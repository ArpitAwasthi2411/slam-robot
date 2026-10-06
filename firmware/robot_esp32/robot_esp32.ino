/*
 * robot_esp32  v2.0  —  ESP32-S3 low-level controller
 * -----------------------------------------------------------------------------
 *  - Reads 2 quadrature encoders and streams raw tick deltas to the Pi (50 Hz)
 *  - Accepts wheel-velocity commands from the Pi and closes a PI speed loop
 *  - FlySky RC has priority over the Pi (manual takeover) + signal-loss failsafe
 *  - Pi command watchdog: no command for 300 ms  -> motors stop
 *  - E-STOP latch ('E'): blocks the Pi AND the RC until released ('R') or the ESP32 resets
 *
 *  Works with arduino-esp32 core 2.x AND 3.x.
 *
 * SERIAL PROTOCOL (115200 baud, newline-terminated ASCII)
 *  ESP32 -> Pi
 *    ODM,<dl>,<dr>,<dt_ms>,<mode>   tick deltas since last ODM, mode 0=IDLE 1=RC 2=PI 3=ESTOP
 *    INFO,<text>                    boot banner / replies
 *  Pi -> ESP32
 *    V,<left_mm_s>,<right_mm_s>     target wheel speeds (closed loop)
 *    P,<left_pwm>,<right_pwm>       raw PWM -255..255 (calibration, open loop)
 *    S                              stop now
 *    E                              e-stop latch ON  (motors off, RC ignored)
 *    R                              release e-stop latch
 *    ?                              print config
 *
 * ARDUINO IDE SETTINGS (Tools menu) for ESP32-S3 Dev Module, board on the
 * native "USB" port (shows up as /dev/ttyACM0 on the Pi):
 *    USB Mode:         Hardware CDC and JTAG
 *    USB CDC On Boot:  Enabled           <-- if Disabled, this file still routes
 *                                            the link to the USB port (see LINK)
 * -----------------------------------------------------------------------------
 */

#include <Arduino.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ---- Which serial object talks to the Pi --------------------------------------
// v1 used `Serial`. With "USB CDC On Boot: Disabled", `Serial` is UART0 (GPIO43/44),
// NOT the USB socket -> the Pi saw /dev/ttyACM0 but received nothing.
// Here we pick the native USB CDC port explicitly in that case.
#if defined(ARDUINO_USB_MODE) && ARDUINO_USB_MODE && defined(ARDUINO_USB_CDC_ON_BOOT) && !ARDUINO_USB_CDC_ON_BOOT
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    #if defined(HWCDC_SERIAL_IS_DEFINED)
      #define LINK HWCDCSerial
    #else
      HWCDC robotUsbLink;          // core 3.0.x only creates HWCDCSerial when CDC-on-boot is on
      #define LINK robotUsbLink
    #endif
  #else
    #define LINK USBSerial         // core 2.x provides this when CDC-on-boot is off
  #endif
#else
  #define LINK Serial
#endif

#define FW_VERSION "2.1"

// ============================ PINS ============================================
#define RC_STEERING_PIN  4    // CH1
#define RC_THROTTLE_PIN  7    // CH2

#define LEFT_RPWM   5
#define LEFT_LPWM   6
#define RIGHT_RPWM  9
#define RIGHT_LPWM  10

#define ENC_LEFT_A   15
#define ENC_LEFT_B   16
#define ENC_RIGHT_A  17
#define ENC_RIGHT_B  18

// ============================ DIRECTION FIXES =================================
// Test 1 (robot on blocks): send "P,120,120" with tools/serial_probe.py.
//   Both wheels must spin FORWARD. If one spins backward, flip its MOTOR_DIR.
// Test 2: roll the robot forward by hand. Both dl and dr must be POSITIVE.
//   If one is negative, flip its ENC_DIR.
#define LEFT_MOTOR_DIR    1
#define RIGHT_MOTOR_DIR   1
#define LEFT_ENC_DIR      1
#define RIGHT_ENC_DIR     1

// ============================ RC CALIBRATION ==================================
#define STEERING_MIN     1105
#define STEERING_CENTER  1504
#define STEERING_MAX     2000
#define THROTTLE_MIN     1212
#define THROTTLE_CENTER  1710
#define THROTTLE_MAX     2000
#define RC_DEADZONE      60
#define RC_TIMEOUT_US    100000UL   // no pulse for 100 ms = receiver lost
#define RC_RANGE_MARGIN  100        // pulses this far outside MIN..MAX = receiver failsafe value
// SET THE RECEIVER FAILSAFE ON THE TRANSMITTER (FS-i6: System/Setup > Failsafe, CH1+CH2 ON,
// with sticks centred). By default FlySky receivers HOLD the last stick position when the
// transmitter is switched off or out of range -> the robot would keep driving.
#define RC_HOLDOFF_MS    1000       // after sticks return to centre, wait before Pi may drive

// ============================ MOTOR / PWM =====================================
#define PWM_FREQ        5000
#define PWM_RES_BITS    8
#define MAX_PWM         200          // hard cap (0..255)

// ============================ SPEED LOOP ======================================
// Calibrate with:  python3 tools/serial_probe.py --calibrate   (robot on blocks)
#define WHEEL_DIAMETER_MM     125.0f
#define TICKS_PER_REV         4740.0f
#define MAX_WHEEL_SPEED_MMS   600.0f  // wheel speed reached at MAX_PWM  (calibrate)
#define PWM_MIN               25.0f   // PWM where the wheel just starts turning (calibrate)
#define KP                    0.15f   // PWM per (mm/s) error
#define KI                    0.40f   // PWM per (mm) integrated error
#define INTEG_LIMIT_PWM       60.0f
#define SPEED_FILTER_ALPHA    0.5f

#define CONTROL_PERIOD_MS     20      // 50 Hz control + ODM output
#define CMD_TIMEOUT_MS        300

// ============================ STATE ===========================================
enum Mode : uint8_t { MODE_IDLE = 0, MODE_RC = 1, MODE_PI = 2, MODE_ESTOP = 3 };

static const float MM_PER_TICK = (PI * WHEEL_DIAMETER_MM) / TICKS_PER_REV;

volatile long enc_left_count = 0;
volatile long enc_right_count = 0;

volatile unsigned long steering_rise = 0, steering_pw = 0, steering_last = 0;
volatile unsigned long throttle_rise = 0, throttle_pw = 0, throttle_last = 0;

struct Motor {
  uint8_t pin_r, pin_l, ch_r, ch_l;
  int8_t dir;
};
Motor motorL = {LEFT_RPWM, LEFT_LPWM, 0, 1, LEFT_MOTOR_DIR};
Motor motorR = {RIGHT_RPWM, RIGHT_LPWM, 2, 3, RIGHT_MOTOR_DIR};

struct WheelLoop {
  float target_mms = 0;
  float speed_mms = 0;   // filtered measurement
  float integ = 0;       // mm
};
WheelLoop loopL, loopR;

long prev_left = 0, prev_right = 0;
long report_dl = 0, report_dr = 0;       // ticks not yet sent to the Pi
unsigned long report_dt = 0;

unsigned long last_control_ms = 0;
unsigned long last_cmd_ms = 0;
unsigned long last_rc_active_ms = 0;
bool have_cmd = false;
bool raw_pwm_mode = false;
bool estop_latched = false;
int raw_pwm_l = 0, raw_pwm_r = 0;
Mode mode = MODE_IDLE;

char rx_buf[64];
uint8_t rx_len = 0;

// ============================ ISRs ============================================
void IRAM_ATTR steeringISR() {
  unsigned long t = micros();
  if (digitalRead(RC_STEERING_PIN)) { steering_rise = t; }
  else { steering_pw = t - steering_rise; steering_last = t; }
}
void IRAM_ATTR throttleISR() {
  unsigned long t = micros();
  if (digitalRead(RC_THROTTLE_PIN)) { throttle_rise = t; }
  else { throttle_pw = t - throttle_rise; throttle_last = t; }
}
// 2x decoding on channel A (keeps the measured 4740 ticks/rev calibration valid)
void IRAM_ATTR encLeftA_ISR() {
  enc_left_count = enc_left_count + ((digitalRead(ENC_LEFT_B) == digitalRead(ENC_LEFT_A)) ? -1 : 1);
}
void IRAM_ATTR encRightA_ISR() {
  enc_right_count = enc_right_count + ((digitalRead(ENC_RIGHT_B) == digitalRead(ENC_RIGHT_A)) ? 1 : -1);
}

// ============================ PWM (core 2.x / 3.x) ============================
void pwmAttach(uint8_t pin, uint8_t ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcAttach(pin, PWM_FREQ, PWM_RES_BITS);
#else
  ledcSetup(ch, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(pin, ch);
#endif
}
void pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(ch, duty);
#endif
}

// pwm: signed, -MAX_PWM..MAX_PWM, positive = robot forward
void setMotor(const Motor &m, int pwm) {
  pwm = constrain(pwm * m.dir, -MAX_PWM, MAX_PWM);
  if (pwm > 0)      { pwmWrite(m.pin_r, m.ch_r, pwm);  pwmWrite(m.pin_l, m.ch_l, 0); }
  else if (pwm < 0) { pwmWrite(m.pin_r, m.ch_r, 0);    pwmWrite(m.pin_l, m.ch_l, -pwm); }
  else              { pwmWrite(m.pin_r, m.ch_r, 0);    pwmWrite(m.pin_l, m.ch_l, 0); }
}

void stopMotors() {
  setMotor(motorL, 0);
  setMotor(motorR, 0);
}

// ============================ RC ==============================================
bool rcValid(volatile unsigned long &last, volatile unsigned long &pw, int minV, int maxV) {
  unsigned long l = last, p = pw;
  return l != 0 && (micros() - l) < RC_TIMEOUT_US &&
         (long)p > (long)(minV - RC_RANGE_MARGIN) && (long)p < (long)(maxV + RC_RANGE_MARGIN);
}

float normalizeRC(unsigned long pw, int minV, int center, int maxV) {
  int v = constrain((int)pw, minV, maxV);
  if (abs(v - center) < RC_DEADZONE) return 0.0f;
  if (v > center) return (float)(v - center - RC_DEADZONE) / (float)(maxV - center - RC_DEADZONE);
  return (float)(v - center + RC_DEADZONE) / (float)(center - minV - RC_DEADZONE);
}

// ============================ SPEED LOOP ======================================
int speedLoop(WheelLoop &w, float dt_s) {
  if (fabsf(w.target_mms) < 1.0f) {     // zero target -> coast to stop, no creep
    w.integ = 0;
    return 0;
  }
  float err = w.target_mms - w.speed_mms;
  w.integ += err * dt_s;
  float ilim = INTEG_LIMIT_PWM / KI;
  w.integ = constrain(w.integ, -ilim, ilim);

  float sgn = w.target_mms > 0 ? 1.0f : -1.0f;
  float ff = PWM_MIN + (fabsf(w.target_mms) / MAX_WHEEL_SPEED_MMS) * (MAX_PWM - PWM_MIN);
  float out = sgn * ff + KP * err + KI * w.integ;
  // never let the PI term reverse the wheel against the commanded direction
  if (out * sgn < 0) out = 0;
  return (int)constrain(out, -(float)MAX_PWM, (float)MAX_PWM);
}

// ============================ SERIAL RX =======================================
long parseLong(const char *&p) {
  while (*p == ',' || *p == ' ') p++;
  return strtol(p, (char **)&p, 10);
}

void handleLine(char *line) {
  const char *p = line + 1;
  switch (line[0]) {
    case 'V': {
      long l = parseLong(p), r = parseLong(p);
      loopL.target_mms = constrain(l, -2000L, 2000L);
      loopR.target_mms = constrain(r, -2000L, 2000L);
      raw_pwm_mode = false;
      have_cmd = true;
      last_cmd_ms = millis();
      break;
    }
    case 'P': {
      raw_pwm_l = constrain(parseLong(p), -255L, 255L);
      raw_pwm_r = constrain(parseLong(p), -255L, 255L);
      raw_pwm_mode = true;
      have_cmd = true;
      last_cmd_ms = millis();
      break;
    }
    case 'S':
      have_cmd = false;
      loopL.target_mms = loopR.target_mms = 0;
      stopMotors();
      break;
    case 'E':
      estop_latched = true;
      have_cmd = false;
      stopMotors();
      break;
    case 'R':
      estop_latched = false;
      break;
    case '?':
      LINK.printf("INFO,robot_esp32 v%s core%d mm_per_tick=%.5f max_mms=%.0f pwm_min=%.0f kp=%.3f ki=%.3f\n",
                  FW_VERSION, ESP_ARDUINO_VERSION_MAJOR, MM_PER_TICK, MAX_WHEEL_SPEED_MMS, PWM_MIN, KP, KI);
      break;
    default:
      break;
  }
}

void pollSerial() {
  while (LINK.available()) {
    char c = (char)LINK.read();
    if (c == '\n' || c == '\r') {
      if (rx_len > 0) {
        rx_buf[rx_len] = '\0';
        handleLine(rx_buf);
        rx_len = 0;
      }
    } else if (rx_len < sizeof(rx_buf) - 1) {
      rx_buf[rx_len++] = c;
    } else {
      rx_len = 0;  // overflow: drop the line
    }
  }
}

// ============================ SETUP ===========================================
void setup() {
  LINK.begin(115200);

  pinMode(RC_STEERING_PIN, INPUT);
  pinMode(RC_THROTTLE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RC_STEERING_PIN), steeringISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_THROTTLE_PIN), throttleISR, CHANGE);

  pwmAttach(motorL.pin_r, motorL.ch_r);
  pwmAttach(motorL.pin_l, motorL.ch_l);
  pwmAttach(motorR.pin_r, motorR.ch_r);
  pwmAttach(motorR.pin_l, motorR.ch_l);
  stopMotors();

  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), encLeftA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encRightA_ISR, CHANGE);

  delay(300);
  last_control_ms = millis();
  LINK.printf("INFO,READY,robot_esp32 v%s\n", FW_VERSION);
}

// ============================ LOOP ============================================
void loop() {
  pollSerial();

  unsigned long now = millis();
  unsigned long dt_ms = now - last_control_ms;
  if (dt_ms < CONTROL_PERIOD_MS) return;
  last_control_ms = now;
  float dt_s = dt_ms / 1000.0f;

  // ---- encoders ----
  noInterrupts();
  long l_now = enc_left_count;
  long r_now = enc_right_count;
  interrupts();
  long dl = (l_now - prev_left) * LEFT_ENC_DIR;
  long dr = (r_now - prev_right) * RIGHT_ENC_DIR;
  prev_left = l_now;
  prev_right = r_now;

  float a = SPEED_FILTER_ALPHA;
  loopL.speed_mms = a * (dl * MM_PER_TICK / dt_s) + (1 - a) * loopL.speed_mms;
  loopR.speed_mms = a * (dr * MM_PER_TICK / dt_s) + (1 - a) * loopR.speed_mms;

  // ---- arbitration: RC > Pi > idle ----
  bool rc_ok = rcValid(steering_last, steering_pw, STEERING_MIN, STEERING_MAX) &&
               rcValid(throttle_last, throttle_pw, THROTTLE_MIN, THROTTLE_MAX);
  float thr = rc_ok ? normalizeRC(throttle_pw, THROTTLE_MIN, THROTTLE_CENTER, THROTTLE_MAX) : 0.0f;
  float str = rc_ok ? normalizeRC(steering_pw, STEERING_MIN, STEERING_CENTER, STEERING_MAX) : 0.0f;
  bool rc_active = (thr != 0.0f || str != 0.0f);
  if (rc_active) last_rc_active_ms = now;

  bool pi_fresh = have_cmd && (now - last_cmd_ms) < CMD_TIMEOUT_MS;
  bool rc_holdoff = last_rc_active_ms != 0 && (now - last_rc_active_ms) < RC_HOLDOFF_MS;

  if (estop_latched) {
    mode = MODE_ESTOP;
    stopMotors();
    loopL.integ = loopR.integ = 0;
    have_cmd = false;
  } else if (rc_active) {
    mode = MODE_RC;
    float ls = constrain(thr + str, -1.0f, 1.0f);
    float rs = constrain(thr - str, -1.0f, 1.0f);
    setMotor(motorL, (int)(ls * MAX_PWM));
    setMotor(motorR, (int)(rs * MAX_PWM));
    loopL.integ = loopR.integ = 0;
  } else if (pi_fresh && !rc_holdoff) {
    mode = MODE_PI;
    if (raw_pwm_mode) {
      setMotor(motorL, raw_pwm_l);
      setMotor(motorR, raw_pwm_r);
    } else {
      setMotor(motorL, speedLoop(loopL, dt_s));
      setMotor(motorR, speedLoop(loopR, dt_s));
    }
  } else {
    mode = MODE_IDLE;
    stopMotors();
    loopL.integ = loopR.integ = 0;
    if (!pi_fresh) have_cmd = false;
  }

  // ---- odometry to Pi (never blocks the control loop) ----
  report_dl += dl;
  report_dr += dr;
  report_dt += dt_ms;
  if (LINK.availableForWrite() >= 40) {
    LINK.printf("ODM,%ld,%ld,%lu,%u\n", report_dl, report_dr, report_dt, (unsigned)mode);
    report_dl = report_dr = 0;
    report_dt = 0;
  } else if (report_dt > 1000) {
    // nobody has been reading for >1 s (Pi rebooting/bridge restarting): drop the backlog
    // instead of sending one huge stale packet later
    report_dl = report_dr = 0;
    report_dt = 0;
  }
}
