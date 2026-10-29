/*
 * robot_esp32_classic  v2.4  —  CLASSIC ESP32 ONLY (ESP32-WROOM-32 DevKit)
 * -----------------------------------------------------------------------------
 *  Same firmware as firmware/robot_esp32 v2.2, with the ESP32-S3 parts removed.
 *  Arduino IDE: Tools > Board > "ESP32 Dev Module", Upload speed 115200 if uploads fail.
 *  If upload says "Failed to communicate with the flash chip": unplug everything except
 *  USB, flash, then reconnect the wiring.
 *
 *  - Reads 2 quadrature encoders and streams raw tick deltas to the Pi (50 Hz)
 *  - Accepts wheel-velocity commands from the Pi and closes a PI speed loop
 *  - FlySky RC has priority over the Pi (manual takeover) + signal-loss failsafe
 *  - Pi command watchdog: no command for 300 ms  -> motors stop
 *  - E-STOP latch ('E' or the mushroom button): blocks the Pi AND the RC until released ('R')
 *  - MPU-6050 gyro (yaw added to ODM), 3x HC-SR04 ultrasonic (US lines), e-stop button input.
 *    Each can be switched off below (USE_IMU / USE_ULTRASONIC / USE_ESTOP_BUTTON).
 *  - v2.4: battery voltage + current (INA226 on the same I2C bus, or a divider on GPIO 36)
 *    -> BAT lines; bumper switches (all in parallel) on GPIO 39 -> BMP lines + instant stop.
 *
 * WIRING (classic ESP32 GPIO)
 *   RC: forward/back channel -> 34, left/right channel -> 35
 *   Left BTS7960  RPWM 25, LPWM 26      Right BTS7960  RPWM 32, LPWM 33
 *   All 4 EN pins: 5V -> e-stop NC contact -> R_EN/L_EN (+10k to GND)
 *   Encoders  left A 18, B 19   right A 4, B 13   (3.3 V)
 *   MPU-6050  SDA 21, SCL 22, VCC 3.3V, AD0 GND
 *   HC-SR04   TRIG L/C/R 14/16/17   ECHO L/C/R -> level shifter -> 23/27/15
 *   E-stop sense GPIO 5 -> NO contact -> GND
 *   INA226    SDA 21, SCL 22 (shared with the MPU), VCC 3.3V, addr 0x40 (A0, A1 to GND)
 *             VBUS -> battery +, IN+ / IN- in series with the battery + lead (see hardware/battery_dock.md)
 *   Battery divider (only if there is no INA226): battery+ -> 100k -> GPIO 36 -> 22k -> GND
 *   Bumpers   GPIO 39 -> all switches (NO) in parallel -> GND, 10k pull-up from 39 to 3.3V
 *             (39 has no internal pull-up)
 *   NEVER use 6-11 (flash), 1/3 (USB to Pi), 0/2/12 (boot).
 *
 * SERIAL PROTOCOL (115200 baud, newline-terminated ASCII)
 *  ESP32 -> Pi
 *    ODM,<dl>,<dr>,<dt_ms>,<mode>[,<dyaw_urad>]   mode 0=IDLE 1=RC 2=PI 3=ESTOP
 *    IMU,<gz_mrad_s>,<ax_mm_s2>,<ay_mm_s2>,<az_mm_s2>
 *    US,<left_mm>,<center_mm>,<right_mm>   0 = no echo / out of range
 *    INFO,<text>
 *    BAT,<mV>,<mA>,<src>   2 Hz; mA > 0 = discharging, < 0 = charging; src 1 = INA226, 2 = divider
 *    BMP,<0|1>             bumper released / pressed (pressed also stops the motors at once)
 *  Pi -> ESP32
 *    V,<left_mm_s>,<right_mm_s>   P,<left_pwm>,<right_pwm>   S stop   E e-stop   R release
 *    C print RC pulses   G re-calibrate gyro   ? print config
 *  v2.3 tuning (used by the mobile app's Tuning Lab):
 *    K,<kp>,<ki>,<pwm_min>,<max_mms>,<accel_mms2>   set speed-loop gains live
 *    W  save them to flash (kept after power-off)      X  back to factory defaults
 *    T,1 / T,0  stream / stop  SPD,<tgtL>,<measL>,<pwmL>,<tgtR>,<measR>,<pwmR>  (50 Hz)
 *    ESP32 -> Pi:  INFO,TUNE,<kp>,<ki>,<pwm_min>,<max_mms>,<accel>
 *  v2.5 front docking + IMU check:
 *    D,1 / D,0  docking mode: the front ultrasonic stop drops from US_STOP_MM to US_STOP_DOCK_MM, only while
 *               the Pi refreshes D,1 (every < 2 s) and only at <= DOCK_MAX_MMS forward (creep speed)
 *    I          I2C scan -> INFO,I2C,0x40,0x68 (which chips answer: MPU-6050 0x68, INA226 0x40)
 * -----------------------------------------------------------------------------
 */

#include <Arduino.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

#if !defined(CONFIG_IDF_TARGET_ESP32)
  #error "This sketch is for the classic ESP32: Tools > Board > ESP32 Dev Module"
#endif

#define LINK Serial               // classic ESP32: USB-UART chip -> /dev/ttyUSB0 on the Pi
#define FW_VERSION "2.5"
#include <Wire.h>
#include <Preferences.h>     // tuning values survive reboot (ESP32 flash)
#include <esp_system.h>      // esp_reset_reason(): why the ESP32 restarted (sent in the READY line)

// ============================ PINS (classic ESP32) ============================
#define BOARD_NAME   "ESP32"
#define RC_STEERING_PIN  35   // left/right stick channel (your wiring: the wire on 35)
#define RC_THROTTLE_PIN  34   // forward/back stick channel (the wire on 34)
#define LEFT_RPWM   25
#define LEFT_LPWM   26
#define RIGHT_RPWM  32
#define RIGHT_LPWM  33
#define ENC_LEFT_A   18
#define ENC_LEFT_B   19
#define ENC_RIGHT_A  4
#define ENC_RIGHT_B  13

// ============================ OPTIONAL SENSORS ================================
// Set to 0 for anything not wired yet. All are safe to leave on with nothing connected:
// a missing IMU is detected at boot, echo pins are pulled down, e-stop pin idles HIGH.
#define USE_IMU            1     // MPU-6050 (GY-521) on I2C
#define USE_ULTRASONIC     1     // 3x HC-SR04, ECHO through a level shifter / divider
#define USE_ESTOP_BUTTON   1     // mushroom e-stop sense input
#define ESTOP_PRESSED_LEVEL LOW  // NO contact to GND, or EN-line divider: both read LOW when pressed

#define USE_INA226         1     // battery monitor on I2C (falls back to the divider if not found)
#define USE_BAT_ADC        0     // 1 = divider on GPIO 36 (only if there is no INA226; leave 0 if 36 is unwired)
#define USE_BUMPER         0     // set 1 once the bumper switches are wired to GPIO 39
#define INA226_ADDR       0x40
#define SHUNT_OHMS        0.01f  // !! common INA226 boards ship with R100 (0.1 ohm = max 0.8 A).
                                 //    Swap it for an R010 (0.01 ohm, 2512 size) for up to 8 A.
#define BAT_CURRENT_SIGN  1      // set -1 if the app shows "charging" while driving
#define BAT_ADC_PIN       36
#define BAT_DIVIDER       ((100.0f + 22.0f) / 22.0f)
#define BAT_PERIOD_MS     500
#define BUMPER_PIN        39
#define BUMPER_HOLD_MS    500    // ignore Pi commands this long after a bump (the Pi replans)
#define I2C_SDA      21
#define I2C_SCL      22
#define ESTOP_PIN    5
#define US_TRIG_L    14
#define US_TRIG_C    16
#define US_TRIG_R    17
#define US_ECHO_L    23
#define US_ECHO_C    27
#define US_ECHO_R    15

#define IMU_ADDR          0x68   // AD0 low (GY-521 default)
#define IMU_YAW_SIGN      1      // set -1 if turning LEFT (CCW seen from above) gives negative gz
#define IMU_PERIOD_US     5000   // 200 Hz gyro integration
#define US_SLOT_MS        25     // one sensor fired every 25 ms -> each sensor ~13 Hz
#define US_MAX_MM         4000
#define US_STOP_MM        150    // AUTO mode: obstacle this close in front -> forward motion removed
// Front docking (D,1 from the Pi): the dock itself is in front of the sensors, so the stop distance is
// relaxed ONLY while docking mode is refreshed (every < 2 s) AND the commanded speed is creep speed.
#define US_STOP_DOCK_MM    30    // docking mode: stop only this close (contacts touch at ~0)
#define DOCK_MAX_MMS       60    // docking mode applies only at or below this forward speed (mm/s)
#define DOCK_MODE_HOLD_MS 2000   // docking mode ends by itself without a fresh D,1

// ============================ DIRECTION FIXES =================================
// Test 1 (robot on blocks): send "P,120,120" with tools/serial_probe.py.
//   Both wheels must spin FORWARD. If one spins backward, flip its MOTOR_DIR.
// Test 2: roll the robot forward by hand. Both dl and dr must be POSITIVE.
//   If one is negative, flip its ENC_DIR.
#define LEFT_MOTOR_DIR    -1   // measured 2026-10-07: left wheel ran backward on forward PWM
#define RIGHT_MOTOR_DIR   1
#define LEFT_ENC_DIR      -1   // measured: --push gave negative left ticks rolling forward
#define RIGHT_ENC_DIR     -1   // measured: right speed negative on forward PWM

// ============================ RC CALIBRATION ==================================
// Measured 2026-10-07 with rc_reader (FS-i6, two sticks: forward/back on one, left/right on the other)
#define STEERING_MIN     1113     // stick full RIGHT
#define STEERING_CENTER  1609
#define STEERING_MAX     1999     // stick full LEFT
#define STEERING_REVERSE 1        // 1 = higher pulse means LEFT (yours). 0 = higher means right
#define THROTTLE_MIN     1034     // full back
#define THROTTLE_CENTER  1540
#define THROTTLE_MAX     1996     // full forward
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
#define TICKS_PER_REV         4970.0f // measured with serial_probe --push (avg of both wheels)
// Defaults below are only used until you tune from the app ("K" command) and save ("W").
#define DEF_MAX_WHEEL_SPEED_MMS 493.0f  // wheel speed reached at MAX_PWM  (calibrate)
#define DEF_PWM_MIN           15.0f   // PWM where the wheel just starts turning (calibrate)
#define DEF_KP                0.15f   // PWM per (mm/s) error
#define DEF_KI                0.40f   // PWM per (mm) integrated error
#define DEF_ACCEL_MMS2        600.0f  // wheel-speed ramp (mm/s^2); 0 = no ramp (jerky)
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

// live-tunable (K command), saved to flash with W
float g_kp = DEF_KP, g_ki = DEF_KI, g_pwm_min = DEF_PWM_MIN;
float g_max_mms = DEF_MAX_WHEEL_SPEED_MMS, g_accel = DEF_ACCEL_MMS2;
bool telemetry = false;            // T,1: stream SPD lines (target/measured/pwm per wheel)
int last_pwm_l = 0, last_pwm_r = 0;
Preferences prefs;

struct WheelLoop {
  float cmd_mms = 0;     // what the Pi asked for
  float target_mms = 0;  // ramped toward cmd_mms at g_accel
  float speed_mms = 0;   // filtered measurement
  float integ = 0;       // mm
};
WheelLoop loopL, loopR;

long prev_left = 0, prev_right = 0;
long report_dl = 0, report_dr = 0;       // ticks not yet sent to the Pi
unsigned long report_dt = 0;

unsigned long last_control_ms = 0;
unsigned long last_cmd_ms = 0;
unsigned long dock_mode_until = 0;      // docking mode (D,1) active until this millis()
unsigned long last_rc_active_ms = 0;
bool have_cmd = false;
bool raw_pwm_mode = false;
bool estop_latched = false;
int raw_pwm_l = 0, raw_pwm_r = 0;
Mode mode = MODE_IDLE;

char rx_buf[64];
uint8_t rx_len = 0;

// IMU state
bool imu_ok = false;
uint8_t imu_whoami = 0;
float gyro_bias_z = 0;          // raw LSB
float gz_rad_s = 0, ax_ms2 = 0, ay_ms2 = 0, az_ms2 = 0;
double yaw_accum = 0;           // rad integrated since last ODM
unsigned long last_imu_us = 0;
uint8_t imu_fail = 0;

// ultrasonic state
volatile unsigned long us_rise[3] = {0, 0, 0}, us_pw[3] = {0, 0, 0};
volatile bool us_done[3] = {false, false, false};
uint16_t us_mm[3] = {0, 0, 0};
uint8_t us_slot = 0;
unsigned long last_us_ms = 0;
const uint8_t US_TRIG[3] = {US_TRIG_L, US_TRIG_C, US_TRIG_R};
const uint8_t US_ECHO[3] = {US_ECHO_L, US_ECHO_C, US_ECHO_R};

// e-stop button
uint8_t estop_btn_count = 0;
bool estop_btn = false;

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
// 2x decoding on channel A (TICKS_PER_REV above was measured with this decoding)
void IRAM_ATTR encLeftA_ISR() {
  enc_left_count = enc_left_count + ((digitalRead(ENC_LEFT_B) == digitalRead(ENC_LEFT_A)) ? -1 : 1);
}
void IRAM_ATTR encRightA_ISR() {
  enc_right_count = enc_right_count + ((digitalRead(ENC_RIGHT_B) == digitalRead(ENC_RIGHT_A)) ? 1 : -1);
}

#if USE_ULTRASONIC
static inline void IRAM_ATTR echoEdge(uint8_t i) {
  unsigned long t = micros();
  if (digitalRead(US_ECHO[i])) { us_rise[i] = t; }
  else if (us_rise[i]) { us_pw[i] = t - us_rise[i]; us_done[i] = true; }
}
void IRAM_ATTR echoL_ISR() { echoEdge(0); }
void IRAM_ATTR echoC_ISR() { echoEdge(1); }
void IRAM_ATTR echoR_ISR() { echoEdge(2); }
#endif

// ============================ IMU (MPU-6050) ===================================
bool imuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool imuRead(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)IMU_ADDR, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// returns raw gz (LSB) and fills accel in m/s^2; false on I2C error
bool imuSample(int16_t &gz_raw) {
  uint8_t b[14];
  if (!imuRead(0x3B, b, 14)) return false;
  int16_t ax = (b[0] << 8) | b[1], ay = (b[2] << 8) | b[3], az = (b[4] << 8) | b[5];
  gz_raw = (b[12] << 8) | b[13];
  const float g = 9.80665f / 16384.0f;          // +-2 g range
  ax_ms2 = ax * g; ay_ms2 = ay * g; az_ms2 = az * g;
  return true;
}

void imuCalibrate() {
  // average the gyro while the robot is still (1 s)
  double sum = 0; int n = 0;
  for (int i = 0; i < 200; i++) {
    int16_t gz;
    if (imuSample(gz)) { sum += gz; n++; }
    delay(5);
  }
  if (n > 50) gyro_bias_z = sum / n;
}

void imuInit() {
#if USE_IMU
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(5);                            // never hang the control loop on a bad bus
  uint8_t who = 0;
  if (!imuRead(0x75, &who, 1) || who == 0x00 || who == 0xFF) { imu_ok = false; return; }
  imu_whoami = who;                              // 0x68 genuine, 0x70/0x72/0x98 common clones
  imu_ok = imuWrite(0x6B, 0x01) &&               // wake, clock = gyro X PLL
           imuWrite(0x1A, 0x03) &&               // DLPF 44 Hz
           imuWrite(0x1B, 0x08) &&               // gyro +-500 dps (65.5 LSB per deg/s)
           imuWrite(0x1C, 0x00);                 // accel +-2 g
  if (imu_ok) { delay(50); imuCalibrate(); last_imu_us = micros(); }
#endif
}

void imuUpdate() {
#if USE_IMU
  if (!imu_ok) return;
  unsigned long now = micros();
  if (now - last_imu_us < IMU_PERIOD_US) return;
  float dt = (now - last_imu_us) * 1e-6f;
  last_imu_us = now;
  int16_t gz;
  if (!imuSample(gz)) {
    if (++imu_fail > 20) { imu_ok = false; LINK.printf("INFO,IMU lost (I2C errors)\n"); }
    return;
  }
  imu_fail = 0;
  const float LSB_TO_RAD = (1.0f / 65.5f) * (PI / 180.0f);
  gz_rad_s = IMU_YAW_SIGN * (gz - gyro_bias_z) * LSB_TO_RAD;
  if (fabsf(gz_rad_s) < 0.003f) gz_rad_s = 0;    // tiny dead band against residual drift
  yaw_accum += gz_rad_s * dt;
#endif
}

// ============================ ULTRASONIC =======================================
void usInit() {
#if USE_ULTRASONIC
  for (int i = 0; i < 3; i++) {
    pinMode(US_TRIG[i], OUTPUT);
    digitalWrite(US_TRIG[i], LOW);
    pinMode(US_ECHO[i], INPUT_PULLDOWN);   // unconnected sensor = no edges = "nothing in range"
  }
  attachInterrupt(digitalPinToInterrupt(US_ECHO_L), echoL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(US_ECHO_C), echoC_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(US_ECHO_R), echoR_ISR, CHANGE);
#endif
}

void usUpdate(unsigned long now_ms) {
#if USE_ULTRASONIC
  if (now_ms - last_us_ms < US_SLOT_MS) return;
  last_us_ms = now_ms;
  // close the previous slot: an echo that never finished = nothing in range
  uint8_t prev = us_slot;
  if (us_done[prev]) {
    unsigned long mm = (unsigned long)(us_pw[prev] * 0.1715f);
    us_mm[prev] = (mm >= 20 && mm <= US_MAX_MM) ? (uint16_t)mm : 0;
  } else {
    us_mm[prev] = 0;
  }
  if (prev == 2 && LINK.availableForWrite() >= 30) {
    LINK.printf("US,%u,%u,%u\n", us_mm[0], us_mm[1], us_mm[2]);
  }
  // fire the next one
  us_slot = (us_slot + 1) % 3;
  us_done[us_slot] = false;
  us_rise[us_slot] = 0;
  digitalWrite(US_TRIG[us_slot], HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG[us_slot], LOW);
#else
  (void)now_ms;
#endif
}

bool dockModeActive() { return dock_mode_until && (long)(dock_mode_until - millis()) > 0; }

// fwd_mms: the commanded forward speed. In docking mode, at creep speed, only a very close echo blocks.
bool frontBlocked(float fwd_mms) {
#if USE_ULTRASONIC
  uint16_t limit = (dockModeActive() && fwd_mms <= DOCK_MAX_MMS) ? US_STOP_DOCK_MM : US_STOP_MM;
  for (int i = 0; i < 3; i++)
    if (us_mm[i] && us_mm[i] < limit) return true;
#else
  (void)fwd_mms;
#endif
  return false;
}

// I2C bus scan (command I): which chips answer (MPU-6050 0x68/0x69, INA226 0x40..0x4F)
void i2cScan() {
  char buf[160];
  int n = snprintf(buf, sizeof(buf), "INFO,I2C");
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      found++;
      if (n < (int)sizeof(buf) - 8) n += snprintf(buf + n, sizeof(buf) - n, ",0x%02X", a);
    }
  }
  if (!found) snprintf(buf + n, sizeof(buf) - n, ",none (check SDA/SCL, 3.3V, GND)");
  LINK.printf("%s\n", buf);
}

// ============================ E-STOP BUTTON ====================================
void estopButtonUpdate() {
#if USE_ESTOP_BUTTON
  bool pressed = digitalRead(ESTOP_PIN) == ESTOP_PRESSED_LEVEL;
  if (pressed) { if (estop_btn_count < 3) estop_btn_count++; }
  else estop_btn_count = 0;
  bool now = estop_btn_count >= 3;                  // 3 control ticks = 60 ms debounce
  if (now && !estop_btn) LINK.printf("INFO,ESTOP button pressed\n");
  if (!now && estop_btn) LINK.printf("INFO,ESTOP button released (still latched: release from the dashboard)\n");
  estop_btn = now;
  if (estop_btn) estop_latched = true;
#endif
}

// ============================ BATTERY (INA226 / divider) =======================
static bool ina_ok = false;
static unsigned long last_bat_ms = 0;
static uint8_t bump_count = 0;
static bool bumped = false;
static unsigned long bump_ms = 0;

static bool i2cWrite16(uint8_t addr, uint8_t reg, uint16_t v) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(v >> 8);
  Wire.write(v & 0xFF);
  return Wire.endTransmission() == 0;
}

static bool i2cRead16(uint8_t addr, uint8_t reg, uint16_t *v) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 2) != 2) return false;
  *v = ((uint16_t)Wire.read() << 8) | Wire.read();
  return true;
}

void batInit() {
#if USE_INA226
  Wire.begin(I2C_SDA, I2C_SCL);                  // harmless if the IMU already started the bus
  Wire.setTimeOut(5);
  uint16_t id = 0;
  // manufacturer ID register 0xFE reads 0x5449 ("TI") on a real INA226
  ina_ok = i2cRead16(INA226_ADDR, 0xFE, &id) && id == 0x5449 &&
           i2cWrite16(INA226_ADDR, 0x00, 0x4527);  // 16 averages, 1.1 ms conversions, continuous
#endif
#if USE_BAT_ADC
  if (!ina_ok) { analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db); }
#endif
  LINK.printf("INFO,battery sensor: %s\n", ina_ok ? "INA226" : (USE_BAT_ADC ? "divider on GPIO 36" : "none"));
}

void batUpdate(unsigned long now) {
  if (now - last_bat_ms < BAT_PERIOD_MS) return;
  last_bat_ms = now;
  long mv = 0, ma = 0;
  int src = 0;
  if (ina_ok) {
    uint16_t bus = 0, shunt = 0;
    if (i2cRead16(INA226_ADDR, 0x02, &bus) && i2cRead16(INA226_ADDR, 0x01, &shunt)) {
      mv = (long)(bus * 1.25f);                                    // 1.25 mV per bit
      float shunt_uv = (int16_t)shunt * 2.5f;                       // 2.5 uV per bit, signed
      ma = (long)(BAT_CURRENT_SIGN * shunt_uv / SHUNT_OHMS / 1000.0f);
      src = 1;
    }
  }
#if USE_BAT_ADC
  if (!src) {
    mv = (long)(analogReadMilliVolts(BAT_ADC_PIN) * BAT_DIVIDER);
    src = mv > 3000 ? 2 : 0;                                        // nothing connected reads ~0
  }
#endif
  if (src && LINK.availableForWrite() >= 32) LINK.printf("BAT,%ld,%ld,%d\n", mv, ma, src);
}

// ============================ BUMPERS ==========================================
void bumperUpdate(unsigned long now) {
#if USE_BUMPER
  bool pressed = digitalRead(BUMPER_PIN) == LOW;
  if (pressed) { if (bump_count < 2) bump_count++; } else bump_count = 0;
  bool b = bump_count >= 2;                          // 40 ms debounce
  if (b && !bumped) { bump_ms = now; have_cmd = false; LINK.printf("BMP,1\n"); }
  if (!b && bumped) LINK.printf("BMP,0\n");
  bumped = b;
#endif
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
  float ilim = INTEG_LIMIT_PWM / fmaxf(g_ki, 0.01f);
  w.integ = constrain(w.integ, -ilim, ilim);

  float sgn = w.target_mms > 0 ? 1.0f : -1.0f;
  float ff = g_pwm_min + (fabsf(w.target_mms) / g_max_mms) * (MAX_PWM - g_pwm_min);
  float out = sgn * ff + g_kp * err + g_ki * w.integ;
  // never let the PI term reverse the wheel against the commanded direction
  if (out * sgn < 0) out = 0;
  return (int)constrain(out, -(float)MAX_PWM, (float)MAX_PWM);
}

// Ramp both wheel targets toward the commanded speeds together, keeping their ratio
// (so the curve the Pi asked for is kept while accelerating) -> no jerks.
void rampTargets(float dt_s) {
  float dl = loopL.cmd_mms - loopL.target_mms, dr = loopR.cmd_mms - loopR.target_mms;
  float big = fmaxf(fabsf(dl), fabsf(dr));
  float step = g_accel * dt_s;
  if (g_accel <= 0 || big <= step) {
    loopL.target_mms = loopL.cmd_mms;
    loopR.target_mms = loopR.cmd_mms;
  } else {
    float k = step / big;
    loopL.target_mms += dl * k;
    loopR.target_mms += dr * k;
  }
}

void printTune() {
  LINK.printf("INFO,TUNE,%.4f,%.4f,%.1f,%.0f,%.0f\n", g_kp, g_ki, g_pwm_min, g_max_mms, g_accel);
}

void loadTune() {
  prefs.begin("tune", true);
  g_kp = prefs.getFloat("kp", DEF_KP);
  g_ki = prefs.getFloat("ki", DEF_KI);
  g_pwm_min = prefs.getFloat("pmin", DEF_PWM_MIN);
  g_max_mms = prefs.getFloat("vmax", DEF_MAX_WHEEL_SPEED_MMS);
  g_accel = prefs.getFloat("acc", DEF_ACCEL_MMS2);
  prefs.end();
}

void saveTune() {
  prefs.begin("tune", false);
  prefs.putFloat("kp", g_kp);
  prefs.putFloat("ki", g_ki);
  prefs.putFloat("pmin", g_pwm_min);
  prefs.putFloat("vmax", g_max_mms);
  prefs.putFloat("acc", g_accel);
  prefs.end();
}

// ============================ SERIAL RX =======================================
float parseFloat(const char *&p) {
  while (*p == ',' || *p == ' ') p++;
  return strtof(p, (char **)&p);
}

long parseLong(const char *&p) {
  while (*p == ',' || *p == ' ') p++;
  return strtol(p, (char **)&p, 10);
}

void handleLine(char *line) {
  const char *p = line + 1;
  switch (line[0]) {
    case 'V': {
      long l = parseLong(p), r = parseLong(p);
      loopL.cmd_mms = constrain(l, -2000L, 2000L);
      loopR.cmd_mms = constrain(r, -2000L, 2000L);
      raw_pwm_mode = false;
      have_cmd = true;
      last_cmd_ms = millis();
      break;
    }
    case 'P': {
      // parse into locals first: constrain() is a macro and would call parseLong() up to 3 times
      long pl = parseLong(p), pr = parseLong(p);
      raw_pwm_l = constrain(pl, -255L, 255L);
      raw_pwm_r = constrain(pr, -255L, 255L);
      raw_pwm_mode = true;
      have_cmd = true;
      last_cmd_ms = millis();
      break;
    }
    case 'S':
      have_cmd = false;
      loopL.target_mms = loopR.target_mms = 0;
      loopL.cmd_mms = loopR.cmd_mms = 0;
      stopMotors();
      break;
    case 'K': {   // K,kp,ki,pwm_min,max_mms,accel   (live tuning; W saves to flash)
      float kp = parseFloat(p), ki = parseFloat(p), pmin = parseFloat(p), vmax = parseFloat(p), acc = parseFloat(p);
      g_kp = constrain(kp, 0.0f, 2.0f);
      g_ki = constrain(ki, 0.0f, 5.0f);
      g_pwm_min = constrain(pmin, 0.0f, 120.0f);
      g_max_mms = constrain(vmax, 100.0f, 2000.0f);
      g_accel = constrain(acc, 0.0f, 5000.0f);
      loopL.integ = loopR.integ = 0;
      printTune();
      break;
    }
    case 'W':
      saveTune();
      LINK.printf("INFO,TUNE_SAVED\n");
      printTune();
      break;
    case 'X':     // factory defaults (and clear flash)
      prefs.begin("tune", false); prefs.clear(); prefs.end();
      loadTune();
      printTune();
      break;
    case 'D': {   // D,1 = docking mode (refresh every < 2 s), D,0 = off
      long on = parseLong(p);
      dock_mode_until = on ? millis() + DOCK_MODE_HOLD_MS : 0;
      if (on && !dock_mode_until) dock_mode_until = 1;     // millis() wrapped to 0: still "on"
      break;
    }
    case 'I':     // I2C scan
      i2cScan();
      break;
    case 'T': {   // T,1 = stream SPD telemetry, T,0 = stop
      long on = parseLong(p);
      telemetry = on != 0;
      break;
    }
    case 'E':
      estop_latched = true;
      have_cmd = false;
      stopMotors();
      break;
    case 'R':
      if (!estop_btn) estop_latched = false;      // a pressed mushroom button cannot be overridden
      else LINK.printf("INFO,ESTOP button still pressed: twist it out first\n");
      break;
    case 'G':
#if USE_IMU
      if (imu_ok) { imuCalibrate(); LINK.printf("INFO,gyro bias %.1f\n", gyro_bias_z); }
#endif
      break;
    case 'C': {   // RC calibration readout: raw pulse widths in microseconds (0 = no signal)
      unsigned long now_us = micros();
      unsigned long sp = steering_pw, tp = throttle_pw, sl = steering_last, tl = throttle_last;
      LINK.printf("INFO,RC,%lu,%lu\n",
                  (sl && now_us - sl < RC_TIMEOUT_US) ? sp : 0UL,
                  (tl && now_us - tl < RC_TIMEOUT_US) ? tp : 0UL);
      break;
    }
    case '?':
      LINK.printf("INFO,robot_esp32 v%s %s core%d mm_per_tick=%.5f max_mms=%.0f pwm_min=%.0f kp=%.3f ki=%.3f accel=%.0f\n",
                  FW_VERSION, BOARD_NAME, ESP_ARDUINO_VERSION_MAJOR, MM_PER_TICK, g_max_mms, g_pwm_min, g_kp, g_ki, g_accel);
      printTune();
      LINK.printf("INFO,uptime_s=%lu\n", millis() / 1000UL);   // the bridge spots a silent reboot from this
      LINK.printf("INFO,dock_mode=%s (stop %d mm at <= %d mm/s)\n", dockModeActive() ? "on" : "off", US_STOP_DOCK_MM, DOCK_MAX_MMS);
      LINK.printf("INFO,i2c SDA=%d SCL=%d (send I to scan the bus)\n", I2C_SDA, I2C_SCL);
      LINK.printf("INFO,sensors imu=%s(who=0x%02X bias=%.1f) ultrasonic=%s estop_button=%s(%s)\n",
                  USE_IMU ? (imu_ok ? "ok" : "FAIL") : "off", imu_whoami, gyro_bias_z,
                  USE_ULTRASONIC ? "on" : "off", USE_ESTOP_BUTTON ? "on" : "off",
                  estop_btn ? "PRESSED" : "released");
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

// Why did we (re)start? BROWNOUT = supply dipped below ~2.4 V (motors pulling the 5 V/3.3 V down),
// POWERON = cold start or USB power lost, EXT = EN/RESET pin (USB-serial DTR/RTS or the button),
// PANIC/INT_WDT/TASK_WDT = firmware crash or hang.
const char *resetReasonName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    default:                return "UNKNOWN";
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

#if USE_ESTOP_BUTTON
  pinMode(ESTOP_PIN, ESTOP_PRESSED_LEVEL == LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
#endif
  usInit();
#if USE_BUMPER
  pinMode(BUMPER_PIN, INPUT);      // input-only pin: needs the external 10k pull-up
#endif
  delay(300);
  imuInit();                       // calibrates the gyro: keep the robot still at power-up
#if USE_IMU
  for (int i = 0; i < 2 && !imu_ok; i++) { delay(200); imuInit(); }   // a slow-starting GY-521: try again
#endif
  batInit();
  loadTune();
  last_control_ms = millis();
  LINK.printf("INFO,READY,robot_esp32 v%s %s reset=%s\n", FW_VERSION, BOARD_NAME, resetReasonName());
  i2cScan();                       // which I2C chips answer (IMU 0x68, INA226 0x40): see docs/troubleshooting.md
  printTune();
}

// ============================ LOOP ============================================
void loop() {
  pollSerial();
  imuUpdate();
  usUpdate(millis());

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

  estopButtonUpdate();
  bumperUpdate(now);
  batUpdate(now);

  // ---- arbitration: RC > Pi > idle ----
  bool rc_ok = rcValid(steering_last, steering_pw, STEERING_MIN, STEERING_MAX) &&
               rcValid(throttle_last, throttle_pw, THROTTLE_MIN, THROTTLE_MAX);
  float thr = rc_ok ? normalizeRC(throttle_pw, THROTTLE_MIN, THROTTLE_CENTER, THROTTLE_MAX) : 0.0f;
  float str = rc_ok ? normalizeRC(steering_pw, STEERING_MIN, STEERING_CENTER, STEERING_MAX) : 0.0f;
  if (STEERING_REVERSE) str = -str;   // str > 0 must mean turn right
  bool rc_active = (thr != 0.0f || str != 0.0f);
  if (rc_active) last_rc_active_ms = now;

  bool pi_fresh = have_cmd && (now - last_cmd_ms) < CMD_TIMEOUT_MS &&
                  !(bump_ms && now - bump_ms < BUMPER_HOLD_MS);
  bool rc_holdoff = last_rc_active_ms != 0 && (now - last_rc_active_ms) < RC_HOLDOFF_MS;

  // anything other than closed-loop Pi driving: next Pi command ramps up from standstill
  bool closed_loop = !estop_latched && !rc_active && pi_fresh && !rc_holdoff && !raw_pwm_mode;
  if (!closed_loop) {
    loopL.target_mms = loopR.target_mms = 0;
    if (!pi_fresh) loopL.cmd_mms = loopR.cmd_mms = 0;
  }

  if (estop_latched) {
    mode = MODE_ESTOP;
    stopMotors();
    last_pwm_l = last_pwm_r = 0;
    loopL.integ = loopR.integ = 0;
    have_cmd = false;
  } else if (rc_active) {
    mode = MODE_RC;
    float ls = constrain(thr + str, -1.0f, 1.0f);
    float rs = constrain(thr - str, -1.0f, 1.0f);
    last_pwm_l = (int)(ls * MAX_PWM);
    last_pwm_r = (int)(rs * MAX_PWM);
    setMotor(motorL, last_pwm_l);
    setMotor(motorR, last_pwm_r);
    loopL.integ = loopR.integ = 0;
  } else if (pi_fresh && !rc_holdoff) {
    mode = MODE_PI;
    if (raw_pwm_mode) {
      last_pwm_l = raw_pwm_l;
      last_pwm_r = raw_pwm_r;
      setMotor(motorL, raw_pwm_l);
      setMotor(motorR, raw_pwm_r);
    } else {
      float avg = 0.5f * (loopL.cmd_mms + loopR.cmd_mms);
      if (frontBlocked(avg)) {
        // low-level safety net under the Pi: remove the forward part, keep turning in place
        if (avg > 0) { loopL.cmd_mms -= avg; loopR.cmd_mms -= avg; }
      }
      rampTargets(dt_s);
      last_pwm_l = speedLoop(loopL, dt_s);
      last_pwm_r = speedLoop(loopR, dt_s);
      setMotor(motorL, last_pwm_l);
      setMotor(motorR, last_pwm_r);
    }
  } else {
    mode = MODE_IDLE;
    stopMotors();
    last_pwm_l = last_pwm_r = 0;
    loopL.integ = loopR.integ = 0;
    if (!pi_fresh) have_cmd = false;
  }

  // ---- odometry to Pi (never blocks the control loop) ----
  report_dl += dl;
  report_dr += dr;
  report_dt += dt_ms;
  if (LINK.availableForWrite() >= 80) {
    if (imu_ok) {
      LINK.printf("ODM,%ld,%ld,%lu,%u,%ld\n", report_dl, report_dr, report_dt, (unsigned)mode,
                  (long)(yaw_accum * 1e6));
      LINK.printf("IMU,%d,%d,%d,%d\n", (int)(gz_rad_s * 1000), (int)(ax_ms2 * 1000),
                  (int)(ay_ms2 * 1000), (int)(az_ms2 * 1000));
      yaw_accum = 0;
    } else {
      LINK.printf("ODM,%ld,%ld,%lu,%u\n", report_dl, report_dr, report_dt, (unsigned)mode);
    }
    report_dl = report_dr = 0;
    report_dt = 0;
    if (telemetry && LINK.availableForWrite() >= 48) {
      // per-wheel: target (ramped) mm/s, measured mm/s, PWM actually applied
      LINK.printf("SPD,%d,%d,%d,%d,%d,%d\n",
                  (int)loopL.target_mms, (int)loopL.speed_mms, last_pwm_l,
                  (int)loopR.target_mms, (int)loopR.speed_mms, last_pwm_r);
    }
  } else if (report_dt > 1000) {
    // nobody has been reading for >1 s (Pi rebooting/bridge restarting): drop the backlog
    // instead of sending one huge stale packet later
    report_dl = report_dr = 0;
    report_dt = 0;
  }
}
