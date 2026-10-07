/*
 * rc_reader — print FlySky RC pulse widths (classic ESP32)
 * Serial Monitor 115200. CH1 (steering) = GPIO 34, CH2 (throttle) = GPIO 35.
 * Prints every 200 ms:  CH1 now / min / max   |   CH2 now / min / max
 * Type  r  + Enter in the Serial Monitor to reset min/max.
 * Motors are held OFF the whole time.
 */
#define CH1_PIN 34   // steering (stick left/right)
#define CH2_PIN 35   // throttle (stick forward/back)

const int motorPins[] = {25, 26, 32, 33};
int min1 = 9999, max1 = 0, min2 = 9999, max2 = 0;

void setup() {
  for (int p : motorPins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  Serial.begin(115200);
  delay(300);
  Serial.println("rc_reader ready. 0 = no signal. Type r to reset min/max.");
}

void loop() {
  int ch1 = pulseIn(CH1_PIN, HIGH, 30000);   // microseconds, 0 if no pulse
  int ch2 = pulseIn(CH2_PIN, HIGH, 30000);

  if (ch1 > 800 && ch1 < 2200) { min1 = min(min1, ch1); max1 = max(max1, ch1); }
  if (ch2 > 800 && ch2 < 2200) { min2 = min(min2, ch2); max2 = max(max2, ch2); }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') { min1 = min2 = 9999; max1 = max2 = 0; Serial.println("-- min/max reset --"); }
  }

  Serial.printf("CH1 steer: %4d  (min %4d max %4d)   |   CH2 throttle: %4d  (min %4d max %4d)\n",
                ch1, min1 == 9999 ? 0 : min1, max1, ch2, min2 == 9999 ? 0 : min2, max2);
  delay(200);
}
