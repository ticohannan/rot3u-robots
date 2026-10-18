// Test sketch for the shared driver (tests/test_driver.py). Host only.
#include <Wire.h>
#include "RobotServoDriver.h"
#include "PCA9685Driver.h"
PCA9685Driver drv(0x40, 16);
RobotServoDriver &servo = drv;
char line[40]; uint8_t n = 0;
void setup() {
  Serial.begin(115200); Wire.begin();
  bool ok = servo.begin(true);
  Serial.print(F("BEGIN ")); Serial.print(ok ? 1 : 0); Serial.print(F(" ADOPTED ")); Serial.println(servo.adopted ? 1 : 0);
  for (uint8_t c = 0; c < 16; c++) { uint16_t us; if (servo.readBack(c, us)) { Serial.print(F("LIVE ")); Serial.print(c); Serial.print(' '); Serial.println(us); } }
}
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n') { if (n < 39) line[n++] = c; continue; }
    line[n] = 0; n = 0;
    int ch = 0, us = 0;
    if (sscanf(line, "W %d %d", &ch, &us) == 2) { servo.write(ch, us); uint16_t r = 0; bool live = servo.readBack(ch, r); Serial.print(F("READ ")); Serial.print(live ? (int)r : -1); Serial.println(); }
    else if (sscanf(line, "R %d", &ch) == 1) { servo.release(ch); uint16_t r = 0; Serial.print(F("LIVE? ")); Serial.println(servo.readBack(ch, r) ? 1 : 0); }
  }
}
