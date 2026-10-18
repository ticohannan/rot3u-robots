// PCA9685Driver.h - register-level PCA9685 servo driver (DD-DRV section 3; PCA9685 register driver rev 1.0)
// No library: the Adafruit library resets the chip in begin(), which drops a holding arm.
// Canonical copy: firmware/common/. Copyright (c) 2026 ticohannan. BSD-3-Clause, see LICENSE.
#ifndef PCA9685_DRIVER_H
#define PCA9685_DRIVER_H
#include <Arduino.h>
#include <Wire.h>
#include "RobotServoDriver.h"

class PCA9685Driver : public RobotServoDriver {
public:
  static const uint8_t REG_MODE1 = 0x00, REG_LED0_ON_L = 0x06, REG_PRESCALE = 0xFE;
  static const uint8_t MODE1_RESTART = 0x80, MODE1_AI = 0x20, MODE1_SLEEP = 0x10;
  static const uint8_t PRESCALE_50HZ = 131;           // round(27 MHz / (4096 * 50)) - 1

  PCA9685Driver(uint8_t addr = 0x40, uint8_t nCh = 16, float oscHz = 27000000.0f)
    : addr_(addr), n_(nCh > 16 ? 16 : nCh), osc_(oscHz) {}

  bool begin(bool adopt) override {
    adopted = false;
    if (!present()) return false;
    int mode1 = readReg(REG_MODE1), pre = readReg(REG_PRESCALE);
    bool running = mode1 >= 0 && (mode1 & MODE1_SLEEP) == 0 && pre > 120 && pre < 145;
    if (running && adopt) {
      ps_ = (uint8_t)pre;
      if ((mode1 & MODE1_AI) == 0) writeReg(REG_MODE1, (uint8_t)mode1 | MODE1_AI);
      adopted = true;
      return true;
    }
    writeReg(REG_MODE1, MODE1_SLEEP);                  // sleep: prescale is writable
    writeReg(REG_PRESCALE, PRESCALE_50HZ);
    writeReg(REG_MODE1, 0x00);                         // wake
    delay(5);
    writeReg(REG_MODE1, MODE1_RESTART | MODE1_AI);
    ps_ = PRESCALE_50HZ;
    for (uint8_t c = 0; c < n_; c++) release(c);
    int after = readReg(REG_PRESCALE);
    return after > 120 && after < 145;
  }
  bool present() override {
    Wire.beginTransmission(addr_);
    return Wire.endTransmission() == 0;
  }
  uint8_t channels() const override { return n_; }
  bool write(uint8_t ch, uint16_t us) override {
    if (ch >= n_) return false;
    return setCounts(ch, 0, usToCounts(clampAbs(us)));
  }
  bool release(uint8_t ch) override {
    if (ch >= n_) return false;
    return setCounts(ch, 0, 4096);                     // OFF_H bit 4 = full off
  }
  bool readBack(uint8_t ch, uint16_t &us) override {
    if (ch >= n_) return false;
    Wire.beginTransmission(addr_);
    Wire.write(REG_LED0_ON_L + 4 * ch);
    if (Wire.endTransmission() != 0) return false;
    Wire.requestFrom((int)addr_, 4);
    if (Wire.available() < 4) return false;
    uint8_t onL = Wire.read(), onH = Wire.read(), offL = Wire.read(), offH = Wire.read();
    uint16_t on  = ((uint16_t)(onH & 0x0F) << 8) | onL;
    uint16_t off = ((uint16_t)(offH & 0x0F) << 8) | offL;
    if ((onH & 0x10) || (offH & 0x10) || on != 0 || off == 0) return false;
    uint16_t v = countsToUs(off);
    // one count (4.9 us) of tolerance: a channel set to exactly 500 us reads back as 499
    if (v + 5 < SERVO_ABS_MIN_US || v > SERVO_ABS_MAX_US + 5) return false;
    us = clampAbs(v);
    return true;
  }
  bool canReadBack() const override { return true; }
  uint16_t resolutionUs() const override { return (uint16_t)(usPerCount() + 0.5f); }
  void diag(Print &out) override {
    out.print(F("# PCA9685 @0x")); out.print(addr_, HEX);
    out.print(F(" MODE1=0x")); out.print(readReg(REG_MODE1), HEX);
    out.print(F(" PRESCALE=")); out.println(readReg(REG_PRESCALE));
  }
  const char *name() const override { return "PCA9685"; }

  // Extras, not part of the interface
  uint8_t prescale() const { return ps_; }
  uint8_t address() const { return addr_; }
  static void scanBus(Print &out) {
    for (uint8_t a = 1; a < 127; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) {
        out.print(F("SCAN,")); out.print(millis()); out.print(F(",0x"));
        if (a < 16) out.print('0');
        out.println(a, HEX);
      }
    }
  }
  int readReg(uint8_t reg) {
    Wire.beginTransmission(addr_); Wire.write(reg);
    if (Wire.endTransmission() != 0) return -1;
    Wire.requestFrom((int)addr_, 1);
    return Wire.available() ? Wire.read() : -1;
  }
  bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr_); Wire.write(reg); Wire.write(val);
    return Wire.endTransmission() == 0;
  }

private:
  uint8_t addr_, n_, ps_ = PRESCALE_50HZ;
  float   osc_;
  float usPerCount() const { return 1000000.0f * (float)((uint16_t)ps_ + 1) / osc_; }
  uint16_t usToCounts(uint16_t us) const { return (uint16_t)((float)us / usPerCount() + 0.5f); }
  uint16_t countsToUs(uint16_t n) const { return (uint16_t)((float)n * usPerCount() + 0.5f); }
  bool setCounts(uint8_t c, uint16_t on, uint16_t off) {
    Wire.beginTransmission(addr_);
    Wire.write(REG_LED0_ON_L + 4 * c);
    Wire.write(on & 0xFF); Wire.write(on >> 8);
    Wire.write(off & 0xFF); Wire.write(off >> 8);
    return Wire.endTransmission() == 0;
  }
};

#endif
