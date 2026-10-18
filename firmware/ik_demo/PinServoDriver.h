// PinServoDriver.h - servos on Arduino pins via the Servo library (DD-DRV section 4)
// Cannot read back, so it never adopts. STATUS: compiled, NOT yet run on hardware.
// Canonical copy: firmware/common/. Copyright (c) 2026 ticohannan. BSD-3-Clause, see LICENSE.
#ifndef PIN_SERVO_DRIVER_H
#define PIN_SERVO_DRIVER_H
#include <Arduino.h>
#include <Servo.h>
#include "RobotServoDriver.h"

class PinServoDriver : public RobotServoDriver {
public:
  static const uint8_t MAX_PINS = 12;
  PinServoDriver(const uint8_t *pins, uint8_t n) : n_(n > MAX_PINS ? MAX_PINS : n) {
    for (uint8_t i = 0; i < n_; i++) pins_[i] = pins[i];
  }
  bool begin(bool) override {
    adopted = false;
    for (uint8_t i = 0; i < n_; i++) release(i);
    return true;
  }
  bool present() override { return true; }
  uint8_t channels() const override { return n_; }
  bool write(uint8_t ch, uint16_t us) override {
    if (ch >= n_) return false;
    uint16_t v = clampAbs(us);
    if (!s_[ch].attached()) {
      // Set the pulse BEFORE attach, so the first pulse is this one and not the library's 1500 us (DRV-1).
      // Before attach the library clamps to its default 544-2400 us; write again once attached.
      s_[ch].writeMicroseconds(v);
      s_[ch].attach(pins_[ch], SERVO_ABS_MIN_US, SERVO_ABS_MAX_US);
    }
    s_[ch].writeMicroseconds(v);
    return true;
  }
  bool release(uint8_t ch) override {
    if (ch >= n_) return false;
    if (s_[ch].attached()) s_[ch].detach();
    return true;
  }
  bool readBack(uint8_t, uint16_t &) override { return false; }
  bool canReadBack() const override { return false; }
  uint16_t resolutionUs() const override { return 1; }
  void diag(Print &out) override { out.print(F("# pin driver, ")); out.print(n_); out.println(F(" channels")); }
  const char *name() const override { return "pins"; }
private:
  uint8_t n_, pins_[MAX_PINS];
  Servo   s_[MAX_PINS];
};

#endif
