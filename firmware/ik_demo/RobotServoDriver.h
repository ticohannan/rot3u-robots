// RobotServoDriver.h - hardware-neutral servo output interface (DD-DRV, docs/design/servo-driver.md)
// Canonical copy: firmware/common/. Copyright (c) 2026 ticohannan. BSD-3-Clause, see LICENSE.
#ifndef ROBOT_SERVO_DRIVER_H
#define ROBOT_SERVO_DRIVER_H
#include <Arduino.h>

const uint16_t SERVO_ABS_MIN_US = 500;    // absolute pulse bounds; sketches add soft limits above these
const uint16_t SERVO_ABS_MAX_US = 2500;

class RobotServoDriver {
public:
  // Cold board: configure, every output off. Running board and adopt = true: keep it running,
  // set adopted = true, and let readBack() report the live channels (DRV-1, DRV-2).
  virtual bool        begin(bool adopt) = 0;
  virtual bool        present() = 0;                         // hardware answers
  virtual uint8_t     channels() const = 0;
  virtual bool        write(uint8_t ch, uint16_t us) = 0;    // clamps to 500-2500 (DRV-3)
  virtual bool        release(uint8_t ch) = 0;               // no pulse at all (DRV-4)
  virtual bool        readBack(uint8_t ch, uint16_t &us) = 0;// false = off, unknown or unsupported
  virtual bool        canReadBack() const = 0;
  virtual uint16_t    resolutionUs() const = 0;
  virtual void        diag(Print &out) = 0;                  // one line of hardware status
  virtual const char *name() const = 0;
  bool adopted = false;
  static uint16_t clampAbs(long us) {
    if (us < SERVO_ABS_MIN_US) return SERVO_ABS_MIN_US;
    if (us > SERVO_ABS_MAX_US) return SERVO_ABS_MAX_US;
    return (uint16_t)us;
  }
};

#endif
