# Design: Servo Driver Interface (DD-DRV)

**Version:** 1.0
**Status:** ACTIVE
**Updated:** 2026-10-17
**Applies to:** `firmware/common/RobotServoDriver.h`, `PCA9685Driver.h`, `PinServoDriver.h`

Both sketches drive servos through one small interface, so a different servo board can be supported
by adding one driver class, without touching the sketches.

## 1. Requirements

| ID | Requirement |
| --- | --- |
| DRV-1 | No output produces a pulse until the sketch writes that channel. `begin()` on a cold board leaves every output off |
| DRV-2 | A driver that can read its outputs back reports the pulse each live channel is already producing, so a sketch can **adopt** a holding arm after a reset instead of letting it drop |
| DRV-3 | `write(ch, us)` sets the pulse width in µs, clamped to the absolute range (500–2500 µs); it returns false if the hardware did not accept it. The sketch's soft limits sit above this |
| DRV-4 | `release(ch)` stops the pulse completely (the servo goes limp) |
| DRV-5 | Errors are returned, not printed. The sketch decides how to report them |
| DRV-6 | The interface is hardware-neutral: no I2C, pin or register details leak into the sketches, except through the driver's own `diag()` |

## 2. Interface

```cpp
class RobotServoDriver {
public:
  virtual bool        begin(bool adopt) = 0;     // adopt = keep live outputs if the hardware allows
  virtual bool        present() = 0;             // hardware answers
  virtual uint8_t     channels() const = 0;
  virtual bool        write(uint8_t ch, uint16_t us) = 0;
  virtual bool        release(uint8_t ch) = 0;
  virtual bool        readBack(uint8_t ch, uint16_t &us) = 0;  // false = off, unknown, or unsupported
  virtual bool        canReadBack() const = 0;
  virtual uint16_t    resolutionUs() const = 0;  // smallest pulse step the hardware makes
  virtual void        diag(Print &out) = 0;      // one-line hardware status for humans
  virtual const char *name() const = 0;
  bool                adopted = false;           // begin() found a running board and kept it
};
```

`begin(adopt)` behaviour:

| Board state at boot | `adopt = true` | `adopt = false` |
| --- | --- | --- |
| Not running (cold) | Configure, all outputs off | Configure, all outputs off |
| Already running at servo rate | Keep running; `adopted = true`; `readBack` reports live channels | Reconfigure, all outputs off |

## 3. PCA9685 driver (`PCA9685Driver`)

Register-level, no library (the Adafruit library resets the chip in `begin()`, which drops a holding
arm). Behaviour carried over unchanged from `servo_tester` v2.3 and the PCA9685 register driver
rev 1.0:

- 50 Hz: prescale 131 with the nominal 27 MHz oscillator (clones vary; calibration is by measurement,
  so only consistency matters). Resolution 4.88 µs per count.
- Running = awake (MODE1 SLEEP clear) and prescale 121–144.
- A channel is live when it is not full-on, not full-off, ON count is 0, OFF count is not 0, and the
  pulse is 500–2500 µs, with one count (4.9 µs) of tolerance: a channel written at exactly 500 µs reads
  back as 499 µs and must still be adopted. (v2.3 and ik_demo v0.4 rejected it, so a wrist roll holding
  at 500 µs was not adopted after a reset.)
- Configure: SLEEP → write prescale → ALL_LED_OFF_H (0xFD) = full-off, so no old pulse resumes →
  wake → 5 ms → RESTART | AI → release each channel.
- Release = full-off bit (OFF_H bit 4).
- Constructor arguments: I2C address (default 0x40), number of channels used (≤ 16), oscillator Hz.
- `diag()` prints MODE1 and PRESCALE.
- `scanBus(Print&)` (extra, not in the interface) lists I2C addresses that answer.

## 4. Pin driver (`PinServoDriver`)

For servos wired straight to Arduino pins, using the Arduino `Servo` library.

- Cannot read back (`canReadBack()` = false), so it can never adopt: after a reset the servos stay
  limp until the sketch writes them.
- `write()` on a released channel sets the pulse **before** `attach()`, so the first pulse is the
  requested one, not the library's 1500 µs default (DRV-1). Before `attach()` the library clamps to its
  default 544–2400 µs, so `write()` sets the pulse again right after attaching.
- `release()` = `detach()`.
- Constructor argument: a pin list.
- Status: compiled; **not yet run on hardware**.

## 5. Choosing a driver in a sketch

```cpp
#if defined(SERVO_DRIVER_PINS)
  #include "PinServoDriver.h"
  const uint8_t PINS[] = {2, 3, 4, 5, 6, 7};
  PinServoDriver drv(PINS, 6);
#else
  #include "PCA9685Driver.h"
  PCA9685Driver drv(0x40, 16);
#endif
RobotServoDriver &servo = drv;
```

`servo_tester` defaults to the PCA9685 with 16 channels; `ik_demo` takes the choice from
`robot_config.h`.

## 6. Adding a driver

1. Write the design change here first: a section like §3 or §4 with the board's adoption behaviour
   and resolution.
2. Add `firmware/common/<Name>Driver.h` implementing every method.
3. Run `python tools/sync_common.py` to copy it into the sketch folders.
4. Add a host shim for the hardware and a test in `tests/test_driver.py`.

## 7. Files and copies

The canonical files live in `firmware/common/`. The Arduino IDE only compiles files inside the
sketch folder, so each sketch folder holds a copy. `tools/sync_common.py` writes the copies and
`tools/sync_common.py --check` (run by the tests) fails if any copy differs. Edit only
`firmware/common/`.

## 8. Tests

| Requirement | Test |
| --- | --- |
| DRV-1 cold boot leaves outputs off | `tests/test_driver.py::test_cold_boot_outputs_off` |
| DRV-2 adopt keeps live channels | `tests/test_driver.py::test_adopt_live_channels` |
| DRV-3 clamp to 500–2500 | `tests/test_driver.py::test_write_clamps` |
| DRV-4 release sets full-off | `tests/test_driver.py::test_release_full_off` |
| Reconfigure never resumes old pulses | `tests/test_driver.py::test_reconfigure_all_off_first` |
| Copies identical | `tests/test_sync.py::test_common_copies_identical` |
| Both drivers compile for the Mega | `tests/test_compile.py` |
