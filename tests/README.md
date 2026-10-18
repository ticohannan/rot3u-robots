# Tests

```
python tests/run_tests.py            # everything
python tests/run_tests.py ik_demo    # one file
pytest tests                         # also works, if pytest is installed
```

| File | Checks |
| --- | --- |
| `test_sync.py` | The shared headers in each sketch folder match `firmware/common/` |
| `test_compile.py` | Both sketches, with both drivers, build for the Mega 2560 with no warnings (needs avr-gcc) |
| `test_driver.py` | The servo driver interface (DD-DRV) |
| `test_servo_tester.py` | `servo_tester` (DD-ST): safety, fits, backlash, export, saved sessions |
| `test_ik_demo.py` | `ik_demo` (DD-IK): the Demo 3 regression, safety, frames, branches, grip depth, flips, goals, programs, backlash |
| `test_roundtrip.py` | An `EXPORT` block compiles into `ik_demo`; every config in `configs/` builds |
| `test_tools.py` | `tools/touch_refit.py` and `tools/rot3u_kin.py` |

## How it works

The sketches are compiled for the PC (`host_build.py`) against small stand-ins for the Arduino core in
`host/shim/`: `Wire` emulates a PCA9685's registers, `EEPROM` is a 4 KB array, and time advances
only when the sketch calls `delay()` (1 ms per `loop()` pass otherwise). `host/host_main.cpp` feeds
commands from the test, logs every servo write, and can save the PCA9685 and EEPROM state between two
runs to imitate a reset with the servo board still powered.

`analyze.py` turns the logged pulses back into joint angles with the robot's calibration and checks the
gripper's real path with an independent Python model (`tools/rot3u_kin.py`), never with the
firmware's own kinematics.

## What you need

- Python 3.8+ with numpy.
- A C++ compiler called `g++` (Linux and macOS have one; on Windows, install MSYS2 or use WSL, or just
  let GitHub run the tests: every push runs them, see the Actions tab).
- For `test_compile.py` only: `avr-gcc`, plus checkouts of ArduinoCore-avr and the Servo library next to
  the repository (`../ext/ArduinoCore-avr`, `../ext/Servo`), or set `ARDUINO_CORE` and `SERVO_LIB`.
  Without them those tests are skipped.
