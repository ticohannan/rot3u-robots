"""Both sketches, with both drivers, compile for the Mega 2560 without warnings (DD-DRV section 8).
Skipped when avr-gcc / ArduinoCore-avr are not installed (CI uses arduino-cli instead)."""
from pathlib import Path
import avr_build

ROOT = Path(__file__).resolve().parent.parent
SKIP = None if avr_build.available() else "avr-gcc or ArduinoCore-avr not found"


def _build(sketch, defines=()):
    ok, msg, size = avr_build.build(ROOT / sketch, list(defines))
    assert ok, msg
    flash, ram = int(size[0]), int(size[1])
    assert flash < 200000 and ram < 6500, (flash, ram)       # Mega: 253952 flash, 8192 RAM
    return flash, ram


def test_servo_tester_pca9685():
    _build("firmware/servo_tester")


def test_servo_tester_pins():
    _build("firmware/servo_tester", ["-DSERVO_DRIVER_PINS"])


def test_ik_demo_pca9685():
    _build("firmware/ik_demo")


def test_ik_demo_pins():
    _build("firmware/ik_demo", ["-DSERVO_DRIVER_PINS"])
