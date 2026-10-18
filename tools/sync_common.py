"""Copy the shared headers from firmware/common/ into every sketch folder (DD-DRV section 7).

    python tools/sync_common.py          write the copies
    python tools/sync_common.py --check  exit 1 if any copy differs (run by the tests)

Edit only firmware/common/. The Arduino IDE compiles only files inside a sketch's own folder,
so each sketch keeps a copy.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
COMMON = ROOT / "firmware" / "common"
SKETCHES = {
    "firmware/servo_tester": ["RobotTypes.h", "RobotServoDriver.h", "PCA9685Driver.h", "PinServoDriver.h"],
    "firmware/ik_demo": ["RobotTypes.h", "RobotServoDriver.h", "PCA9685Driver.h", "PinServoDriver.h"],
    "tests/host/driver_sketch": ["RobotServoDriver.h", "PCA9685Driver.h"],
}


def main(check: bool) -> int:
    bad = 0
    for sketch, files in SKETCHES.items():
        for name in files:
            src = (COMMON / name).read_bytes()
            dst = ROOT / sketch / name
            if check:
                if not dst.exists() or dst.read_bytes() != src:
                    print(f"OUT OF SYNC: {sketch}/{name}")
                    bad += 1
            elif not dst.exists() or dst.read_bytes() != src:
                dst.write_bytes(src)
                print(f"copied {name} -> {sketch}/")
    if check and not bad:
        print("all shared headers in sync")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main("--check" in sys.argv))
