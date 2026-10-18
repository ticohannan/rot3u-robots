"""Compile a sketch folder for the Arduino Mega 2560 with avr-gcc, the way the Arduino IDE does
(minus automatic prototype generation, so the sketches declare functions before use).

    python tests/avr_build.py firmware/servo_tester [-DSERVO_DRIVER_PINS ...]

Needs avr-gcc and a checkout of ArduinoCore-avr (env ARDUINO_CORE, default ../ext/ArduinoCore-avr)
and, for the pin driver, the Servo library (env SERVO_LIB, default ../ext/Servo/src).
Prints flash and RAM use. Exit code 0 = built without warnings in the sketch.
"""
import os, subprocess, sys, shutil, tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CORE = Path(os.environ.get("ARDUINO_CORE", ROOT.parent / "ext" / "ArduinoCore-avr"))
SERVO = Path(os.environ.get("SERVO_LIB", ROOT.parent / "ext" / "Servo" / "src"))
CACHE = ROOT / "tests" / "build" / "core"
MCU = ["-mmcu=atmega2560", "-DF_CPU=16000000L", "-DARDUINO=10819", "-DARDUINO_AVR_MEGA2560",
       "-DARDUINO_ARCH_AVR", "-Os", "-g", "-ffunction-sections", "-fdata-sections", "-flto", "-fno-fat-lto-objects"]
CXX = ["-std=gnu++11", "-fpermissive", "-fno-exceptions", "-fno-threadsafe-statics", "-Wno-error=narrowing"]


def available() -> bool:
    return shutil.which("avr-g++") is not None and (CORE / "cores" / "arduino").is_dir()


def inc():
    i = [CORE / "cores/arduino", CORE / "variants/mega", CORE / "libraries/Wire/src",
         CORE / "libraries/Wire/src/utility", CORE / "libraries/EEPROM/src", SERVO]
    return [f"-I{p}" for p in i]


def run(cmd):
    r = subprocess.run([str(c) for c in cmd], capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError(" ".join(map(str, cmd))[:300] + "\n" + r.stderr)
    return r


def core_objects():
    CACHE.mkdir(parents=True, exist_ok=True)
    srcs = list((CORE / "cores/arduino").glob("*.c")) + list((CORE / "cores/arduino").glob("*.cpp")) + \
        list((CORE / "cores/arduino").glob("*.S")) + list((CORE / "libraries/Wire/src").glob("*.cpp")) + \
        list((CORE / "libraries/Wire/src/utility").glob("*.c")) + list((SERVO / "avr").glob("*.cpp"))
    objs = []
    for s in srcs:
        o = CACHE / (s.parent.name + "_" + s.name + ".o")
        if not o.exists():
            if s.suffix == ".c":
                run(["avr-gcc", "-c", *MCU, "-w", "-std=gnu11", *inc(), s, "-o", o])
            elif s.suffix == ".S":
                run(["avr-gcc", "-c", "-x", "assembler-with-cpp", *MCU, *inc(), s, "-o", o])
            else:
                run(["avr-g++", "-c", *MCU, "-w", *CXX, *inc(), s, "-o", o])
        objs.append(o)
    return objs


def build(sketch_dir: Path, defines=()):
    sketch_dir = Path(sketch_dir)
    ino = sketch_dir / (sketch_dir.name + ".ino")
    out = Path(tempfile.mkdtemp(prefix="avr_"))
    cpp = out / "sketch.cpp"
    cpp.write_text('#include <Arduino.h>\n#line 1 "' + ino.name + '"\n' + ino.read_text())
    r = subprocess.run(["avr-g++", "-c", *MCU, *CXX, "-Wall", "-Wextra", *defines, f"-I{sketch_dir}", *inc(),
                        str(cpp), "-o", str(out / "sketch.o")], capture_output=True, text=True)
    warnings = [l for l in r.stderr.splitlines() if ino.name in l and ("warning" in l or "error" in l)]
    if r.returncode:
        return False, r.stderr, None
    run(["avr-gcc", *MCU, "-Wl,--gc-sections", "-fuse-linker-plugin", "-o", out / "sketch.elf",
         out / "sketch.o", *core_objects(), "-lm"])
    size = run(["avr-size", "-C", "--mcu=atmega2560", out / "sketch.elf"]).stdout
    flash = [l for l in size.splitlines() if l.startswith("Program")][0]
    ram = [l for l in size.splitlines() if l.startswith("Data")][0]
    return not warnings, "\n".join(warnings), (flash.split()[1], ram.split()[1])


if __name__ == "__main__":
    ok, msg, sz = build(Path(sys.argv[1]), sys.argv[2:])
    print(msg)
    if sz:
        print(f"flash {sz[0]} bytes, RAM {sz[1]} bytes")
    sys.exit(0 if ok else 1)
