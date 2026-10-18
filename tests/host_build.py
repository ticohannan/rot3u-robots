"""Build and run a sketch on the PC with the shims in tests/host/shim (emulated PCA9685 + EEPROM).

    build(sketch_dir, name, defines, replace) -> path of the executable
    run(exe, commands, state=None) -> (stdout text, parsed log)
"""
import os, re, subprocess, tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
HOST = ROOT / "tests" / "host"
BUILD = ROOT / "tests" / "build"


def build(sketch_dir, name=None, defines=(), replace=None, extra_files=None):
    """replace: list of (regex, text) applied to the .ino before compiling.
       extra_files: {filename: text} written into the build folder ahead of the sketch folder
       (e.g. a replacement robot_config.h)."""
    sketch_dir = ROOT / sketch_dir
    name = name or sketch_dir.name
    out = BUILD / name
    out.mkdir(parents=True, exist_ok=True)
    src = (sketch_dir / (sketch_dir.name + ".ino")).read_text()
    for pat, txt in (replace or []):
        new = re.sub(pat, lambda m: txt, src, flags=re.S)
        if new == src:
            raise ValueError(f"pattern not found: {pat[:60]}")
        src = new
    (out / "sketch_host.cpp").write_text(src)
    for fn, txt in (extra_files or {}).items():
        (out / fn).write_text(txt)
    exe = out / ("host_" + name)
    cmd = ["g++", "-O1", "-std=c++17", "-Wno-narrowing", f"-I{out}", f"-I{HOST / 'shim'}", f"-I{sketch_dir}",
           *defines, "-o", str(exe), str(HOST / "host_main.cpp")]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError(r.stderr[-4000:])
    return exe


class Log:
    def __init__(self, path):
        self.events = []
        for line in Path(path).read_text().splitlines():
            if len(line) < 2:
                continue
            self.events.append((line[0], line[2:]))

    def writes(self):
        return [(int(r.split()[0]), int(r.split()[1])) for t, r in self.events if t == "W"]

    def writes_t(self):
        return [tuple(int(v) for v in r.split()) for t, r in self.events if t == "W"]

    def outputs(self):
        return [r for t, r in self.events if t == "O"]


def run(exe, commands, state=None, timeout=600):
    logf = tempfile.NamedTemporaryFile(delete=False, suffix=".log").name
    args = [str(exe), logf] + ([str(state)] if state else [])
    r = subprocess.run(args, input="\n".join(commands) + "\n", capture_output=True, text=True, timeout=timeout)
    if r.returncode:
        raise RuntimeError(f"host run failed ({r.returncode}): {r.stderr[-2000:]}")
    log = Log(logf)
    os.unlink(logf)
    return r.stdout, log
