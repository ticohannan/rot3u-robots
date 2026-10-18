"""DD-CFG: a servo_tester EXPORT block pasted into robot_config.h compiles and drives ik_demo unchanged."""
import re

import configs as C
import host_build as hb
import test_servo_tester as ST

# Sir Pinch-a-lot's calibration, re-measured: points generated from its lines, read to 0.1 deg
TRUE = [(0, "base", 1475.6, 10.901, 0), (1, "shoulder", 408.8, 10.625, 1), (2, "elbow", 332.1, -11.587, 1),
        (3, "wrist", 519.5, -11.111, 1), (4, "roll", 500.0, 11.111, 0)]
ENDS = {0: (500, 2500), 1: (500, 2500), 2: (500, 2050), 3: (500, 1500), 4: (500, 2500)}


def session():
    cmds = ["ARM A"]
    for ch, name, u0, k, load in TRUE:
        lo, hi = ENDS[ch]
        cmds += [f"LIM {ch} {max(lo, 500)} {min(hi, 2500)}", f"SEL {ch}", f"JT ROT {name}", f"LOAD {load}", "START 1500"]
        for us in (lo + 150, 1500, hi - 150):
            cmds += [f"GO {us}", f"PT {round((us - u0) / k, 1)}"]
        # the roll's ends are the tester's bounds, not contacts: no margin (Sir Pinch-a-lot's real roll limits)
        cmds += ["MARGIN 0" if name == "roll" else "MARGIN 30", f"GO {lo}", "LO", f"GO {hi}", "HI", "MARGIN 30"]
    cmds += ["LIM 5 1124 1950", "SEL 5", "JT GRIP gripper", "START 1150", "SETP OPEN", "GO 1580", "SETP HOLD",
             "GO 1124", "LO", "GO 1950", "HI", "EXPORT"]
    return cmds


def test_export_compiles():
    out, _ = ST.run(session())
    block = out[out.index("// ---- BEGIN SERVO_TESTER EXPORT"):out.index("// ---- END SERVO_TESTER EXPORT ----") + 40]
    robot = re.sub(r"const JointCal JOINTS\[\] = \{.*?\n\};\nconst GripCal GRIP = \{[^}]*\};[^\n]*", lambda m: block,
                   C.ROBOT, flags=re.S)
    assert robot != C.ROBOT
    exe = hb.build("firmware/ik_demo", name="roundtrip", extra_files={"robot_config.h": robot})
    cal, _ = hb.run(exe, ["CAL", "P 10 3 0"])
    for ch, name, u0, k, load in TRUE:
        m = re.search(r"# joint \d " + name + r" \(ch " + str(ch) + r"\): us = ([\d.]+) \+ (-?[\d.]+)", cal)
        assert m and abs(float(m.group(1)) - u0) < 1.5 and abs(float(m.group(2)) - k) < 0.02, (name, m and m.groups())
    assert "gripper ch 5 open 1150, hold 1580" in cal
    assert ">>> 2 solutions detected" in cal


def test_saved_configs_compile():
    """Every robot config in configs/ builds with the Demo 3 task, and the template refuses to move."""
    root = C.ROOT / "configs"
    for i, rc in enumerate(sorted(root.rglob("robot_config*.h"))):
        exe = hb.build("firmware/ik_demo", name=f"cfg{i}", extra_files={"robot_config.h": rc.read_text()})
        out, log = hb.run(exe, ["HOME", "CAL"])
        if "template" in rc.name:
            assert "CAL_VALID is false" in out and all(off == 4096 for _, off in log.writes()), rc
        else:
            assert "CAL_VALID = true" in out, rc
    for i, tc in enumerate(sorted(root.rglob("task_config*.h"))):
        exe = hb.build("firmware/ik_demo", name=f"task{i}", extra_files={"task_config.h": tc.read_text()})
        out, _ = hb.run(exe, ["CAL"])
        assert "ready pose th*" in out, tc
