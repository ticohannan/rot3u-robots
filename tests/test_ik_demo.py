"""ik_demo v1.0 against DD-IK (docs/design/ik_demo.md). Every motion is checked with the independent
Python model (tools/rot3u_kin.py) on the pulses the sketch actually wrote."""
import math
import re
import tempfile
from pathlib import Path

import numpy as np

import analyze as A
import configs as C
import host_build as hb
import rot3u_kin as K

ROOT = Path(__file__).resolve().parent.parent
_cache = {}


def build(name, robot=None, task=None):
    key = (name, robot, task)
    if key not in _cache:
        extra = {}
        if robot:
            extra["robot_config.h"] = robot
        if task:
            extra["task_config.h"] = task
        _cache[key] = hb.build("firmware/ik_demo", name=name, extra_files=extra)
    return _cache[key]


def models(robot=None, task=None):
    d = Path(tempfile.mkdtemp())
    (d / "r.h").write_text(robot or C.ROBOT)
    (d / "t.h").write_text(task or C.TASK)
    return K.read_robot(d / "r.h"), K.read_task(d / "t.h")


DEMO3 = ["HOME", "P 10 3 0", "T 1", "T 2", "S 2", "GO", "P 12 -8 0", "GO", "P 14 6 0", "GO", "STACK 0",
         "P -2 12 0", "GO", "P -2 -13 0", "GO"]
DEMO3_PICKS = [(10, 3), (12, -8), (14, 6), (-2, 12), (-2, -13)]
DEMO3_LAYERS = [0, 1, 2, 0, 1]


def check_transfers(trs, picks, layers, rob, task, gu=12.5, dest=(5, 5), tol=2.0, tilt=1.5):
    assert len(trs) == len(picks), f"{len(trs)} transfers"
    for t, (px, py), n in zip(trs, picks, layers):
        assert t.get("ok"), t["plan"]
        e_pick = math.hypot((t["pick"][0] - px) * 10, (t["pick"][1] - py) * 10)
        e_place = math.hypot((t["place"][0] - dest[0]) * 10, (t["place"][1] - dest[1]) * 10)
        assert e_pick < tol and abs(t["pick"][2] - gu) < tol, (px, py, t["pick"])
        assert e_place < tol and abs(t["place"][2] - (n * 25 + gu)) < tol, (n, t["place"])
        assert t["tilt"] < tilt, t["tilt"]
        assert t["min_tip"] > 2.5 and t["min_elbow"] > 19.5, (t["min_tip"], t["min_elbow"])
        assert t["maxstep"] <= 16, t["maxstep"]


# ------------------------------------------------------------------------- regression
def test_demo3_regression():
    out, log = hb.run(build("sir"), DEMO3)
    assert out.count(">>> 2 solutions detected") == 5
    rob, task = models()
    check_transfers(A.transfers(log, rob, task), DEMO3_PICKS, DEMO3_LAYERS, rob, task)
    assert "# done - tower count is now 3" in out and out.strip().endswith("tower count is now 2")


def test_program_run():                                  # IK-G6: the start program is the Demo 3 script
    out, log = hb.run(build("sir"), ["LIST", "@idle", "RUN"])
    assert "# RUN finished" in out, out[-2000:]
    assert "# WAIT: lift the tower off" in out
    rob, task = models()
    check_transfers(A.transfers(log, rob, task), DEMO3_PICKS, DEMO3_LAYERS, rob, task)


def test_program_record_and_stop():
    out, log = hb.run(build("sir"), ["PROG", "ECHO hello", "HOME", "WAIT stop here", "P 10 3 0", "END", "LIST",
                                     "@idle x", "RUN"])
    assert "# hello" in out and "# RUN stopped at line 3" in out
    assert "target: top block" not in out.split("RUN stopped")[0].split(">> RUN")[-1]


# ---------------------------------------------------------------------------- safety
def test_cal_invalid_never_moves():                      # IK-S1
    robot = C.ROBOT.replace("#define CAL_VALID  true", "#define CAL_VALID  false")
    out, log = hb.run(build("calinv", robot=robot), ["DRY 0", "HOME", "P 10 3 0", "GO", "J 0 90 -90 -45 0", "G O"])
    assert "DRY stays on" in out and "the arm will not move" in out
    assert all(off == 4096 for _, off in log.writes()), "a pulse was written with CAL_VALID false"


def test_keypress_stops():                               # IK-S6
    out, log = hb.run(build("sir"), ["HOME", "P 10 3 0", "@keyat 2500", "GO"])
    assert "STOPPED by keypress" in out and "sequence stopped" in out
    stop_t = [i for i, (t, r) in enumerate(log.events) if t == "O" and "STOPPED" in r][0]
    assert not [e for e in log.events[stop_t:] if e[0] == "W"], "writes after the stop"


def test_path_check_refuses_stack_collision():           # IK-S3
    out, log = hb.run(build("sir"), ["HOME", "STK 12 -8 1", "STK 9 -1 9", "MV 12 -8 5 5"])
    assert "transfer refused before moving" in out or "no complete transfer" in out, out[-1500:]
    n_home = len([1 for t, r in log.events if t == "W"])
    out2, log2 = hb.run(build("sir"), ["HOME"])
    assert n_home == len([1 for t, r in log2.events if t == "W"]), "the refused transfer moved the arm"


def test_carry_raises_over_stack():                      # IK-S3, DD-IK section 4 step 5
    base_out, _ = hb.run(build("sir"), ["HOME", "STK 12 -8 1", "MV 12 -8 5 5"])
    out, log = hb.run(build("sir"), ["HOME", "STK 12 -8 1", "STK 8 -2 2", "MV 12 -8 5 5"])
    rob, task = models()
    trs = A.transfers(log, rob, task, grasp_up=12.5)
    assert len(trs) == 1 and trs[0]["ok"], out[-1500:]
    near = [z for x, y, z in trs[0]["carry"] if math.hypot((x - 8) * 10, (y + 2) * 10) < 30]
    assert near and all(z > 50 + 5 for z in near), min(near) if near else None
    zp = lambda o: float(re.search(r"carry at (\d+) mm", o).group(1))
    assert zp(out) > zp(base_out), (zp(out), zp(base_out))


# ------------------------------------------------------------------------- generality
def test_frame_rotation_invariance():                    # IK-G1
    out1, log1 = hb.run(build("sir"), ["HOME", "P 10 3 0", "GO"])
    # the same physical layout described on a sheet turned 90 deg, with its origin elsewhere
    rho, bx, by = 90.0, 3.0, -4.0
    def tp(X, Y):
        x, y = (X + 5.2) * 10, (Y + 1.1) * 10                      # base frame, mm (original frame rot 0)
        r = math.radians(rho)
        return bx + (math.cos(r) * x - math.sin(r) * y) / 10, by + (math.sin(r) * x + math.cos(r) * y) / 10
    (px, py), (dx, dy) = tp(10, 3), tp(5, 5)
    out2, log2 = hb.run(build("sir"), [f"FRAME {bx} {by} {rho} 0", f"DEST {dx:.3f} {dy:.3f}", "HOME",
                                       f"P {px:.3f} {py:.3f} 0", "GO"])
    w1 = [w for w in log1.writes() if w[1] < 4096]
    w2 = [w for w in log2.writes() if w[1] < 4096]
    assert len(w1) == len(w2)
    assert max(abs(a[1] - b[1]) for a, b in zip(w1, w2)) <= 1, "same physical moves expected"


def test_back_branch():                                  # IK-G2
    robot = C.robot_wide()
    exe = build("wide", robot=robot)
    out, _ = hb.run(exe, ["P 10 3 0"])
    n_any = int(re.search(r">>> (\d+) solutions? detected", out).group(1))
    out, log = hb.run(exe, ["BRANCH B", "HOME", "P 10 3 0", "GO"])
    n_back = int(re.search(r">>> (\d+) solutions? detected", out).group(1))
    oks = re.findall(r"SOL (\d)  (\S+)\s.*OK$", out, re.M)
    assert n_any > n_back >= 1 and all(b.startswith("back") for _, b in oks), (n_any, oks)
    assert "with back/" in out
    rob, task = models(robot=robot)
    # the invented wide-range servos move 0.9 deg per PCA9685 count, so a few mm of quantisation is expected
    check_transfers(A.transfers(log, rob, task), [(10, 3)], [0], rob, task, tol=5.0, tilt=3.0)
    out, _ = hb.run(exe, ["BRANCH F", "P 10 3 0"])
    assert all(b.startswith("front") for _, b in re.findall(r"SOL (\d)  (\S+)\s.*OK$", out, re.M))


def test_grip_depth():                                   # IK-G3
    out, log = hb.run(build("sir"), ["GRIPAT 10", "HOME", "P 10 3 0", "GO", "GRIPAT 20", "P 10 3 0"])
    rob, task = models()
    gu = max(12.5, 10 + 3.0)
    t = A.transfers(log, rob, task, grip_depth=10)
    check_transfers(t, [(10, 3)], [0], rob, task, gu=gu)
    assert "grasp at the very top of the block" in out


def test_flip_policy():                                  # IK-G4
    robot = C.robot_wide()
    task = C.task_with(APPROACH_LIST="0, -90")          # a side grasp (0) is tried first
    exe = build("flip", robot=robot, task=task)
    rob, tk = models(robot=robot, task=task)
    res = {}
    for f in (0, 1, 2):
        out, log = hb.run(exe, [f"FLIP {f}", "DEST 12 14", "HOME", "P 22 0 0", "GO"])
        trs = A.transfers(log, rob, tk)
        assert len(trs) == 1 and trs[0].get("ok"), out[-1500:]
        res[f] = (re.search(r"block arrives ([a-z ]+),", out).group(1), trs[0])
    assert res[0][0] == "upright" and res[0][1]["up_z"] > 0.99, res[0]
    assert res[1][0] == "upright" and res[1][1]["up_z"] > 0.99, res[1]
    assert res[2][0] == "upside down" and res[2][1]["up_z"] < -0.99, res[2]
    for _, t in res.values():
        assert math.hypot((t["place"][0] - 12) * 10, (t["place"][1] - 14) * 10) < 5 and abs(t["place"][2] - 12.5) < 5


def test_world_mv_tower():                               # IK-G5, IK-G6
    out, log = hb.run(build("sir"), ["HOME", "STK 10 3 2", "STK 12 -8 1", "STK -2 12 1", "TOWER 5 5 3",
                                     "MV 5 5 -2 12", "WORLD"])
    rob, task = models()
    trs = A.transfers(log, rob, task)
    assert len(trs) == 4 and all(t.get("ok") for t in trs), out[-2500:]
    picks = [(10, 3, 37.5), (10, 3, 12.5), (12, -8, 12.5), (5, 5, 62.5)]
    places = [(5, 5, 12.5), (5, 5, 37.5), (5, 5, 62.5), (-2, 12, 37.5)]
    for t, p, q in zip(trs, picks, places):
        assert np.linalg.norm((t["pick"] - p) * [10, 10, 1]) < 2.0, (t["pick"], p)
        assert np.linalg.norm((t["place"] - q) * [10, 10, 1]) < 2.0, (t["place"], q)
    world = out.split(">> WORLD")[1]
    assert "[5.0, 5.0]: 2 block(s)" in world and "[-2.0, 12.0]: 2 block(s)" in world and "[10.0, 3.0]: 0" in world


# ---------------------------------------------------------------------------- backlash
LASH = (2.0, 3.0, 3.0, 3.0, 0.0)


def test_zero_lash_no_change():                          # IK-L4
    _, l1 = hb.run(build("sir"), ["HOME", "P 10 3 0", "GO"])
    _, l2 = hb.run(build("sir"), ["LASH ON", "HOME", "P 10 3 0", "GO"])
    assert l1.writes() == l2.writes()


def test_lash_off_is_centre_line():                      # IK-L1
    robot = C.robot_with_lash(LASH)
    _, l1 = hb.run(build("sir"), ["HOME", "P 10 3 0", "GO"])
    _, l2 = hb.run(build("lash", robot=robot), ["HOME", "P 10 3 0", "GO"])
    assert l1.writes() == l2.writes()


def test_lash_sides():                                   # IK-L2: compensation cancels the play of a true arm
    robot = C.robot_with_lash(LASH)
    exe = build("lash", robot=robot)
    rob, task = models(robot=robot)
    m = A.masses(robot, C.TASK)
    errs = {}
    for mode in ("OFF", "ON"):
        out, log = hb.run(exe, [f"LASH {mode}", "HOME", "P 10 3 0", "GO", "P -2 12 0", "GO"])
        res = A.true_positions(log, rob, task, m)
        e = []
        for r, pick, n in zip(res, [(10, 3), (-2, 12)], [0, 1]):
            e.append(np.linalg.norm((r[3] - (pick[0], pick[1], 12.5)) * [10, 10, 1]))
            e.append(np.linalg.norm((r[7] - (5, 5, 12.5 + 25 * n)) * [10, 10, 1]))
        errs[mode] = max(e)
    assert errs["ON"] < 1.5 and errs["OFF"] > 3.0, errs


def test_lash_suggestion():                              # IK-L3
    out, _ = hb.run(build("lash", robot=C.robot_with_lash(LASH)), ["P 10 3 0"])
    m = re.search(r"change the grasp pulses by ((?:-?\d+ ?){5}) us; without it the grasp point lands about ([\d.]+) mm off", out)
    assert m, out[-800:]
    d = [int(v) for v in m.group(1).split()]
    assert any(abs(v) >= 10 for v in d[:4]) and float(m.group(2)) > 3.0, m.groups()
    out, _ = hb.run(build("sir"), ["P 10 3 0"])
    assert "not measured for this robot" in out
