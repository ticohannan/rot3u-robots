"""PC tools: touch_refit recovers injected offsets; rot3u_kin agrees with the firmware's printed FK."""
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

import host_build as hb
import rot3u_kin as K
import touch_refit as TR

ROOT = Path(__file__).resolve().parent.parent
ROBOT = ROOT / "firmware/ik_demo/robot_config.h"
TASK = ROOT / "firmware/ik_demo/task_config.h"


def synthetic_touches(true_d):
    """Run M at six points in ik_demo, read W, and compute where an arm with offsets true_d would put the tips."""
    pts = [(8, -4), (0, 12), (10, 2), (6, -10), (-2, -12), (5, 5)]
    exe = hb.build("firmware/ik_demo", name="sir")
    cmds = ["HOME"]
    for x, y in pts:
        cmds += [f"M {x} {y} 0.5", "W"]
    out, _ = hb.run(exe, cmds)
    pulses = [[int(v) for v in l.split()[2:7]] for l in out.splitlines() if l.startswith("# pulses")]
    rob, task = K.read_robot(ROBOT), K.read_task(TASK)
    touches = []
    for us in pulses:
        th = K.pulses_to_deg(us, rob) + true_d
        p = TR.tip_paper(th, rob, task, (task["FRAME_BX_CM"], task["FRAME_BY_CM"]))
        touches.append((us, p[0], p[1], p[2]))
    return rob, task, touches


def test_touch_refit_recovers_offsets():
    true_d = np.array([-2.9, 0.0, 0.0, 3.3, 0.0])
    rob, task, touches = synthetic_touches(true_d)
    d, b, res = TR.refit(rob, task, touches, joints=(0, 3), prior_deg=10)
    assert np.allclose(d[[0, 3]], true_d[[0, 3]], atol=0.15), d
    assert np.abs(res).max() < 0.5


def test_touch_refit_cli():
    rob, task, touches = synthetic_touches(np.array([-2.0, 0, 0, 0, 0]))
    f = Path(tempfile.mkdtemp()) / "t.csv"
    f.write_text("# us0..us4, X, Y, tip\n" + "\n".join(",".join(str(v) for v in [*us, x, y, z]) for us, x, y, z in touches))
    r = subprocess.run([sys.executable, str(ROOT / "tools/touch_refit.py"), str(ROBOT), str(TASK), str(f), "--joints", "0"],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    m = re.search(r'\{\s*0, J_ROT,\s*([\d.]+)', r.stdout)
    assert m and abs(float(m.group(1)) - (1475.6 + 10.901 * 2.0)) < 2.0, r.stdout


def test_reference_fk_matches_firmware():
    exe = hb.build("firmware/ik_demo", name="sir")
    out, _ = hb.run(exe, ["HOME", "J 20 80 -100 -60 0", "W"])
    th = [float(v) for v in re.search(r"# th\* deg\s+(.*)", out).group(1).split()]
    X, Y, Z = (float(v) for v in re.search(r"grasp point paper \[(-?[\d.]+), (-?[\d.]+)\] cm, height (-?[\d.]+)", out).groups())
    rob, task = K.read_robot(ROBOT), K.read_task(TASK)
    p = K.base_to_paper(K.fk(th, rob["geo"], rob["geo"]["ARM_TIP"] - task["GRIP_DEPTH_MM"])[:3, 3], task)
    assert abs(p[0] - X) < 0.06 and abs(p[1] - Y) < 0.06 and abs(p[2] - Z) < 0.06, (p, X, Y, Z)
