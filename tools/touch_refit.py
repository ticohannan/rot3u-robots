"""Touch-test refit: correct joint offsets (and optionally the base-axis position) from where the jaw
tips really landed, for a ROT3U arm (docs/guides/joint-mapping.md, section 4).

1. In ik_demo, for each of 4-8 grid points spread over the work area: `M x y 0.5`, then `W`.
2. Write one line per point in a CSV file:  us0,us1,us2,us3,us4,X_cm,Y_cm,tip_mm
   (the five pulses from the W line, then where the jaw tips really are on the paper, and their height).
3. python tools/touch_refit.py firmware/ik_demo/robot_config.h firmware/ik_demo/task_config.h touches.csv
       [--joints 0,1,2,3] [--base-xy] [--prec-xy 2.5] [--prec-z 1.0]

It prints the fitted offsets, the residual at every point, and replacement JOINTS rows (only US0 changes)
plus a FRAME line if --base-xy was used. With few points, fit few joints: base and wrist (0,3) is a good
start; several joint sets often fit equally well (see the residuals), and any of them corrects the work
area that was measured.
Copyright (c) 2026 ticohannan. BSD-3-Clause, see LICENSE.
"""
import argparse
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rot3u_kin as K  # noqa: E402


def tip_paper(th, rob, task, base_xy):
    T = K.fk(th, rob["geo"], rob["geo"]["ARM_TIP"])
    t = dict(task, FRAME_BX_CM=base_xy[0], FRAME_BY_CM=base_xy[1])
    return K.base_to_paper(T[:3, 3], t)


def refit(rob, task, touches, joints=(0, 1, 2, 3), base_xy=False, prec_xy=2.5, prec_z=1.0, prior_deg=3.0, prior_mm=6.0):
    """touches: list of (pulses[5], X_cm, Y_cm, tip_mm). Returns (dtheta[5] deg, base (x, y) cm, residuals mm)."""
    th0 = [K.pulses_to_deg(us, rob) for us, *_ in touches]
    meas = np.array([[x * 10, y * 10, z] for _, x, y, z in touches], float)
    sig = np.array([prec_xy, prec_xy, prec_z])
    free = list(joints) + (["bx", "by"] if base_xy else [])
    pri = np.array([prior_deg if f not in ("bx", "by") else prior_mm for f in free])

    def model(p):
        d = np.zeros(5)
        b = [task["FRAME_BX_CM"], task["FRAME_BY_CM"]]
        for f, v in zip(free, p):
            if f == "bx":
                b[0] += v / 10
            elif f == "by":
                b[1] += v / 10
            else:
                d[f] = v
        return np.array([tip_paper(t + d, rob, task, b) * [10, 10, 1] for t in th0]), d, b

    p = np.zeros(len(free))
    for _ in range(25):
        m0, _, _ = model(p)
        r = np.r_[((meas - m0) / sig).ravel(), -p / pri]
        J = []
        for i in range(len(p)):
            e = np.zeros(len(p)); e[i] = 1e-3
            J.append(((model(p + e)[0] - model(p - e)[0]) / sig).ravel() / 2e-3)
        Jm = np.vstack([np.array(J).T, np.diag(1 / pri)])
        p = p + np.linalg.lstsq(Jm, r, rcond=None)[0]
    m, d, b = model(p)
    return d, b, meas - m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("robot"); ap.add_argument("task"); ap.add_argument("csv")
    ap.add_argument("--joints", default="0,1,2,3"); ap.add_argument("--base-xy", action="store_true")
    ap.add_argument("--prec-xy", type=float, default=2.5); ap.add_argument("--prec-z", type=float, default=1.0)
    a = ap.parse_args()
    rob, task = K.read_robot(a.robot), K.read_task(a.task)
    touches = []
    for row in csv.reader(open(a.csv)):
        if not row or row[0].strip().startswith("#"):
            continue
        v = [float(x) for x in row]
        touches.append((v[:5], v[5], v[6], v[7]))
    joints = [int(j) for j in a.joints.split(",") if j != ""]
    d, b, res = refit(rob, task, touches, joints, a.base_xy, a.prec_xy, a.prec_z)
    print("fitted joint offsets (actual = commanded + d), deg:", np.round(d, 2))
    for i, r in enumerate(res):
        print(f"  point {i + 1}: miss x {r[0]:+5.1f} y {r[1]:+5.1f} z {r[2]:+5.1f} mm")
    xy = np.hypot(res[:, 0], res[:, 1])
    print(f"residual xy: rms {np.sqrt(np.mean(xy ** 2)):.1f} mm, worst {xy.max():.1f} mm")
    print("\nreplacement rows (US0 = US0 - USPD * d):")
    for j, dd in zip(rob["joints"], d):
        u0 = j["us0"] - j["uspd"] * dd
        print(f'  {{ {j["ch"]:2d}, {j["type"]}, {u0:7.1f}, {j["uspd"]:8.3f}, {j["min"]:4d}, {j["max"]:4d}, '
              f'{j["lash"]:5.2f}, {j["load"]}, "{j["name"]}" }},')
    if a.base_xy:
        print(f"\nbase axis: FRAME {b[0]:.1f} {b[1]:.1f} {task['FRAME_ROT_DEG']:.0f} {task['FRAME_Z0_MM']:.0f}")


if __name__ == "__main__":
    main()
