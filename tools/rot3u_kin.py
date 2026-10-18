"""ROT3U reference kinematics in Python (independent of the firmware), plus config readers.

The tests use it to check what ik_demo commands; you can use it to check a pose by hand:

    python tools/rot3u_kin.py firmware/ik_demo/robot_config.h 1812 1693 1451 1457 500

prints the joint angles and the grasp point for those pulses (base frame, mm).
Model and conventions: docs/robots/rot3u.md. Spong DH: A = Rot(z,th) Trans(z,d) Trans(x,r) Rot(x,alpha).
Copyright (c) 2026 ticohannan. BSD-3-Clause, see LICENSE.
"""
import re
import sys
from pathlib import Path

import numpy as np


def _num(txt, name, default=None):
    m = re.search(r"\b" + name + r"\s*=\s*(-?[\d.]+)", txt)
    if not m:
        if default is None:
            raise KeyError(name)
        return default
    return float(m.group(1))


def read_robot(path):
    """Joints (kinematic order), gripper and geometry from a robot_config.h."""
    txt = Path(path).read_text()
    body = re.search(r"JointCal\s+JOINTS\[\]\s*=\s*\{(.*?)\n\};", txt, re.S).group(1)
    joints = []
    for row in re.findall(r"\{([^{}]*)\}", body):
        f = [x.strip() for x in row.split(",")]
        joints.append(dict(ch=int(f[0]), type=f[1], us0=float(f[2]), uspd=float(f[3]), min=int(f[4]),
                           max=int(f[5]), lash=float(f[6]), load=int(f[7]), name=f[8].strip('"')))
    g = [int(x) for x in re.search(r"GripCal\s+GRIP\s*=\s*\{([^}]*)\}", txt).group(1).split(",")]
    geo = {k: _num(txt, k) for k in ("ARM_D1", "ARM_R2", "ARM_R3", "ARM_TIP")}
    geo["JAW_HALF_MM"] = _num(txt, "JAW_HALF_MM", 0.0)
    return dict(joints=joints, grip=dict(ch=g[0], open=g[1], hold=g[2], min=g[3], max=g[4]), geo=geo)


def read_task(path):
    txt = Path(path).read_text()
    keys = ["FRAME_BX_CM", "FRAME_BY_CM", "FRAME_ROT_DEG", "FRAME_Z0_MM", "BLOCK_X_MM", "BLOCK_Y_MM",
            "BLOCK_Z_MM", "GRIP_DEPTH_MM", "DEST_X_CM", "DEST_Y_CM", "MIN_TIP_Z_MM", "CLEAR_MM"]
    return {k: _num(txt, k) for k in keys}


def A(t, d, r, a):
    ct, st, ca, sa = np.cos(t), np.sin(t), np.cos(a), np.sin(a)
    return np.array([[ct, -st * ca, st * sa, r * ct], [st, ct * ca, -ct * sa, r * st], [0, sa, ca, d], [0, 0, 0, 1]])


def fk(th_deg, geo, tool):
    """Frame 5 (grasp point at distance `tool` from the wrist axis) for joint angles in degrees."""
    t = np.radians(th_deg)
    return (A(t[0], geo["ARM_D1"], 0, np.pi / 2) @ A(t[1], 0, geo["ARM_R2"], 0) @ A(t[2], 0, geo["ARM_R3"], 0)
            @ A(t[3] + np.pi / 2, 0, 0, np.pi / 2) @ A(t[4], tool, 0, 0))


def elbow(th_deg, geo):
    t = np.radians(th_deg)
    return (A(t[0], geo["ARM_D1"], 0, np.pi / 2) @ A(t[1], 0, geo["ARM_R2"], 0))[:3, 3]


def pulses_to_deg(us, robot):
    return np.array([(u - j["us0"]) / j["uspd"] for u, j in zip(us, robot["joints"])])


def base_to_paper(p, task):
    r = np.radians(task["FRAME_ROT_DEG"])
    x, y, z = p
    return np.array([(np.cos(r) * x - np.sin(r) * y) / 10 + task["FRAME_BX_CM"],
                     (np.sin(r) * x + np.cos(r) * y) / 10 + task["FRAME_BY_CM"], z + task["FRAME_Z0_MM"]])


if __name__ == "__main__":
    if len(sys.argv) != 7:
        print(__doc__)
        sys.exit(1)
    rob = read_robot(sys.argv[1])
    th = pulses_to_deg([float(v) for v in sys.argv[2:7]], rob)
    T = fk(th, rob["geo"], rob["geo"]["ARM_TIP"])
    print("joint angles deg:", np.round(th, 2))
    print("jaw tip, base frame mm:", np.round(T[:3, 3], 1), " tool axis:", np.round(T[:3, 2], 3))
