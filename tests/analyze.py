"""Check what ik_demo commanded, using the independent Python model in tools/rot3u_kin.py.

transfers(log, robot, task, grip_depth) -> one dict per GO/MV/TOWER transfer:
    pick      grasp point (paper cm, cm, mm) at the end of the descent (step 3)
    place     grasp point at the end of the lowering (step 7)
    tilt      angle between the block's up axis after placing and vertical, deg
    plan      the "# GO:" line the sketch printed
    min_tip   lowest jaw-tip height over the whole transfer, mm
    min_elbow lowest elbow height, mm
    carry     list of (paper x, y, lowest point of the carried block) while carrying (steps 5-7)
    maxstep   largest single-update pulse change on channels 0-4, us
"""
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import rot3u_kin as K  # noqa: E402

UPC = 1e6 * 132 / 27e6          # us per PCA9685 count at prescale 131


def transfers(log, robot, task, grip_depth=None, grasp_up=None):
    gd = task["GRIP_DEPTH_MM"] if grip_depth is None else grip_depth
    geo = robot["geo"]
    tool = geo["ARM_TIP"] - gd
    chans = [j["ch"] for j in robot["joints"]]
    cur = {}
    out, tr, step, prev = [], None, None, None
    for tag, rest in log.events:
        if tag == "O":
            if rest.startswith("# GO:"):
                tr = dict(plan=rest, steps={}, min_tip=1e9, min_elbow=1e9, carry=[], maxstep=0)
                out.append(tr)
            m = re.match(r"# step (\d)/9", rest)
            if m and tr is not None:
                step = int(m.group(1))
            if rest.startswith("# done") or rest.startswith("# sequence stopped"):
                step = None
                if tr is not None:
                    tr["ok"] = rest.startswith("# done")
                    tr = None
        elif tag == "W":
            ch, off = map(int, rest.split()[:2])
            if off < 4096:
                cur[ch] = off * UPC
        elif tag == "T" and tr is not None and step is not None and all(c in cur for c in chans):
            us = [cur[c] for c in chans]
            if prev is not None:
                tr["maxstep"] = max(tr["maxstep"], max(abs(a - b) for a, b in zip(us, prev)))
            prev = us
            th = K.pulses_to_deg(us, robot)
            T = K.fk(th, geo, tool)
            Ttip = K.fk(th, geo, geo["ARM_TIP"])
            tip_z = Ttip[2, 3] + task["FRAME_Z0_MM"]
            el_z = K.elbow(th, geo)[2] + task["FRAME_Z0_MM"]
            tr["min_tip"] = min(tr["min_tip"], tip_z)
            tr["min_elbow"] = min(tr["min_elbow"], el_z)
            p = K.base_to_paper(T[:3, 3], task)
            tr["steps"].setdefault(step, []).append((p, T[:3, :3]))
            if 5 <= step <= 7 and grasp_up is not None:
                tr["carry"].append((p[0], p[1], p[2] - grasp_up))
    for tr in out:
        s = tr["steps"]
        if 3 in s and 7 in s:
            tr["pick"] = s[3][-1][0]
            tr["place"] = s[7][-1][0]
            Rrel = s[7][-1][1] @ s[3][-1][1].T
            tr["tilt"] = float(np.degrees(np.arccos(np.clip(Rrel[2, 2], -1, 1))))
            tr["up_z"] = float(Rrel[2, 2])
    return out


# ---------------------------------------------------------------- a "true" arm with backlash
def masses(robot_txt, task_txt):
    g = lambda t, k: K._num(t, k)
    return dict(upper=g(robot_txt, "MASS_UPPER_G"), elbow=g(robot_txt, "MASS_ELBOW_G"), fore=g(robot_txt, "MASS_FORE_G"),
                wrist=g(robot_txt, "MASS_WRIST_G"), tool=g(robot_txt, "MASS_TOOL_G"), com=g(robot_txt, "TOOL_COM_MM"),
                band=g(robot_txt, "BAL_BAND_GMM"), block=g(task_txt, "BLOCK_MASS_G"))


def gravity_sides(th, geo, tool, m, carrying):
    """Independent re-implementation of DD-IK section 5: side of each pitch joint under gravity.
    Torque about joint j in the arm plane: tau = -sum(m_k * (u_k - u_j)) over masses outboard of j."""
    t2 = np.radians(th[1]); t23 = t2 + np.radians(th[2]); ph = t23 + np.radians(th[3])
    sh = np.array([0.0, geo["ARM_D1"]])
    el = sh + geo["ARM_R2"] * np.array([np.cos(t2), np.sin(t2)])
    wr = el + geo["ARM_R3"] * np.array([np.cos(t23), np.sin(t23)])
    d = np.array([np.cos(ph), np.sin(ph)])
    masses_ = {"upper": ((sh + el) / 2, m["upper"]), "elbow": (el, m["elbow"]), "fore": ((el + wr) / 2, m["fore"]),
               "wrist": (wr, m["wrist"]), "tool": (wr + m["com"] * d, m["tool"]),
               "block": (wr + tool * d, m["block"] if carrying else 0.0)}
    outboard = {1: ("upper", "elbow", "fore", "wrist", "tool", "block"), 2: ("fore", "wrist", "tool", "block"),
                3: ("wrist", "tool", "block")}
    axis = {1: sh, 2: el, 3: wr}
    sides = {}
    for j, names in outboard.items():
        tau = -sum(masses_[n][1] * (masses_[n][0][0] - axis[j][0]) for n in names)
        sides[j] = 0 if abs(tau) <= m["band"] else int(np.sign(tau))
    return sides


def true_positions(log, robot, task, m, grip_depth=None):
    """Grasp points at the end of steps 3 and 7 for an arm whose joints really sit LASH/2 to one side:
    gravity side for LOAD joints (outside the balance band), otherwise opposite to the last motion."""
    gd = task["GRIP_DEPTH_MM"] if grip_depth is None else grip_depth
    geo = robot["geo"]; tool = geo["ARM_TIP"] - gd
    js = robot["joints"]; chans = [j["ch"] for j in js]
    cur, lastdir, step, carrying, res, tr = {}, {c: 0 for c in chans}, None, False, [], None
    def actual():
        us = [cur[c] for c in chans]
        th = K.pulses_to_deg(us, robot)
        g = gravity_sides(th, geo, tool, m, carrying)
        for i, j in enumerate(js):
            side = g.get(i, 0) if j["load"] else 0
            if side == 0:
                side = -lastdir[j["ch"]]
            th[i] += 0.5 * j["lash"] * side
        return K.base_to_paper(K.fk(th, geo, tool)[:3, 3], task)
    for tag, rest in log.events:
        if tag == "O":
            if rest.startswith("# GO:"):
                tr = {}; res.append(tr)
            mm = re.match(r"# step (\d)/9", rest)
            if mm:
                if tr is not None and step in (3, 7) and all(c in cur for c in chans):
                    tr[step] = actual()
                step = int(mm.group(1)); carrying = 5 <= step <= 7
        elif tag == "W":
            ch, off = map(int, rest.split()[:2])
            if off < 4096:
                us = off * UPC
                if ch in cur and abs(us - cur[ch]) > 0.1 and ch in lastdir:
                    lastdir[ch] = int(np.sign(us - cur[ch]) * np.sign(next(j["uspd"] for j in js if j["ch"] == ch)))
                cur[ch] = us
    return res
