"""servo_tester v3.0 against DD-ST (docs/design/servo_tester.md), on the emulated PCA9685 + EEPROM."""
import math
import re
import tempfile
from pathlib import Path

import host_build as hb

EXE = None
UPC = 1e6 * 132 / 27e6


def exe():
    global EXE
    if EXE is None:
        EXE = hb.build("firmware/servo_tester")
    return EXE


def run(cmds, state=None):
    if state is None:
        state = Path(tempfile.mkdtemp()) / "state.bin"   # fresh EEPROM every test unless shared
    return hb.run(exe(), cmds, state=state)


def fit_line_of(out, ch):
    m = [l for l in out.splitlines() if l.startswith(f"FIT,A,{ch},")][-1].split(",")
    return dict(n=int(m[3]), us0=float(m[4]), uspd=float(m[5]), lash=float(m[6]), rms=float(m[7]), worst=float(m[8]))


def pt_lines(out):
    return [l.split(",") for l in out.splitlines() if l.startswith("PT,")]


# ------------------------------------------------------------------ safety (unchanged from v2.3)
def test_no_pulse_before_start():                       # ST-S1
    out, log = run(["SEL 0", "PT 5", "GO 1600", "SEL 3", "LO"])
    assert all(off == 4096 for _, off in log.writes()), "a pulse went out before START"
    assert out.count("NOT_STARTED") == 3


def test_adopt_on_boot():                               # ST-S2
    st = Path(tempfile.mkdtemp()) / "s.bin"
    run(["SEL 3", "START 1500", "GO 1600"], state=st)
    out, log = run([], state=st)
    assert re.search(r"POS,\d+,A,3,1(59[5-9]|60[0-5]),ADOPTED", out), out
    assert log.writes() == [], "adoption must not write"


def test_soft_limit_clamp():                            # ST-S3
    out, log = run(["SEL 2", "START 1000", "GO 2000"])
    assert "POS" in out and ",2,1300,START" in out
    assert re.search(r",2,1700,IDLE", out)
    assert all(off * UPC < 1705 for ch, off in log.writes() if off < 4096)


def test_slew_rate():                                   # ST-S4
    out, log = run(["SEL 1", "START 1500", "RATE 5 20", "GO 1700"])
    w = [(off, t) for ch, off, t in log.writes_t() if ch == 1 and off < 4096]
    for (a, ta), (b, tb) in zip(w[1:], w[2:]):
        assert abs(b - a) * UPC <= 5 + UPC and tb - ta >= 20, (a, b, ta, tb)


def test_limits_not_restored_at_boot():                 # ST-S8
    st = Path(tempfile.mkdtemp()) / "s.bin"
    run(["LIM 0 500 2500", "SEL 0", "START 1500", "GO 600", "LO", "GO 2400", "HI", "OFF 0"], state=st)
    out, _ = run(["SEL 0", "STAT", "LIMRESTORE", "STAT"], state=st)
    stats = [l for l in out.splitlines() if l.startswith("# arm=")]
    assert "lim=1300-1700" in stats[0], stats
    assert "lim=630-2370" in stats[1], stats


# ------------------------------------------------------------------ mapping
def test_lo_hi_pull_limits_in():                        # ST-M2
    out, _ = run(["LIM 0 500 2500", "SEL 0", "START 1500", "GO 600", "LO", "MARGIN 50", "GO 2400", "HI"])
    lims = re.findall(r"LIMS,\d+,A,0,(\d+),(\d+)", out)
    assert ("630", "2500") in lims and ("630", "2350") in lims, lims


def test_pth_ptd_conversions():                         # ST-M3
    out, _ = run(["SEL 1", "START 1500", "PTH 10 60 100", "REF 10", "PTH 10 60 100", "REF 0",
                  "PTD -50 86.6025", "PTD 0 -100"])
    vals = [float(p[7]) for p in pt_lines(out)]
    assert abs(vals[0] - 30.0) < 0.01 and abs(vals[1] - 20.0) < 0.01, vals
    assert abs(vals[2] - 120.0) < 0.01 and abs(vals[3] + 90.0) < 0.01, vals


def test_circle_fit():                                  # ST-M3 (PTXY)
    u0, k, cx, cy, R = 1475.6, 10.901, -5.2, -1.1, 15.3
    cmds = ["LIM 0 500 2500", "SEL 0", "JT ROT base", "START 1500"]
    for us in (900, 1300, 1700, 2100):
        a = math.radians((us - u0) / k)
        cmds += [f"GO {us}", f"PTXY {cx + R * math.cos(a):.3f} {cy + R * math.sin(a):.3f}"]
    out, _ = run(cmds + ["FIT", "EXPORT"])
    f = fit_line_of(out, 0)
    assert abs(f["us0"] - u0) < 1.0 and abs(f["uspd"] - k) < 0.02, f
    assert "axis at paper [-5.20, -1.10]" in out, out[-1500:]
    assert "FRAME -5.2 -1.1 0" in out


def _lash_cmds(u0, k, lash, extra_sides=True):
    """Points taken with APPR from both sides; the true joint sits h = lash/2 on the side opposite the motion."""
    h = lash / 2
    cmds = ["LIM 1 500 2500", "SEL 1", "JT ROT shoulder", "START 1500", "PREC 0.5"]
    for us in (800, 1200, 1600, 2000, 2300):
        for d in ("+", "-") if extra_sides else ("+",):
            side = (-1 if d == "+" else 1) * (1 if k > 0 else -1)   # value terms: lags behind the joint's motion
            cmds += [f"APPR {d} {us}", f"PT {(us - u0) / k + h * side:.3f}"]
    return cmds


def test_fit_recovers_line():                           # ST-M5
    u0, k = 408.8, 10.625
    out, _ = run(_lash_cmds(u0, k, 0.0, extra_sides=False) + ["FIT"])
    f = fit_line_of(out, 1)
    assert abs(f["us0"] - u0) < 0.1 and abs(f["uspd"] - k) < 0.001 and f["rms"] < 0.01, f


def test_fit_recovers_lash():                           # ST-M5, ST-M4
    u0, k = 408.8, 10.625
    out, _ = run(_lash_cmds(u0, k, 3.0) + ["FIT"])
    f = fit_line_of(out, 1)
    assert abs(f["lash"] - 3.0) < 0.05 and abs(f["us0"] - u0) < 0.5, f
    assert "(fitted, significant)" in out


def test_auto_side_from_motion():                       # ST-M4
    out, _ = run(["LIM 1 500 2500", "SEL 1", "START 1500", "APPR + 1400", "PT 90", "APPR - 1400", "PT 91",
                  "SIDE -", "GO 1500", "PT 92", "SIDE A", "LOAD 1", "PT 93"])
    sides = [p[8] for p in pt_lines(out)]
    assert sides == ["-1A", "1A", "-1", "-1A"], sides
    assert "gravity-loaded joint with SIDE A" in out


def test_fit_warnings():                                # ST-M5
    out, _ = run(["SEL 2", "START 1400", "PT 10", "GO 1600", "PT 70", "FIT"])
    assert "only 2 points" in out and "unusual slope" in out
    out, _ = run(["SEL 2", "START 1350", "PREC 0.5", "PT 10", "GO 1450", "PT 25", "GO 1550", "PT 26", "GO 1650", "PT 50", "FIT"])
    assert "scatter is larger" in out
    out, _ = run(["LIM 2 500 2500", "SEL 2", "START 1500", "GO 600", "LO", "GO 1400", "PT 80", "GO 1500", "PT 90",
                  "GO 1600", "PT 100", "FIT"])
    assert "extrapolating to the LO end" in out


def test_wig():                                         # ST-M6
    out, _ = run(["SEL 1", "START 1500", "WIG 2 6 100", "SEL 6", "JT LIN slide", "WIG 3 4.5"])
    lashes = re.findall(r"# backlash ([\d.]+) (deg|mm)", out)
    assert abs(float(lashes[0][0]) - math.degrees(0.04)) < 0.01 and lashes[0][1] == "deg", lashes
    assert lashes[1] == ("1.50", "mm"), lashes


def _mapping_session():
    cmds = ["ARM A", "LIM 0 500 2500", "SEL 0", "JT ROT base", "START 1500"]
    for us, v in ((900, -52.8), (1500, 2.2), (2100, 57.3)):
        cmds += [f"GO {us}", f"PT {v}"]
    cmds += ["GO 530", "LO", "GO 2470", "HI", "WIG 1 3 100",
             "SEL 3", "JT ROT wrist", "LOAD 1", "START 1400", "PT -80", "GO 1300", "PT -71",
             "SEL 5", "JT GRIP gripper", "LIM 5 1100 1950", "START 1150", "SETP OPEN", "GO 1580", "SETP HOLD",
             "SEL 7", "JT ROT spare", "START 1500", "PT 0"]
    return cmds


def parse_export(out):
    block = out[out.index("// ---- BEGIN SERVO_TESTER EXPORT"):out.index("// ---- END SERVO_TESTER EXPORT ----")]
    rows = re.findall(r"\{\s*(\d+), (J_ROT|J_LIN),\s*(-?[\d.]+),\s*(-?[\d.]+),\s*(\d+),\s*(\d+),\s*([\d.]+), (\d), \"(\w+)\" \}", block)
    grip = re.search(r"const GripCal GRIP = \{ (\d+), (\d+), (\d+), (\d+), (\d+) \};", block)
    return block, rows, grip


def test_export_format():                               # ST-M7, DD-CFG section 4
    out, _ = run(_mapping_session() + ["EXPORT"])
    block, rows, grip = parse_export(out)
    assert [r[0] for r in rows] == ["0", "3"], rows     # ch7 has one point: not exported as a row
    base = rows[0]
    assert base[1] == "J_ROT" and abs(float(base[3]) - 10.9) < 0.1 and base[4:6] == ("560", "2440"), base
    assert abs(float(base[6]) - math.degrees(0.02)) < 0.01 and base[8] == "base"
    assert rows[1][7] == "1" and rows[1][8] == "wrist"
    assert grip and grip.groups() == ("5", "1150", "1580", "1100", "1950"), grip and grip.groups()
    assert "ch7 spare: 1 pts - NOT MAPPED" in block
    assert "CFG_FORMAT 1" in block


def test_session_survives_reset():                      # ST-M8
    st = Path(tempfile.mkdtemp()) / "s.bin"
    out1, _ = run(_mapping_session() + ["EXPORT"], state=st)
    out2, log = run(["SESSION", "EXPORT"], state=st)
    assert "saved mapping session restored: arm A, 6 points" in out2, out2[:800]
    b1, r1, g1 = parse_export(out1)
    b2, r2, g2 = parse_export(out2)
    assert r1 == r2 and g1.groups() == g2.groups()
    assert "NOT applied" in out2


def test_fit_lash_negative_slope():                     # ST-M5 with a reversed servo (negative USPD)
    u0, k = 332.1, -11.587
    cmds = [c.replace("LIM 1", "LIM 2").replace("SEL 1", "SEL 2").replace("shoulder", "elbow") for c in _lash_cmds(u0, k, 3.0)]
    out, _ = run(cmds + ["FIT"])
    f = fit_line_of(out, 2)
    assert abs(f["lash"] - 3.0) < 0.05 and abs(f["uspd"] - k) < 0.01, f


def test_start_probe_refused_on_live_channel():         # ST-S1 / ST-S4
    out, log = run(["SEL 1", "START 1300", "START 1700", "PROBE 1600"])
    assert out.count("ERR") >= 2 and "already holding" in out
    w = [off for ch, off in log.writes() if ch == 1 and off < 4096]
    assert len(w) == 1, w


def test_key_pauses_sweep_dwell():                      # ST-S6: a key during a dwell (not only during motion)
    out, log = run(["SEL 1", "START 1500", "@keyat 2600  ", "SWEEP 1450 1550 25 3000"])
    assert "# paused" in out and "sweep complete" not in out, out[-800:]


def test_bad_channel_refused():                         # ST-S9
    out, log = run(["SEL 15", "START 1500", "OFF 99"])
    assert "BAD_CH" in out and "released ch" not in out
    assert log.writes()[-1][1] != 4096


def test_export_invalid_limits():                       # ST-M7: MIN >= MAX never exported as a row
    out, _ = run(["LIM 0 500 2500", "SEL 0", "JT ROT base", "START 1500", "GO 1200", "PT -25", "GO 1700", "PT 21",
                  "GO 1500", "PT 2", "GO 2000", "LO", "LIM 0 500 2500", "GO 1000", "HI", "EXPORT"])
    block, rows, grip = parse_export(out)
    assert rows == [] and "INVALID" in block, block
