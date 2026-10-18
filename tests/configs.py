"""Robot and task configs used only by the tests (written into the host build folder)."""
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ROBOT = (ROOT / "firmware/ik_demo/robot_config.h").read_text()
TASK = (ROOT / "firmware/ik_demo/task_config.h").read_text()

# A ROT3U-shaped arm with wide joint ranges (360-degree servos), so front and behind-the-back
# branches and side grasps are all reachable. Invented numbers; not a real robot.
WIDE_JOINTS = """const JointCal JOINTS[] = {
  {  0, J_ROT, 1500.0,   5.500,  520, 2480,  LASH0, 0, "base"     },
  {  1, J_ROT,  1500.0,  5.500,  520, 2480,  LASH1, 1, "shoulder" },
  {  2, J_ROT,  1500.0, -5.500,  520, 2480,  LASH2, 1, "elbow"    },
  {  3, J_ROT,  1500.0, -5.500,  520, 2480,  LASH3, 1, "wrist"    },
  {  4, J_ROT,   800.0,  5.500,  520, 2480,  0.00, 0, "roll"     },
};"""


def robot_wide(lash=(0, 0, 0, 0)):
    j = WIDE_JOINTS
    for i, v in enumerate(lash):
        j = j.replace(f"LASH{i}", f"{v:.2f}")
    return replace_joints(ROBOT, j)


def robot_with_lash(lash):
    """The real Sir Pinch-a-lot calibration with invented backlash values per joint."""
    txt = ROBOT
    names = ["base", "shoulder", "elbow", "wrist", "roll"]
    for n, v in zip(names, lash):
        import re
        txt = re.sub(r'(\{[^{}]*?,\s*)0\.00(,\s*\d,\s*"' + n + '")', lambda m: m.group(1) + f"{v:.2f}" + m.group(2), txt)
    return txt


def replace_joints(robot_txt, joints_block):
    import re
    new = re.sub(r"const JointCal JOINTS\[\] = \{.*?\n\};", lambda m: joints_block, robot_txt, flags=re.S)
    assert new != robot_txt
    return new


def task_with(**kv):
    """Demo 3 task config with some constants replaced, e.g. task_with(FRAME_ROT_DEG=90)."""
    import re
    txt = TASK
    for k, v in kv.items():
        if k == "APPROACH_LIST":
            txt, n = re.subn(r"const float APPROACH_LIST\[\] = \{[^}]*\};", f"const float APPROACH_LIST[] = {{{v}}};", txt)
        elif k in ("BRANCH_POLICY", "FLIP_POLICY"):
            txt, n = re.subn(r"#define " + k + r"\s+\w+", f"#define {k} {v}", txt)
        elif k == "START_PROGRAM":
            txt, n = re.subn(r"const char START_PROGRAM\[\] PROGMEM =.*?;\n", f"const char START_PROGRAM[] PROGMEM = {v};\n", txt, flags=re.S)
        elif k == "INIT_WORLD":
            txt, n = re.subn(r"const float INIT_WORLD\[\]\[3\] = \{.*?\};", f"const float INIT_WORLD[][3] = {{ {v} }};", txt)
        else:
            txt, n = re.subn(r"(const float " + k + r"\s*=\s*)-?[\d.]+", lambda m: m.group(1) + str(v), txt)
            if n == 0:
                txt, n = re.subn(r"(\b" + k + r"\s*=\s*)-?[\d.]+", lambda m: m.group(1) + str(v), txt)
        assert n, k
    return txt
