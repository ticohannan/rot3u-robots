# Design: Robot Configuration Format (DD-CFG)

**Version:** 1.0
**Status:** ACTIVE
**Updated:** 2026-10-17
**Applies to:** `firmware/common/RobotTypes.h`, `servo_tester` v3.x `EXPORT`, `ik_demo` v1.x `robot_config.h` and `task_config.h`

This document defines the one data format shared by the two programs. `servo_tester` measures a
robot and prints an **export block**; that block is pasted into the robot's `robot_config.h`;
`ik_demo` reads it. No JSON, no file system: the format is C++ source, so the Arduino IDE checks it
when the sketch compiles.

## 1. Files

| File | What it holds | Changes when |
| --- | --- | --- |
| `firmware/common/RobotTypes.h` | The structs and enums below. Copied into every sketch folder | The format changes (bump `CFG_FORMAT`) |
| `robot_config.h` (in the `ik_demo` sketch folder) | One physical robot: joints, gripper, geometry, mass model, driver choice | The robot is rebuilt or re-calibrated |
| `task_config.h` (in the `ik_demo` sketch folder) | One assignment or workspace: paper frame, blocks, places, clearances, policies, start program | The assignment or the paper changes |
| `configs/*.h` | Saved copies of the two files above for each robot and task | A robot or task is added |

`robot_config.h` and `task_config.h` are deliberately separate: moving the paper or changing the
assignment never touches a calibration, and re-calibrating never touches the task.

## 2. Joint model

Every actuated joint, rotary or linear, follows one straight line from pulse width to joint value:

```
value = (us - US0) / USPD                 the joint's "centre-line" value
actual = value + (LASH / 2) * side        side = +1 or -1, which side of its play the joint sits on
```

| Symbol | Meaning | Units |
| --- | --- | --- |
| `US0` | Pulse at value 0 | µs |
| `USPD` | Pulse change per unit of value. Its **sign** carries the joint's direction | µs/deg (rotary) or µs/mm (linear) |
| `LASH` | Full free play (backlash, slop, dead band combined) measured at the joint | deg or mm |
| `side` | +1 if the joint is pushed to the + end of its play, −1 if to the − end | — |
| `LOAD` | 1 = gravity-loaded (horizontal axis): `side` comes from the gravity torque. 0 = `side` comes from the direction of the last motion | — |

`US0` and `USPD` describe the **centre** of the play. A program that ignores lash (`side = 0`) is
therefore wrong by at most `LASH/2` at any pose, instead of by `LASH` on one side of a balance point.

**Direction convention for `side`.** A joint dragged in the + direction lags behind the command, so
it sits on the − side of its play: `side = −sign(direction of motion)`. A gravity-loaded joint sits
on the side gravity pushes it toward: `side = sign(gravity torque about the joint's + axis)`.

The joint value's zero and sign are set by the **kinematic model** the robot uses (for ROT3U, see
`docs/robots/rot3u.md`). `servo_tester` does not know the model. The person mapping the joint must
use the model's convention when entering measurements. The mapping guide says how.

## 3. Types (`RobotTypes.h`)

```cpp
#define CFG_FORMAT 1                        // bump when this file changes incompatibly

enum JointType : uint8_t { J_ROT = 0, J_LIN = 1, J_GRIP = 2 };

struct JointCal {
  uint8_t     ch;      // driver channel
  uint8_t     type;    // J_ROT or J_LIN
  float       us0;     // US0, us
  float       uspd;    // USPD, us per deg or per mm (sign = direction)
  uint16_t    minUs;   // safe pulse limits; the IK treats them as joint limits
  uint16_t    maxUs;
  float       lash;    // full backlash, deg or mm (0 = not measured)
  uint8_t     load;    // 1 = gravity-loaded
  const char *name;
};

struct GripCal {
  uint8_t  ch;
  uint16_t openUs;     // jaws clear of a block
  uint16_t holdUs;     // jaws gripping a block
  uint16_t minUs;      // safe pulse limits
  uint16_t maxUs;
};
```

## 4. The export block

`servo_tester` command `EXPORT` prints exactly this shape (values are an example):

```cpp
// ---- BEGIN SERVO_TESTER EXPORT (servo_tester 3.0, arm A, CFG_FORMAT 1) ----
// ch0 base: 4 pts, rms 0.4 deg, worst 0.7 deg, span -60..75 deg, lash 1.2 deg (WIG)
// ch5 gripper: open 1150, hold 1580
const JointCal JOINTS[] = {
  //  ch  type     US0      USPD    MIN   MAX   LASH  LOAD  name
  {  0, J_ROT, 1475.6,  10.901,  530, 2470,  1.20,  0, "base"     },
  {  1, J_ROT,  408.8,  10.625,  530, 2470,  0.00,  1, "shoulder" },
};
const GripCal GRIP = { 5, 1150, 1580, 1124, 1920 };
// base axis from the PTXY circle fit (paper cm): FRAME -5.2 -1.1 0
// ---- END SERVO_TESTER EXPORT ----
```

Rules:

- One `JointCal` row per mapped non-gripper channel, in **channel order**. `ik_demo` uses the rows in
  **kinematic order** (base outward). When the channels are wired in kinematic order, the two orders are
  the same; otherwise the user re-orders the rows by hand.
- `MIN`/`MAX` are the range ends found with `LO`/`HI`, pulled inward by the export margin (`MARGIN`,
  default 30 µs). A channel without marked ends exports the last `LIM` set for it (saved with the
  session), or else its live soft limits, and its comment line says so.
- `LASH` is the `WIG` measurement if one was made, otherwise the lash fitted from the points if it is
  significant, otherwise 0.
- The comment lines carry the method and the quality of each fit, so a pasted block keeps its evidence.
- A channel with fewer than 2 points is exported as a comment, not a row, so the sketch does not
  compile with a made-up calibration.

## 5. `robot_config.h` contents

```cpp
#define ROBOT_NAME "..."
#define CAL_VALID  true          // false: ik_demo solves and prints but never moves
#define KIN_MODEL  KIN_ROT3U     // kinematic model (only KIN_ROT3U is implemented)
// #define SERVO_DRIVER_PINS     // servos on Arduino pins instead of a PCA9685
#define PCA_ADDR   0x40
#define SERVO_PIN_LIST {2, 3, 4, 5, 6, 7}
<export block: JOINTS[] in kinematic order, GRIP>
const float ARM_D1, ARM_R2, ARM_R3, ARM_TIP;   // geometry of the model, mm (docs/robots/rot3u.md)
const float JAW_HALF_MM;                       // half the jaw height across the tool axis, mm
const float VIA_DEG[3];                        // shoulder, elbow, wrist of a tall safe pose, deg
const float MASS_UPPER_G, MASS_ELBOW_G, MASS_FORE_G, MASS_WRIST_G, MASS_TOOL_G;  // gravity model, g
const float TOOL_COM_MM, BAL_BAND_GMM;         // gripper centre of mass; balance band, g*mm
#define LASH_COMP_DEFAULT false                // ik_demo starts with backlash compensation off
```

## 6. `task_config.h` contents

| Item | Meaning | Runtime command |
| --- | --- | --- |
| `FRAME_BX_CM`, `FRAME_BY_CM` | Robot base axis on the paper, cm | `FRAME bx by rot z0` |
| `FRAME_ROT_DEG` | Direction of the robot's base-frame x axis on the paper, CCW from paper +x | `FRAME` |
| `FRAME_Z0_MM` | Height of the base-frame origin above the paper, mm | `FRAME` |
| `BLOCK_X_MM`, `BLOCK_Y_MM`, `BLOCK_Z_MM` | Block size; `Z` is the height when upright | `BLOCK sx sy sz` |
| `BLOCK_MASS_G` | Block mass for the gravity model | — |
| `GRIP_DEPTH_MM` | Distance from the jaw tips back to the middle of the gripping area | `GRIPAT mm` |
| `DEST_X_CM`, `DEST_Y_CM` | Default place location | `DEST x y` |
| `KEEPOUT_R_MM`, `CLEAR_MM`, `MIN_TIP_Z_MM`, `MIN_ELBOW_Z_MM` | Safety distances | — |
| `READY_R_MM`, `READY_Z_MM` | Ready pose: grasp point distance from the base axis and height | — |
| `APPROACH_LIST[]` | Gripper approach angles to try, deg (−90 = straight down) | `A deg` |
| `LEVEL_TOL_DEG` | A placed block within this tilt counts as level (upright, upside down, on its side) | — |
| `BRANCH_POLICY` | `BR_ANY`, `BR_FRONT` or `BR_BACK` | `BRANCH A|F|B` |
| `FLIP_POLICY` | `FLIP_NO` (block must stay upright), `FLIP_OK` (any orientation, upright preferred), `FLIP_PREFER` (flipped preferred, for demonstrations) | `FLIP 0|1|2` |
| `INIT_WORLD[]` | Stacks present at start: x, y, number of blocks | `STK x y n` |
| `START_PROGRAM` | Optional command program, loaded at boot | `PROG`, `RUN` |

## 7. Compatibility

- `CFG_FORMAT` is checked at compile time: `ik_demo` refuses to compile against a `RobotTypes.h`
  with a different value.
- A config written for format 1 stays valid as long as `CFG_FORMAT` is 1. Adding a field to a
  struct is an incompatible change.

## 8. Tests

| Requirement | Test |
| --- | --- |
| The export block compiles in `ik_demo` unchanged | `tests/test_roundtrip.py::test_export_compiles` |
| `EXPORT` values reproduce the fitted line to 0.1 µs | `tests/test_servo_tester.py::test_fit_recovers_line` |
| Gripper and unmapped channels exported as specified | `tests/test_servo_tester.py::test_export_format` |
