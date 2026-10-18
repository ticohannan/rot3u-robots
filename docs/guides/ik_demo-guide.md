# Guide: Using ik_demo

**Version:** 1.0
**Status:** ACTIVE
**Updated:** 2026-10-17
**Applies to:** `ik_demo` v1.0 on a ROT3U-type arm

Design reference: `docs/design/ik_demo.md`. Robot conventions: `docs/robots/rot3u.md`.

## 1. Set up the two config files

Both live in `firmware/ik_demo/` next to the sketch.

| File | Holds | Comes from |
| --- | --- | --- |
| `robot_config.h` | The arm: calibration (`JOINTS[]`, `GRIP`), geometry, via pose, mass model, driver | `servo_tester` `EXPORT` (joint-mapping guide) |
| `task_config.h` | The assignment: paper frame, block size, grip depth, destination, safety distances, approach angles, policies, starting stacks, a start program | You |

Saved sets live in `configs/`. To switch robots or assignments, copy the pair you want into
`firmware/ik_demo/` and upload.

Until the calibration is real, keep `#define CAL_VALID false`: the sketch then solves and prints but
never moves.

## 2. First run

1. Upload `firmware/ik_demo/ik_demo.ino`. Serial Monitor at 115200 baud, Newline.
2. `CAL`: check every joint's range in degrees and the ready pose.
3. Support the arm; `HOME`. Idle channels get their first pulse one at a time, base outward.
4. Rehearse anything new with `DRY 1` (prints only), then `AIR 40` (every height 40 mm higher), then
   for real with `AIR 0`.
5. Any key stops a move and holds.

## 3. Picking by coordinates (the Demo 3 style)

```
P 10 3 0        top block at paper (10, 3), its bottom at z = 0 cm  -> lists SOL 1..8, ">>> N solutions detected"
T 1             preview solution 1 at the hover pose (no gripping)
T 2             preview solution 2
S 2             select solution 2
GO              pick it and place it on the stack at DEST (default (5, 5))
STACK 0         tell it the destination is empty again
```

(x, y) is the **centre** of the block's footprint. z is the height of the bottom of the top block, so a
block on the paper is z = 0 and the top of a 2-stack is z = 2.5 (cm).

Each `P` also prints what backlash compensation would change (section 7).

`P` plans every solution's complete transfer before it answers, so on the Mega it may take a second or
two, and `MAP` much longer (estimates; not yet timed on the board).

## 4. Goals beyond one pick

Tell the sketch what is on the paper, then give goals:

```
STK 10 3 2      a stack of 2 blocks at (10, 3)
STK 12 -8 1     one block at (12, -8)
WORLD           list the stacks
TOWER 5 5 3     build a 3-high tower at (5, 5) from the other stacks, in the order declared
MV 5 5 -2 12    move the top block of (5, 5) onto the stack at (-2, 12)
```

Declared stacks are also obstacles: every joint move is checked against them, and the carry height
rises to clear them. Blocks you have not declared are invisible to the path check.

## 5. Programs

```
PROG
ECHO my demo
HOME
P 10 3 0
GO
WAIT next block at (12,-8), then Enter
P 12 -8 0
GO
END
RUN
```

`LIST` shows the program; `PCLR` clears it. `task_config.h` can hold a `START_PROGRAM` that is loaded at
boot: the Demo 3 config holds the Demo 3 video script, so `RUN` plays it. `RUN` stops at the first
command that fails or when you press a key. At a `WAIT`, Enter continues and `X` stops.

## 6. Changing the setup without re-calibrating

| Change | Command (and the `task_config.h` constant to make it permanent) |
| --- | --- |
| The paper moved or turned | `FRAME bx by rot z0`: the base axis position on the paper, the direction of the robot's base x axis on the paper (CCW from paper +x), the base height (`FRAME_*`) |
| Work behind the robot | `BRANCH B` (or `F`, `A` = any) (`BRANCH_POLICY`) |
| Grip with a different part of the jaws | `GRIPAT mm`: jaw tips to the middle of the gripping area (`GRIP_DEPTH_MM`) |
| Different blocks | `BLOCK sx sy sz` mm (`BLOCK_*_MM`) |
| Another destination | `DEST x y` (`DEST_*_CM`) |
| Allow flipped blocks | `FLIP 1` (any level orientation, upright preferred) or `FLIP 2` (flipped preferred); `FLIP 0` = upright only (`FLIP_POLICY`) |
| Fix the approach angle | `A -75` (−90 = straight down); `A AUTO` for automatic |

**Front and behind the back.** Behind-the-back solutions exist only if the joints' ranges allow them
(a ROT3U base usually turns about 180°, and its shoulder must lean back past vertical). When a move
between front and back would sweep through the paper, the sketch goes through the via pose
(`VIA_DEG` in `robot_config.h`) and says so.

**Flipping.** A block turns over only when the gripper's orientation at the place differs from the
pick by a turn about a horizontal axis: for example a side grasp (approach angle 0) with the wrist
rolled 180° between pick and place. A straight-down gripper never flips a block. The sketch reports
how each block will arrive ("upright", "on its side", "upside down") and never places one tilted.

## 7. Backlash: measure, then decide

1. Measure each joint's play with `servo_tester` (`WIG`, or points from both sides) and export again.
2. In `ik_demo`, `P` prints for the selected solution:
   `# backlash: compensation would change the grasp pulses by 0 21 -14 9 0 us; without it the grasp point lands about 4.2 mm off.`
3. Choose: `LASH ON` applies the correction to every move; `LASH OFF` (the default) uses the centre of
   each joint's play, so the error never exceeds half the play.
4. Confirm with a touch test (`M x y 0.5`) in both modes before relying on it.

The correction uses gravity for the shoulder, elbow and wrist (from the mass model in
`robot_config.h`), including the carried block, and the direction of motion for the base and roll, and
for any joint near its balance point.

## 8. When something goes wrong

| Message | Meaning and fix |
| --- | --- |
| `0 solutions detected` | Every branch is out of reach, out of a joint's range, or has no complete place. Read the reasons per `SOL`; try `MAP z` to see where picks work |
| `transfer refused before moving` | The plan could not be completed (a vertical line, the place, or the carry path). Nothing moved |
| `path check: ... not moving` | A joint move would hit the paper or a stack, even through the via pose. `HOME` first, or remove the obstacle from the plan |
| `jump guard` | The arm is not where the line move expected (it was moved by hand, or a joint slipped). `HOME` |
| `arm not holding - use HOME first` | Some channel has no pulse yet |
| `could not return to the ready pose` | The block was placed; the arm waits above the destination. `HOME` when clear |
