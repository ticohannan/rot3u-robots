# Design: ik_demo (DD-IK)

**Version:** 1.0
**Status:** ACTIVE
**Updated:** 2026-10-17
**Applies to:** `firmware/ik_demo/ik_demo.ino` v1.0, `robot_config.h`, `task_config.h`

`ik_demo` moves blocks with a ROT3U-type arm by inverse kinematics. The person types positions on
graph paper, or whole goals ("move the top block from here to there", "build a 3-high tower
here"). It lists the IK solutions, lets the person pick one, and runs pick-and-place sequences with
automatic grip and release, checking every move against the paper, the stacks it knows about, and
the joint limits.

Lineage: v1.0 generalises v0.4 (the Robot Demo 3 sketch). Every v0.4 command still works the same
way; the Demo 3 video script runs unchanged (test `test_demo3_regression`).

## 1. Requirements

### Safety

| ID | Requirement |
| --- | --- |
| IK-S1 | While `CAL_VALID` is false the sketch solves and prints but never moves (`DRY` is forced on) |
| IK-S2 | Every pulse is clamped to the channel's soft limits as the last step before the driver |
| IK-S3 | Every joint-space move is sampled through forward kinematics first (24 points); it is refused if the jaws, a carried block or the elbow would go below the paper, or would pass through a known stack. If the direct move fails, it goes through the **via pose** (`VIA_DEG` in `robot_config.h`): up to it with base and roll unchanged, base and roll turn there, then down to the target, each leg checked the same way |
| IK-S9 | A transfer is planned completely before anything moves: every vertical line (descend, lift, lower, clear) is solved every 2.5 mm, and the carry move is path-checked. A transfer that cannot be completed is refused with the arm still |
| IK-S4 | Straight-line moves stop if any joint would jump more than `JUMP_US` between points (the arm is not where the plan expects) |
| IK-S5 | The elbow is kept at least `MIN_BEND` from straight (singular) |
| IK-S6 | Any key stops a move and holds. During `RUN`, any key stops the program |
| IK-S7 | On boot, live channels are adopted (DD-DRV). The first `HOME` gives a first pulse to idle channels one at a time, base outward |
| IK-S8 | `AIR mm` raises every commanded height for rehearsal; the world model is not changed while AIR is non-zero |
| IK-S10 | The lowest jaw point is the tip, lowered by `JAW_HALF_MM` × the tool's horizontal component (a tilted or horizontal gripper reaches lower than its tip) |

### Generality (new in v1.0)

| ID | Requirement |
| --- | --- |
| IK-G1 | The paper frame (base axis position, paper rotation, base height) is set in `task_config.h` and can be changed at run time with `FRAME`, without re-calibrating |
| IK-G2 | Front and behind-the-back arm branches are both solved. `BRANCH F|B|A` restricts the branches used for picking and placing |
| IK-G3 | The part of the jaws that grips is set by `GRIP_DEPTH_MM` / `GRIPAT`: the grasp point is that far back from the jaw tips |
| IK-G4 | Every transfer computes the block's orientation after placing from the tool rotations at pick and place. Within `LEVEL_TOL_DEG` (task config, default 3°) it is upright, upside down or on its side; anything else is *tilted* and always refused. `FLIP 0` accepts only upright; `FLIP 1` accepts any level orientation but prefers upright; `FLIP 2` prefers flipped. The height of a flipped block is its new vertical extent |
| IK-G5 | A world model of stacks (position, count, top height) is kept. `GO` and the goal commands update it. Path checks use it |
| IK-G6 | Goals beyond the Demo 3 task: `MV` (top block from one stack to another), `TOWER` (build a stack of n from the other known stacks), and stored programs (`PROG` … `END`, `RUN`, with `WAIT` and `ECHO` steps) |
| IK-G7 | Block size is configurable (cuboid) |

### Backlash

| ID | Requirement |
| --- | --- |
| IK-L1 | The calibration line is the centre of each joint's play (DD-CFG §2). With compensation off, pulses come from the centre line |
| IK-L2 | With compensation on, pulse = US0 + USPD·(θ − (LASH/2)·side). For gravity-loaded joints, side = sign of the gravity torque from the mass model, including the carried block; inside the balance band (\|torque\| < `BAL_BAND_NMM`) and for unloaded joints, side = −sign(direction of the move into the pose) |
| IK-L3 | After every `P`, the sketch prints the correction compensation would apply to the selected solution (µs per joint) and the predicted grasp-point error without it (mm), so the person can choose `LASH ON` or `LASH OFF` |
| IK-L4 | With `LASH` = 0 for every joint, compensation changes nothing |

## 2. Kinematic model

Only the ROT3U model (`KIN_ROT3U`) is implemented: a 5-joint arm (base yaw, shoulder, elbow, wrist
pitch, wrist roll) plus gripper, Spong DH convention, frames and angle conventions in
`docs/robots/rot3u.md`. The model is isolated in the `kin*` functions (`kinSolve`, `kinFK`,
`kinToolR`, `kinGravitySide`). A new model is added by implementing those four and a `KIN_*` switch,
after its design is written here.

IK for grasp point p in the base frame, approach angle ψ in the arm plane (−90° = straight down),
branch side s = +1 (front) or −1 (behind the back), elbow e = up or down:

```
θ1  = Atan2(x = s·px, y = s·py)                          C: atan2(s*py, s*px)
u   = s·(√(px² + py²) − d·cos ψ)                         wrist centre along x1;  d = ARM_TIP − grip depth
v   = pz − d·sin ψ − d1
D   = (u² + v² − r2² − r3²) / (2·r2·r3)                   |D| ≤ cos(MIN_BEND)
θ3  = Atan2(x = D, y = σ·√(1 − D²))                      σ = −s for elbow up, +s for elbow down
θ2  = Atan2(x = u, y = v) − Atan2(x = r2 + r3·cos θ3, y = r3·sin θ3)
φ   = Atan2(x = s·cos ψ, y = sin ψ)                      approach direction in the x1 frame
θ4* = φ − θ2 − θ3
θ5  = the multiples of 180° inside the roll range (jaws close across the arm plane): roll A = the one
      nearest 0, roll B = the next
```

Each candidate is wrapped into its joint's range (360° equivalents), checked against the limits, and
checked by forward kinematics (≤ 1 mm). Solutions are numbered 1–8: 1 front/elbow-up roll A,
2 front/elbow-up roll B, 3–4 front/elbow-down, 5–6 back/elbow-up, 7–8 back/elbow-down.

## 3. Frames

- **Paper frame:** cm, the graph paper's axes. `x, y` on the paper, `z` up from the paper.
- **Base frame:** mm, origin on the base axis at `FRAME_Z0_MM` above the paper, x axis along the base
  joint's zero direction.
- Paper → base: d = (X − bx, Y − by)·10; x = cos ρ·dx + sin ρ·dy; y = −sin ρ·dx + cos ρ·dy;
  z = Zpaper − z0, where ρ = `FRAME_ROT_DEG` is the direction of the base x axis on the paper, CCW
  from paper +x.

A position typed as `P x y z` is the **centre of the block's footprint**; `z` is the height of the
**bottom of the top block** (a block on the paper is z = 0).

## 4. Transfer planner

A transfer moves the top block of a source stack to the top of a destination stack.

1. **Pick heights.** Bottom zb. Grasp height above the bottom gu = max(sz/2, grip depth + `MIN_TIP_Z_MM`),
   so the jaw tips never go below the block's own bottom; a warning is printed if gu > sz − 3 mm
   (gripping the very top). Hover = zb + sz + `CLEAR_MM` + grip depth (tips clear the stack).
2. **Pick candidates.** For each approach angle ψ (automatic: the `APPROACH_LIST` order, closest to
   straight down first; or fixed with `A`) and each allowed branch: grasp and hover reachable.
3. **Place candidates** for a pick candidate: each approach angle and allowed branch at the
   destination. The block rotation R = R_place·R_pickᵀ (tool rotations from FK). The block's new
   vertical extent ez = Σ|R₃ⱼ|·sⱼ, the new grasp height above the destination top
   = ez/2 + (R·g)_z, where g is the grasp point's offset from the block centre at pick. Orientation
   class: upright if R₃₃ > 0.9, upside down if R₃₃ < −0.9, otherwise on its side. Accepted per
   `FLIP`. Place grasp, place hover (+`CLEAR_MM` + 10 mm for the carried block) and the clear-up
   pose must be reachable.
4. **Preference order** among complete candidates: orientation (upright first for `FLIP` 0/1, flipped
   first for 2), then pick ψ closest to −90°, then place ψ equal to pick ψ, then the place branch and
   roll equal to the pick's.
5. **Carry heights.** The carry pose above the destination is at zP = place hover + 0, 10, … 80 mm.
   The lift pose above the source is at the highest zL ≤ max(zP, pick hover) that is reachable with the
   elbow ≥ `LIFT_BEND` from straight, but not below the pick hover. The joint move from the lift pose to
   the carry pose must pass the path check (IK-S3) against every stack, and the lift and lower lines
   must solve (IK-S9). The first zP that works is used. If none works, the transfer is refused before
   any motion.
6. **Sequence:** open; hover above the source; descend; grip; lift to the carry height; carry; lower
   onto the destination; release; clear straight up; return to the ready pose.
7. **World update** after release (only when not `DRY` and `AIR` = 0): source count −1 (its top drops
   by sz), destination count +1 (its top rises by ez).

`P x y z` + `GO` is one transfer with the source at (x, y, z) and the destination at `DEST`. A solution
counts toward "N solutions detected" when the whole transfer from that pick can be planned (steps 1–5,
including the carry path check). The moves from the ready pose to the pick hover and back are checked
when they happen, from wherever the arm is; the via pose (IK-S3) handles a branch change.

**Approach angle.** `P` uses the first angle in `APPROACH_LIST` for which some branch has a complete
plan, and lists all 8 solutions at that angle. The ready pose prefers straight down (−90°), then the list.

## 5. Backlash model (IK-L2)

Gravity torque about pitch joint j (shoulder, elbow, wrist) in the arm plane (u outward along x1,
v up): τ = −Σ mₖ·g·(uₖ − uⱼ) over the masses outboard of joint j. Masses (from `robot_config.h`):
`MASS_ELBOW_G` at the elbow axis, `MASS_WRIST_G` at the wrist axis, `MASS_TOOL_G` at `TOOL_COM_MM`
along the tool, the block (`BLOCK_MASS_G`) at the grasp point while carried, and half of each link's
`MASS_UPPER_G`/`MASS_FORE_G` at its midpoint. Positive τ turns the joint toward +θ, so side = sign(τ).
Joints with `LOAD = 0` (base, roll) use the motion direction.

During a move, compensation is evaluated at each target point, with the motion direction of that
step; `lastSide` is kept per joint for steps where a joint does not move.

## 6. Commands

### 6.1 From v0.4 (unchanged behaviour)

| Command | Action |
| --- | --- |
| `P x y z` | Solve for the top block at paper (x, y, z); list solutions; "N solutions detected" |
| `S k` | Select solution k (1–8) |
| `GO` | Pick the target with the selected solution, place it at `DEST` |
| `T k` | Preview solution k at the hover pose (no gripping) |
| `STACK n` | Set the number of blocks at `DEST` |
| `MAP z` | Solution-count map for a block bottom at z cm (automatic approach angle) |
| `HOME` | Ready pose (straight-up retreat first if low) |
| `M x y z` | Jaw tips over paper (x, y) at height z cm (touch test) |
| `J a b c d e` | Joint angles θ1..θ5*, deg |
| `G O|C|us` | Gripper open / hold / pulse |
| `W` | Where: pulses, angles, grasp point on the paper |
| `AIR mm`, `DRY 1|0`, `SPD n`, `CAL`, `!` | As v0.4 |
| `A deg` | Fix the approach angle, −90 to 90 (not 0). `A AUTO` (or `A 0`, as in v0.4) = automatic |

### 6.2 New in v1.0

| Command | Action |
| --- | --- |
| `FRAME [bx by rot z0]` | Show or set the paper frame |
| `BLOCK sx sy sz` | Block size, mm |
| `GRIPAT mm` | Grip depth from the jaw tips |
| `DEST x y` | Destination for `GO` |
| `BRANCH A|F|B` | Any / front only / behind the back only |
| `FLIP 0|1|2` | Transport orientation policy (IK-G4) |
| `LASH ON|OFF` | Backlash compensation |
| `STK x y n` | Declare a stack of n blocks at (x, y); n = 0 removes it; `STK CLR` clears all |
| `WORLD` | List the stacks |
| `MV x1 y1 x2 y2` | Transfer the top block of the stack at (x1, y1) to the stack at (x2, y2) |
| `TOWER x y n` | Transfer blocks from the other stacks, in the order declared, until (x, y) has n |
| `PROG` … `END` | Record a program (up to 24 lines); `RUN` plays it; `LIST`; `PCLR` clears |
| `WAIT text` | (in a program) print the text and wait for Enter |
| `ECHO text` | (in a program) print the text |

## 7. Configuration items used

From `robot_config.h`: `JOINTS[5]` (kinematic order), `GRIP`, `ARM_D1`, `ARM_R2`, `ARM_R3`, `ARM_TIP`,
`JAW_HALF_MM`, `VIA_DEG[3]`, the mass model (`MASS_UPPER_G`, `MASS_ELBOW_G`, `MASS_FORE_G`,
`MASS_WRIST_G`, `MASS_TOOL_G`, `TOOL_COM_MM`, `BAL_BAND_GMM`), `LASH_COMP_DEFAULT`, `CAL_VALID`,
the driver choice. From `task_config.h`: everything in DD-CFG §6 plus `LEVEL_TOL_DEG`.

## 8. Tests

| Requirement | Test (`tests/test_ik_demo.py`) |
| --- | --- |
| IK-S1 | `test_cal_invalid_never_moves` |
| IK-S3, IK-S9 | `test_path_check_refuses_stack_collision`, `test_carry_raises_over_stack`, `test_flip_policy` (uses the via pose) |
| IK-S6 | `test_keypress_stops` |
| IK-G1 | `test_frame_rotation_invariance` |
| IK-G2 | `test_back_branch` |
| IK-G3 | `test_grip_depth` |
| IK-G4 | `test_flip_policy` |
| IK-G5, IK-G6 | `test_world_mv_tower`, `test_program_run` |
| IK-L1–L4 | `test_lash_off_is_centre_line`, `test_lash_sides`, `test_lash_suggestion`, `test_zero_lash_no_change` |
| Regression | `test_demo3_regression` (the Demo 3 video script, FK-checked) |
| Reference model agrees with the firmware | `tests/test_tools.py::test_reference_fk_matches_firmware` |
| Export block compiles and drives `ik_demo` | `tests/test_roundtrip.py::test_export_compiles` |
