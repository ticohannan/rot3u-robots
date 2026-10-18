# ROT3U Arm: Model and Conventions

**Version:** 1.0
**Status:** ACTIVE
**Updated:** 2026-10-17
**Applies to:** the `KIN_ROT3U` model in `ik_demo`; mapping a ROT3U-type arm with `servo_tester`

The ROT3U is a low-cost 6-servo desktop arm: base yaw, shoulder, elbow, wrist pitch, wrist roll and a
parallel gripper, usually with MG996R servos. Arms sold under this name differ in detail, so every
number below is **per arm** and must be measured. This page fixes the **angle conventions** the
software uses, so that a measurement means the same thing to `servo_tester` and to `ik_demo`.

## 1. DH model (Spong convention)

A_i = Rot(z, θ_i) · Trans(z, d_i) · Trans(x, r_i) · Rot(x, α_i). z_(i−1) is the axis of joint i.

| Link | α | r | d | θ |
| --- | --- | --- | --- | --- |
| 1 base | +90° | 0 | `ARM_D1` | θ1 |
| 2 shoulder | 0 | `ARM_R2` | 0 | θ2 |
| 3 elbow | 0 | `ARM_R3` | 0 | θ3 |
| 4 wrist pitch | +90° | 0 | 0 | θ4* + 90° |
| 5 wrist roll | 0 | 0 | `ARM_TIP` − grip depth | θ5 |

Column order note: `robotica` and exam DH tables use r, α, d, θ.

- `ARM_D1`: base plate to the shoulder axis. `ARM_R2`: shoulder axis to elbow axis. `ARM_R3`: elbow
  axis to wrist-pitch axis. `ARM_TIP`: wrist-pitch axis to the **jaw tips**, along the gripper.
  Measure by hand with a ruler touching both reference points (photographs foreshorten).
- The wrist-pitch and roll axes intersect, so the IK decouples into position and orientation.
- The +90° on link 4 keeps θ4* = 0 when the gripper is in line with the forearm. Software that takes
  θ4* must add it before building A4.
- Home (all angles 0): arm straight out horizontally; grasp point at
  (`ARM_R2` + `ARM_R3` + `ARM_TIP` − grip depth, 0, `ARM_D1`).

Reference implementation: `tools/rot3u_kin.py` (also used by the tests).

## 2. Joint angle conventions — what to measure

All angles in degrees. "Outward" means the horizontal direction the arm is reaching (the base frame's
x₁ direction). "Absolute angle" of a link = its angle above horizontal-outward, measured in the arm's
vertical plane: 0 = horizontal pointing outward, +90 = straight up, 180 = horizontal pointing back
(behind the back), −90 = straight down.

| Joint | Value | How to measure (servo_tester) |
| --- | --- | --- |
| θ1 base | Direction of the arm's vertical plane, CCW seen from above, from the base frame x axis | `PTXY`: jaw tips on the grid paper at ≥ 3 pulses; the circle fit finds the base axis and the angles from paper +x. Use paper +x as the base x axis (`FRAME_ROT_DEG = 0`) |
| θ2 shoulder | Absolute angle of the upper arm (shoulder axis → elbow axis) | `PTD dx dz` with the inner mark on the shoulder axis and the outer on the elbow axis; `REF 0` |
| θ3 elbow | Absolute angle of the forearm **minus** that of the upper arm. Folding down is negative | Hold the shoulder still; measure the upper arm once (that is `REF`); then `PTD` along the forearm (elbow axis → wrist axis) |
| θ4* wrist | Absolute angle of the gripper (wrist axis → jaw tips) **minus** that of the forearm. Straight down with a level forearm is −90 | `REF` = forearm absolute angle; `PTD` along the gripper |
| θ5 roll | Rotation about the gripper axis. 0 = jaws close **across** the arm's plane (jaw faces parallel to it); +90 = turned clockwise seen from above with the gripper pointing down | `PT 0` with the jaws across the plane, `PT 90` / `PT -90` turned. A square or the grid lines is enough |
| gripper | Open and holding pulses | `JT GRIP`, `SETP OPEN`, `SETP HOLD` (on a real block) |

Signs check: with the arm in front, raising the upper arm increases θ2; bending the elbow downward
makes θ3 more negative; tipping the jaws down makes θ4* more negative.

**Behind the back.** The same conventions hold when the arm leans past vertical: the upper arm's
absolute angle goes above 90°, and `PTD` reports it correctly because dx becomes negative. `PTH`
(heights only) cannot tell 60° from 120°, so use `PTD` on any joint that crosses vertical.

## 3. Gravity load and the side of the play

Shoulder, elbow and wrist pitch carry weight: set `LOAD 1` on them; base and roll get `LOAD 0`.

For a loaded joint, the joint sits on the side its load pushes it to. To tag a point, ask: **if this
servo went limp now, which way would the link fall?** Toward increasing θ → `SIDE +`; toward
decreasing θ → `SIDE -`.

| Pose | Shoulder | Elbow | Wrist |
| --- | --- | --- | --- |
| Arm reaching forward and down (most picks) | `-` | `-` (forearm and gripper outward of the elbow) | `-` if the gripper's weight is outward of the wrist axis, `+` if inward |
| Upper arm leaning back past vertical | `+` | depends on where the forearm's weight is | as above |

The **balance point** is where the side flips. Near it the joint can rest anywhere in its play, so
take points clearly on each side, not at it. `ik_demo` computes the side from a mass model
(`MASS_*` in `robot_config.h`); inside the balance band (`BAL_BAND_GMM`) it falls back to the direction
of motion.

## 4. Known differences between arms

| Arm | Notes |
| --- | --- |
| Sir Pinch-a-lot (arm A), Build Rev B, aluminium horns | d1 113, r2 104, r3 97, tip 174 mm (grasp surface 170 with rubber pads). Wrist bracket moved 90° on its horn: the brackets collide above 1500 µs on ch3, so the wrist cannot fold past about −86°; picks use a gripper tilted outward (automatic approach angle). Base axis is not under the centre of the platform. Config: `configs/rot3u_sir_pinch_a_lot_revB/` |
| Don Pinzas (arm B) | Not yet mapped |
