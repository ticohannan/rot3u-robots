# Guide: Mapping a Robot's Joints with servo_tester

**Version:** 1.0
**Status:** ACTIVE
**Updated:** 2026-10-17
**Applies to:** `servo_tester` v3.0; any servo-driven robot (rotary or linear joints, grippers)

This guide takes a newly built or rebuilt robot from "nothing known" to a calibration block that
`ik_demo` compiles in. It assumes only a **hand ruler** (a protractor or a phone level app helps but is
optional), graph paper, and some tape. Allow about 15–20 minutes per joint the first time.

Design reference: `docs/design/servo_tester.md`. ROT3U angle conventions: `docs/robots/rot3u.md`.

## 0. Before you start (safety)

1. Servo power on its own supply, with the switch within reach. Servo current does **not** come from
   the Arduino's 5 V pin.
2. Decide the robot's **joint conventions** first (zero and positive direction for every joint). For a
   ROT3U they are fixed in `docs/robots/rot3u.md` §2. For another robot, write them down in a
   `docs/robots/<robot>.md` page before measuring anything.
3. Support the arm by hand or rest it on something soft whenever a joint is first given a pulse: its
   position is unknown until then.
4. One joint at a time. Moves are slow by default (5 µs per 20 ms); leave it that way while mapping.

## 1. Upload and connect

1. Open `firmware/servo_tester/servo_tester.ino` in the Arduino IDE, board **Arduino Mega or Mega
   2560**, and upload.
2. Serial Monitor: **115200 baud**, line ending **Newline**.
3. The banner says `# READY servo_tester v3.0`, then either `no saved mapping session` or `saved
   mapping session restored`. If the robot was already holding a pose, its channels are **adopted** and
   keep holding.
4. Type `ARM A` (any letter: it tags the data lines).

If a previous session is restored and you want to start over: `NEW YES`.

## 2. For each joint, base outward

### 2.1 Define it

```
SEL 1                 select channel 1
JT ROT shoulder       type ROT (degrees) or LIN (mm) or GRIP, and a name of up to 8 characters
LOAD 1                1 if gravity pulls on it (horizontal axis), else 0
PREC 2                your reading precision in degrees (or mm): 2 is honest for a hand ruler
```

### 2.2 First pulse and the safe range

1. Support the link. `START 1500` (or the pulse it was last at). The default soft limits are 1300–1700 µs.
2. Open the window a little at a time, watching for contact, binding, a cable pulling tight, or the end
   of *useful* travel: `LIM 1 1150 1850`, then `JOG` (`+`/`-`, `3` = 10 µs steps, `q` quits).
3. At the low end, stop **before** anything touches: `LO`. At the high end: `HI`. The live soft limit
   is pulled in by `MARGIN` (30 µs) at once.
4. If an end is the tester's own bound (500 or 2500 µs) and nothing touches, type `MARGIN 0` before
   `LO`/`HI` for that joint, then `MARGIN 30` afterwards. Otherwise the margin cuts off real travel.
5. **Cover behind the back too.** If the joint can carry the arm past vertical (a shoulder leaning
   back), the range ends are wherever it really stops on *that* side, not where front-side work stops.

### 2.3 Measurement points

Take **3 to 5 points spread over the whole range**: near `LO`, near `HI`, and in between. Include a
point past vertical if the joint goes there. At each point: `GO us`, wait for `# at ... us`, measure,
then enter one of:

| You have | Command | Notes |
| --- | --- | --- |
| A protractor or phone level on the link | `PT angle` | The angle in the robot's convention |
| A ruler: height of two marks on the link, L apart along it | `PTH h1 h2 L` | h2 = the outer mark. Only within about ±70° of horizontal |
| A ruler: horizontal and vertical offset of the outer mark from the inner | `PTD dx dz` | dx outward +, dz up +. Works at any angle, including past vertical |
| Jaw tips over grid paper (a base / vertical axis) | `PTXY x y` | Needs ≥ 3 points; finds the axis position too |
| A linear joint | `PT mm` | Ruler reading of the slide |

**Relative joints** (an elbow measured as forearm minus upper arm): keep the inboard joint still,
measure the inboard link's absolute angle once, type `REF that_angle`, then measure the outer link
with `PTH`/`PTD`. The point stored is the difference. Set `REF 0` again before measuring an absolute
link.

**The side of the play.** Every point is tagged with which side of its free play the joint sits on:

- Not gravity-loaded (base, roll): leave `SIDE A`. Use `APPR + us` (arrive from below) and
  `APPR - us` (arrive from above) at the same pulse, and take a point after each. The fit then
  measures the play.
- Gravity-loaded: before each point, ask which way the link would fall if the servo went limp; toward
  increasing angle → `SIDE +`, decreasing → `SIDE -`. Take points on **both sides of the balance
  point**, for example the arm leaning forward and leaning back.

### 2.4 Backlash, directly

Hold a pulse in the middle of the range. Put a ruler against the link at a known distance R from the
joint axis (the farther, the better). Push the link gently one way and read the position a; push the
other way and read b. Then `WIG a b R` (for a linear joint, `WIG a b`). For a loaded joint, do this
once on each side of the balance point; the larger value is the honest one.

### 2.5 Fit and check

`FIT` prints the line, the lash, the residual at every point and warnings. Then:

| Warning | What to do |
| --- | --- |
| only 2 points | Add a third point |
| unusual slope | Check the units and that the right link was measured (hobby servos are about 10–11.5 µs/deg) |
| scatter larger than your precision | `PTS`, find the worst point, re-measure it (`PTDEL n`) |
| extrapolating | Add a point near that range end |
| lash not identifiable | Take points on both sides of the play, or use `WIG` |

### 2.6 Gripper

```
SEL 5
JT GRIP gripper
LIM 5 1100 1950
START 1300            then JOG to open the jaws well clear of a block
SETP OPEN
GO 1550               then JOG until the jaws hold a block firmly without the servo straining (buzzing)
SETP HOLD
```

## 3. Export

1. `EXPORT` prints a block between `BEGIN SERVO_TESTER EXPORT` and `END`.
2. Copy it into the robot's `robot_config.h`, replacing the old `JOINTS[]` and `GRIP` lines.
3. Put the rows in **kinematic order** (base first). If the channels are wired in that order already,
   nothing changes.
4. If a `FRAME` line is suggested (from `PTXY`), put those numbers in `task_config.h`.
5. Keep the comment lines: they record how each joint was measured and how well it fitted.
6. Save a copy under `configs/<robot>/` and commit it.

## 4. Touch test (with ik_demo)

Joint-by-joint readings leave errors of a few millimetres at the gripper. A touch test measures the
end result and corrects the offsets.

1. Upload `ik_demo`, `HOME`, then for 6 points spread over the work area: `M x y 0.5`, then `W`.
2. Read where the jaw tips really are on the grid (±2.5 mm is fine) and their height.
3. Make a CSV file, one line per point: the first five pulses from `W` (channels 0–4; the sixth is the
   gripper), then X cm, Y cm, tip height mm.
4. `python tools/touch_refit.py firmware/ik_demo/robot_config.h firmware/ik_demo/task_config.h touches.csv --joints 0,3`
5. Paste the replacement rows it prints, upload, and repeat one or two points to confirm.

Fit few joints from few points: base plus wrist (`0,3`) is a good start. Different joint sets can fit
equally well; any of them corrects the measured area.

## 5. Working with a hand ruler

- **Use a long lever.** Tape a straight stick or a 30 cm ruler along the link and measure its ends:
  1 mm of reading error on a 300 mm lever is 0.2°, on a 50 mm link it is 1.1°.
- **Heights from the table** with a ruler standing vertically on it beat diagonal measurements.
- **Parallax:** put your eye level with the mark, or read both ends from the same side.
- **More points beat one careful point.** Scatter averages out roughly as 1/√n. Five honest points at
  ±2° give a better line than two at ±1°.
- **State the precision** (`PREC`). The fit compares its scatter with it and tells you when a reading is
  probably a mistake rather than noise.
- **Grid paper is an instrument.** `PTXY` on 1 cm paper at a 15 cm radius gives about 1° per 2.5 mm.
