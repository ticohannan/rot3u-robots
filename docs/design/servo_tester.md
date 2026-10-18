# Design: servo_tester (DD-ST)

**Version:** 3.0
**Status:** ACTIVE
**Updated:** 2026-10-17
**Applies to:** `firmware/servo_tester/servo_tester.ino` v3.0

`servo_tester` is the first program to run on a new or rebuilt robot. It moves one joint at a time,
under tight safety limits, while a person finds each joint's safe range, measures what the joint
actually does at a few pulse widths, and measures its free play. It fits a line to the measurements
and prints a configuration block that `ik_demo` (or any other program) compiles in directly.

It is generic: it knows nothing about the robot's geometry. Joints may be rotary (degrees), linear
(millimetres) or grippers (pulse settings only). The measurement tools assumed are a hand ruler and,
optionally, a protractor or a phone level app.

Lineage: v3.0 keeps every command and safety behaviour of v2.3 and adds the mapping commands.

## 1. Requirements

### Safety (unchanged from v2.3; RULES 1.3)

| ID | Requirement |
| --- | --- |
| ST-S1 | No channel outputs a pulse until `START` (or `PROBE`). Outputs stay off at boot |
| ST-S2 | On boot, channels already producing a valid pulse are **adopted**, not reset. `INIT` is the only way to force outputs off by reconfiguring |
| ST-S3 | Every channel has soft limits, default 1300–1700 µs, inside the absolute 500–2500 µs. Commands outside are clamped |
| ST-S4 | Moves are slew-rate limited (default 5 µs per 20 ms; ceiling 25 µs per step, floor 10 ms) |
| ST-S5 | Only the selected channel moves. One joint at a time |
| ST-S6 | Any keypress during a move pauses it and holds |
| ST-S7 | No automatic release by default (`HOLD 0`); the watchdog only pauses |
| ST-S8 | Saved limits are never applied at boot. `LIMRESTORE` applies them on request |

### Mapping (new in v3.0)

| ID | Requirement |
| --- | --- |
| ST-M1 | Each channel can be given a type (`ROT`, `LIN`, `GRIP`), a name, and a gravity-load flag |
| ST-M2 | The found ends of a joint's range are recorded (`LO`, `HI`) and the live soft limit is pulled inward by a margin at once |
| ST-M3 | A measurement point pairs the current pulse with a joint value entered in one of four ways: direct value; two heights along a link; two offsets along a link; a tip position on a grid |
| ST-M4 | Each point is tagged with the side of the joint's play: automatically from the direction of the last motion, or set by hand for gravity-loaded joints |
| ST-M5 | `FIT` fits value = (us − US0)/USPD + (LASH/2)·side by least squares, and reports residuals, scatter against the stated reading precision, the lash and whether it is significant, and warnings for unusual slopes and extrapolation |
| ST-M6 | A direct backlash measurement (`WIG`) converts a pushed-both-ways displacement into lash |
| ST-M7 | `EXPORT` prints the block defined in DD-CFG §4 |
| ST-M8 | Mapping data survives a reset: it is saved to EEPROM after every change and restored at boot |
| ST-M9 | Lower-precision instruments are supported: every method states how its reading error turns into joint-value error, and the fit compares scatter with the stated precision |

## 2. Hardware

Arduino Mega 2560 or any board with ≥ 4 KB RAM and ≥ 2 KB EEPROM, plus a servo driver supported by
DD-DRV (default PCA9685 at 0x40). Serial at 115200 baud; line ending Newline or None.

## 3. Commands

Commands are whole lines, case-insensitive. Numbers may be decimal.

### 3.1 Unchanged from v2.3

| Command | Action |
| --- | --- |
| `HELP`, `?` | Command list |
| `SCAN` | I2C bus scan; adopt live channels if the driver is already running |
| `INIT` | Force reconfigure — all outputs go off |
| `PING` | Driver status (`diag`) |
| `STAT` | Selected channel: pulse, limits, rate, state, joint type |
| `ARM id` | Tag data lines with an arm letter |
| `SEL ch` | Select a channel |
| `LIM ch min max` | Soft limits, µs |
| `RATE us ms` | Slew step and period |
| `HOLD ms` | Auto-release after idle (0 = never, default) |
| `WD 0|1` | Pause a move if the PC goes quiet |
| `START us` | First pulse on the selected channel |
| `GO us` | Ramp to a pulse |
| `PROBE us` | Pulse for 200 ms, then release |
| `SWEEP min max step dwell` | Sweep and return |
| `JOG` | `+`/`-` step, `1`–`4` = 1/5/10/25 µs, `m` mark, `q` quit |
| `R`, `X` | Resume a paused move; stop and hold |
| `MARK label` | Log the pulse with a label |
| `OFF [ch]`, `!` | Release a channel; release all |

### 3.2 Joint definition

| Command | Action |
| --- | --- |
| `JT ROT|LIN|GRIP [name]` | Type and name of the selected channel (name ≤ 8 characters) |
| `LOAD 0|1` | Gravity-loaded flag: 1 for a joint whose axis is horizontal and which carries weight |
| `REF deg` | Absolute angle of the **inboard** link, subtracted from `PTH`/`PTD` readings, so a joint angle can be measured as the difference of two absolute link angles. Default 0 |
| `SIDE +|-|A` | Side tag for the next points: `+`/`-` by hand, `A` automatic from the last motion (default) |
| `PREC v` | Your reading precision in the joint's units (deg or mm). Used to judge the scatter. Default 1 |

### 3.3 Range

| Command | Action |
| --- | --- |
| `LO` / `HI` | The current pulse is the low / high end of the safe range (contact, binding, or the end of useful travel). The live soft limit becomes that end ± `MARGIN`, and that margin is saved with the end |
| `MARGIN us` | Margin for the next `LO`/`HI` (default 30). Use `MARGIN 0` when an end is the tester's 500/2500 µs bound rather than a contact, or the margin will cut off travel the joint really has |
| `LIMRESTORE` | Apply the saved range ends (± their margins) to the live soft limits of every channel |

### 3.4 Measurement points

| Command | Joint value recorded | Use when |
| --- | --- | --- |
| `PT v` | `v` directly | Protractor, level app, or a ruler reading of a linear joint |
| `PTH h1 h2 L` | asin((h2 − h1)/L) − REF, deg | Ruler: heights above the table of two marks on the link, the outer one `h2`, `L` apart along the link. Within ±70° of horizontal |
| `PTD dx dz` | Atan2(x = dx, y = dz) − REF, deg (C: atan2(dz, dx)) | Ruler: horizontal offset (outward +) and vertical offset (up +) of the outer mark from the inner mark. Any angle, including past vertical |
| `PTXY x y` | Tip position on grid paper, cm | A vertical-axis joint (a base). Needs ≥ 3 points; `FIT` fits a circle, finds the axis and converts each point to an angle from the paper +x axis |
| `APPR +|- us [span]` | — | Approach `us` from below (`+`) or above (`-`), starting `span` µs away (default 60), so the next point is taken with a known side of the play |
| `PTS` | — | List the selected channel's points with residuals from the last fit |
| `PTDEL n`, `PTCLR` | — | Delete point n, or all points of the selected channel |

Each point prints a data line `PT,<ms>,<arm>,<ch>,<n>,<us>,<kind>,<value>,<side>`.

Precision of the ruler methods (stated in the guide and printed with each point):

- `PTH`: an error δh in each height gives about 57.3·√2·δh / (L·cos θ) degrees. With δh = 1 mm and
  L = 100 mm at θ = 0: 0.8°. It grows without bound toward vertical, hence the ±70° rule.
- `PTD`: about 57.3·√2·δ / √(dx² + dz²) degrees, the same at every angle.
- `PTXY`: an error δ in each coordinate gives about 57.3·δ / radius degrees.

A longer lever is the cure in every case: tape a straight stick or a long ruler to the link and
measure its ends.

### 3.5 Backlash

| Command | Action |
| --- | --- |
| `WIG a b [R]` | Free play measured directly. Hold the commanded pulse, push the link gently one way and read a position `a`, then the other way and read `b`, at distance `R` from the joint axis (same units). Rotary: lash = \|b − a\|/R rad, stored in degrees. Linear: lash = \|b − a\| mm (no `R`) |

The fitted lash (§4) and the `WIG` lash are reported side by side; `EXPORT` prefers `WIG`.

### 3.6 Gripper

| Command | Action |
| --- | --- |
| `SETP OPEN` / `SETP HOLD` | The current pulse is the gripper's open / holding setting (channel type `GRIP`) |

### 3.7 Fit, export, session

| Command | Action |
| --- | --- |
| `FIT [ch]` | Fit one channel (default: selected) and print the report. Prints `FIT,<arm>,<ch>,<n>,<us0>,<uspd>,<lash>,<rms>,<worst>` |
| `EXPORT` | Fit every channel that has data and print the export block (DD-CFG §4) |
| `SESSION` | Summary of the saved data: channels, types, points, ends, lash |
| `NEW YES` | Erase the saved session (RAM and EEPROM). Live limits are not changed |

## 4. Fit

For channel c with points (us_i, v_i, s_i), s_i ∈ {−1, 0, +1}:

- `PTXY` points are first converted. Circle by linear least squares (Kåsa): solve
  [2x 2y 1]·[cx cy k]ᵀ = x² + y², radius = √(k + cx² + cy²). Angle_i = Atan2(x = X_i − cx, y = Y_i − cy)
  (C: atan2(Y − cy, X − cx)), unwrapped along increasing pulse. `PTXY` points and value points cannot be
  mixed in one channel; `FIT` says so.
- Model: us = US0 + USPD·v + c·s. Linear least squares on [1, v, s] when both sides are present and
  n ≥ 4, else on [1, v]. Then lash = 2·(−c/USPD).
- Residual in joint units: r_i = v_i − ((us_i − US0)/USPD − (c/USPD)·s_i).
- Uncertainty: σ² = Σr²·USPD² / (n − p) in µs², covariance σ²(AᵀA)⁻¹; the lash is "significant" when
  it exceeds twice its standard error. A negative or insignificant lash is reported and dropped.

Warnings:

| Condition | Message gist |
| --- | --- |
| n = 2 | Exact line, nothing checks it: add a third point |
| \|USPD\| outside 5–20 µs/deg (rotary) | Unusual slope: check units, or a reading taken on the wrong link |
| rms > 2 × `PREC` | Scatter larger than your reading precision: look at the worst point |
| A range end more than 25 % of the points' span beyond them | Extrapolating: add a point near that end, including the behind-the-back side if the joint goes there |
| Gravity-loaded joint with only one side tagged | Lash not identifiable from points: take points on both sides of the balance point, or use `WIG` |

## 5. Data saved to EEPROM

Header (magic `ST3`, layout version 3, arm, margin), then per channel: type, load flag, side mode,
name, REF, PREC, range ends with their margins, `WIG` lash, gripper open/hold, and the last `LIM` set;
then up to 60 points (channel, kind, side, pulse, two floats). Saved after every change. Restored at
boot with a one-line summary; a session saved by a different layout is discarded. Live soft limits are
**not** restored (ST-S8). The last `LIM` is kept only so `EXPORT` is the same after a reset.

## 6. Not in scope

- Continuous-rotation servos (no position to map) and servos with feedback wires (a feedback reading
  would replace the human measurement; possible later through a driver `readBack`).
- Multi-joint motion. Coordinated motion belongs in the application (`ik_demo`).
- Robot geometry. The kinematic model's angle conventions are documented per robot
  (`docs/robots/`), and the person mapping the joint applies them.

## 7. Tests

| Requirement | Test (`tests/test_servo_tester.py`) |
| --- | --- |
| ST-S1 | `test_no_pulse_before_start` |
| ST-S2 | `test_adopt_on_boot` |
| ST-S3 | `test_soft_limit_clamp` |
| ST-S4 | `test_slew_rate` |
| ST-S8 | `test_limits_not_restored_at_boot` |
| ST-M2 | `test_lo_hi_pull_limits_in` |
| ST-M3 | `test_pth_ptd_conversions`, `test_circle_fit` |
| ST-M4 | `test_auto_side_from_motion` |
| ST-M5 | `test_fit_recovers_line`, `test_fit_recovers_lash`, `test_fit_warnings` |
| ST-M6 | `test_wig` |
| ST-M7 | `test_export_format` |
| ST-M8 | `test_session_survives_reset` |
