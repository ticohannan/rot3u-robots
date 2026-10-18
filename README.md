# rot3u-robots

Arduino firmware and tools for calibrating and driving low-cost servo robot arms, starting with the
**ROT3U** 6-servo desktop arm. Built for a robotics course (ECE 6311) and written to be reused.

| Part | What it does |
| --- | --- |
| [`firmware/servo_tester`](firmware/servo_tester) | Maps a robot one joint at a time, safely: finds each joint's range, measures pulse-to-angle (or mm) with a hand ruler, measures backlash, and prints a calibration block for other programs |
| [`firmware/ik_demo`](firmware/ik_demo) | Inverse-kinematics pick and place on graph paper: lists the IK solutions, lets you choose one, picks and places blocks, builds towers, runs programs, optionally compensates backlash |
| [`firmware/common`](firmware/common) | The shared servo-driver interface (PCA9685 board, or servos on Arduino pins) and config types |
| [`tools`](tools) | PC tools: reference kinematics, touch-test refit, header sync |
| [`configs`](configs) | Saved robot calibrations and task setups, plus templates |
| [`docs`](docs) | Design documents, robot conventions, guides |
| [`tests`](tests) | Simulations of both sketches on a PC, checked against an independent model |

## Quick start

1. **Hardware:** Arduino Mega 2560, a PCA9685 16-channel servo board at I2C address 0x40, servos on
   channels 0–5 (base, shoulder, elbow, wrist pitch, wrist roll, gripper), a separate 5–6 V servo supply.
2. **Map the robot:** upload `firmware/servo_tester` and follow
   [docs/guides/joint-mapping.md](docs/guides/joint-mapping.md). It ends with an `EXPORT` block.
3. **Configure:** paste the block into `firmware/ik_demo/robot_config.h`; set the paper frame and the
   task in `firmware/ik_demo/task_config.h` (templates in `configs/templates/`).
4. **Run:** upload `firmware/ik_demo`, then `HOME`, `P 10 3 0`, `GO`. See
   [docs/guides/ik_demo-guide.md](docs/guides/ik_demo-guide.md).

Git on a Windows laptop: [docs/guides/git-on-windows.md](docs/guides/git-on-windows.md).

## Status

`servo_tester` v3.0 and `ik_demo` v1.0 are compiled for the Mega 2560 and pass the PC test suite
(`python tests/run_tests.py`; GitHub Actions runs it on every push). **They have not yet been run on
hardware in this form.** They build on `servo_tester` v2.3 (verified on a bench servo) and `ik_demo`
v0.4 (used on the arm "Sir Pinch-a-lot" for Robot Demo 3). See [CHANGELOG.md](CHANGELOG.md).

## Safety

These programs move real arms. Keep the servo power switch within reach, support an arm whose
position is unknown before its first pulse, and rehearse new sequences with `DRY 1` and `AIR 40`
before running them at full height. The firmware never sends a pulse before you start a channel,
clamps every pulse to soft limits, limits speed, and stops on any keypress.

## Changing the code

Design document first, then code, then a test: see [CONTRIBUTING.md](CONTRIBUTING.md).

## Authorship

Author: ticohannan, who is responsible for this code and documentation. Prepared with the
assistance of AI tools; every change is reviewed, tested and accepted by the author.

## Licence

BSD 3-Clause, see [LICENSE](LICENSE).
