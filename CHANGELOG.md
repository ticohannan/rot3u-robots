# Changelog

Newest first. Each row names the versions delivered and what has actually been run.

| Date | Change | Status |
| --- | --- | --- |
| 2026-10-17 | **Repository created.** `servo_tester` v3.0: v2.3's safety model and commands unchanged, plus joint types, range ends (`LO`/`HI`), four measurement methods for a hand ruler (`PT`, `PTH`, `PTD`, `PTXY`), side-of-play tagging, least-squares fit with backlash, `WIG` backlash, gripper settings, `EXPORT` config block, EEPROM-saved sessions. `ik_demo` v1.0: generalises v0.4 (Robot Demo 3) with `robot_config.h` / `task_config.h`, run-time paper frame, front and behind-the-back branches, grip depth, flip policy, world model of stacks, `MV` / `TOWER` goals, programs, backlash compensation with a printed suggestion, via pose. Shared `RobotServoDriver` interface with PCA9685 and pin drivers. Design documents, guides, 55 tests, CI. An independent review before release found and fixed: NaN angles reaching the pulse maths, `START`/`PROBE` jumping on a live channel, `HOME` skipping the path check when one channel was idle, keys not pausing a sweep's dwell, wrong backlash sign for reversed servos, `OFF 99` releasing channel 15, old pulses resuming during a PCA9685 reconfigure, silent truncation of program lines, a stack-table overflow, exportable MIN ≥ MAX | Compiled for the Mega 2560; all tests pass on the PC. **NOT YET RUN ON HARDWARE** in this form (the v2.3 tester behaviour and the v0.4 J/M checks it builds on were run on Sir Pinch-a-lot) |

## Lineage (before this repository)

| Date | Version | Note |
| --- | --- | --- |
| 2026-10-11 | `ik_demo` v0.4 | Robot Demo 3 sketch for Sir Pinch-a-lot, touch-test calibration (base −2.9°, wrist +3.3°) |
| 2026-09-20 | `servo_tester` v2.3 | Register-level PCA9685, adoption on boot, verified on the bench servo |

## Fixes found while generalising

- A channel written at exactly 500 µs read back as 499 µs and was not adopted after a reset (v2.3,
  v0.4). The PCA9685 driver now allows one count of tolerance.
- v0.4 could in principle place with a different approach angle or roll than it picked with, tilting the
  block; v1.0 computes the block's orientation and refuses tilted placements.
- v0.4 checked the carry path only against the tower; v1.0 checks every declared stack.
