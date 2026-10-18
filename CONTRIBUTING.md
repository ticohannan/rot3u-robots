# Contributing and the change process

**Version:** 1.0
**Updated:** 2026-10-17

This repository holds firmware that moves real arms. The rules below keep the code, the documents
and the tests telling the same story.

## 1. Every change goes: design document → code → tests

1. **Design first.** Write the change into the relevant design document in `docs/design/` before
   touching code: the requirement (with an ID such as `IK-G7`), the command or format, the behaviour at
   its edges. Bump the document's version.
2. **Implement** exactly what the document now says. If implementing shows the design was wrong, fix
   the document first, then the code.
3. **Test.** Add or change a test that checks the requirement, and list it in the document's *Tests*
   table. A requirement with no test is not done.
4. **Run the tests**: `python tests/run_tests.py`. They also run on GitHub for every push.
5. **Record it**: bump `FW_VERSION` in the sketch (patch = fix, minor = new command or behaviour) and
   add a row to `CHANGELOG.md`.

| Document | Covers |
| --- | --- |
| `docs/design/servo-driver.md` | The servo driver interface and each driver |
| `docs/design/config-format.md` | `RobotTypes.h`, the export block, `robot_config.h`, `task_config.h` |
| `docs/design/servo_tester.md` | `servo_tester` |
| `docs/design/ik_demo.md` | `ik_demo` |
| `docs/robots/<robot>.md` | One robot family's model and angle conventions |

## 2. Safety rules (never relaxed without a design change and a test)

- No output pulses before it is explicitly started; adopt live outputs after a reset instead of
  resetting them.
- Every channel has soft limits, applied as the last step before the driver.
- Moves are slew-rate limited; any key stops them.
- No automatic release of a gravity-loaded joint by default.
- Default soft limits (1300–1700 µs), absolute bounds (500–2500 µs) and default slew rates do not change
  without a design document change saying why.

## 3. Shared code

`firmware/common/` holds the canonical shared headers. The Arduino IDE only compiles files inside a
sketch's own folder, so each sketch folder has copies. Edit only `firmware/common/`, then run
`python tools/sync_common.py`. The tests fail if a copy differs.

## 4. Status words

Say what has actually been run: *compiled*, *tested on the PC*, *run on <arm> on <date>*, or *NOT YET
RUN ON HARDWARE*. Put it in the sketch's header comment and in the changelog row.

## 5. Measurements

Every number recorded about a robot carries its method and uncertainty (for example "ruler, ±1 mm").
Calibration blocks keep the comment lines `servo_tester` writes.
