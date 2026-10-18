// robot_config.h - ONE PHYSICAL ROBOT (format: docs/design/config-format.md, CFG_FORMAT 1)
// Sir Pinch-a-lot: ROT3U 6-DOF arm A, Build Revision B (aluminium horns), MG996R servos.
// Saved copy: configs/rot3u_sir_pinch_a_lot_revB/robot_config.h
// Copyright (c) 2026 ticohannan. BSD-3-Clause, see LICENSE.
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
#include "RobotTypes.h"

#define ROBOT_NAME "Sir Pinch-a-lot (ROT3U arm A, Rev B)"
#define CAL_VALID  true            // false: solve and print only, the arm never moves
#define KIN_MODEL  KIN_ROT3U       // kinematic model: docs/robots/rot3u.md

// ---- servo driver (docs/design/servo-driver.md)
// #define SERVO_DRIVER_PINS       // servos on Arduino pins instead of a PCA9685
#define PCA_ADDR   0x40
#define SERVO_PIN_LIST {2, 3, 4, 5, 6, 7}   // used only with SERVO_DRIVER_PINS, one pin per channel 0..5

// ---- joints, in KINEMATIC order (base, shoulder, elbow, wrist pitch, wrist roll)
// Angle conventions: docs/robots/rot3u.md. Calibration 2026-10-11: base circle fit (3 grid points),
// shoulder 3 points, elbow 2 points, wrist 3 points after the 90 deg bracket move, roll 2 points;
// touch-test refit (6 points): base -2.9 deg, wrist +3.3 deg. Residual 1.7 mm rms, worst 3.1 mm.
// LASH not measured yet (0 = compensation has no effect). Measure with servo_tester WIG.
const JointCal JOINTS[] = {
  //  ch  type     US0      USPD    MIN   MAX   LASH  LOAD  name
  {  0, J_ROT, 1475.6,   10.901,  530, 2470,  0.00, 0, "base"     },
  {  1, J_ROT,  408.8,   10.625,  530, 2470,  0.00, 1, "shoulder" },
  {  2, J_ROT,  332.1,  -11.587,  530, 2020,  0.00, 1, "elbow"    },
  {  3, J_ROT,  519.5,  -11.111,  530, 1470,  0.00, 1, "wrist"    },  // brackets collide above 1500
  {  4, J_ROT,  500.0,   11.111,  500, 2500,  0.00, 0, "roll"     },  // 500/2500 = jaws sideways
};
const GripCal GRIP = { 5, 1150, 1580, 1124, 1920 };   // holds a 25 mm cube at 1570, releases at 1500

// ---- geometry of the ROT3U model, mm (measured by hand 2026-10-10, +/- 1 mm)
const float ARM_D1  = 113.0;       // base plate to shoulder axis
const float ARM_R2  = 104.0;       // shoulder axis to elbow axis
const float ARM_R3  = 97.0;        // elbow axis to wrist-pitch axis
const float ARM_TIP = 174.0;       // wrist-pitch axis to the jaw tips, along the tool
const float JAW_HALF_MM = 0.0;     // half the jaw height across the tool axis (not measured: 0)

// ---- via pose: shoulder, elbow, wrist angles (deg) of a tall, safe pose. A joint move that would hit
// the paper or a stack goes through it instead (base and roll turn while the arm is up there).
const float VIA_DEG[3] = {90.0, -30.0, -60.0};

// ---- mass model for the gravity side of the backlash (grams, rough is fine)
const float MASS_UPPER_G = 30.0;   // upper-arm link (bracket), at its midpoint
const float MASS_ELBOW_G = 60.0;   // elbow servo
const float MASS_FORE_G  = 30.0;   // forearm link, at its midpoint
const float MASS_WRIST_G = 120.0;  // wrist-pitch and roll servos
const float MASS_TOOL_G  = 80.0;   // gripper with its servo
const float TOOL_COM_MM  = 60.0;   // gripper centre of mass, from the wrist-pitch axis
const float BAL_BAND_GMM = 500.0;  // |gravity torque| below this (g*mm) = balanced: side from motion

#define LASH_COMP_DEFAULT false    // start with backlash compensation off (LASH ON to enable)

#endif
