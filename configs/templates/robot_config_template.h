// robot_config.h - TEMPLATE for a new ROT3U-type robot (format: docs/design/config-format.md)
// 1. Measure the geometry (docs/robots/rot3u.md) and fill in the ARM_* lines.
// 2. Map every joint with servo_tester (docs/guides/joint-mapping.md) and paste its EXPORT block
//    over the JOINTS[] and GRIP lines below, rows in kinematic order.
// 3. Only then set CAL_VALID to true.
// Copyright (c) 2026 ticohannan. BSD-3-Clause, see LICENSE.
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
#include "RobotTypes.h"

#define ROBOT_NAME "new ROT3U arm (template)"
#define CAL_VALID  false           // false: solve and print only, the arm never moves
#define KIN_MODEL  KIN_ROT3U       // kinematic model: docs/robots/rot3u.md

// ---- servo driver (docs/design/servo-driver.md)
// #define SERVO_DRIVER_PINS       // servos on Arduino pins instead of a PCA9685
#define PCA_ADDR   0x40
#define SERVO_PIN_LIST {2, 3, 4, 5, 6, 7}   // used only with SERVO_DRIVER_PINS, one pin per channel 0..5

// ---- joints, in KINEMATIC order (base, shoulder, elbow, wrist pitch, wrist roll)
// PLACEHOLDERS (nominal MG996R slope, narrow limits) - replace with the servo_tester EXPORT block.
const JointCal JOINTS[] = {
  //  ch  type     US0      USPD    MIN   MAX   LASH  LOAD  name
  {  0, J_ROT, 1500.0,   11.000, 1300, 1700,  0.00, 0, "base"     },
  {  1, J_ROT,  510.0,   11.000, 1300, 1700,  0.00, 1, "shoulder" },
  {  2, J_ROT,  500.0,  -11.000, 1300, 1700,  0.00, 1, "elbow"    },
  {  3, J_ROT,  500.0,  -11.000, 1300, 1700,  0.00, 1, "wrist"    },
  {  4, J_ROT,  500.0,   11.000, 1300, 1700,  0.00, 0, "roll"     },
};
const GripCal GRIP = { 5, 1300, 1500, 1300, 1700 };

// ---- geometry of the ROT3U model, mm - MEASURE (these are Sir Pinch-a-lot's)
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
