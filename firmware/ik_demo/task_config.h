// task_config.h - ONE ASSIGNMENT / WORKSPACE (format: docs/design/config-format.md section 6)
// Robot Demo 3 (ECE 6311): pick blocks typed as [x, y, z] on 1 cm graph paper, place at [5, 5, 0].
// Saved copy: configs/tasks/demo3_task_config.h
// Copyright (c) 2026 ticohannan. BSD-3-Clause, see LICENSE.
#ifndef TASK_CONFIG_H
#define TASK_CONFIG_H

// ---- paper frame (FRAME bx by rot z0 changes these at run time)
const float FRAME_BX_CM   = -5.2;  // base axis on the paper, cm (servo_tester PTXY circle fit)
const float FRAME_BY_CM   = -1.1;
const float FRAME_ROT_DEG = 0.0;   // direction of the base frame x axis on the paper, CCW from paper +x
const float FRAME_Z0_MM   = 0.0;   // base-frame origin height above the paper

// ---- blocks
const float BLOCK_X_MM = 25.0, BLOCK_Y_MM = 25.0, BLOCK_Z_MM = 25.0;
const float BLOCK_MASS_G = 10.0;
const float GRIP_DEPTH_MM = 4.0;   // jaw tips to the middle of the gripping area (rubber pads)

// ---- destination of GO
const float DEST_X_CM = 5.0, DEST_Y_CM = 5.0;

// ---- safety distances, mm
const float KEEPOUT_R_MM   = 115.0;  // grasp point stays this far from the base axis
const float CLEAR_MM       = 15.0;   // clearance above blocks while moving
const float MIN_TIP_Z_MM   = 3.0;    // jaw tips stay this high above the paper
const float MIN_ELBOW_Z_MM = 20.0;   // elbow axis stays this high above the paper
const float READY_R_MM     = 120.0;  // ready pose: grasp point this far out, toward DEST,
const float READY_Z_MM     = 95.0;   //   at this height (clears a 3-block tower)

// ---- approach angles to try, deg in the arm plane (-90 = straight down), best first
const float APPROACH_LIST[] = {-90, -85, -80, -75, -70, -65, -60, -55};

// ---- a block counts as level (upright, upside down or on its side) within this tilt, deg
const float LEVEL_TOL_DEG = 3.0;

// ---- policies
#define BRANCH_POLICY BR_ANY       // BR_ANY | BR_FRONT | BR_BACK
#define FLIP_POLICY   FLIP_NO      // FLIP_NO | FLIP_OK | FLIP_PREFER

// ---- stacks on the paper at start: {x cm, y cm, blocks}; rows with 0 blocks are ignored.
// DEST is added automatically. Declaring the blocks lets the path check steer around them.
const float INIT_WORLD[][3] = { {0, 0, 0} };

// ---- program loaded at boot (RUN plays it). One command per line, <= 48 characters.
// The Demo 3 video script: 5 single blocks at (10,3) (12,-8) (14,6) (-2,12) (-2,-13),
// each turned so one face points at the base axis (docs/guides/ik_demo-guide.md).
const char START_PROGRAM[] PROGMEM =
  "ECHO Robot Demo 3: 5 picks, 3-block tower\n"
  "HOME\n"
  "P 10 3 0\n"
  "T 1\n"
  "T 2\n"
  "S 2\n"
  "GO\n"
  "P 12 -8 0\n"
  "GO\n"
  "P 14 6 0\n"
  "GO\n"
  "WAIT lift the tower off (5,5), then Enter\n"
  "STACK 0\n"
  "P -2 12 0\n"
  "GO\n"
  "P -2 -13 0\n"
  "GO\n";

#endif
