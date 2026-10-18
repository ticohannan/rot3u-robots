// RobotTypes.h - shared configuration types (DD-CFG, docs/design/config-format.md)
// Canonical copy: firmware/common/. Sketch folders hold copies made by tools/sync_common.py.
// Copyright (c) 2026 ticohannan. BSD-3-Clause, see LICENSE.
#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H
#include <stdint.h>

#define CFG_FORMAT 1                      // bump when these structs change incompatibly

enum JointType : uint8_t { J_ROT = 0, J_LIN = 1, J_GRIP = 2 };

// value = (us - us0) / uspd ;  actual = value + (lash / 2) * side
struct JointCal {
  uint8_t     ch;      // driver channel
  uint8_t     type;    // J_ROT (deg) or J_LIN (mm)
  float       us0;     // pulse at value 0, us
  float       uspd;    // us per deg (or per mm); the sign carries the direction
  uint16_t    minUs;   // safe pulse limits (also the IK joint limits)
  uint16_t    maxUs;
  float       lash;    // full backlash, deg or mm (0 = not measured)
  uint8_t     load;    // 1 = gravity-loaded: side from the gravity torque
  const char *name;
};

struct GripCal {
  uint8_t  ch;
  uint16_t openUs;     // jaws clear of a block
  uint16_t holdUs;     // jaws gripping a block
  uint16_t minUs;
  uint16_t maxUs;
};

#endif
