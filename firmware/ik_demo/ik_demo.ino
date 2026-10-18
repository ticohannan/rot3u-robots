/*
  ik_demo.ino  v1.0 - inverse-kinematics pick and place for ROT3U-type arms
  Design: docs/design/ik_demo.md (DD-IK).  Guide: docs/guides/ik_demo-guide.md
  Copyright (c) 2026 ticohannan. BSD-3-Clause, see LICENSE.

  The robot is described in robot_config.h (from servo_tester EXPORT) and the assignment in
  task_config.h. Servo output goes through the RobotServoDriver interface (DD-DRV).

  STATUS: v1.0 compiled for the ATmega2560 and tested on a PC against an independent DH model
  (tests/test_ik_demo.py), including the Robot Demo 3 script. v0.4, which it generalises, ran its
  J and M checks on Sir Pinch-a-lot; v1.0 is NOT YET RUN ON HARDWARE.
  While CAL_VALID is false the sketch solves and prints but never moves the arm.

  Serial Monitor: 115200 baud, line ending Newline (None also works). HELP lists the commands.
  Paper coordinates are cm. For P, z is the height of the BOTTOM of the top block (on the paper: 0),
  and (x, y) is the centre of the block's footprint.
  Atan2 note: C atan2(y, x) is class Atan2(x, y). Every call below is commented.
*/

#define FW_VERSION "1.0"

#include <Wire.h>
#include <math.h>
#include "RobotTypes.h"
#include "robot_config.h"

#define KIN_ROT3U 1
enum { BR_ANY = 0, BR_FRONT = 1, BR_BACK = 2 };
enum { FLIP_NO = 0, FLIP_OK = 1, FLIP_PREFER = 2 };
#include "task_config.h"

#if CFG_FORMAT != 1
  #error "RobotTypes.h CFG_FORMAT does not match this sketch (expects 1)"
#endif
#if KIN_MODEL != KIN_ROT3U
  #error "only the ROT3U kinematic model is implemented (docs/design/ik_demo.md section 2)"
#endif

#include "RobotServoDriver.h"
#if defined(SERVO_DRIVER_PINS)
  #include "PinServoDriver.h"
  const uint8_t PIN_LIST[] = SERVO_PIN_LIST;
  PinServoDriver drv(PIN_LIST, sizeof(PIN_LIST));
#else
  #include "PCA9685Driver.h"
  PCA9685Driver drv(PCA_ADDR, 16);
#endif
RobotServoDriver &servo = drv;

const uint8_t NJ = 5;                      // arm joints (the ROT3U model)
static_assert(sizeof(JOINTS) / sizeof(JOINTS[0]) == NJ, "ROT3U needs 5 JOINTS rows in kinematic order");
const uint8_t NS = NJ + 1;                 // output slots: joints 0..4, gripper 5
const uint8_t GS = NJ;                     // gripper slot

// ---- motion constants (unchanged from v0.4)
const uint16_t STEP_MS   = 20;
const float    LIN_STEP  = 1.5;            // mm per update on straight up/down moves (40 ms each)
const uint16_t JUMP_US   = 80;
const uint16_t LIN_JMAX  = 12;
const float    MIN_BEND  = 10.0;
const float    LIFT_BEND = 20.0;
const uint8_t  N_APPROACH = sizeof(APPROACH_LIST) / sizeof(APPROACH_LIST[0]);

enum { S_OK = 0, W_REACH, W_LIMIT, W_ROLL, W_KEEPOUT, W_LOW, W_FK, W_HOVER, W_PLACE, W_BRANCH };

struct Sol {
  float    th[NJ];                         // joint angles theta1*..theta5*, deg
  uint16_t us[NJ];                         // centre-line pulses (no backlash correction)
  uint8_t  why, bad;
};

// ---- runtime state
uint16_t cur[NS];
bool     active[NS];
uint16_t sim[NS];                          // simulated pulses while DRY is on
int8_t   lastSide[NJ];
bool     dryRun = true, lashOn = LASH_COMP_DEFAULT, cmdFail = false, calBad = false;
float    airMm = 0.0;
uint8_t  rateUs = 5;

float    frBx = FRAME_BX_CM, frBy = FRAME_BY_CM, frRot = FRAME_ROT_DEG, frZ0 = FRAME_Z0_MM;
float    blk[3] = {BLOCK_X_MM, BLOCK_Y_MM, BLOCK_Z_MM};
float    gripDepth = GRIP_DEPTH_MM;
float    destX = DEST_X_CM, destY = DEST_Y_CM;
uint8_t  branchPolicy = BRANCH_POLICY, flipPolicy = FLIP_POLICY;
float    approachDeg = -90.0;
bool     approachAuto = true;

// ---- world model (IK-G5)
const uint8_t MAX_STACKS = 12;
struct Stack { float x, y, top, topH; uint8_t n; };
Stack    stk[MAX_STACKS];
uint8_t  nStk = 0;

// ---- current target (P) and its solutions
Sol      sols[8];
uint8_t  solWhy[8];
bool     haveTgt = false;
float    tgtX = 0, tgtY = 0, tgtZb = 0;    // paper cm, cm, block bottom mm
float    tgtPsi = -90;
int8_t   sel = -1;

// ---- programs (IK-G6)
const uint8_t PROG_LINES = 24, PROG_LEN = 49;   // 48 characters per line
char     prog[PROG_LINES][PROG_LEN];
uint8_t  progN = 0;
bool     recording = false, running = false;

char     buf[64];
uint8_t  blen = 0;
uint32_t lastByte = 0;

// ================================================================== basics
float wrap180(float a) {
  if (!isfinite(a)) return NAN;            // IK-S11: never loop on inf
  while (a > 180.0f)   a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}
float sgnf(float v) { return v > 0 ? 1.0f : (v < 0 ? -1.0f : 0.0f); }

uint8_t slotCh(uint8_t s) { return s < NJ ? JOINTS[s].ch : GRIP.ch; }
uint16_t slotMin(uint8_t s) { return s < NJ ? JOINTS[s].minUs : GRIP.minUs; }
uint16_t slotMax(uint8_t s) { return s < NJ ? JOINTS[s].maxUs : GRIP.maxUs; }
uint16_t clampSlot(uint8_t s, long us) {
  if (us < slotMin(s)) us = slotMin(s);
  if (us > slotMax(s)) us = slotMax(s);
  return (uint16_t)us;
}
void applyUs(uint8_t s, uint16_t us) {   // IK-S2: last line of defence
  us = clampSlot(s, us);
  servo.write(slotCh(s), us); cur[s] = us; active[s] = true;
}
void releaseSlot(uint8_t s) { servo.release(slotCh(s)); active[s] = false; }
void releaseAll() {
  for (uint8_t s = 0; s < NS; s++) releaseSlot(s);
  Serial.println(F("# all channels released"));
}
void emitPos(uint8_t s, const char *tag) {
  Serial.print(F("POS,")); Serial.print(millis()); Serial.print(F(",A,"));
  Serial.print(slotCh(s)); Serial.print(','); Serial.print(cur[s]); Serial.print(','); Serial.println(tag);
}

// ============================================================ calibration
float halfLash(uint8_t j) { return lashOn ? 0.5f * JOINTS[j].lash : 0.0f; }
float usToDeg(uint8_t j, float us) { return (us - JOINTS[j].us0) / JOINTS[j].uspd; }
uint16_t degToUs(uint8_t j, float th, int8_t side) {
  float u = JOINTS[j].us0 + JOINTS[j].uspd * (th - halfLash(j) * side);
  return (uint16_t)(u + 0.5f);
}
float limLo(uint8_t j) { float a = usToDeg(j, JOINTS[j].minUs), b = usToDeg(j, JOINTS[j].maxUs); return a < b ? a : b; }
float limHi(uint8_t j) { float a = usToDeg(j, JOINTS[j].minUs), b = usToDeg(j, JOINTS[j].maxUs); return a > b ? a : b; }
// Best estimate of the actual joint angle at the current pulse (centre line + the side it sits on).
float curDeg(uint8_t j, const uint16_t p[]) { return usToDeg(j, p[j]) + halfLash(j) * lastSide[j]; }

// The 360-degree equivalent of th nearest the middle of joint j's range; false if outside the range.
bool fitJoint(uint8_t j, float &th, uint16_t &us) {
  if (!isfinite(th)) return false;         // IK-S11
  float lo = limLo(j), hi = limHi(j), mid = 0.5f * (lo + hi);
  th = mid + wrap180(th - mid);
  if (th < lo - 0.01f || th > hi + 0.01f) return false;
  us = degToUs(j, th, 0);
  return true;
}

// ================================================================== frames
// Paper (cm, cm, mm above the paper) -> base frame (mm). IK-G1.
void toBase(float X, float Y, float zPaper, float &x, float &y, float &z) {
  float dx = (X - frBx) * 10.0f, dy = (Y - frBy) * 10.0f, c = cosf(frRot * DEG_TO_RAD), s = sinf(frRot * DEG_TO_RAD);
  x = c * dx + s * dy;
  y = -s * dx + c * dy;
  z = zPaper - frZ0;
}
void toPaper(float x, float y, float z, float &X, float &Y, float &zPaper) {
  float c = cosf(frRot * DEG_TO_RAD), s = sinf(frRot * DEG_TO_RAD);
  X = (c * x - s * y) / 10.0f + frBx;
  Y = (s * x + c * y) / 10.0f + frBy;
  zPaper = z + frZ0;
}

// ============================================================ kinematics
// ROT3U model (docs/robots/rot3u.md). tool = wrist axis to grasp point, mm.
float toolLen() { return ARM_TIP - gripDepth; }

struct Pose2 { float u[5], v[5]; };        // arm plane: shoulder, elbow, wrist, grasp, tip (u along x1, v up)
void kinPlane(const float th[NJ], Pose2 &p) {
  float t2 = th[1] * DEG_TO_RAD, t23 = t2 + th[2] * DEG_TO_RAD, ph = t23 + th[3] * DEG_TO_RAD;
  p.u[0] = 0;                              p.v[0] = ARM_D1;
  p.u[1] = ARM_R2 * cosf(t2);              p.v[1] = ARM_D1 + ARM_R2 * sinf(t2);
  p.u[2] = p.u[1] + ARM_R3 * cosf(t23);    p.v[2] = p.v[1] + ARM_R3 * sinf(t23);
  p.u[3] = p.u[2] + toolLen() * cosf(ph);  p.v[3] = p.v[2] + toolLen() * sinf(ph);
  p.u[4] = p.u[2] + ARM_TIP * cosf(ph);    p.v[4] = p.v[2] + ARM_TIP * sinf(ph);
}
// Grasp point (base frame, mm), tip height, elbow height, and the lowest jaw point.
void kinFK(const float th[NJ], float &x, float &y, float &z, float &tipZ, float &elbowZ, float &lowZ) {
  Pose2 p; kinPlane(th, p);
  float t1 = th[0] * DEG_TO_RAD;
  x = cosf(t1) * p.u[3]; y = sinf(t1) * p.u[3]; z = p.v[3];
  tipZ = p.v[4]; elbowZ = p.v[1];
  float ph = (th[1] + th[2] + th[3]) * DEG_TO_RAD;
  lowZ = tipZ - JAW_HALF_MM * fabs(cosf(ph));
}
// Tool rotation matrix (columns x, y, z of frame 5 in the base frame) from the DH chain.
void kinToolR(const float th[NJ], float R[3][3]) {
  const float al[5] = {HALF_PI, 0, 0, HALF_PI, 0};
  float M[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  for (uint8_t i = 0; i < 5; i++) {
    float t = th[i] * DEG_TO_RAD + (i == 3 ? HALF_PI : 0.0f);
    float ct = cosf(t), st = sinf(t), ca = cosf(al[i]), sa = sinf(al[i]);
    float A[3][3] = {{ct, -st * ca, st * sa}, {st, ct * ca, -ct * sa}, {0, sa, ca}}, N[3][3];
    for (uint8_t r = 0; r < 3; r++)
      for (uint8_t c = 0; c < 3; c++) N[r][c] = M[r][0] * A[0][c] + M[r][1] * A[1][c] + M[r][2] * A[2][c];
    memcpy(M, N, sizeof(M));
  }
  memcpy(R, M, sizeof(M));
}
// Gravity side of the pitch joints (IK-L2). side[j] = +1/-1, or 0 inside the balance band / unloaded.
void kinGravitySide(const float th[NJ], bool carrying, int8_t side[NJ]) {
  Pose2 p; kinPlane(th, p);
  float ph = (th[1] + th[2] + th[3]) * DEG_TO_RAD;
  float mu[6] = {0.5f * p.u[1], p.u[1], 0.5f * (p.u[1] + p.u[2]), p.u[2], p.u[2] + TOOL_COM_MM * cosf(ph), p.u[3]};
  float m[6]  = {MASS_UPPER_G, MASS_ELBOW_G, MASS_FORE_G, MASS_WRIST_G, MASS_TOOL_G, carrying ? BLOCK_MASS_G : 0.0f};
  uint8_t first[NJ] = {0, 0, 2, 4, 0};     // first outboard mass index for joints 1, 2, 3
  for (uint8_t j = 0; j < NJ; j++) {
    side[j] = 0;
    if (!JOINTS[j].load || j == 0 || j == 4) continue;
    float uj = p.u[j - 1], tau = 0;        // joint j axis: shoulder (j=1), elbow (2), wrist (3)
    for (uint8_t k = first[j]; k < 6; k++) tau -= m[k] * (mu[k] - uj);
    if (fabs(tau) > BAL_BAND_GMM) side[j] = tau > 0 ? 1 : -1;
  }
}

bool branchAllowed(uint8_t b) {
  uint8_t k = b & 3;
  if (branchPolicy == BR_FRONT) return k < 2;
  if (branchPolicy == BR_BACK) return k >= 2;
  return true;
}

// One IK solution. x, y, z = grasp point, base frame, mm. psi = approach angle in the arm plane.
// b = k + 4*r.  k: 0 front/elbow-up, 1 front/elbow-down, 2 back/elbow-up, 3 back/elbow-down; r: roll A/B.
uint8_t kinSolve(float x, float y, float z, float psi, uint8_t b, Sol &s) {
  uint8_t k = b & 3, r = b >> 2;
  s.bad = 255;
  for (uint8_t j = 0; j < NJ; j++) { s.th[j] = NAN; s.us[j] = 0; }
  if (!isfinite(x) || !isfinite(y) || !isfinite(z) || !isfinite(psi)) return s.why = W_REACH;   // IK-S11
  float rxy = sqrtf(x * x + y * y), d = toolLen();
  if (rxy < KEEPOUT_R_MM) return s.why = W_KEEPOUT;
  float side = (k < 2) ? 1.0f : -1.0f;
  bool  up   = (k % 2) == 0;
  float ps = psi * DEG_TO_RAD;
  float u = side * (rxy - d * cosf(ps));   // wrist centre along x1
  float v = z - d * sinf(ps) - ARM_D1;
  float D = (u * u + v * v - ARM_R2 * ARM_R2 - ARM_R3 * ARM_R3) / (2.0f * ARM_R2 * ARM_R3);
  if (D > cosf(MIN_BEND * DEG_TO_RAD) || D < -1.0f) return s.why = W_REACH;
  float sg = (up ? -1.0f : 1.0f) * side;
  float t1 = atan2f(side * y, side * x);                 // class Atan2(x = side*x, y = side*y)
  float t3 = atan2f(sg * sqrtf(1.0f - D * D), D);        // class Atan2(x = D, y = +/-sqrt(1-D^2))
  float t2 = atan2f(v, u)                                // class Atan2(x = u, y = v)
           - atan2f(ARM_R3 * sinf(t3), ARM_R2 + ARM_R3 * cosf(t3));  // minus Atan2(x = r2 + r3 c3, y = r3 s3)
  float phi = atan2f(sinf(ps), side * cosf(ps));         // class Atan2(x = side cos psi, y = sin psi)
  float t4 = phi - t2 - t3;
  s.th[0] = wrap180(t1 * RAD_TO_DEG);
  s.th[1] = wrap180(t2 * RAD_TO_DEG);
  s.th[2] = wrap180(t3 * RAD_TO_DEG);
  s.th[3] = wrap180(t4 * RAD_TO_DEG);
  for (uint8_t j = 0; j < 4; j++)
    if (!fitJoint(j, s.th[j], s.us[j])) { s.bad = j; return s.why = W_LIMIT; }
  // wrist roll: jaws close across the arm plane, theta5 = n*180; roll A nearest 0, roll B the next
  float lo = limLo(4), hi = limHi(4), best = 0, second = 0;
  uint8_t found = 0;
  for (int8_t n = -4; n <= 4; n++) {
    float c = 180.0f * n;
    if (c < lo - 0.01f || c > hi + 0.01f) continue;
    if (found == 0 || fabs(c) < fabs(best)) { second = best; best = c; }
    else if (found == 1 || fabs(c) < fabs(second)) second = c;
    found++;
  }
  if (found <= r) { s.bad = 4; return s.why = W_ROLL; }
  s.th[4] = (r == 1) ? second : best;
  s.us[4] = degToUs(4, s.th[4], 0);
  float fx, fy, fz, tz, ez, lz;
  kinFK(s.th, fx, fy, fz, tz, ez, lz);
  if (ez < MIN_ELBOW_Z_MM - frZ0) return s.why = W_LOW;
  if (lz < MIN_TIP_Z_MM - frZ0 - 0.01f) return s.why = W_LOW;
  float e = sqrtf((fx - x) * (fx - x) + (fy - y) * (fy - y) + (fz - z) * (fz - z));
  if (e > 1.0f) return s.why = W_FK;       // self-check: FK must land on the target
  return s.why = S_OK;
}

uint8_t solvePaper(float X, float Y, float zPaper, float psi, uint8_t b, Sol &s) {
  float x, y, z;
  toBase(X, Y, zPaper, x, y, z);
  return kinSolve(x, y, z, psi, b, s);
}

// ================================================================== world
int8_t findStack(float X, float Y) {
  for (uint8_t i = 0; i < nStk; i++) if (fabs(stk[i].x - X) < 0.5f && fabs(stk[i].y - Y) < 0.5f) return i;
  return -1;
}
int8_t setStack(float X, float Y, uint8_t n) {
  int8_t i = findStack(X, Y);
  if (i < 0) {
    if (nStk >= MAX_STACKS) return -1;
    i = nStk++;
    stk[i].x = X; stk[i].y = Y;
  }
  stk[i].n = n; stk[i].top = n * blk[2]; stk[i].topH = blk[2];
  return i;
}
void removeStack(float X, float Y) {
  int8_t i = findStack(X, Y);
  if (i < 0) return;
  for (uint8_t k = i; k + 1 < nStk; k++) stk[k] = stk[k + 1];
  nStk--;
}
int8_t destStack() { int8_t i = findStack(destX, destY); return i >= 0 ? i : setStack(destX, destY, 0); }
float destTop() { int8_t i = findStack(destX, destY); return i >= 0 ? stk[i].top : 0.0f; }
uint8_t destCount() { int8_t i = destStack(); return i >= 0 ? stk[i].n : 0; }

void printWorld() {
  Serial.print(F("# world: ")); Serial.print(nStk); Serial.println(F(" stack(s)  (x y cm: blocks, top mm)"));
  for (uint8_t i = 0; i < nStk; i++) {
    Serial.print(F("#   [")); Serial.print(stk[i].x, 1); Serial.print(F(", ")); Serial.print(stk[i].y, 1);
    Serial.print(F("]: ")); Serial.print(stk[i].n); Serial.print(F(" block(s), top ")); Serial.print(stk[i].top, 1);
    if (fabs(stk[i].x - destX) < 0.5f && fabs(stk[i].y - destY) < 0.5f) Serial.print(F("   <- DEST"));
    Serial.println();
  }
}

// ============================================================== printing
void printBranch(uint8_t b) {
  switch (b & 3) {
    case 0:  Serial.print(F("front/elbow-up  ")); break;
    case 1:  Serial.print(F("front/elbow-down")); break;
    case 2:  Serial.print(F("back/elbow-up   ")); break;
    default: Serial.print(F("back/elbow-down ")); break;
  }
  Serial.print((b >> 2) ? F(" roll B") : F(" roll A"));
}
// Displayed solution numbers 1..8: 1 = front/up roll A, 2 = front/up roll B, 3 = front/down roll A, ...
uint8_t numToB(uint8_t n) { return ((n - 1) >> 1) + 4 * ((n - 1) & 1); }
uint8_t bToNum(uint8_t b) { return 2 * (b & 3) + (b >> 2) + 1; }

void printWhy(uint8_t why, uint8_t bad) {
  switch (why) {
    case S_OK:      Serial.print(F("OK")); break;
    case W_REACH:   Serial.print(F("rejected: out of reach")); break;
    case W_LIMIT:   Serial.print(F("rejected: joint ")); Serial.print(bad + 1);
                    Serial.print(F(" outside its range (")); Serial.print(limLo(bad), 0);
                    Serial.print(F(" to ")); Serial.print(limHi(bad), 0); Serial.print(F(")")); break;
    case W_ROLL:    Serial.print(F("rejected: wrist roll cannot square the jaws")); break;
    case W_KEEPOUT: Serial.print(F("rejected: inside the base keep-out")); break;
    case W_LOW:     Serial.print(F("rejected: elbow or jaws too low")); break;
    case W_FK:      Serial.print(F("rejected: FK self-check failed")); break;
    case W_HOVER:   Serial.print(F("rejected: hover pose above it unreachable")); break;
    case W_PLACE:   Serial.print(F("rejected: no valid place at DEST from this pick")); break;
    case W_BRANCH:  Serial.print(F("rejected: branch not allowed (BRANCH)")); break;
  }
}
void printAngles(const float th[NJ]) {
  for (uint8_t j = 0; j < NJ; j++) { Serial.print(' '); if (isnan(th[j])) Serial.print('-'); else Serial.print(th[j], 1); }
}

// =============================================================== motion
bool keyStopped = false;
bool keyStop() {
  if (!Serial.available()) return false;
  keyStopped = true;
  delay(5);
  while (Serial.available()) Serial.read();
  blen = 0;
  Serial.println(F("# STOPPED by keypress - holding position"));
  cmdFail = true;
  return true;
}

bool currentPulses(uint16_t p[NS]) {
  for (uint8_t s = 0; s < NS; s++) {
    if (dryRun) { p[s] = sim[s]; continue; }
    if (s < NJ && !active[s]) { Serial.println(F("# arm not holding - use HOME first")); return false; }
    p[s] = cur[s];
  }
  return true;
}

// Pulses for target angles th, arriving from angles thFrom (IK-L2). Updates lastSide when 'commit'.
void targetPulses(const float th[NJ], const float thFrom[NJ], bool carrying, uint16_t out[NJ], bool commit) {
  int8_t g[NJ];
  kinGravitySide(th, carrying, g);
  for (uint8_t j = 0; j < NJ; j++) {
    int8_t side = g[j];
    if (side == 0) {
      float d = th[j] - thFrom[j];
      side = fabs(d) > 0.05f ? (int8_t)(-sgnf(d)) : lastSide[j];
    }
    out[j] = degToUs(j, th[j], side);
    if (commit) lastSide[j] = side;
  }
}

// The listed slots (bit mask) move together and arrive together, cosine speed profile.
bool moveJ(const uint16_t tgt[NS], uint8_t mask) {
  uint16_t t[NS], s0[NS];
  long d[NS], maxd = 0;
  for (uint8_t c = 0; c < NS; c++) {
    d[c] = 0; t[c] = cur[c]; s0[c] = cur[c];
    if (!(mask & (1 << c))) continue;
    if (!active[c] && !dryRun) {
      Serial.print(F("# ch ")); Serial.print(slotCh(c)); Serial.println(F(" is not holding - use HOME first"));
      cmdFail = true;
      return false;
    }
    t[c] = clampSlot(c, tgt[c]);
    d[c] = (long)t[c] - (long)cur[c];
    if (labs(d[c]) > maxd) maxd = labs(d[c]);
  }
  if (dryRun) {
    Serial.print(F("# DRY move ->"));
    for (uint8_t c = 0; c < NS; c++) {
      Serial.print(' ');
      if (mask & (1 << c)) { t[c] = clampSlot(c, tgt[c]); sim[c] = t[c]; Serial.print(t[c]); } else Serial.print('-');
    }
    Serial.println();
    return true;
  }
  if (maxd == 0) return true;
  uint16_t n = (uint16_t)ceil((float)maxd * 1.5708f / (float)rateUs);
  if (n < 1) n = 1;
  for (uint16_t i = 1; i <= n; i++) {
    float f = 0.5f - 0.5f * cosf(PI * (float)i / (float)n);
    for (uint8_t c = 0; c < NS; c++) {
      if (!d[c]) continue;
      uint16_t v = (uint16_t)((long)s0[c] + (long)floor((float)d[c] * f + 0.5f));
      if (v != cur[c]) applyUs(c, v);
    }
    if (keyStop()) return false;
    delay(STEP_MS);
  }
  for (uint8_t c = 0; c < NS; c++) if (d[c] && cur[c] != t[c]) applyUs(c, t[c]);
  return true;
}

bool moveGrip(uint16_t us) {
  uint16_t t[NS];
  t[GS] = us;
  return moveJ(t, 1 << GS);
}

float graspUp();

// Check a joint-space move by sampling it through FK (IK-S3): paper, elbow, and every known stack.
bool anyNonFinite(const float a[NJ], const float b[NJ]) {
  for (uint8_t j = 0; j < NJ; j++) if (!isfinite(a[j]) || !isfinite(b[j])) return true;
  return false;
}

bool pathClear(const float thA[NJ], const float thB[NJ], bool carrying) {
  if (!CAL_VALID) return true;
  if (anyNonFinite(thA, thB)) { Serial.println(F("# path check: a joint angle is not a number - not moving")); return false; }
  float gu = graspUp();
  for (uint8_t i = 1; i <= 24; i++) {      // the start pose is where the arm already is
    float f = (float)i / 24.0f, th[NJ];
    for (uint8_t j = 0; j < NJ; j++) th[j] = thA[j] + (thB[j] - thA[j]) * f;
    float x, y, z, tz, ez, lz, X, Y, zp;
    kinFK(th, x, y, z, tz, ez, lz);
    toPaper(x, y, z, X, Y, zp);
    float low = carrying ? zp - gu : lz + frZ0;
    const __FlashStringHelper *msg = NULL;
    if (ez + frZ0 < MIN_ELBOW_Z_MM)       msg = F("elbow would hit the paper");
    else if (low < MIN_TIP_Z_MM - 0.5f)   msg = F("jaws or block would hit the paper");
    else {
      float rad = 0.6f * (blk[0] + blk[1]);
      for (uint8_t k = 0; k < nStk; k++) {
        if (stk[k].n == 0) continue;
        float dx = (X - stk[k].x) * 10.0f, dy = (Y - stk[k].y) * 10.0f;
        if (sqrtf(dx * dx + dy * dy) < rad && low < stk[k].top + 5.0f) { msg = F("would clip a stack"); break; }
      }
    }
    if (msg) {
      Serial.print(F("# path check: ")); Serial.print(msg);
      Serial.print(F(" at ")); Serial.print(i * 100 / 24); Serial.println(F("% of the move - not moving"));
      return false;
    }
  }
  return true;
}

bool pathClearQuiet(const float thA[NJ], const float thB[NJ], bool carrying);

// One checked joint move between two angle sets (backlash compensation applied on arrival).
bool legJ(const float from[NJ], const float to[NJ], bool carrying) {
  if (anyNonFinite(from, to)) { Serial.println(F("# joint angle is not a number - not moving")); cmdFail = true; return false; }
  uint16_t t[NS];
  targetPulses(to, from, carrying, t, true);
  if (!moveJ(t, 0x1F)) { cmdFail = true; return false; }
  return true;
}

// Joint move to solution s from where the arm is, with the path check and backlash compensation.
// If the direct move fails the path check, go through the via pose (robot_config.h VIA_DEG): lift to it
// with the base and roll unchanged, turn base and roll there, then move to the target. Each leg is checked.
bool goSol(const Sol &s, bool carrying) {
  uint16_t p[NS];
  if (!currentPulses(p)) { cmdFail = true; return false; }
  float from[NJ];
  for (uint8_t j = 0; j < NJ; j++) from[j] = curDeg(j, p);
  if (!CAL_VALID || pathClearQuiet(from, s.th, carrying)) return legJ(from, s.th, carrying);
  float v1[NJ], v2[NJ];
  bool viaOk = true;
  for (uint8_t j = 0; j < NJ; j++) {
    v1[j] = (j >= 1 && j <= 3) ? VIA_DEG[j - 1] : from[j];
    v2[j] = (j >= 1 && j <= 3) ? VIA_DEG[j - 1] : s.th[j];
    uint16_t u;
    float a = v1[j], b = v2[j];
    if (!fitJoint(j, a, u) || !fitJoint(j, b, u)) viaOk = false;
  }
  if (viaOk && pathClearQuiet(from, v1, carrying) && pathClearQuiet(v2, s.th, carrying)) {
    Serial.println(F("# direct move blocked - going through the via pose"));
    return legJ(from, v1, carrying) && legJ(v1, v2, carrying) && legJ(v2, s.th, carrying);
  }
  pathClear(from, s.th, carrying);         // prints why the direct move is refused
  cmdFail = true;
  return false;
}

// Straight vertical move of the grasp point at paper [X, Y], branch b, approach psi, heights in paper mm.
bool moveV(float X, float Y, float z0, float z1, float psi, uint8_t b, bool carrying) {
  float dz = z1 - z0;
  uint16_t n = (uint16_t)ceil(fabs(dz) / LIN_STEP);
  if (n < 1) n = 1;
  Sol s;
  uint16_t p[NS];
  if (!currentPulses(p)) { cmdFail = true; return false; }
  float prev[NJ];
  for (uint8_t j = 0; j < NJ; j++) prev[j] = curDeg(j, p);
  for (uint16_t i = 1; i <= n; i++) {
    float z = z0 + dz * (float)i / (float)n;
    if (solvePaper(X, Y, z, psi, b, s) != S_OK) {
      Serial.print(F("# line move blocked at height ")); Serial.print(z, 1);
      Serial.print(F(" mm: ")); printWhy(s.why, s.bad); Serial.println();
      cmdFail = true;
      return false;
    }
    uint16_t t[NJ];
    targetPulses(s.th, prev, carrying, t, true);
    memcpy(prev, s.th, sizeof(prev));
    if (dryRun) { for (uint8_t c = 0; c < NJ; c++) sim[c] = clampSlot(c, t[c]); continue; }
    long maxd = 0;
    for (uint8_t c = 0; c < NJ; c++) {
      long d = labs((long)t[c] - (long)cur[c]);
      if (d > JUMP_US) {
        Serial.print(F("# jump guard on ch ")); Serial.print(slotCh(c));
        Serial.println(F(" - the arm is not where the line move expects; stopped"));
        cmdFail = true;
        return false;
      }
      if (d > maxd) maxd = d;
    }
    uint8_t nsub = (uint8_t)((maxd + LIN_JMAX - 1) / LIN_JMAX);
    if (nsub < 1) nsub = 1;
    uint16_t a[NJ];
    for (uint8_t c = 0; c < NJ; c++) a[c] = cur[c];
    for (uint8_t m = 1; m <= nsub; m++) {
      for (uint8_t c = 0; c < NJ; c++) {
        uint16_t v = (uint16_t)((long)a[c] + ((long)t[c] - (long)a[c]) * m / nsub);
        if (v != cur[c]) applyUs(c, v);
      }
      if (keyStop()) return false;
      delay(2 * STEP_MS);
    }
  }
  if (dryRun) {
    Serial.print(F("# DRY line at [")); Serial.print(X, 1); Serial.print(','); Serial.print(Y, 1);
    Serial.print(F("] from ")); Serial.print(z0, 1); Serial.print(F(" to ")); Serial.print(z1, 1); Serial.println(F(" mm"));
  }
  return true;
}

// ======================================================= transfer planner
// Grasp height above the bottom of the block: mid-height, raised so the jaw tips stay above the bottom.
float graspUp() {
  float g = 0.5f * blk[2], m = gripDepth + MIN_TIP_Z_MM;
  return g > m ? g : m;
}

struct Plan {
  float psiP, psiQ;                        // pick and place approach angles
  uint8_t bP, bQ;                          // pick and place branches
  float zb, zG, zH, pzb, pG, pH, pUp;      // heights, paper mm
  float zL, zP;                            // lift height above the pick, carry height above the place
  float ez;                                // block height after placing
  uint8_t orient;                          // 0 upright, 1 on its side, 2 upside down (3 tilted: rejected)
  float sx, sy, dx, dy;                    // source and destination, paper cm
};

const __FlashStringHelper *orientName(uint8_t o) {
  return o == 0 ? F("upright") : o == 1 ? F("on its side") : F("upside down");
}

bool orientOk(uint8_t o) { return flipPolicy != FLIP_NO || o == 0; }

// Every point of a vertical line solves (checked every 2.5 mm), so a planned line move cannot stop halfway.
bool lineOk(float X, float Y, float z0, float z1, float psi, uint8_t b) {
  Sol s;
  uint8_t n = (uint8_t)(fabs(z1 - z0) / 2.5f) + 1;
  for (uint8_t i = 0; i <= n; i++)
    if (solvePaper(X, Y, z0 + (z1 - z0) * i / n, psi, b, s) != S_OK) return false;
  return true;
}

// Find the best place for a block picked with (bP, psiP) at pick grasp angles thPick. IK-G4.
bool planPlace(Plan &pl, const float thPick[NJ]) {
  float Rp[3][3], Rq[3][3], Rr[3][3];
  kinToolR(thPick, Rp);
  float gu = graspUp(), dTop = destTop() + airMm;
  float best = 1e9;
  bool  got = false;
  Sol s;
  // candidates in preference order: the pick's own approach and branch first (index 0), then the rest
  for (uint8_t ia = 0; ia <= N_APPROACH; ia++) {
    float psiQ = ia == 0 ? pl.psiP : APPROACH_LIST[ia - 1];
    if (ia > 0 && (!approachAuto || psiQ == pl.psiP)) continue;
    for (uint8_t bi = 0; bi <= 8; bi++) {
      uint8_t b = bi == 0 ? pl.bP : bi - 1;
      if (bi > 0 && b == pl.bP) continue;
      if (!branchAllowed(b)) continue;
      if (solvePaper(pl.dx, pl.dy, dTop + 0.5f * blk[2], psiQ, b, s) != S_OK) continue;
      kinToolR(s.th, Rq);
      for (uint8_t r = 0; r < 3; r++)     // Rr = Rq * Rp^T
        for (uint8_t c = 0; c < 3; c++) Rr[r][c] = Rq[r][0] * Rp[c][0] + Rq[r][1] * Rp[c][1] + Rq[r][2] * Rp[c][2];
      // block's up axis after the transfer: upright, upside down, on its side, or tilted (never accepted)
      float cl = cosf(LEVEL_TOL_DEG * DEG_TO_RAD), sl = sinf(LEVEL_TOL_DEG * DEG_TO_RAD);
      uint8_t o = Rr[2][2] > cl ? 0 : (Rr[2][2] < -cl ? 2 : (fabs(Rr[2][2]) < sl && (fabs(Rr[2][0]) > cl || fabs(Rr[2][1]) > cl) ? 1 : 3));
      if (o == 3 || !orientOk(o)) continue;
      float ez = fabs(Rr[2][0]) * blk[0] + fabs(Rr[2][1]) * blk[1] + fabs(Rr[2][2]) * blk[2];
      float pG = dTop + 0.5f * ez + Rr[2][2] * (gu - 0.5f * blk[2]);
      float pH = pG + CLEAR_MM + 10.0f, pUp = dTop + ez + CLEAR_MM + gripDepth;
      Sol g, h, u;
      if (solvePaper(pl.dx, pl.dy, pG, psiQ, b, g) != S_OK) continue;
      float x, y, z, tz, ez2, lz;
      kinFK(g.th, x, y, z, tz, ez2, lz);
      if (lz + frZ0 < dTop + (dTop > 0.5f ? 1.0f : MIN_TIP_Z_MM) - 0.01f) continue;   // jaws into the stack below
      if (solvePaper(pl.dx, pl.dy, pH, psiQ, b, h) != S_OK) continue;
      if (solvePaper(pl.dx, pl.dy, pUp, psiQ, b, u) != S_OK) continue;
      if (!lineOk(pl.dx, pl.dy, pH, pG, psiQ, b) || !lineOk(pl.dx, pl.dy, pG, pUp, psiQ, b)) continue;
      float score = 0;
      bool flipped = o != 0;
      if (flipPolicy == FLIP_PREFER) score += flipped ? 0 : 1000; else score += flipped ? 1000 : 0;
      score += fabs(psiQ - pl.psiP) + (b != pl.bP ? 1.0f : 0.0f) + ((b & 3) != (pl.bP & 3) ? 2.0f : 0.0f);
      if (score < best) {
        best = score; got = true;
        pl.psiQ = psiQ; pl.bQ = b; pl.pG = pG; pl.pH = pH; pl.pUp = pUp; pl.ez = ez; pl.orient = o;
        pl.pzb = dTop;
        if (score == 0) return true;       // nothing can beat the preferred candidate
      }
    }
  }
  return got;
}

// Carry heights (section 4 step 5). Carry pose above the place at zP = place hover + 0, 10, ... 80 mm;
// lift pose above the pick at the highest zL <= max(zP, pick hover) that is reachable with the elbow
// LIFT_BEND from straight, but not below the pick hover. The joint move between them must be clear.
bool planCarry(Plan &pl, bool checkPath) {
  for (float extra = 0; extra <= 80.0f; extra += 10.0f) {
    Sol a, c;
    float zP = pl.pH + extra;
    if (solvePaper(pl.dx, pl.dy, zP, pl.psiQ, pl.bQ, c) != S_OK) continue;
    float zL = zP > pl.zH ? zP : pl.zH;
    while (zL > pl.zH && (solvePaper(pl.sx, pl.sy, zL, pl.psiP, pl.bP, a) != S_OK || fabs(a.th[2]) < LIFT_BEND)) zL -= 5.0f;
    if (zL < pl.zH) zL = pl.zH;
    if (solvePaper(pl.sx, pl.sy, zL, pl.psiP, pl.bP, a) != S_OK) continue;
    if (!lineOk(pl.sx, pl.sy, pl.zG, zL, pl.psiP, pl.bP) || !lineOk(pl.dx, pl.dy, zP, pl.pH, pl.psiQ, pl.bQ)) continue;
    if (checkPath && !pathClearQuiet(a.th, c.th, true)) continue;
    pl.zL = zL; pl.zP = zP;
    return true;
  }
  return false;
}

// A complete plan for picking the block at (sx, sy, bottom zb) with branch bP at approach psiP.
uint8_t planTransfer(Plan &pl, float sx, float sy, float zbCm10, float psiP, uint8_t bP, bool checkPath) {
  pl.sx = sx; pl.sy = sy; pl.dx = destX; pl.dy = destY; pl.psiP = psiP; pl.bP = bP;
  pl.zb = zbCm10 + airMm;
  pl.zG = pl.zb + graspUp();
  pl.zH = pl.zb + blk[2] + CLEAR_MM + gripDepth;
  if (!branchAllowed(bP)) return W_BRANCH;
  Sol g, h;
  uint8_t w = solvePaper(sx, sy, pl.zG, psiP, bP, g);
  if (w != S_OK) return w;
  if (solvePaper(sx, sy, pl.zH, psiP, bP, h) != S_OK || !lineOk(sx, sy, pl.zH, pl.zG, psiP, bP)) return W_HOVER;
  if (!planPlace(pl, g.th)) return W_PLACE;
  if (!planCarry(pl, checkPath)) return W_PLACE;
  return S_OK;
}

bool pathClearQuiet(const float thA[NJ], const float thB[NJ], bool carrying) {
  // same test as pathClear, without printing (used while planning)
  if (anyNonFinite(thA, thB)) return false;
  float gu = graspUp(), rad = 0.6f * (blk[0] + blk[1]);
  for (uint8_t i = 1; i <= 24; i++) {
    float f = (float)i / 24.0f, th[NJ];
    for (uint8_t j = 0; j < NJ; j++) th[j] = thA[j] + (thB[j] - thA[j]) * f;
    float x, y, z, tz, ez, lz, X, Y, zp;
    kinFK(th, x, y, z, tz, ez, lz);
    toPaper(x, y, z, X, Y, zp);
    float low = carrying ? zp - gu : lz + frZ0;
    if (ez + frZ0 < MIN_ELBOW_Z_MM || low < MIN_TIP_Z_MM - 0.5f) return false;
    for (uint8_t k = 0; k < nStk; k++) {
      if (stk[k].n == 0) continue;
      float dx = (X - stk[k].x) * 10.0f, dy = (Y - stk[k].y) * 10.0f;
      if (sqrtf(dx * dx + dy * dy) < rad && low < stk[k].top + 5.0f) return false;
    }
  }
  return true;
}

// ============================================================ ready pose
bool readySol(Sol &s) {
  float px, py, pz;
  toBase(destX, destY, 0, px, py, pz);
  float a = atan2f(py, px);                // class Atan2(x = px, y = py): direction of DEST
  float x = READY_R_MM * cosf(a), y = READY_R_MM * sinf(a);
  for (uint8_t i = 0; i <= N_APPROACH; i++) {          // straight down first, then the task's list
    float psi = i == 0 ? -90.0f : APPROACH_LIST[i - 1];
    for (uint8_t b = 0; b < 8; b++)
      if (branchAllowed(b) && kinSolve(x, y, READY_Z_MM - frZ0, psi, b, s) == S_OK) return true;
  }
  return false;
}

bool lineOk(float X, float Y, float z0, float z1, float psi, uint8_t b);

// If the jaws are low, back straight up first, so HOME after a stop does not drag a block.
bool retreatUp() {
  float th[NJ], x, y, z, tz, ez, lz, X, Y, zp;
  for (uint8_t j = 0; j < NJ; j++) th[j] = curDeg(j, cur);
  kinFK(th, x, y, z, tz, ez, lz);
  toPaper(x, y, z, X, Y, zp);
  float topMax = 0;
  for (uint8_t k = 0; k < nStk; k++) if (stk[k].top > topMax) topMax = stk[k].top;
  float zUp = topMax + blk[2] + CLEAR_MM + gripDepth;
  if (zp >= zUp) return true;
  int8_t kb = -1; long best = 40; float psiB = -90;
  for (uint8_t ia = 0; ia < N_APPROACH; ia++) {
    for (uint8_t b = 0; b < 8; b++) {     // which solution is the arm in now?
      Sol s;
      if (solvePaper(X, Y, zp, APPROACH_LIST[ia], b, s) != S_OK) continue;
      long m = 0;
      for (uint8_t c = 0; c < NJ; c++) { long d = labs((long)s.us[c] - (long)cur[c]); if (d > m) m = d; }
      if (m < best) { best = m; kb = b; psiB = APPROACH_LIST[ia]; }
    }
  }
  if (kb < 0) { Serial.println(F("# cannot work out a straight-up retreat from here - moving directly")); return true; }
  // as high as the straight line allows (the joint limits can end it early); the next move is path-checked
  while (zUp > zp + 5.0f && !lineOk(X, Y, zp, zUp, psiB, kb)) zUp -= 5.0f;
  if (zUp <= zp + 5.0f) { Serial.println(F("# no straight-up retreat possible here - the next move is path-checked")); return true; }
  Serial.print(F("# backing straight up first, to ")); Serial.print(zUp, 0); Serial.println(F(" mm"));
  return moveV(X, Y, zp, zUp, psiB, kb, false);
}

void doHome() {
  Sol r;
  if (!readySol(r)) { Serial.println(F("# ready pose is not reachable with this calibration")); cmdFail = true; return; }
  if (dryRun) { Serial.print(F("# DRY ready pose th* =")); printAngles(r.th); Serial.println(); uint16_t t[NS]; memcpy(t, r.us, sizeof(r.us)); moveJ(t, 0x1F); return; }
  bool cold = false;
  for (uint8_t c = 0; c < NS; c++) if (!active[c]) cold = true;
  if (!cold) {
    if (!retreatUp()) return;
    if (goSol(r, false)) Serial.println(F("# at the ready pose"));
    return;
  }
  // IK-S12: first pulses on the idle channels only; channels already holding then move the normal way
  bool anyArmHolding = false;
  for (uint8_t c = 0; c < NJ; c++) if (active[c]) anyArmHolding = true;
  Serial.println(F("# first pulse on idle channels, base outward - arm should be supported near the ready pose"));
  for (uint8_t c = 0; c < NS; c++) {
    if (active[c]) continue;
    applyUs(c, c < NJ ? r.us[c] : GRIP.openUs);
    emitPos(c, "START");
    delay(400);
    if (keyStop()) return;
  }
  for (uint8_t j = 0; j < NJ; j++) lastSide[j] = 0;
  if (anyArmHolding) {
    if (!retreatUp()) return;
    if (goSol(r, false)) Serial.println(F("# at the ready pose"));
    return;
  }
  Serial.println(F("# at the ready pose"));
}

// ============================================================== commands
void printLashSuggestion(const Sol &g, float psi, uint8_t b) {
  bool any = false;
  for (uint8_t j = 0; j < NJ; j++) if (JOINTS[j].lash > 0) any = true;
  if (!any) { Serial.println(F("# backlash: not measured for this robot (LASH = 0) - nothing to compensate")); return; }
  Sol h;
  if (solvePaper(tgtX, tgtY, tgtZb + blk[2] + CLEAR_MM + gripDepth + airMm, psi, b, h) != S_OK) return;
  bool keep = lashOn; lashOn = true;
  uint16_t comp[NJ];
  int8_t saved[NJ]; memcpy(saved, lastSide, sizeof(saved));
  targetPulses(g.th, h.th, false, comp, true);           // descending from the hover pose
  int8_t sd[NJ]; memcpy(sd, lastSide, sizeof(sd)); memcpy(lastSide, saved, sizeof(saved));
  lashOn = keep;
  float act[NJ];
  for (uint8_t j = 0; j < NJ; j++) act[j] = g.th[j] + 0.5f * JOINTS[j].lash * sd[j];
  float x0, y0, z0, x1, y1, z1, a, c, e;
  kinFK(g.th, x0, y0, z0, a, c, e);
  kinFK(act, x1, y1, z1, a, c, e);
  float err = sqrtf((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
  Serial.print(F("# backlash: compensation would change the grasp pulses by"));
  for (uint8_t j = 0; j < NJ; j++) { Serial.print(' '); Serial.print((int)comp[j] - (int)g.us[j]); }
  Serial.print(F(" us; without it the grasp point lands about ")); Serial.print(err, 1);
  Serial.print(F(" mm off.  LASH is ")); Serial.print(lashOn ? F("ON") : F("OFF"));
  Serial.println(F(" (LASH ON | LASH OFF)"));
}

void printSolutions() {
  uint8_t n = 0;
  float x, y, z;
  toBase(tgtX, tgtY, tgtZb + graspUp() + airMm, x, y, z);
  Serial.print(F("# target: top block at paper [")); Serial.print(tgtX, 1); Serial.print(F(", "));
  Serial.print(tgtY, 1); Serial.print(F(", ")); Serial.print(tgtZb / 10.0f, 1);
  Serial.print(F("] cm -> grasp point (")); Serial.print(x, 1); Serial.print(F(", "));
  Serial.print(y, 1); Serial.print(F(", ")); Serial.print(z, 1); Serial.println(F(") mm, base frame"));
  for (uint8_t m = 1; m <= 8; m++) {
    uint8_t b = numToB(m);
    if (b >= 4 && solWhy[b] != S_OK && solWhy[b] == solWhy[b - 4]) continue;   // same reason as roll A
    Serial.print(F("SOL ")); Serial.print(m); Serial.print(F("  ")); printBranch(b);
    if (sols[b].why != W_REACH && sols[b].why != W_KEEPOUT && !isnan(sols[b].th[0])) { Serial.print(F("  th* =")); printAngles(sols[b].th); }
    Serial.print(F("   ")); printWhy(solWhy[b], sols[b].bad); Serial.println();
    if (solWhy[b] == S_OK) n++;
  }
  Serial.print(F(">>> ")); Serial.print(n);
  Serial.print(n == 1 ? F(" solution detected") : F(" solutions detected"));
  if (sel >= 0) { Serial.print(F("   (selected: SOL ")); Serial.print(bToNum(sel)); Serial.print(F(")")); }
  Serial.println();
  if (n) Serial.println(F("# S k to change the selection, T k to preview, GO to pick and place"));
  else cmdFail = true;
}

void doSolve(float X, float Y, float Zcm) {
  tgtX = X; tgtY = Y; tgtZb = Zcm * 10.0f; haveTgt = true; sel = -1;
  if (graspUp() > blk[2] - 3.0f) Serial.println(F("# ! the grip depth puts the grasp at the very top of the block"));
  Plan pl;
  bool got = false;
  for (uint8_t i = 0; i < N_APPROACH && !got; i++) {
    float psi = approachAuto ? APPROACH_LIST[i] : approachDeg;
    if (!approachAuto && i > 0) break;
    for (uint8_t b = 0; b < 8 && !got; b++) if (planTransfer(pl, X, Y, tgtZb, psi, b, true) == S_OK) { got = true; tgtPsi = psi; }
  }
  if (!got) tgtPsi = approachAuto ? APPROACH_LIST[0] : approachDeg;
  Serial.print(F("# approach ")); Serial.print(tgtPsi, 0); Serial.print(F(" deg (")); Serial.print(tgtPsi + 90.0f, 0);
  Serial.println(F(" deg outward from straight down)"));
  for (uint8_t m = 1; m <= 8; m++) {
    uint8_t b = numToB(m);
    solvePaper(X, Y, tgtZb + graspUp() + airMm, tgtPsi, b, sols[b]);
    solWhy[b] = planTransfer(pl, X, Y, tgtZb, tgtPsi, b, true);
    if (solWhy[b] == S_OK && sel < 0) sel = b;
  }
  printSolutions();
  if (sel >= 0) printLashSuggestion(sols[sel], tgtPsi, sel);
}

void doSelect(int k) {
  if (!haveTgt) { Serial.println(F("# P x y z first")); cmdFail = true; return; }
  if (k < 1 || k > 8) { Serial.println(F("# S 1-8")); cmdFail = true; return; }
  uint8_t b = numToB(k);
  if (solWhy[b] != S_OK) { Serial.print(F("# SOL ")); Serial.print(k); Serial.print(F(" is ")); printWhy(solWhy[b], sols[b].bad); Serial.println(); cmdFail = true; return; }
  sel = b;
  Serial.print(F("# selected SOL ")); Serial.print(k); Serial.print(F(" ")); printBranch(sel); Serial.println();
}

void doPreview(int k) {
  if (!haveTgt) { Serial.println(F("# P x y z first")); cmdFail = true; return; }
  if (k < 1 || k > 8 || solWhy[numToB(k)] != S_OK) { Serial.println(F("# that solution is not valid")); cmdFail = true; return; }
  Sol h;
  float zH = tgtZb + blk[2] + CLEAR_MM + gripDepth + airMm;
  if (solvePaper(tgtX, tgtY, zH, tgtPsi, numToB(k), h) != S_OK) {
    Serial.print(F("# hover pose not reachable now: ")); printWhy(h.why, h.bad); Serial.println(F(" - P again"));
    cmdFail = true; return;
  }
  Serial.print(F("# preview SOL ")); Serial.print(k); Serial.print(F(" at the hover pose, th* =")); printAngles(h.th); Serial.println();
  goSol(h, false);
}

void stepMsg(uint8_t i, const __FlashStringHelper *m) {
  Serial.print(F("# step ")); Serial.print(i); Serial.print(F("/9: ")); Serial.println(m);
}

// Execute one transfer from source (sx, sy, bottom zb mm) to DEST with pick branch bP at approach psiP.
bool runTransfer(float sx, float sy, float zb, float psiP, uint8_t bP) {
  Plan pl;
  uint8_t w = planTransfer(pl, sx, sy, zb, psiP, bP, true);
  if (w != S_OK) {
    Serial.print(F("# transfer refused before moving: ")); printWhy(w, 255);
    Serial.println(F(" (or no carry height clears the known stacks)"));
    cmdFail = true;
    return false;
  }
  int8_t srcI = findStack(sx, sy);
  Serial.print(F("# GO: pick with SOL ")); Serial.print(bToNum(bP)); Serial.print(F(" at ")); Serial.print(psiP, 0);
  Serial.print(F(" deg, place at [")); Serial.print(pl.dx, 1); Serial.print(F(", ")); Serial.print(pl.dy, 1);
  Serial.print(F("] on ")); Serial.print(pl.pzb, 1); Serial.print(F(" mm with ")); printBranch(pl.bQ);
  Serial.print(F(" at ")); Serial.print(pl.psiQ, 0); Serial.print(F(" deg, block arrives ")); Serial.print(orientName(pl.orient));
  Serial.print(F(", lift to ")); Serial.print(pl.zL, 0); Serial.print(F(" mm, carry at ")); Serial.print(pl.zP, 0); Serial.println(F(" mm"));
  Sol s;
  stepMsg(1, F("open the gripper"));        if (!moveGrip(GRIP.openUs)) goto fail;
  stepMsg(2, F("hover above the block"));   if (solvePaper(sx, sy, pl.zH, psiP, bP, s) != S_OK || !goSol(s, false)) goto fail;
  stepMsg(3, F("descend"));                 if (!moveV(sx, sy, pl.zH, pl.zG, psiP, bP, false)) goto fail;
  stepMsg(4, F("grip"));                    if (!moveGrip(GRIP.holdUs)) goto fail; if (!dryRun) delay(400);
  if (srcI >= 0 && !dryRun && airMm == 0.0f && stk[srcI].n > 0) {   // the block has left its stack
    stk[srcI].n--; stk[srcI].top = pl.zb; stk[srcI].topH = blk[2];
  }
  stepMsg(5, F("lift"));                    if (!moveV(sx, sy, pl.zG, pl.zL, psiP, bP, true)) goto fail;
  stepMsg(6, F("carry"));                   if (solvePaper(pl.dx, pl.dy, pl.zP, pl.psiQ, pl.bQ, s) != S_OK || !goSol(s, true)) goto fail;
  stepMsg(7, F("lower onto the destination")); if (!moveV(pl.dx, pl.dy, pl.zP, pl.pG, pl.psiQ, pl.bQ, true)) goto fail;
  stepMsg(8, F("release"));                 if (!moveGrip(GRIP.openUs)) goto fail; if (!dryRun) delay(300);
  if (!dryRun && airMm == 0.0f) {          // the block is on the destination now
    int8_t d = destStack();
    if (d >= 0) { stk[d].n++; stk[d].top = pl.pzb + pl.ez; stk[d].topH = pl.ez; }
    else Serial.println(F("# (destination not tracked: stack table full)"));
  }
  stepMsg(9, F("clear and return"));        if (!moveV(pl.dx, pl.dy, pl.pG, pl.pUp, pl.psiQ, pl.bQ, false)) goto fail;
  if (!readySol(s) || !goSol(s, false)) {
    Serial.println(F("# (could not return to the ready pose - the arm waits above the destination; HOME when clear)"));
    if (!keyStopped) cmdFail = false;      // the transfer itself is complete
  }
  Serial.print(F("# done - tower count is now ")); Serial.println(destCount());
  return true;
fail:
  Serial.println(F("# sequence stopped - arm holding. Fix it, then HOME (or G O to drop the block)"));
  cmdFail = true;
  return false;
}

void doGo() {
  if (!haveTgt) { Serial.println(F("# P x y z first")); cmdFail = true; return; }
  if (sel < 0 || solWhy[sel] != S_OK) { Serial.println(F("# no valid solution selected")); cmdFail = true; return; }
  runTransfer(tgtX, tgtY, tgtZb, tgtPsi, sel);
}

// Best pick (approach closest to straight down, any allowed branch) for the top block of stack i.
bool autoTransfer(uint8_t i) {
  float zb = stk[i].top - stk[i].topH;
  Plan pl;
  for (uint8_t ia = 0; ia < N_APPROACH; ia++) {
    float psi = approachAuto ? APPROACH_LIST[ia] : approachDeg;
    if (!approachAuto && ia > 0) break;
    for (uint8_t b = 0; b < 8; b++)
      if (planTransfer(pl, stk[i].x, stk[i].y, zb, psi, b, true) == S_OK) return runTransfer(stk[i].x, stk[i].y, zb, psi, b);
  }
  Serial.print(F("# no complete transfer from [")); Serial.print(stk[i].x, 1); Serial.print(F(", "));
  Serial.print(stk[i].y, 1); Serial.println(F("]"));
  cmdFail = true;
  return false;
}

void doMove(float x1, float y1, float x2, float y2) {
  int8_t i = findStack(x1, y1);
  if (i < 0 || stk[i].n == 0) { Serial.println(F("# no known stack there - declare it with STK x y n")); cmdFail = true; return; }
  float kx = destX, ky = destY;
  destX = x2; destY = y2;
  if (destStack() < 0) { Serial.println(F("# too many stacks (12) - no room for the destination")); cmdFail = true; }
  else autoTransfer((uint8_t)findStack(x1, y1));
  destX = kx; destY = ky;
}

void doTower(float x, float y, uint8_t n) {
  float kx = destX, ky = destY;
  destX = x; destY = y;
  int8_t d = destStack();
  if (d < 0) { Serial.println(F("# too many stacks (12) - no room for the tower")); cmdFail = true; destX = kx; destY = ky; return; }
  while (stk[d].n < n && !cmdFail) {
    int8_t src = -1;
    for (uint8_t i = 0; i < nStk; i++) if (i != (uint8_t)d && stk[i].n > 0) { src = i; break; }
    if (src < 0) { Serial.println(F("# no blocks left in the other stacks")); cmdFail = true; break; }
    if (dryRun || airMm != 0.0f) {
      Serial.println(F("# DRY/AIR: one transfer shown; the world model is not updated, so TOWER stops here"));
      autoTransfer((uint8_t)src);
      break;
    }
    if (!autoTransfer((uint8_t)src)) break;
  }
  destX = kx; destY = ky;
  if (!cmdFail) { Serial.print(F("# tower at [")); Serial.print(x, 1); Serial.print(F(", ")); Serial.print(y, 1); Serial.print(F("] has ")); Serial.print(stk[d].n); Serial.println(F(" blocks")); }
}

void doMoveTip(float X, float Y, float zTipCm) {
  int8_t k = -1;
  Sol s, a;
  float psiK = -90, gx = X, gy = Y, zg = 0;
  for (uint8_t i = 0; i < N_APPROACH && k < 0; i++) {
    float psi = approachAuto ? APPROACH_LIST[i] : approachDeg;
    if (!approachAuto && i > 0) break;
    float ps = psi * DEG_TO_RAD;
    zg = zTipCm * 10.0f - gripDepth * sinf(ps) + airMm;     // grasp point above the tip
    for (uint8_t b = 0; b < 8 && k < 0; b++) {
      if (!branchAllowed(b)) continue;
      float bx, by, bz; toBase(X, Y, 0, bx, by, bz);
      float r = sqrtf(bx * bx + by * by), back = gripDepth * cosf(ps);   // the tool points outward: grasp is inward of the tip
      if (r < 1.0f) continue;
      float gbx = bx * (r - back) / r, gby = by * (r - back) / r, pz;
      toPaper(gbx, gby, 0, gx, gy, pz);
      if (solvePaper(gx, gy, zg, psi, b, s) == S_OK && solvePaper(gx, gy, zg + 20.0f, psi, b, a) == S_OK) { k = b; psiK = psi; }
    }
  }
  if (k < 0) { Serial.print(F("# no solution for M: ")); printWhy(s.why, s.bad); Serial.println(); cmdFail = true; return; }
  Serial.print(F("# M with ")); printBranch(k); Serial.print(F(", approach ")); Serial.print(psiK, 0);
  Serial.print(F(" deg, th* =")); printAngles(s.th); Serial.println();
  if (goSol(a, false) && moveV(gx, gy, zg + 20.0f, zg, psiK, k, false))
    Serial.println(F("# there - read where the jaw tips are on the grid"));
}

void doJoint(float a[NJ]) {
  uint16_t t[NS];
  for (uint8_t j = 0; j < NJ; j++) {
    if (!fitJoint(j, a[j], t[j])) {
      Serial.print(F("# joint ")); Serial.print(j + 1); Serial.print(F(" outside its range ("));
      Serial.print(limLo(j), 1); Serial.print(F(" to ")); Serial.print(limHi(j), 1); Serial.println(F(" deg)"));
      cmdFail = true;
      return;
    }
  }
  float x, y, z, tz, ez, lz, X, Y, zp;
  kinFK(a, x, y, z, tz, ez, lz);
  toPaper(x, y, z, X, Y, zp);
  Serial.print(F("# J -> grasp point paper [")); Serial.print(X, 1); Serial.print(F(", "));
  Serial.print(Y, 1); Serial.print(F("] cm, height ")); Serial.print(zp, 1); Serial.println(F(" mm"));
  Sol s; memcpy(s.th, a, sizeof(s.th)); memcpy(s.us, t, sizeof(s.us));
  goSol(s, false);
}

void doWhere() {
  float th[NJ], x, y, z, tz, ez, lz, X, Y, zp;
  const uint16_t *p = dryRun ? sim : cur;
  for (uint8_t j = 0; j < NJ; j++) th[j] = curDeg(j, p);
  kinFK(th, x, y, z, tz, ez, lz);
  toPaper(x, y, z, X, Y, zp);
  Serial.print(F("# pulses   ")); for (uint8_t c = 0; c < NS; c++) { Serial.print(' '); Serial.print(p[c]); Serial.print(active[c] || dryRun ? "" : "(off)"); }
  Serial.println();
  Serial.print(F("# th* deg ")); printAngles(th); Serial.println();
  Serial.print(F("# grasp point paper [")); Serial.print(X, 1); Serial.print(F(", ")); Serial.print(Y, 1);
  Serial.print(F("] cm, height ")); Serial.print(zp, 1); Serial.print(F(" mm (tips ")); Serial.print(tz + frZ0, 1); Serial.println(F(" mm)"));
  Serial.print(F("# DEST [")); Serial.print(destX, 1); Serial.print(F(", ")); Serial.print(destY, 1);
  Serial.print(F("] tower count ")); Serial.print(destCount());
  Serial.print(F(", AIR ")); Serial.print(airMm, 0); Serial.print(F(" mm, DRY ")); Serial.print(dryRun ? 1 : 0);
  Serial.print(F(", SPD ")); Serial.print(rateUs); Serial.print(F(", LASH ")); Serial.println(lashOn ? F("ON") : F("OFF"));
}

void printFrame() {
  Serial.print(F("# FRAME: base axis at paper [")); Serial.print(frBx, 2); Serial.print(F(", ")); Serial.print(frBy, 2);
  Serial.print(F("] cm, base x axis at ")); Serial.print(frRot, 1); Serial.print(F(" deg on the paper, z0 "));
  Serial.print(frZ0, 1); Serial.println(F(" mm"));
}

void doCal() {
  Serial.print(F("# ")); Serial.print(F(ROBOT_NAME)); Serial.print(F(", CAL_VALID = ")); Serial.println(CAL_VALID ? F("true") : F("false - no motion"));
  for (uint8_t j = 0; j < NJ; j++) {
    Serial.print(F("# joint ")); Serial.print(j + 1); Serial.print(' '); Serial.print(JOINTS[j].name);
    Serial.print(F(" (ch ")); Serial.print(JOINTS[j].ch); Serial.print(F("): us = ")); Serial.print(JOINTS[j].us0, 1);
    Serial.print(F(" + ")); Serial.print(JOINTS[j].uspd, 3); Serial.print(F(" * th   range ")); Serial.print(limLo(j), 1);
    Serial.print(F(" to ")); Serial.print(limHi(j), 1); Serial.print(F(" deg, lash ")); Serial.print(JOINTS[j].lash, 2);
    Serial.println(JOINTS[j].load ? F(" (gravity side)") : F(" (motion side)"));
  }
  Serial.print(F("# gripper ch ")); Serial.print(GRIP.ch); Serial.print(F(" open ")); Serial.print(GRIP.openUs);
  Serial.print(F(", hold ")); Serial.println(GRIP.holdUs);
  printFrame();
  Serial.print(F("# block ")); Serial.print(blk[0], 1); Serial.print('x'); Serial.print(blk[1], 1); Serial.print('x');
  Serial.print(blk[2], 1); Serial.print(F(" mm, grip depth ")); Serial.print(gripDepth, 1); Serial.print(F(" mm, BRANCH "));
  Serial.print(branchPolicy == BR_ANY ? 'A' : branchPolicy == BR_FRONT ? 'F' : 'B'); Serial.print(F(", FLIP "));
  Serial.print(flipPolicy); Serial.print(F(", LASH ")); Serial.print(lashOn ? F("ON") : F("OFF"));
  Serial.print(F(", approach ")); if (approachAuto) Serial.println(F("auto")); else Serial.println(approachDeg, 0);
  Sol r;
  if (readySol(r)) {
    Serial.print(F("# ready pose th* =")); printAngles(r.th);
    Serial.print(F("   pulses")); for (uint8_t c = 0; c < NJ; c++) { Serial.print(' '); Serial.print(r.us[c]); }
    Serial.println();
  } else Serial.println(F("# ready pose not reachable"));
}

void doMap(float zcm) {
  float zb = zcm * 10.0f;
  Serial.print(F("# valid pick solutions for a block bottom at z = ")); Serial.print(zcm, 1);
  Serial.println(F(" cm, best approach angle per cell   (. none, B base keep-out, 2 cm grid)"));
  for (int Y = 20; Y >= -20; Y -= 2) {
    if (Y >= 0 && Y < 10) Serial.print(' ');
    if (Y > -10) Serial.print(' ');
    Serial.print(Y); Serial.print(F(" |"));
    for (int X = -20; X <= 20; X += 2) {
      uint8_t n = 0;
      bool keep = false;
      Sol s;
      for (uint8_t ia = 0; ia < N_APPROACH && n == 0 && !keep; ia++) {
        float psi = approachAuto ? APPROACH_LIST[ia] : approachDeg;
        if (!approachAuto && ia > 0) break;
        for (uint8_t b = 0; b < 8; b++) {
          if (!branchAllowed(b)) continue;
          uint8_t w = solvePaper(X, Y, zb + graspUp(), psi, b, s);
          if (w == W_KEEPOUT) { keep = true; break; }
          if (w == S_OK && solvePaper(X, Y, zb + blk[2] + CLEAR_MM + gripDepth, psi, b, s) == S_OK) n++;
        }
      }
      Serial.print(' ');
      Serial.print(keep ? 'B' : (n ? (char)('0' + n) : '.'));
    }
    Serial.println();
  }
  Serial.println(F("# columns: x = -20 to +20 cm left to right, 2 cm apart; B marks the base"));
}

void printHelp() {
  Serial.println(F("# P x y z | S k | GO | T k | STACK n | MAP z | HOME | M x y z | J a b c d e"));
  Serial.println(F("# G O|C|us | W | AIR mm | DRY 1|0 | SPD n | A deg | CAL | !   (any key stops a move)"));
  Serial.println(F("# FRAME [bx by rot z0] | BLOCK sx sy sz | GRIPAT mm | DEST x y | BRANCH A|F|B | FLIP 0|1|2 | LASH ON|OFF"));
  Serial.println(F("# STK x y n | STK CLR | WORLD | MV x1 y1 x2 y2 | TOWER x y n"));
  Serial.println(F("# PROG ... END | RUN | LIST | PCLR | WAIT text | ECHO text"));
}

float nextF(bool &ok) {                  // IK-S11: only plain numbers (no nan, inf)
  char *t = strtok(NULL, " ,[]");
  if (!t) { ok = false; return 0; }
  char *e;
  float v = strtod(t, &e);
  if (e == t || *e != 0 || !isfinite(v)) { ok = false; Serial.print(F("# not a number: ")); Serial.println(t); return 0; }
  return v;
}

// ============================================================== programs
void loadStartProgram() {
  progN = 0;
  uint8_t len = 0;
  for (uint16_t i = 0;; i++) {
    char c = pgm_read_byte(START_PROGRAM + i);
    if (c == 0 || c == '\n') {
      if (len && progN < PROG_LINES) { prog[progN][len] = 0; progN++; }
      len = 0;
      if (c == 0) break;
      continue;
    }
    if (progN < PROG_LINES && len < PROG_LEN - 1) prog[progN][len++] = c;
    else if (progN < PROG_LINES) { Serial.print(F("# WARNING start program line ")); Serial.print(progN + 1); Serial.println(F(" too long - cut off")); while (pgm_read_byte(START_PROGRAM + i + 1) && pgm_read_byte(START_PROGRAM + i + 1) != '\n') i++; }
  }
}
void listProgram() {
  Serial.print(F("# program: ")); Serial.print(progN); Serial.println(F(" line(s)"));
  for (uint8_t i = 0; i < progN; i++) { Serial.print(F("#  ")); Serial.print(i + 1); Serial.print(F(": ")); Serial.println(prog[i]); }
}

void handleLine(char *line);

bool waitEnter() {                       // returns false if the person typed X (stop)
  while (true) {
    if (Serial.available()) {
      delay(5);
      char c = Serial.read();
      bool stop = (c == 'x' || c == 'X');
      while (Serial.available()) Serial.read();
      return !stop;
    }
    delay(10);
  }
}

void runProgram() {
  if (!progN) { Serial.println(F("# no program (PROG ... END to record one)")); return; }
  running = true;
  cmdFail = false; keyStopped = false;
  char l[PROG_LEN];
  for (uint8_t i = 0; i < progN; i++) {
    strcpy(l, prog[i]);
    Serial.print(F("# RUN ")); Serial.print(i + 1); Serial.print(F(": ")); Serial.println(l);
    handleLine(l);
    if (cmdFail) { Serial.print(F("# RUN stopped at line ")); Serial.println(i + 1); running = false; return; }
  }
  running = false;
  Serial.println(F("# RUN finished"));
}

// ============================================================== dispatcher
void handleLine(char *line) {
  if (recording) {
    char tmp[8]; strncpy(tmp, line, 7); tmp[7] = 0;
    for (char *p = tmp; *p; p++) *p = toupper(*p);
    if (!strncmp(tmp, "END", 3)) { recording = false; Serial.print(F("# program recorded: ")); Serial.print(progN); Serial.println(F(" line(s) - RUN plays it")); return; }
    if (progN >= PROG_LINES) { Serial.println(F("# program full")); return; }
    if (strlen(line) > PROG_LEN - 1) { Serial.print(F("# line refused: longer than ")); Serial.print(PROG_LEN - 1); Serial.println(F(" characters")); return; }
    strncpy(prog[progN], line, PROG_LEN - 1); prog[progN][PROG_LEN - 1] = 0; progN++;
    Serial.print(F("#  ")); Serial.print(progN); Serial.print(F(": ")); Serial.println(prog[progN - 1]);
    return;
  }
  char raw[64]; strncpy(raw, line, 63); raw[63] = 0;
  char *cmd = strtok(line, " ,[]");
  if (!cmd) return;
  for (char *p = cmd; *p; p++) *p = toupper(*p);
  bool ok = true;
  if (!running) { cmdFail = false; keyStopped = false; }

  if (!strcmp(cmd, "?") || !strcmp(cmd, "HELP")) { printHelp(); return; }
  if (!strcmp(cmd, "!"))    { releaseAll(); return; }
  if (!strcmp(cmd, "W"))    { doWhere(); return; }
  if (!strcmp(cmd, "CAL"))  { doCal(); return; }
  if (!strcmp(cmd, "HOME")) { doHome(); return; }
  if (!strcmp(cmd, "GO"))   { doGo(); return; }
  if (!strcmp(cmd, "WORLD")) { printWorld(); return; }
  if (!strcmp(cmd, "PROG")) { recording = true; progN = 0; Serial.println(F("# recording: one command per line, END to finish")); return; }
  if (!strcmp(cmd, "RUN"))  { if (running) { Serial.println(F("# RUN inside RUN ignored")); return; } runProgram(); return; }
  if (!strcmp(cmd, "LIST")) { listProgram(); return; }
  if (!strcmp(cmd, "PCLR")) { progN = 0; Serial.println(F("# program cleared")); return; }
  if (!strcmp(cmd, "ECHO")) { char *t = raw + 4; while (*t == ' ') t++; Serial.print(F("# ")); Serial.println(t); return; }
  if (!strcmp(cmd, "WAIT")) {
    char *t = raw + 4; while (*t == ' ') t++;
    Serial.print(F("# WAIT: ")); Serial.print(t); Serial.println(F("   (Enter = continue, X = stop)"));
    if (!waitEnter()) { Serial.println(F("# stopped")); cmdFail = true; }
    return;
  }
  if (!strcmp(cmd, "P")) {
    float x = nextF(ok), y = nextF(ok), z = nextF(ok);
    if (!ok) { Serial.println(F("# P x y z   (paper cm; z = bottom of the top block)")); cmdFail = true; return; }
    doSolve(x, y, z); return;
  }
  if (!strcmp(cmd, "S"))    { float k = nextF(ok); if (ok) doSelect((int)k); return; }
  if (!strcmp(cmd, "T"))    { float k = nextF(ok); if (ok) doPreview((int)k); return; }
  if (!strcmp(cmd, "STACK")) {
    float n = nextF(ok);
    if (!ok || n < 0 || n > 6) { Serial.println(F("# STACK 0-6")); cmdFail = true; return; }
    if (setStack(destX, destY, (uint8_t)n) < 0) { Serial.println(F("# too many stacks (12)")); cmdFail = true; return; }
    Serial.print(F("# tower count ")); Serial.println((int)n); return;
  }
  if (!strcmp(cmd, "MAP"))  { float z = nextF(ok); doMap(ok ? z : 0.0f); return; }
  if (!strcmp(cmd, "M")) {
    float x = nextF(ok), y = nextF(ok), z = nextF(ok);
    if (!ok) { Serial.println(F("# M x y z   (paper cm; z = jaw-tip height above the paper, cm)")); cmdFail = true; return; }
    doMoveTip(x, y, z); return;
  }
  if (!strcmp(cmd, "J")) {
    float a[NJ];
    for (uint8_t j = 0; j < NJ; j++) a[j] = nextF(ok);
    if (!ok) { Serial.println(F("# J th1 th2 th3 th4 th5   (degrees)")); cmdFail = true; return; }
    doJoint(a); return;
  }
  if (!strcmp(cmd, "G")) {
    char *t = strtok(NULL, " ");
    if (!t) { Serial.println(F("# G O | G C | G us")); return; }
    uint16_t us = (toupper(t[0]) == 'O') ? GRIP.openUs : (toupper(t[0]) == 'C') ? GRIP.holdUs : (uint16_t)atoi(t);
    if (!active[GS] && !dryRun) { applyUs(GS, us); emitPos(GS, "START"); return; }
    moveGrip(us); return;
  }
  if (!strcmp(cmd, "AIR")) {
    float a = nextF(ok);
    if (!ok || a < 0 || a > 100) { Serial.println(F("# AIR 0-100 (mm)")); return; }
    airMm = a; haveTgt = false; Serial.print(F("# AIR ")); Serial.print(airMm, 0);
    Serial.println(F(" mm - every commanded height is raised; P again to re-solve")); return;
  }
  if (!strcmp(cmd, "DRY")) {
    float d = nextF(ok);
    if (!ok) { Serial.println(F("# DRY 1 | 0")); return; }
    if (d == 0 && (!CAL_VALID || calBad)) { Serial.println(F("# CAL_VALID is false or the limits are invalid - DRY stays on")); return; }
    dryRun = (d != 0);
    memcpy(sim, cur, sizeof(cur));
    Serial.print(F("# DRY ")); Serial.println(dryRun ? 1 : 0); return;
  }
  if (!strcmp(cmd, "A")) {
    char *t = strtok(NULL, " ");
    bool autoWord = t && isalpha(t[0]);
    float a = t ? atof(t) : 0; ok = t != NULL;
    if (autoWord) a = 0;
    if (!ok || (a != 0 && (a < -90 || a > 90))) { Serial.println(F("# A deg, -90 (straight down) to 90 (not 0); A AUTO (or A 0) = automatic")); return; }
    approachAuto = (a == 0); if (!approachAuto) approachDeg = a; haveTgt = false;
    if (approachAuto) Serial.println(F("# approach automatic - P again to re-solve"));
    else { Serial.print(F("# approach fixed at ")); Serial.print(approachDeg, 0); Serial.println(F(" deg - P again to re-solve")); }
    return;
  }
  if (!strcmp(cmd, "SPD")) {
    float s = nextF(ok);
    if (!ok || s < 1 || s > 15) { Serial.println(F("# SPD 1-15")); return; }
    rateUs = (uint8_t)s; Serial.print(F("# SPD ")); Serial.println(rateUs); return;
  }
  if (!strcmp(cmd, "FRAME")) {
    float a = nextF(ok), b = nextF(ok), c = nextF(ok), d = nextF(ok);
    if (ok) { frBx = a; frBy = b; frRot = c; frZ0 = d; haveTgt = false; }
    else Serial.println(F("# FRAME bx by rot z0 sets it (cm, cm, deg, mm)"));
    printFrame(); return;
  }
  if (!strcmp(cmd, "BLOCK")) {
    float a = nextF(ok), b = nextF(ok), c = nextF(ok);
    if (!ok || a <= 0 || b <= 0 || c <= 0) { Serial.println(F("# BLOCK sx sy sz (mm)")); return; }
    blk[0] = a; blk[1] = b; blk[2] = c; haveTgt = false;
    Serial.print(F("# block ")); Serial.print(a, 1); Serial.print('x'); Serial.print(b, 1); Serial.print('x'); Serial.print(c, 1); Serial.println(F(" mm"));
    return;
  }
  if (!strcmp(cmd, "GRIPAT")) {
    float a = nextF(ok);
    if (!ok || a < 0 || a > ARM_TIP - 50) { Serial.println(F("# GRIPAT mm (jaw tips to the gripping area)")); return; }
    gripDepth = a; haveTgt = false;
    Serial.print(F("# grip depth ")); Serial.print(a, 1); Serial.print(F(" mm: grasp point ")); Serial.print(toolLen(), 1);
    Serial.println(F(" mm from the wrist axis - P again to re-solve")); return;
  }
  if (!strcmp(cmd, "DEST")) {
    float a = nextF(ok), b = nextF(ok);
    if (!ok) { Serial.println(F("# DEST x y (paper cm)")); return; }
    destX = a; destY = b; destStack(); haveTgt = false;
    Serial.print(F("# destination [")); Serial.print(a, 1); Serial.print(F(", ")); Serial.print(b, 1);
    if (destStack() < 0) { Serial.println(F("# too many stacks (12) - DEST not tracked")); cmdFail = true; }
    Serial.print(F("], ")); Serial.print(destCount()); Serial.println(F(" block(s) there")); return;
  }
  if (!strcmp(cmd, "BRANCH")) {
    char *t = strtok(NULL, " ");
    if (!t) { Serial.println(F("# BRANCH A|F|B")); return; }
    char c = toupper(t[0]);
    branchPolicy = c == 'F' ? BR_FRONT : c == 'B' ? BR_BACK : BR_ANY; haveTgt = false;
    Serial.print(F("# branches: ")); Serial.println(branchPolicy == BR_FRONT ? F("front only") : branchPolicy == BR_BACK ? F("behind the back only") : F("any")); return;
  }
  if (!strcmp(cmd, "FLIP")) {
    float a = nextF(ok);
    if (!ok || a < 0 || a > 2) { Serial.println(F("# FLIP 0 (upright only) | 1 (any, upright preferred) | 2 (flipped preferred)")); return; }
    flipPolicy = (uint8_t)a; haveTgt = false;
    Serial.print(F("# FLIP ")); Serial.println(flipPolicy); return;
  }
  if (!strcmp(cmd, "LASH")) {
    char *t = strtok(NULL, " ");
    if (!t) { Serial.println(F("# LASH ON | LASH OFF")); return; }
    lashOn = (toupper(t[1]) == 'N');
    Serial.print(F("# backlash compensation ")); Serial.println(lashOn ? F("ON") : F("OFF")); return;
  }
  if (!strcmp(cmd, "STK")) {
    char *t = strtok(NULL, " ,");
    if (t && toupper(t[0]) == 'C') { nStk = 0; destStack(); Serial.println(F("# stacks cleared")); return; }
    char *e = NULL;
    float x = t ? strtod(t, &e) : 0, y = nextF(ok), n = nextF(ok);
    if (t && (e == t || *e != 0 || !isfinite(x))) ok = false;
    if (!t || !ok || n < 0 || n > 9) { Serial.println(F("# STK x y n   (n = 0 empties it) | STK CLR")); return; }
    if (n == 0) removeStack(x, y);
    else if (setStack(x, y, (uint8_t)n) < 0) { Serial.println(F("# too many stacks (12)")); cmdFail = true; }
    destStack();
    printWorld(); return;
  }
  if (!strcmp(cmd, "MV")) {
    float a = nextF(ok), b = nextF(ok), c = nextF(ok), d = nextF(ok);
    if (!ok) { Serial.println(F("# MV x1 y1 x2 y2")); cmdFail = true; return; }
    doMove(a, b, c, d); return;
  }
  if (!strcmp(cmd, "TOWER")) {
    float a = nextF(ok), b = nextF(ok), n = nextF(ok);
    if (!ok || n < 1 || n > 9) { Serial.println(F("# TOWER x y n")); cmdFail = true; return; }
    doTower(a, b, (uint8_t)n); return;
  }
  Serial.print(F("# unknown command: ")); Serial.println(cmd);
  cmdFail = true;
  printHelp();
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (c == '\r') { delay(2); if (Serial.peek() == '\n') Serial.read(); }   // CR+LF is one line ending
      if (blen) { buf[blen] = 0; blen = 0; handleLine(buf); }
      continue;
    }
    if (blen < sizeof(buf) - 1) buf[blen++] = c;
    lastByte = millis();
  }
  if (blen && millis() - lastByte > 60) { buf[blen] = 0; blen = 0; handleLine(buf); }
}

// ============================================================ setup/loop
void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(200);
  Serial.print(F("# READY ik_demo v" FW_VERSION " (")); Serial.print(F(ROBOT_NAME)); Serial.print(F("), driver "));
  Serial.println(servo.name());
  dryRun = !CAL_VALID;
  for (uint8_t s = 0; s < NS; s++) if (slotMin(s) >= slotMax(s)) {
    Serial.print(F("# ERROR ch ")); Serial.print(slotCh(s)); Serial.println(F(" has MIN >= MAX in robot_config.h - DRY forced on"));
    calBad = true;
  }
  if (calBad) dryRun = true;
  for (uint8_t s = 0; s < NS; s++) { cur[s] = 1500; active[s] = false; }
  for (uint8_t j = 0; j < NJ; j++) lastSide[j] = 0;
  if (!servo.begin(true)) Serial.println(F("# WARNING servo driver did not answer - check wiring"));
  if (servo.adopted) {
    for (uint8_t s = 0; s < NS; s++) {
      uint16_t us;
      if (!servo.readBack(slotCh(s), us)) continue;
      cur[s] = us; active[s] = true;
      emitPos(s, "ADOPTED");
      if (CAL_VALID && (us < slotMin(s) || us > slotMax(s))) {
        Serial.print(F("# WARNING ch ")); Serial.print(slotCh(s));
        Serial.println(F(" is outside its limits - the first move will snap it inside; support the arm"));
      }
    }
    Serial.println(F("# adopted live channels, nothing moved"));
  } else Serial.println(F("# servo driver configured, outputs off"));
  memcpy(sim, cur, sizeof(cur));
  nStk = 0;
  for (uint8_t i = 0; i < sizeof(INIT_WORLD) / sizeof(INIT_WORLD[0]); i++)
    if (INIT_WORLD[i][2] > 0) setStack(INIT_WORLD[i][0], INIT_WORLD[i][1], (uint8_t)INIT_WORLD[i][2]);
  destStack();
  loadStartProgram();
  if (!CAL_VALID) Serial.println(F("# CAL_VALID is false: solving and printing only, the arm will not move"));
  else Serial.println(F("# calibration loaded. Tower count starts at 0 - use STACK n if blocks are already there"));
  if (progN) { Serial.print(F("# start program loaded (")); Serial.print(progN); Serial.println(F(" lines): LIST shows it, RUN plays it")); }
  printHelp();
}

void loop() {
  readSerial();
}
