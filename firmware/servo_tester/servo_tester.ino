/*
  servo_tester.ino  v3.0 - joint mapping and calibration for servo-driven robots
  Design: docs/design/servo_tester.md (DD-ST).  Guide: docs/guides/joint-mapping.md
  Copyright (c) 2026 ticohannan. BSD-3-Clause, see LICENSE.

  Target: Arduino Mega 2560 (or any board with >= 4 KB RAM and >= 2 KB EEPROM)
  Servo output through the RobotServoDriver interface (DD-DRV): PCA9685 @ 0x40 by default,
  or servos on Arduino pins (define SERVO_DRIVER_PINS below).

  STATUS: v3.0 compiled for the ATmega2560 and tested on a PC with an emulated PCA9685 and
  EEPROM (tests/test_servo_tester.py). The v2.3 safety behaviour it keeps was verified on the
  bench servo (2026-09-20). The new mapping commands are NOT YET RUN ON HARDWARE.

  Serial: 115200 baud. Commands are whole lines; line ending Newline or None.

  Safety model (unchanged from v2.3):
    - No channel outputs a pulse until you START it.
    - On boot, channels already producing a valid pulse are adopted, not reset.
    - Every channel has soft limits (default 1300-1700 us); commands outside are clamped.
    - Moves are slew-rate limited; any keypress pauses a move.
    - HOLD is OFF by default: a gravity-loaded joint drops when it goes limp.
    - Saved range ends are never applied at boot (LIMRESTORE applies them).

  Data lines (CSV) for logging on a PC:
    POS,<ms>,<arm>,<ch>,<us>,<state>        MARK,<ms>,<arm>,<ch>,<us>,<label>
    LIMS,<ms>,<arm>,<ch>,<min>,<max>        SCAN,<ms>,<addr_hex>
    PT,<ms>,<arm>,<ch>,<n>,<us>,<kind>,<value>,<side>
    FIT,<arm>,<ch>,<n>,<us0>,<uspd>,<lash>,<rms>,<worst>
    ERR,<ms>,<code>,<text>
  Lines starting with '#' are for people.
*/

#define FW_VERSION "3.0"

#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include "RobotTypes.h"
#include "RobotServoDriver.h"

// ------------------------------------------------------------ servo driver
// #define SERVO_DRIVER_PINS              // uncomment to use servos on Arduino pins
#if defined(SERVO_DRIVER_PINS)
  #include "PinServoDriver.h"
  const uint8_t SERVO_PINS[] = {2, 3, 4, 5, 6, 7, 8, 9};
  PinServoDriver drv(SERVO_PINS, sizeof(SERVO_PINS));
#else
  #include "PCA9685Driver.h"
  PCA9685Driver drv(0x40, 16);
#endif
RobotServoDriver &servo = drv;

// ---------------------------------------------------------------- constants
const uint8_t  NUM_CH        = 16;
const uint16_t ABS_MIN_US    = SERVO_ABS_MIN_US;
const uint16_t ABS_MAX_US    = SERVO_ABS_MAX_US;
const uint16_t DEF_MIN_US    = 1300;
const uint16_t DEF_MAX_US    = 1700;
const uint16_t CENTER_US     = 1500;
const uint16_t MAX_RATE_STEP = 25;
const uint16_t MIN_RATE_MS   = 10;
const uint16_t WD_MS         = 2000;
const uint16_t LINE_IDLE_MS  = 60;
const uint16_t PROBE_MS      = 200;
const uint8_t  MAX_PTS       = 60;
const uint8_t  NAME_LEN      = 9;            // 8 characters + terminator

// ------------------------------------------------------------------- state
struct Chan {
  uint16_t minUs, maxUs, curUs;
  bool     active;
  int8_t   dir;                              // direction of the last change: +1, -1, 0
  uint32_t lastChange;
};
Chan chn[NUM_CH];

// Mapping data, saved to EEPROM (DD-ST section 5)
enum PtKind : uint8_t { K_V = 0, K_H = 1, K_D = 2, K_XY = 3 };
struct Meta {
  uint8_t  type;                             // J_ROT, J_LIN, J_GRIP; 255 = not set
  uint8_t  load;
  int8_t   sideMode;                         // 0 = auto, +1, -1
  char     name[NAME_LEN];
  float    ref;                              // inboard link absolute angle, deg
  float    prec;                             // reading precision, joint units
  uint16_t loUs, hiUs;                       // found range ends, 0 = not set
  float    wig;                              // WIG lash, < 0 = not measured
  uint16_t openUs, holdUs;                   // gripper settings, 0 = not set
  uint16_t limMin, limMax;                   // last LIM set for this channel, 0 = none (export fallback)
  uint8_t  loMargin, hiMargin;               // MARGIN in force when LO / HI was recorded
};
struct Point {
  uint8_t  ch, kind;
  int8_t   side;
  uint16_t us;
  float    a, b;                             // value (deg/mm) in a; for K_XY: x in a, y in b (cm)
};
struct Session {
  char     magic[3];
  uint8_t  layout;
  char     arm;
  uint16_t margin;
  Meta     meta[NUM_CH];
  uint8_t  npts;
  Point    pts[MAX_PTS];
};
Session ses;
const uint8_t SES_LAYOUT = 3;

struct FitRes {
  bool  ok;
  uint8_t n, p;
  float us0, uspd, lash, lashSe, rms, worst, vmin, vmax, cx, cy, rad;
  bool  lashSig, both, xy;
  int8_t worstIdx;
};
FitRes fitr;
float resid[MAX_PTS];                        // residual per point of the last fit (by point index)

uint8_t  sel      = 0;
bool     pcaOk    = false;
uint16_t rateStep = 5;
uint16_t rateMs   = 20;
uint16_t holdMs   = 0;
bool     wdOn     = false;
uint32_t lastRx   = 0;

enum MState { M_IDLE, M_MOVING, M_PAUSED };
MState   mstate     = M_IDLE;
uint16_t mTarget    = CENTER_US;
uint32_t nextStepAt = 0;
uint16_t stepsSincePrint = 0;

bool     sweeping   = false;
uint8_t  sweepLeg   = 0;
uint16_t sweepMin, sweepMax, sweepStep, sweepDwell, sweepHome;
uint32_t dwellUntil = 0;

bool     probing    = false;
uint8_t  probeCh    = 0;
uint32_t probeUntil = 0;

bool     apprPending = false;                // APPR: second leg target
uint16_t apprTarget  = 0;

bool     jogMode = false;
uint16_t jogStep = 10;

char     buf[64];
uint8_t  blen = 0;
uint32_t lastByte = 0;

// ------------------------------------------------------------- utilities
float wrap180(float a) {
  while (a > 180.0f)   a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}
bool isRot(uint8_t c) { return ses.meta[c].type != J_LIN && ses.meta[c].type != J_GRIP; }
const __FlashStringHelper *unitOf(uint8_t c) { return isRot(c) ? F("deg") : F("mm"); }
const __FlashStringHelper *typeName(uint8_t t) {
  return t == J_ROT ? F("ROT") : t == J_LIN ? F("LIN") : t == J_GRIP ? F("GRIP") : F("unset");
}

// ---------------------------------------------------------------- session
void saveSession() { EEPROM.put(0, ses); }

void clearSession() {
  memset(&ses, 0, sizeof(ses));
  ses.magic[0] = 'S'; ses.magic[1] = 'T'; ses.magic[2] = '3';
  ses.layout = SES_LAYOUT; ses.arm = 'A'; ses.margin = 30;
  for (uint8_t c = 0; c < NUM_CH; c++) {
    ses.meta[c].type = 255; ses.meta[c].prec = 1.0f; ses.meta[c].wig = -1.0f;
  }
}

bool loadSession() {
  EEPROM.get(0, ses);
  if (ses.magic[0] != 'S' || ses.magic[1] != 'T' || ses.magic[2] != '3' ||
      ses.layout != SES_LAYOUT || ses.npts > MAX_PTS) { clearSession(); return false; }
  return true;
}

uint8_t countPts(uint8_t c) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < ses.npts; i++) if (ses.pts[i].ch == c) n++;
  return n;
}

// ------------------------------------------------------------------ output
void emitPos(uint8_t c, const char *state) {
  Serial.print(F("POS,"));   Serial.print(millis());
  Serial.print(',');         Serial.print(ses.arm);
  Serial.print(',');         Serial.print(c);
  Serial.print(',');         Serial.print(chn[c].curUs);
  Serial.print(',');         Serial.println(state);
}
void emitMark(uint8_t c, const char *label) {
  Serial.print(F("MARK,"));  Serial.print(millis());
  Serial.print(',');         Serial.print(ses.arm);
  Serial.print(',');         Serial.print(c);
  Serial.print(',');         Serial.print(chn[c].curUs);
  Serial.print(',');         Serial.println(label);
}
void emitLims(uint8_t c) {
  Serial.print(F("LIMS,"));  Serial.print(millis());
  Serial.print(',');         Serial.print(ses.arm);
  Serial.print(',');         Serial.print(c);
  Serial.print(',');         Serial.print(chn[c].minUs);
  Serial.print(',');         Serial.println(chn[c].maxUs);
}
void emitErr(const char *code, const __FlashStringHelper *text) {
  Serial.print(F("ERR,"));   Serial.print(millis());
  Serial.print(',');         Serial.print(code);
  Serial.print(',');         Serial.println(text);
}

// ------------------------------------------------------------------ motion
void applyUs(uint8_t c, uint16_t us) {
  if (!servo.write(c, us)) { emitErr("DRV", F("write failed")); return; }
  if (chn[c].active && us != chn[c].curUs) chn[c].dir = us > chn[c].curUs ? 1 : -1;
  chn[c].curUs      = us;
  chn[c].active     = true;
  chn[c].lastChange = millis();
}
void releaseCh(uint8_t c) {
  servo.release(c);
  chn[c].active = false;
  chn[c].dir = 0;
}
void releaseAll() {
  for (uint8_t i = 0; i < NUM_CH; i++) releaseCh(i);
  mstate = M_IDLE; sweeping = false; probing = false; apprPending = false;
  Serial.println(F("# all channels released"));
}
uint16_t clampToLimits(uint8_t c, long us) {
  if (us < chn[c].minUs) us = chn[c].minUs;
  if (us > chn[c].maxUs) us = chn[c].maxUs;
  return (uint16_t)us;
}
void startMove(uint16_t target) {
  if (!chn[sel].active) { emitErr("NOT_STARTED", F("use START us first")); return; }
  mTarget = clampToLimits(sel, target);
  mstate = M_MOVING; nextStepAt = millis(); stepsSincePrint = 0;
}
void stopMove(const char *why) {
  mstate = M_IDLE; sweeping = false; apprPending = false;
  Serial.print(F("# stopped: ")); Serial.println(why);
  emitPos(sel, "IDLE");
}

// ------------------------------------------------------------------- sweep
void advanceSweep() {
  if (sweepLeg > 2) { sweeping = false; Serial.println(F("# sweep complete")); return; }
  uint16_t legTarget = (sweepLeg == 0) ? sweepMin : (sweepLeg == 1) ? sweepMax : sweepHome;
  uint16_t cur = chn[sel].curUs;
  if (cur == legTarget) { sweepLeg++; return; }
  long d = (long)legTarget - (long)cur, s = d;
  if (s >  (long)sweepStep) s =  (long)sweepStep;
  if (s < -(long)sweepStep) s = -(long)sweepStep;
  startMove((uint16_t)((long)cur + s));
}

// -------------------------------------------------------- scan / adoption
uint8_t adoptLiveChannels() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < NUM_CH && i < servo.channels(); i++) {
    uint16_t us;
    if (!servo.readBack(i, us)) continue;
    chn[i].curUs = us; chn[i].active = true; chn[i].dir = 0; chn[i].lastChange = millis();
    if (us < chn[i].minUs || us > chn[i].maxUs) {
      long lo = (long)us - 100, hi = (long)us + 100;
      chn[i].minUs = (uint16_t)(lo < (long)ABS_MIN_US ? ABS_MIN_US : lo);
      chn[i].maxUs = (uint16_t)(hi > (long)ABS_MAX_US ? ABS_MAX_US : hi);
      emitLims(i);
    }
    emitPos(i, "ADOPTED");
    n++;
  }
  return n;
}

void scanAndCheck(bool forceInit) {
#if !defined(SERVO_DRIVER_PINS)
  Serial.println(F("# I2C scan"));
  PCA9685Driver::scanBus(Serial);
#endif
  if (!servo.present()) {
    pcaOk = false;
    emitErr("NO_DRV", F("servo driver does not answer - check SDA/SCL, VCC, address jumpers"));
    return;
  }
  if (!forceInit) {
    bool ok = servo.begin(true);
    if (ok && servo.adopted) {
      pcaOk = true;
      uint8_t n = adoptLiveChannels();
      servo.diag(Serial);
      Serial.print(F("# driver already running - adopted ")); Serial.print(n);
      Serial.println(F(" live channel(s), no reset, joints keep holding"));
      Serial.println(F("# use INIT to force a reconfigure (all outputs go off)"));
      return;
    }
    pcaOk = ok;
  } else {
    Serial.println(F("# configuring the driver - all outputs will go off"));
    pcaOk = servo.begin(false);
  }
  for (uint8_t i = 0; i < NUM_CH; i++) { chn[i].active = false; chn[i].dir = 0; }
  servo.diag(Serial);
  Serial.println(pcaOk ? F("# driver OK, outputs off") : F("# DRIVER CONFIG FAILED"));
}

// ----------------------------------------------------------------- points
void printPoint(uint8_t i) {
  const Point &p = ses.pts[i];
  Serial.print(F("PT,")); Serial.print(millis()); Serial.print(','); Serial.print(ses.arm);
  Serial.print(','); Serial.print(p.ch); Serial.print(','); Serial.print(i + 1);
  Serial.print(','); Serial.print(p.us); Serial.print(',');
  Serial.print(p.kind == K_V ? F("V") : p.kind == K_H ? F("H") : p.kind == K_D ? F("D") : F("XY"));
  Serial.print(','); Serial.print(p.a, 2);
  if (p.kind == K_XY) { Serial.print(';'); Serial.print(p.b, 2); }
  Serial.print(','); Serial.println(p.side);
}

bool addPoint(uint8_t kind, float a, float b) {
  if (!chn[sel].active) { emitErr("NOT_STARTED", F("the channel must be holding a pulse")); return false; }
  if (ses.npts >= MAX_PTS) { emitErr("FULL", F("point store full - PTDEL or PTCLR some")); return false; }
  Meta &m = ses.meta[sel];
  int8_t side = m.sideMode;
  if (side == 0) side = -chn[sel].dir;      // dragged + lags: sits on the - side of its play
  if (m.load && m.sideMode == 0)
    Serial.println(F("# note: gravity-loaded joint with SIDE A - say which way it would fall: SIDE + or SIDE -"));
  Point &p = ses.pts[ses.npts];
  p.ch = sel; p.kind = kind; p.side = side; p.us = chn[sel].curUs; p.a = a; p.b = b;
  ses.npts++;
  saveSession();
  printPoint(ses.npts - 1);
  return true;
}

void listPoints(uint8_t c) {
  Serial.print(F("# ch ")); Serial.print(c); Serial.print(F(" points (n us kind value side resid "));
  Serial.print(unitOf(c)); Serial.println(F(")"));
  for (uint8_t i = 0; i < ses.npts; i++) {
    const Point &p = ses.pts[i];
    if (p.ch != c) continue;
    Serial.print(F("#  ")); Serial.print(i + 1); Serial.print(' '); Serial.print(p.us); Serial.print(' ');
    Serial.print(p.kind == K_V ? F("V ") : p.kind == K_H ? F("H ") : p.kind == K_D ? F("D ") : F("XY "));
    Serial.print(p.a, 2);
    if (p.kind == K_XY) { Serial.print(','); Serial.print(p.b, 2); }
    Serial.print(F(" side ")); Serial.print(p.side);
    if (fitr.ok && !isnan(resid[i])) { Serial.print(F("  resid ")); Serial.print(resid[i], 2); }
    Serial.println();
  }
}

void deletePoint(uint8_t idx) {             // idx is 0-based
  for (uint8_t i = idx; i + 1 < ses.npts; i++) ses.pts[i] = ses.pts[i + 1];
  ses.npts--;
}

// ---------------------------------------------------------------------- fit
// Solve the symmetric p x p system M x = r (p <= 3) by Gaussian elimination. Returns false if singular.
bool solveSym(float M[3][3], float r[3], uint8_t p, float x[3], float inv[3][3]) {
  float A[3][7];
  for (uint8_t i = 0; i < p; i++) {
    for (uint8_t j = 0; j < p; j++) A[i][j] = M[i][j];
    for (uint8_t j = 0; j < p; j++) A[i][p + j] = (i == j) ? 1.0f : 0.0f;
    A[i][2 * p] = r[i];
  }
  for (uint8_t c = 0; c < p; c++) {
    uint8_t piv = c;
    for (uint8_t i = c + 1; i < p; i++) if (fabs(A[i][c]) > fabs(A[piv][c])) piv = i;
    if (fabs(A[piv][c]) < 1e-9f) return false;
    if (piv != c) for (uint8_t j = 0; j <= 2 * p; j++) { float t = A[c][j]; A[c][j] = A[piv][j]; A[piv][j] = t; }
    float d = A[c][c];
    for (uint8_t j = 0; j <= 2 * p; j++) A[c][j] /= d;
    for (uint8_t i = 0; i < p; i++) {
      if (i == c) continue;
      float f = A[i][c];
      for (uint8_t j = 0; j <= 2 * p; j++) A[i][j] -= f * A[c][j];
    }
  }
  for (uint8_t i = 0; i < p; i++) {
    x[i] = A[i][2 * p];
    for (uint8_t j = 0; j < p; j++) inv[i][j] = A[i][p + j];
  }
  return true;
}

// Joint value of point i (deg or mm). For K_XY it needs the circle fit (fitr.cx, cy).
float pointValue(uint8_t i) {
  const Point &p = ses.pts[i];
  if (p.kind != K_XY) return p.a;
  return atan2f(p.b - fitr.cy, p.a - fitr.cx) * RAD_TO_DEG;   // class Atan2(x = X - cx, y = Y - cy)
}

bool fitChannel(uint8_t c, bool verbose) {
  memset(&fitr, 0, sizeof(fitr));
  for (uint8_t i = 0; i < MAX_PTS; i++) resid[i] = NAN;
  uint8_t idx[MAX_PTS], n = 0, nxy = 0;
  for (uint8_t i = 0; i < ses.npts; i++) if (ses.pts[i].ch == c) { idx[n++] = i; if (ses.pts[i].kind == K_XY) nxy++; }
  fitr.n = n;
  if (n < 2) { if (verbose) { Serial.print(F("# ch ")); Serial.print(c); Serial.println(F(": need at least 2 points")); } return false; }
  if (nxy && nxy != n) { if (verbose) Serial.println(F("# PTXY points cannot be mixed with value points on one channel - PTDEL one kind")); return false; }
  // sort by pulse (insertion sort) - used for unwrapping and for the span
  for (uint8_t i = 1; i < n; i++) { uint8_t k = idx[i]; int8_t j = i - 1; while (j >= 0 && ses.pts[idx[j]].us > ses.pts[k].us) { idx[j + 1] = idx[j]; j--; } idx[j + 1] = k; }
  float v[MAX_PTS];
  if (nxy) {
    if (n < 3) { if (verbose) Serial.println(F("# PTXY needs at least 3 points for the circle")); return false; }
    float mx = 0, my = 0;
    for (uint8_t k = 0; k < n; k++) { mx += ses.pts[idx[k]].a; my += ses.pts[idx[k]].b; }
    mx /= n; my /= n;
    float M[3][3] = {{0}}, r[3] = {0}, x[3], inv[3][3];
    for (uint8_t k = 0; k < n; k++) {
      float X = ses.pts[idx[k]].a - mx, Y = ses.pts[idx[k]].b - my, row[3] = {2 * X, 2 * Y, 1}, rhs = X * X + Y * Y;
      for (uint8_t i = 0; i < 3; i++) { r[i] += row[i] * rhs; for (uint8_t j = 0; j < 3; j++) M[i][j] += row[i] * row[j]; }
    }
    if (!solveSym(M, r, 3, x, inv)) { if (verbose) Serial.println(F("# circle fit failed - points in a line? spread them further")); return false; }
    fitr.cx = x[0] + mx; fitr.cy = x[1] + my; fitr.rad = sqrtf(x[2] + x[0] * x[0] + x[1] * x[1]);
    fitr.xy = true;
    float prev = 0;
    for (uint8_t k = 0; k < n; k++) {
      float a = pointValue(idx[k]);
      v[k] = (k == 0) ? a : prev + wrap180(a - prev);
      prev = v[k];
    }
  } else {
    for (uint8_t k = 0; k < n; k++) v[k] = ses.pts[idx[k]].a;
  }
  bool hasP = false, hasM = false;
  for (uint8_t k = 0; k < n; k++) { if (ses.pts[idx[k]].side > 0) hasP = true; if (ses.pts[idx[k]].side < 0) hasM = true; }
  fitr.both = hasP && hasM;
  uint8_t p = (fitr.both && n >= 4) ? 3 : 2;
  // centre the values for conditioning
  float vm = 0; for (uint8_t k = 0; k < n; k++) vm += v[k]; vm /= n;
  float M[3][3] = {{0}}, r[3] = {0}, x[3], inv[3][3];
  for (uint8_t k = 0; k < n; k++) {
    float row[3] = {1.0f, v[k] - vm, (float)ses.pts[idx[k]].side};
    float us = ses.pts[idx[k]].us;
    for (uint8_t i = 0; i < p; i++) { r[i] += row[i] * us; for (uint8_t j = 0; j < p; j++) M[i][j] += row[i] * row[j]; }
  }
  if (!solveSym(M, r, p, x, inv)) { if (verbose) Serial.println(F("# fit failed - all points at one value? spread them out")); return false; }
  fitr.uspd = x[1];
  if (fabs(fitr.uspd) < 1e-6f) { if (verbose) Serial.println(F("# fit failed - zero slope")); return false; }
  fitr.us0 = x[0] - x[1] * vm;
  float cs = (p == 3) ? x[2] : 0.0f;
  float ssr = 0; fitr.worst = 0; fitr.worstIdx = -1;
  fitr.vmin = v[0]; fitr.vmax = v[0];
  for (uint8_t k = 0; k < n; k++) {
    float pred = (ses.pts[idx[k]].us - fitr.us0 - cs * ses.pts[idx[k]].side) / fitr.uspd;
    float e = v[k] - pred;
    resid[idx[k]] = e; ssr += e * e;
    if (fabs(e) > fitr.worst) { fitr.worst = fabs(e); fitr.worstIdx = idx[k]; }
    if (v[k] < fitr.vmin) fitr.vmin = v[k];
    if (v[k] > fitr.vmax) fitr.vmax = v[k];
  }
  fitr.p = p;
  fitr.rms = sqrtf(ssr / n);
  fitr.lash = 0; fitr.lashSe = 0; fitr.lashSig = false;
  if (p == 3) {
    float h = -cs / fitr.uspd;                           // half lash, joint units
    float s2 = (n > p) ? ssr * fitr.uspd * fitr.uspd / (n - p) : 0;   // us^2
    float se = sqrtf(fabs(s2 * inv[2][2])) / fabs(fitr.uspd);
    fitr.lash = 2 * h; fitr.lashSe = 2 * se;
    fitr.lashSig = (fitr.lash > 0) && (n > p) && (fitr.lash > 2 * fitr.lashSe);
  }
  fitr.ok = true;
  return true;
}

float exportLash(uint8_t c) {
  if (ses.meta[c].wig >= 0) return ses.meta[c].wig;
  if (fitr.ok && fitr.lashSig) return fitr.lash;
  return 0.0f;
}

void printFit(uint8_t c) {
  Meta &m = ses.meta[c];
  Serial.print(F("# FIT ch ")); Serial.print(c); Serial.print(' '); Serial.print(m.name[0] ? m.name : "-");
  Serial.print(F(" (")); Serial.print(typeName(m.type == 255 ? (uint8_t)J_ROT : m.type));
  if (m.load) Serial.print(F(", gravity-loaded"));
  Serial.print(F("): ")); Serial.print(fitr.n); Serial.print(F(" points"));
  Serial.println(fitr.both ? F(", both sides of the play") : F(""));
  if (fitr.xy) {
    Serial.print(F("#   circle: axis at paper [")); Serial.print(fitr.cx, 2); Serial.print(F(", "));
    Serial.print(fitr.cy, 2); Serial.print(F("] cm, radius ")); Serial.print(fitr.rad, 2);
    Serial.println(F(" cm; angles from paper +x"));
  }
  Serial.print(F("#   us = ")); Serial.print(fitr.us0, 1); Serial.print(F(" + ")); Serial.print(fitr.uspd, 3);
  Serial.print(F(" * ")); Serial.print(unitOf(c)); Serial.print(F("     (")); Serial.print(1.0f / fitr.uspd, 4);
  Serial.print(' '); Serial.print(unitOf(c)); Serial.println(F("/us)"));
  Serial.print(F("#   lash: "));
  if (fitr.p == 3) {
    Serial.print(fitr.lash, 2); Serial.print(F(" +/- ")); Serial.print(fitr.lashSe, 2);
    Serial.print(fitr.lashSig ? F(" (fitted, significant)") : F(" (fitted, not significant - dropped)"));
  } else Serial.print(fitr.both ? F("not fitted (needs at least 4 points with both sides)") : F("not fitted (needs points on both sides of the play)"));
  if (m.wig >= 0) { Serial.print(F("   WIG: ")); Serial.print(m.wig, 2); }
  Serial.print(F("   export: ")); Serial.print(exportLash(c), 2); Serial.print(' '); Serial.println(unitOf(c));
  Serial.print(F("#   residuals: rms ")); Serial.print(fitr.rms, 2); Serial.print(F(", worst ")); Serial.print(fitr.worst, 2);
  Serial.print(F(" (point ")); Serial.print(fitr.worstIdx + 1); Serial.print(F(")   your precision "));
  Serial.print(m.prec, 2); Serial.println(fitr.rms > 2 * m.prec ? F(": TOO MUCH SCATTER") : F(": OK"));
  Serial.print(F("#   points span ")); Serial.print(fitr.vmin, 1); Serial.print(F(" .. ")); Serial.print(fitr.vmax, 1);
  if (m.loUs || m.hiUs) {
    Serial.print(F("; range ends"));
    if (m.loUs) { Serial.print(F(" LO ")); Serial.print((m.loUs - fitr.us0) / fitr.uspd, 1); }
    if (m.hiUs) { Serial.print(F(" HI ")); Serial.print((m.hiUs - fitr.us0) / fitr.uspd, 1); }
  }
  Serial.println();
  // warnings
  if (fitr.n == 2) Serial.println(F("# ! only 2 points: the line is exact and nothing checks it - add a third"));
  if (isRot(c) && (fabs(fitr.uspd) < 5.0f || fabs(fitr.uspd) > 20.0f))
    Serial.println(F("# ! unusual slope (hobby servos are about 10-11.5 us/deg): check units or the link measured"));
  if (fitr.rms > 2 * m.prec)
    Serial.println(F("# ! scatter is larger than your reading precision: re-check the worst point (PTS)"));
  float span = fitr.vmax - fitr.vmin;
  for (uint8_t e = 0; e < 2; e++) {
    uint16_t u = e ? m.hiUs : m.loUs;
    if (!u || span <= 0) continue;
    float ve = (u - fitr.us0) / fitr.uspd;
    if (ve < fitr.vmin - 0.25f * span || ve > fitr.vmax + 0.25f * span) {
      Serial.print(F("# ! extrapolating to the ")); Serial.print(e ? F("HI") : F("LO"));
      Serial.println(F(" end: add a point near it (include the behind-the-back side if the joint goes there)"));
    }
  }
  if (m.load && !fitr.both)
    Serial.println(F("# ! gravity-loaded but only one side tagged: take points on both sides of the balance point, or use WIG"));
  Serial.print(F("FIT,")); Serial.print(ses.arm); Serial.print(','); Serial.print(c); Serial.print(',');
  Serial.print(fitr.n); Serial.print(','); Serial.print(fitr.us0, 1); Serial.print(','); Serial.print(fitr.uspd, 3);
  Serial.print(','); Serial.print(exportLash(c), 2); Serial.print(','); Serial.print(fitr.rms, 2);
  Serial.print(','); Serial.println(fitr.worst, 2);
}

// ------------------------------------------------------------------- export
void printPad(long v, uint8_t w) {
  char b[12]; snprintf(b, sizeof b, "%ld", v);
  for (uint8_t i = strlen(b); i < w; i++) Serial.print(' ');
  Serial.print(b);
}
void printPadF(float v, uint8_t w, uint8_t d) {
  char b[16]; dtostrf(v, w, d, b); Serial.print(b);
}

// Export limits: the found ends pulled in by the margin; else the last LIM set (saved); else the live limits.
void exportLims(uint8_t c, uint16_t &lo, uint16_t &hi) {
  Meta &m = ses.meta[c];
  lo = m.loUs ? m.loUs + m.loMargin : (m.limMin ? m.limMin : chn[c].minUs);
  hi = m.hiUs ? m.hiUs - m.hiMargin : (m.limMax ? m.limMax : chn[c].maxUs);
}

void doExport() {
  Serial.print(F("// ---- BEGIN SERVO_TESTER EXPORT (servo_tester " FW_VERSION ", arm "));
  Serial.print(ses.arm); Serial.print(F(", CFG_FORMAT ")); Serial.print(CFG_FORMAT); Serial.println(F(") ----"));
  int8_t gripCh = -1, xyCh = -1;
  float xyx = 0, xyy = 0;
  bool any = false;
  // comment lines first (method and quality), then the rows
  for (uint8_t c = 0; c < NUM_CH; c++) {
    Meta &m = ses.meta[c];
    uint8_t n = countPts(c);
    if (m.type == J_GRIP) {
      Serial.print(F("// ch")); Serial.print(c); Serial.print(' '); Serial.print(m.name[0] ? m.name : "gripper");
      Serial.print(F(": open ")); Serial.print(m.openUs); Serial.print(F(", hold ")); Serial.println(m.holdUs);
      if (gripCh < 0) gripCh = c;
      continue;
    }
    if (m.type == 255 && n == 0 && !m.loUs && !m.hiUs) continue;
    Serial.print(F("// ch")); Serial.print(c); Serial.print(' '); Serial.print(m.name[0] ? m.name : "-"); Serial.print(F(": "));
    if (!fitChannel(c, false)) { Serial.print(n); Serial.println(F(" pts - NOT MAPPED, no row exported")); continue; }
    Serial.print(fitr.n); Serial.print(F(" pts")); if (fitr.xy) Serial.print(F(" (grid circle)"));
    Serial.print(F(", rms ")); Serial.print(fitr.rms, 2); Serial.print(F(", worst ")); Serial.print(fitr.worst, 2);
    Serial.print(' '); Serial.print(unitOf(c)); Serial.print(F(", span ")); Serial.print(fitr.vmin, 0);
    Serial.print(F("..")); Serial.print(fitr.vmax, 0); Serial.print(F(", lash ")); Serial.print(exportLash(c), 2);
    Serial.print(m.wig >= 0 ? F(" (WIG)") : (fitr.lashSig ? F(" (fitted)") : F(" (not measured)")));
    Serial.println(m.loUs && m.hiUs ? F("") : F("; range ends not marked (LO/HI): soft limits exported"));
    if (fitr.xy && xyCh < 0) { xyCh = c; xyx = fitr.cx; xyy = fitr.cy; }
  }
  Serial.println(F("const JointCal JOINTS[] = {"));
  Serial.println(F("  //  ch  type     US0      USPD    MIN   MAX   LASH  LOAD  name"));
  for (uint8_t c = 0; c < NUM_CH; c++) {
    Meta &m = ses.meta[c];
    if (m.type == J_GRIP) continue;
    if (!fitChannel(c, false)) continue;
    uint16_t lo, hi;
    exportLims(c, lo, hi);
    Serial.print(F("  { ")); printPad(c, 2); Serial.print(m.type == J_LIN ? F(", J_LIN, ") : F(", J_ROT, "));
    printPadF(fitr.us0, 7, 1); Serial.print(F(", ")); printPadF(fitr.uspd, 8, 3); Serial.print(F(", "));
    printPad(lo, 4); Serial.print(F(", ")); printPad(hi, 4); Serial.print(F(", "));
    printPadF(exportLash(c), 5, 2); Serial.print(F(", ")); Serial.print(m.load ? 1 : 0);
    Serial.print(F(", \"")); Serial.print(m.name[0] ? m.name : "j"); Serial.println(F("\" },"));
    any = true;
  }
  Serial.println(F("};"));
  if (!any) Serial.println(F("// (no channel had enough points - the JOINTS[] above is empty)"));
  if (gripCh >= 0) {
    Meta &m = ses.meta[gripCh];
    uint16_t lo, hi;
    exportLims(gripCh, lo, hi);
    Serial.print(F("const GripCal GRIP = { ")); Serial.print(gripCh); Serial.print(F(", "));
    Serial.print(m.openUs); Serial.print(F(", ")); Serial.print(m.holdUs); Serial.print(F(", "));
    Serial.print(lo); Serial.print(F(", ")); Serial.print(hi); Serial.println(F(" };"));
  } else Serial.println(F("// no GRIP channel defined (JT GRIP, SETP OPEN, SETP HOLD)"));
  if (xyCh >= 0) {
    Serial.print(F("// base axis from the PTXY circle fit (paper cm): FRAME "));
    Serial.print(xyx, 1); Serial.print(' '); Serial.print(xyy, 1); Serial.println(F(" 0"));
  }
  Serial.println(F("// ---- END SERVO_TESTER EXPORT ----"));
  fitr.ok = false;
}

void doSession() {
  Serial.print(F("# session: arm ")); Serial.print(ses.arm); Serial.print(F(", margin "));
  Serial.print(ses.margin); Serial.print(F(" us, ")); Serial.print(ses.npts); Serial.print('/');
  Serial.print(MAX_PTS); Serial.println(F(" points"));
  for (uint8_t c = 0; c < NUM_CH; c++) {
    Meta &m = ses.meta[c];
    uint8_t n = countPts(c);
    if (m.type == 255 && !n && !m.loUs && !m.hiUs && !m.openUs) continue;
    Serial.print(F("#  ch ")); Serial.print(c); Serial.print(' '); Serial.print(typeName(m.type));
    Serial.print(' '); Serial.print(m.name[0] ? m.name : "-"); Serial.print(F(" load ")); Serial.print(m.load);
    Serial.print(F(" pts ")); Serial.print(n); Serial.print(F(" LO ")); Serial.print(m.loUs);
    Serial.print(F(" HI ")); Serial.print(m.hiUs);
    if (m.wig >= 0) { Serial.print(F(" WIG ")); Serial.print(m.wig, 2); }
    if (m.type == J_GRIP) { Serial.print(F(" open ")); Serial.print(m.openUs); Serial.print(F(" hold ")); Serial.print(m.holdUs); }
    Serial.println();
  }
}

// ------------------------------------------------------------------- help
void printHelp() {
  Serial.println(F("# --- info ---"));
  Serial.println(F("#  SCAN | INIT (ALL OUTPUTS OFF) | PING | STAT | SESSION"));
  Serial.println(F("# --- setup ---"));
  Serial.println(F("#  ARM id | SEL ch | LIM ch min max | RATE us ms | HOLD ms | WD 0|1"));
  Serial.println(F("# --- motion (selected channel only) ---"));
  Serial.println(F("#  START us | GO us | PROBE us | SWEEP min max step dwell | JOG | R | X"));
  Serial.println(F("#  APPR +|- us [span]  approach us from below (+) or above (-)"));
  Serial.println(F("#  MARK label | OFF [ch] | !  (release all)"));
  Serial.println(F("# --- mapping ---"));
  Serial.println(F("#  JT ROT|LIN|GRIP [name] | LOAD 0|1 | REF deg | SIDE +|-|A | PREC v"));
  Serial.println(F("#  LO | HI (range ends) | MARGIN us | LIMRESTORE"));
  Serial.println(F("#  PT v | PTH h1 h2 L | PTD dx dz | PTXY x y | PTS | PTDEL n | PTCLR"));
  Serial.println(F("#  WIG a b [R] (backlash) | SETP OPEN|HOLD (gripper)"));
  Serial.println(F("#  FIT [ch] | EXPORT | NEW YES (erase the saved session)"));
  Serial.println(F("# any keypress during a move pauses it"));
}

void printStat() {
  Meta &m = ses.meta[sel];
  Serial.print(F("# arm="));   Serial.print(ses.arm);
  Serial.print(F(" ch="));     Serial.print(sel);
  Serial.print(F(" us="));     Serial.print(chn[sel].curUs);
  Serial.print(F(" lim="));    Serial.print(chn[sel].minUs); Serial.print('-'); Serial.print(chn[sel].maxUs);
  Serial.print(F(" rate="));   Serial.print(rateStep); Serial.print(F("us/")); Serial.print(rateMs);
  Serial.print(F("ms hold=")); Serial.print(holdMs);
  Serial.print(F(" wd="));     Serial.print(wdOn ? 1 : 0);
  Serial.print(F(" active=")); Serial.print(chn[sel].active ? 1 : 0);
  Serial.print(F(" type="));   Serial.print(typeName(m.type));
  Serial.print(F(" name="));   Serial.print(m.name[0] ? m.name : "-");
  Serial.print(F(" side="));   Serial.print(m.sideMode == 0 ? F("A") : (m.sideMode > 0 ? F("+") : F("-")));
  Serial.print(F(" pts="));    Serial.print(countPts(sel));
  Serial.print(F(" state="));
  Serial.println(mstate == M_MOVING ? F("MOVING") : mstate == M_PAUSED ? F("PAUSED") : F("IDLE"));
}

// ---------------------------------------------------------------- commands
bool nextInt(long &out) {
  char *t = strtok(NULL, " ");
  if (t == NULL) return false;
  out = atol(t);
  return true;
}
bool nextF(float &out) {
  char *t = strtok(NULL, " ,");
  if (t == NULL) return false;
  out = atof(t);
  return true;
}

void handleMapping(char *cmd);

void handleLine(char *line) {
  char *cmd = strtok(line, " ");
  if (cmd == NULL) return;
  for (char *p = cmd; *p; p++) *p = toupper(*p);

  if (!strcmp(cmd, "HELP") || !strcmp(cmd, "?")) { printHelp(); return; }
  if (!strcmp(cmd, "SCAN")) { scanAndCheck(false); return; }
  if (!strcmp(cmd, "INIT")) { scanAndCheck(true);  return; }
  if (!strcmp(cmd, "!"))    { releaseAll();        return; }
  if (!strcmp(cmd, "HB"))   { return; }
  if (!strcmp(cmd, "SESSION")) { doSession(); return; }
  if (!strcmp(cmd, "EXPORT"))  { doExport(); return; }
  if (!strcmp(cmd, "NEW")) {
    char *t = strtok(NULL, " ");
    if (!t || strcmp(t, "YES")) { Serial.println(F("# NEW YES erases every point and setting saved for this session")); return; }
    clearSession(); saveSession(); Serial.println(F("# session erased (live limits unchanged)")); return;
  }

  if (!pcaOk) { emitErr("NO_DRV", F("run SCAN first")); return; }

  if (!strcmp(cmd, "PING")) { servo.diag(Serial); return; }
  if (!strcmp(cmd, "STAT")) { printStat(); return; }

  if (!strcmp(cmd, "ARM")) {
    char *t = strtok(NULL, " ");
    if (t == NULL) { emitErr("BAD_ARG", F("ARM id")); return; }
    ses.arm = toupper(t[0]); saveSession();
    Serial.print(F("# arm ")); Serial.println(ses.arm);
    return;
  }
  if (!strcmp(cmd, "SEL")) {
    long n;
    if (!nextInt(n) || n < 0 || n >= NUM_CH) { emitErr("BAD_CH", F("0-15")); return; }
    sel = (uint8_t)n; mstate = M_IDLE; sweeping = false; apprPending = false;
    Serial.print(F("# channel ")); Serial.print(sel); Serial.print(' '); Serial.print(typeName(ses.meta[sel].type));
    Serial.print(' '); Serial.println(ses.meta[sel].name[0] ? ses.meta[sel].name : "");
    emitLims(sel);
    return;
  }
  if (!strcmp(cmd, "LIM")) {
    long c, lo, hi;
    if (!nextInt(c) || !nextInt(lo) || !nextInt(hi)) { emitErr("BAD_ARG", F("LIM ch min max")); return; }
    if (c < 0 || c >= NUM_CH) { emitErr("BAD_CH", F("0-15")); return; }
    if (lo < ABS_MIN_US || hi > ABS_MAX_US || lo >= hi) { emitErr("BAD_LIM", F("outside 500-2500 or min>=max")); return; }
    chn[c].minUs = (uint16_t)lo; chn[c].maxUs = (uint16_t)hi;
    ses.meta[c].limMin = (uint16_t)lo; ses.meta[c].limMax = (uint16_t)hi; saveSession();
    emitLims((uint8_t)c);
    return;
  }
  if (!strcmp(cmd, "RATE")) {
    long s, m;
    if (!nextInt(s) || !nextInt(m)) { emitErr("BAD_ARG", F("RATE us_per_step ms_per_step")); return; }
    if (s < 1) s = 1;
    if (s > MAX_RATE_STEP) { s = MAX_RATE_STEP; Serial.println(F("# step clamped to ceiling")); }
    if (m < MIN_RATE_MS)   { m = MIN_RATE_MS;   Serial.println(F("# period clamped to floor")); }
    rateStep = (uint16_t)s; rateMs = (uint16_t)m;
    Serial.print(F("# rate ")); Serial.print(rateStep); Serial.print(F("us/")); Serial.print(rateMs); Serial.println(F("ms"));
    return;
  }
  if (!strcmp(cmd, "HOLD")) {
    long m;
    if (!nextInt(m) || m < 0) { emitErr("BAD_ARG", F("HOLD ms")); return; }
    holdMs = (uint16_t)m; Serial.print(F("# hold ")); Serial.println(holdMs);
    return;
  }
  if (!strcmp(cmd, "WD")) {
    long v;
    if (!nextInt(v)) { emitErr("BAD_ARG", F("WD 0|1")); return; }
    wdOn = (v != 0); Serial.print(F("# watchdog ")); Serial.println(wdOn ? 1 : 0);
    return;
  }
  if (!strcmp(cmd, "START")) {
    long us;
    if (!nextInt(us)) { emitErr("BAD_ARG", F("START us")); return; }
    applyUs(sel, clampToLimits(sel, us));
    chn[sel].dir = 0;
    mstate = M_IDLE;
    emitPos(sel, "START");
    return;
  }
  if (!strcmp(cmd, "GO")) {
    long us;
    if (!nextInt(us)) { emitErr("BAD_ARG", F("GO us")); return; }
    if (!chn[sel].active) { emitErr("NOT_STARTED", F("use START us first")); return; }
    if (us < ABS_MIN_US) us = ABS_MIN_US;
    if (us > ABS_MAX_US) us = ABS_MAX_US;
    startMove((uint16_t)us);
    return;
  }
  if (!strcmp(cmd, "PROBE")) {
    long us;
    if (!nextInt(us)) { emitErr("BAD_ARG", F("PROBE us")); return; }
    applyUs(sel, clampToLimits(sel, us));
    probing = true; probeCh = sel; probeUntil = millis() + PROBE_MS;
    emitPos(sel, "PROBE");
    return;
  }
  if (!strcmp(cmd, "SWEEP")) {
    long lo, hi, st, dw;
    if (!nextInt(lo) || !nextInt(hi) || !nextInt(st) || !nextInt(dw)) { emitErr("BAD_ARG", F("SWEEP min max step dwell_ms")); return; }
    if (!chn[sel].active) { emitErr("NOT_STARTED", F("use START us first")); return; }
    sweepMin = clampToLimits(sel, lo); sweepMax = clampToLimits(sel, hi);
    sweepStep = (st < 1) ? 1 : (uint16_t)st; sweepDwell = (uint16_t)dw;
    sweepHome = chn[sel].curUs; sweepLeg = 0; sweeping = true; dwellUntil = millis();
    Serial.println(F("# sweep started - any key pauses"));
    return;
  }
  if (!strcmp(cmd, "JOG")) {
    if (!chn[sel].active) { emitErr("NOT_STARTED", F("use START us first")); return; }
    jogMode = true;
    Serial.print(F("# jog on ch ")); Serial.print(sel); Serial.print(F(", step ")); Serial.print(jogStep);
    Serial.println(F("us  (+/- 1-4 m q)"));
    return;
  }
  if (!strcmp(cmd, "MARK")) { char *t = strtok(NULL, " "); emitMark(sel, t ? t : "mark"); return; }
  if (!strcmp(cmd, "OFF")) {
    long c;
    uint8_t target = nextInt(c) ? (uint8_t)constrain(c, 0L, (long)NUM_CH - 1) : sel;
    releaseCh(target);
    if (target == sel) { mstate = M_IDLE; apprPending = false; }
    Serial.print(F("# released ch ")); Serial.println(target);
    return;
  }
  if (!strcmp(cmd, "R")) {
    if (mstate == M_PAUSED) { mstate = M_MOVING; nextStepAt = millis(); Serial.println(F("# resumed")); }
    return;
  }
  if (!strcmp(cmd, "X")) { stopMove("X"); return; }
  handleMapping(cmd);
}

void handleMapping(char *cmd) {
  Meta &m = ses.meta[sel];
  float a, b, cR;
  if (!strcmp(cmd, "JT")) {
    char *t = strtok(NULL, " "), *nm = strtok(NULL, " ");
    if (!t) { emitErr("BAD_ARG", F("JT ROT|LIN|GRIP [name]")); return; }
    for (char *p = t; *p; p++) *p = toupper(*p);
    if (!strcmp(t, "ROT")) m.type = J_ROT; else if (!strcmp(t, "LIN")) m.type = J_LIN;
    else if (!strcmp(t, "GRIP")) m.type = J_GRIP; else { emitErr("BAD_ARG", F("JT ROT|LIN|GRIP [name]")); return; }
    if (nm) { strncpy(m.name, nm, NAME_LEN - 1); m.name[NAME_LEN - 1] = 0; }
    saveSession();
    Serial.print(F("# ch ")); Serial.print(sel); Serial.print(F(" is ")); Serial.print(typeName(m.type));
    Serial.print(' '); Serial.println(m.name);
    return;
  }
  if (!strcmp(cmd, "LOAD")) {
    long v; if (!nextInt(v)) { emitErr("BAD_ARG", F("LOAD 0|1")); return; }
    m.load = v ? 1 : 0; saveSession();
    Serial.print(F("# ch ")); Serial.print(sel); Serial.println(m.load ? F(" gravity-loaded") : F(" not gravity-loaded"));
    return;
  }
  if (!strcmp(cmd, "REF")) {
    if (!nextF(a)) { emitErr("BAD_ARG", F("REF deg")); return; }
    m.ref = a; saveSession();
    Serial.print(F("# REF ")); Serial.print(a, 1); Serial.println(F(" deg is subtracted from PTH/PTD readings on this channel"));
    return;
  }
  if (!strcmp(cmd, "SIDE")) {
    char *t = strtok(NULL, " ");
    if (!t) { emitErr("BAD_ARG", F("SIDE +|-|A")); return; }
    m.sideMode = (t[0] == '+') ? 1 : (t[0] == '-') ? -1 : 0; saveSession();
    Serial.print(F("# side ")); Serial.println(m.sideMode == 0 ? F("automatic (from the last motion)") : (m.sideMode > 0 ? F("+") : F("-")));
    return;
  }
  if (!strcmp(cmd, "PREC")) {
    if (!nextF(a) || a <= 0) { emitErr("BAD_ARG", F("PREC v (deg or mm, > 0)")); return; }
    m.prec = a; saveSession(); Serial.print(F("# reading precision ")); Serial.print(a, 2); Serial.print(' '); Serial.println(unitOf(sel));
    return;
  }
  if (!strcmp(cmd, "MARGIN")) {
    long v; if (!nextInt(v) || v < 0 || v > 200) { emitErr("BAD_ARG", F("MARGIN 0-200")); return; }
    ses.margin = (uint16_t)v; saveSession(); Serial.print(F("# margin ")); Serial.print(ses.margin);
    Serial.println(F(" us for the next LO / HI (MARGIN 0 when an end is a tester bound, not a contact)"));
    return;
  }
  if (!strcmp(cmd, "LO") || !strcmp(cmd, "HI")) {
    if (!chn[sel].active) { emitErr("NOT_STARTED", F("the channel must be holding a pulse")); return; }
    bool lo = (cmd[0] == 'L');
    uint16_t u = chn[sel].curUs;
    if (lo) { m.loUs = u; m.loMargin = (uint8_t)ses.margin; } else { m.hiUs = u; m.hiMargin = (uint8_t)ses.margin; }
    if (m.loUs && m.hiUs && m.loUs >= m.hiUs) Serial.println(F("# ! LO is not below HI - LO is the LOW pulse end, whatever the angle does"));
    long lim = lo ? (long)u + ses.margin : (long)u - ses.margin;
    if (lo && lim < chn[sel].maxUs) chn[sel].minUs = (uint16_t)lim;
    if (!lo && lim > chn[sel].minUs) chn[sel].maxUs = (uint16_t)lim;
    saveSession();
    emitMark(sel, lo ? "LO" : "HI");
    emitLims(sel);
    Serial.print(F("# range end recorded; live soft limit pulled in by ")); Serial.print(ses.margin); Serial.println(F(" us"));
    return;
  }
  if (!strcmp(cmd, "LIMRESTORE")) {
    for (uint8_t c = 0; c < NUM_CH; c++) {
      Meta &mm = ses.meta[c];
      if (!mm.loUs && !mm.hiUs) continue;
      uint16_t lo, hi;
      exportLims(c, lo, hi);
      if (lo < hi) { chn[c].minUs = lo; chn[c].maxUs = hi; emitLims(c); }
    }
    return;
  }
  if (!strcmp(cmd, "PT")) {
    if (m.type == J_GRIP) { emitErr("BAD_ARG", F("gripper channel: use SETP OPEN|HOLD")); return; }
    if (!nextF(a)) { emitErr("BAD_ARG", F("PT value")); return; }
    addPoint(K_V, a, 0); return;
  }
  if (!strcmp(cmd, "PTH")) {
    if (!isRot(sel)) { emitErr("BAD_ARG", F("PTH is for rotary joints")); return; }
    if (!nextF(a) || !nextF(b) || !nextF(cR) || cR <= 0) { emitErr("BAD_ARG", F("PTH h1 h2 L  (outer mark h2, L apart along the link)")); return; }
    float s = (b - a) / cR;
    if (fabs(s) > 1.0f) { emitErr("BAD_ARG", F("|h2 - h1| is more than L - check the readings")); return; }
    float ang = asinf(s) * RAD_TO_DEG;
    if (fabs(s) > 0.94f) Serial.println(F("# ! within 20 deg of vertical: heights are imprecise here, use PTD"));
    Serial.print(F("# link angle ")); Serial.print(ang, 1); Serial.print(F(" deg; 1 mm of height error = about "));
    Serial.print(57.3f * 1.414f / (cR * cosf(ang * DEG_TO_RAD)), 2); Serial.println(F(" deg (if h and L are in mm)"));
    addPoint(K_H, ang - m.ref, 0); return;
  }
  if (!strcmp(cmd, "PTD")) {
    if (!isRot(sel)) { emitErr("BAD_ARG", F("PTD is for rotary joints")); return; }
    if (!nextF(a) || !nextF(b) || (a == 0 && b == 0)) { emitErr("BAD_ARG", F("PTD dx dz  (outer mark minus inner mark: outward +, up +)")); return; }
    float ang = atan2f(b, a) * RAD_TO_DEG;          // class Atan2(x = dx, y = dz)
    Serial.print(F("# link angle ")); Serial.print(ang, 1); Serial.print(F(" deg; 1 mm of error = about "));
    Serial.print(57.3f * 1.414f / sqrtf(a * a + b * b), 2); Serial.println(F(" deg (if dx, dz are in mm)"));
    addPoint(K_D, wrap180(ang - m.ref), 0); return;
  }
  if (!strcmp(cmd, "PTXY")) {
    if (!isRot(sel)) { emitErr("BAD_ARG", F("PTXY is for rotary joints")); return; }
    if (!nextF(a) || !nextF(b)) { emitErr("BAD_ARG", F("PTXY x y  (tip position on the grid, cm)")); return; }
    addPoint(K_XY, a, b); return;
  }
  if (!strcmp(cmd, "PTS")) { listPoints(sel); return; }
  if (!strcmp(cmd, "PTDEL")) {
    long n; if (!nextInt(n) || n < 1 || n > ses.npts) { emitErr("BAD_ARG", F("PTDEL n  (number from PTS)")); return; }
    deletePoint((uint8_t)(n - 1)); saveSession(); fitr.ok = false;
    Serial.print(F("# point ")); Serial.print(n); Serial.println(F(" deleted (later points renumber)"));
    return;
  }
  if (!strcmp(cmd, "PTCLR")) {
    for (int i = ses.npts - 1; i >= 0; i--) if (ses.pts[i].ch == sel) deletePoint((uint8_t)i);
    saveSession(); fitr.ok = false;
    Serial.print(F("# points of ch ")); Serial.print(sel); Serial.println(F(" cleared"));
    return;
  }
  if (!strcmp(cmd, "APPR")) {
    char *t = strtok(NULL, " ");
    long us, span = 60;
    if (!t || (t[0] != '+' && t[0] != '-') || !nextInt(us)) { emitErr("BAD_ARG", F("APPR +|- us [span]")); return; }
    nextInt(span);
    if (!chn[sel].active) { emitErr("NOT_STARTED", F("use START us first")); return; }
    uint16_t tgt = clampToLimits(sel, us);
    long first = (t[0] == '+') ? (long)tgt - span : (long)tgt + span;
    apprTarget = tgt; apprPending = true;
    startMove(clampToLimits(sel, first));
    if (mTarget == tgt) Serial.println(F("# ! the approach start is clamped onto the target by the soft limits - no known side"));
    Serial.print(F("# approaching ")); Serial.print(tgt); Serial.println(t[0] == '+' ? F(" from below") : F(" from above"));
    return;
  }
  if (!strcmp(cmd, "WIG")) {
    if (!nextF(a) || !nextF(b)) { emitErr("BAD_ARG", F("WIG a b [R]")); return; }
    float lash;
    if (isRot(sel)) {
      if (!nextF(cR) || cR <= 0) { emitErr("BAD_ARG", F("rotary joint: WIG a b R  (R = distance from the axis, same units)")); return; }
      lash = fabs(b - a) / cR * RAD_TO_DEG;
    } else lash = fabs(b - a);
    m.wig = lash; saveSession();
    Serial.print(F("# backlash ")); Serial.print(lash, 2); Serial.print(' '); Serial.print(unitOf(sel));
    Serial.println(F(" (WIG) - EXPORT uses this value"));
    return;
  }
  if (!strcmp(cmd, "SETP")) {
    char *t = strtok(NULL, " ");
    if (m.type != J_GRIP) { emitErr("BAD_ARG", F("JT GRIP first")); return; }
    if (!t || !chn[sel].active) { emitErr("BAD_ARG", F("SETP OPEN|HOLD, with the channel holding a pulse")); return; }
    if (toupper(t[0]) == 'O') m.openUs = chn[sel].curUs; else m.holdUs = chn[sel].curUs;
    saveSession(); emitMark(sel, toupper(t[0]) == 'O' ? "OPEN" : "HOLD");
    return;
  }
  if (!strcmp(cmd, "FIT")) {
    long c = sel; nextInt(c);
    if (c < 0 || c >= NUM_CH) { emitErr("BAD_CH", F("0-15")); return; }
    if (ses.meta[c].type == J_GRIP) { Serial.println(F("# gripper channel: nothing to fit (SESSION shows open/hold)")); return; }
    if (fitChannel((uint8_t)c, true)) printFit((uint8_t)c);
    return;
  }
  Serial.print(F("# unknown command: ")); Serial.println(cmd);
}

// ------------------------------------------------------------- jog handling
void handleJogKey(char k) {
  switch (k) {
    case '+': case '=': applyUs(sel, clampToLimits(sel, (long)chn[sel].curUs + jogStep)); emitPos(sel, "JOG"); break;
    case '-': case '_': applyUs(sel, clampToLimits(sel, (long)chn[sel].curUs - jogStep)); emitPos(sel, "JOG"); break;
    case '1': jogStep = 1;  Serial.println(F("# step 1us"));  break;
    case '2': jogStep = 5;  Serial.println(F("# step 5us"));  break;
    case '3': jogStep = 10; Serial.println(F("# step 10us")); break;
    case '4': jogStep = 25; Serial.println(F("# step 25us")); break;
    case 'm': case 'M': emitMark(sel, "jog"); break;
    case '!': releaseAll(); jogMode = false; break;
    case 'q': case 'Q': jogMode = false; Serial.println(F("# jog off")); break;
    default: break;
  }
}

// -------------------------------------------------------------- serial read
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    lastRx = millis();
    if (jogMode) { handleJogKey(c); continue; }
    if (mstate == M_MOVING) {                 // any keypress pauses
      while (Serial.available()) Serial.read();
      mstate = M_PAUSED;
      Serial.println(F("# paused - holding (R to resume, X to stop)"));
      emitPos(sel, "PAUSED");
      blen = 0;
      return;
    }
    if (c == '\n' || c == '\r') {
      if (blen > 0) { buf[blen] = '\0'; blen = 0; handleLine(buf); }
      continue;
    }
    if (blen < sizeof(buf) - 1) buf[blen++] = c;
    lastByte = millis();
  }
  if (blen > 0 && millis() - lastByte > LINE_IDLE_MS) { buf[blen] = '\0'; blen = 0; handleLine(buf); }
}

// ------------------------------------------------------------- main service
void service() {
  uint32_t now = millis();
  if (probing && now >= probeUntil) { releaseCh(probeCh); probing = false; Serial.println(F("# probe released")); }
  if (mstate == M_MOVING) {
    if (wdOn && (now - lastRx) > WD_MS) {
      mstate = M_PAUSED; sweeping = false;
      Serial.println(F("# watchdog: no traffic, paused"));
      emitPos(sel, "PAUSED");
    } else if (now >= nextStepAt) {
      long d = (long)mTarget - (long)chn[sel].curUs;
      if (d == 0) {
        mstate = M_IDLE;
        emitPos(sel, "IDLE");
        if (sweeping) dwellUntil = now + sweepDwell;
        if (apprPending) {
          apprPending = false;
          if (chn[sel].curUs != apprTarget) startMove(apprTarget);
          else Serial.println(F("# at the target"));
        } else if (!sweeping) {
          Serial.print(F("# at ")); Serial.print(chn[sel].curUs);
          Serial.print(F(" us, side of play if measured now: "));
          int8_t sd = ses.meta[sel].sideMode ? ses.meta[sel].sideMode : -chn[sel].dir;
          Serial.println(sd > 0 ? F("+") : sd < 0 ? F("-") : F("unknown"));
        }
      } else {
        long s = d;
        if (s >  (long)rateStep) s =  (long)rateStep;
        if (s < -(long)rateStep) s = -(long)rateStep;
        applyUs(sel, (uint16_t)((long)chn[sel].curUs + s));
        nextStepAt = now + rateMs;
        if (++stepsSincePrint >= 5) { stepsSincePrint = 0; emitPos(sel, "MOVING"); }
      }
    }
  }
  if (sweeping && mstate == M_IDLE && now >= dwellUntil) advanceSweep();
  if (holdMs > 0 && mstate != M_MOVING) {
    for (uint8_t i = 0; i < NUM_CH; i++) {
      if (chn[i].active && (now - chn[i].lastChange) > holdMs) {
        releaseCh(i);
        Serial.print(F("# hold timeout, released ch ")); Serial.println(i);
      }
    }
  }
}

// --------------------------------------------------------------- setup/loop
void setup() {
#if defined(__AVR__) && defined(MCUSR)
  uint8_t mcusr = MCUSR;
  MCUSR = 0;
#endif
  Serial.begin(115200);
  Wire.begin();
  for (uint8_t i = 0; i < NUM_CH; i++) {
    chn[i].minUs = DEF_MIN_US; chn[i].maxUs = DEF_MAX_US; chn[i].curUs = CENTER_US;
    chn[i].active = false; chn[i].dir = 0; chn[i].lastChange = 0;
  }
  delay(200);
#if defined(__AVR__) && defined(MCUSR)
  Serial.print(F("# MCUSR=0x")); Serial.print(mcusr, HEX); Serial.print(F(" ->"));
  if (mcusr & (1 << PORF))  Serial.print(F(" power-on"));
  if (mcusr & (1 << EXTRF)) Serial.print(F(" external/DTR"));
  if (mcusr & (1 << BORF))  Serial.print(F(" brownout"));
  if (mcusr & (1 << WDRF))  Serial.print(F(" watchdog"));
  if (mcusr == 0)           Serial.print(F(" none, or bootloader cleared it"));
  Serial.println();
#endif
  Serial.print(F("# READY servo_tester v" FW_VERSION ", driver ")); Serial.println(servo.name());
  if (loadSession()) {
    Serial.print(F("# saved mapping session restored: arm ")); Serial.print(ses.arm); Serial.print(F(", "));
    Serial.print(ses.npts); Serial.println(F(" points (SESSION for details, NEW YES to erase)"));
    Serial.println(F("# saved range ends are NOT applied to the live limits - LIMRESTORE applies them"));
  } else {
    Serial.println(F("# no saved mapping session - starting a new one"));
    saveSession();
  }
  scanAndCheck(false);
  Serial.println(F("# idle channels stay off until START - type HELP for commands"));
  lastRx = millis();
}

void loop() {
  readSerial();
  service();
}
