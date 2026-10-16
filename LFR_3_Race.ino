/*
  ============================================================================
   LFR_3_Race.ino  —  Roboveda'26 "Lakshmanarekha" race firmware
   Hardware : Techgeeks Blueprint 01 (Arduino Nano + TB6612FNG) + ARC-16
  ============================================================================

   BUTTONS (bot sitting idle)
     LB  (left button)  : auto-calibrate. Place bot CENTERED on a straight
                          line first. It sweeps left/right over the line,
                          saves to EEPROM, then re-centres. LED1 on while busy.
     RB  short press    : run SAFE profile  (qualifier / first runs)
     RB  hold > 1 s     : run RACE profile  (LED1 lights while held = RACE armed)
     LB  during a run   : stop (practice only — touching = penalty in competition)

   LEDS
     3 quick blinks at power-on : calibration loaded from EEPROM, ready
     LED1/LED2 alternating      : NOT calibrated — press LB first
     LED2 solid                 : running
     LED1+LED2 slow blink       : HOLD (finished / lifted / lost) — put it back
                                  on the line and it resumes by itself

   SENSOR CONVENTION
     ARC-16 index 0 = RIGHTMOST, 15 = LEFTMOST (Techgeeks manual).
     Position 0 … 15000, centre 7500. error > 0  → line is to the LEFT.

   Everything you are expected to tune is in the "USER SETTINGS" block.
  ============================================================================
*/
#include <EEPROM.h>

// ============================== PINS (Blueprint 01) ==========================
#define PWMA   3     // left motor speed   (Motor A = LEFT)
#define AIN2   4
#define AIN1   5
#define STDBY  6
#define BIN1   7
#define BIN2   8
#define PWMB   9     // right motor speed  (Motor B = RIGHT)
#define LED1   11
#define LED2   12
#define BTN_L  2
#define BTN_R  10
// ARC-16 on the sensor rail: S0..S3 -> A0..A3, E -> A4, SIG -> A5
#define MUX_EN A4
#define SIG    A5

// ============================== USER SETTINGS ===============================
// ---- hardware orientation (fix in code instead of rewiring) ----
#define SWAP_MOTORS         1   // 1 = Motor A terminal drives the RIGHT wheel (left/right swapped)
#define INVERT_LEFT_MOTOR   0   // 1 if left wheel spins backwards on "forward"
#define INVERT_RIGHT_MOTOR  0
#define SENSOR_REVERSED     0   // 1 if sensor 0 is on the LEFT side of your bot
const float LEFT_GAIN  = 1.00;  // trim if the bot drifts on a straight (e.g. 0.96)
const float RIGHT_GAIN = 1.00;

// ---- speed/feel profiles ----
struct Profile {
  int   base;      // PWM on a straight
  int   minSpeed;  // PWM floor in sharp curves
  int   maxPwm;    // ceiling for either wheel
  int   revMax;    // how hard the inner wheel may reverse
  float kp;        // proportional gain  (per position unit, range ±7500)
  float kd;        // derivative gain    (per 2 ms loop)
  int   turnPwm;   // pivot speed when searching for the line
  int   brakePwm;  // reverse pulse before a pivot
};
//                    base minSp maxPwm revMax   kp     kd   turn brake
Profile SAFE_PROF = { 130,  70,   210,   120,  0.030, 0.90, 150, 120 };
Profile RACE_PROF = { 200,  95,   255,   170,  0.045, 1.40, 190, 180 };

// ---- track logic ----
#define LEFT   1
#define RIGHT -1
#define TURN_BIAS        RIGHT  // side taken at a T or Y when PLAN gives none
const char PLAN[] = "";         // optional per-decision plan, e.g. "RRL" (see doc)
#define LINE_W           3      // sensors covered by the 3 cm line (measure: sketch 2)
#define THRESH           500    // normalised black threshold (0..1000)
#define ALLBLACK_N       10     // this many sensors black counts toward the END tile
#define DARK_N           9      // this many black = hold course, no branch decisions
#define WIDE_N           (LINE_W + 3)
#define HINT_MS          250    // how long a seen side branch stays "fresh"
#define T_WINDOW_MS      80     // L and R branches seen this close together = T
#define GAP_ENABLE       1
#define GAP_ERR          1500   // only treat as a gap if we were nearly centred
#define GAP_MS           120    // drive straight this long looking for the line
#define LOST_TIMEOUT_MS  1200   // searching longer than this -> HOLD
#define BRAKE_MS         25     // reverse pulse length before a pivot
#define BRAKE_ABOVE      120    // only brake if we were faster than this PWM
#define START_IGNORE_MS  800    // ignore all-black while leaving the START pad
#define END_DIST_MM      45     // dark patch this long (mm travelled) = END tile.
                                // A 3 cm cross-line reads ~23 mm, a 10 cm END block ~83 mm.
const float TOP_SPEED_MPS = 1.2;  // bot speed at PWM 255 — measure it (doc, step 2)
#define RESUME_ARM_MS    1000   // in HOLD: must see nothing/all-black this long...
#define RESUME_MS        600    // ...then a clean centred line this long to resume
#define AUTO_RESUME      1
#define LOOP_DETECT      1
#define CURVE_RATIO      0.18   // steering ratio above this = "on a curve" (see doc)
#define LOOP_MM          100    // curving one way this far (mm) = "on a loop" (half of an 8 cm-radius circle is 250 mm)
#define DECISION_HOLD_MS 150
#define RAMP_MS          250    // soft start
#define START_DELAY_MS   800    // after releasing RB, take your hand away
#define RB_LONG_MS       1000

// ---- timing / low level ----
#define LOOP_US          2000   // control loop period (500 Hz)
#define MUX_SETTLE_US    6
#define FAST_ADC         1      // ADC prescaler 32 (~30 us/read)
#define HIGH_PWM_FREQ    1      // ~31 kHz PWM on pins 3 and 9 (quiet, smooth)

// ---- optional battery compensation (needs a divider into A6, see doc) ----
#define USE_VBAT         0
#define VBAT_PIN         A6
const float VBAT_DIVIDER = 2.0;  // (R1+R2)/R2, e.g. 10k/10k
const float VBAT_NOMINAL = 7.8;  // pack voltage you tuned at

#define DEBUG_SERIAL     0       // 1 = print state over USB (bench only)
// ============================================================================

// ------------------------------- calibration --------------------------------
struct CalData {
  uint16_t magic;
  uint8_t  blackHigh;
  int16_t  mn[16];
  int16_t  mx[16];
};
const uint16_t CAL_MAGIC = 0xA16C;
#define MIN_SPAN 120
CalData cal;
uint32_t scaleQ10[16];   // (1000 << 10) / span ; 0 = dead sensor
bool calibrated = false;

// ------------------------------- sensor state -------------------------------
int16_t  raw[16];
int16_t  norm[16];
uint16_t mask;
uint8_t  nActive;
uint8_t  segS[8], segE[8], nSeg;
int32_t  segC[8];
#define CENTER_MASK 0x03C0   // sensors 6..9

// ------------------------------- control state ------------------------------
enum State : uint8_t { ST_IDLE, ST_FOLLOW, ST_GAP, ST_RECOVER, ST_HOLD };
State state = ST_IDLE;
Profile *P = &SAFE_PROF;

int32_t  lastPos = 7500;
int32_t  lastErr = 0;
float    dFilt = 0, errFilt = 0, steerF = 0;
int      lastSpeed = 0;
uint32_t runStartMs = 0, rampStartMs = 0;

uint32_t hintLms = 0, hintRms = 0;       // when a side branch was last seen
int8_t   decisionSide = 0;               // locked choice at a fork / loop exit
uint32_t decisionMs = 0;
uint8_t  decisionIdx = 0;
uint32_t lastDecisionMs = 0;
int8_t   lastDecision = 0;

bool     loopMode = false;
int8_t   loopOut = 0;
int8_t   curveSign = 0;
float    curveMm = 0, straightMm = 0;

int8_t   recDir = 0;
uint32_t lostMs = 0, brakeUntil = 0, gapUntil = 0;
float    blackMm = 0;

uint32_t holdArmMs = 0, resumeMs = 0;
bool     holdArmed = false;

float    vbatGain = 1.0;

// forward declarations (keeps every Arduino IDE version happy)
int32_t centroid(int8_t a, int8_t b);
void    lineLost(uint32_t ms);
void    stepFollow(uint32_t ms);

// =============================== LOW LEVEL ==================================
static inline int readRaw(uint8_t ch) {
  PORTC = (PORTC & 0xF0) | (ch & 0x0F);   // A0..A3 = S0..S3 at once, A4 (E) stays LOW
  delayMicroseconds(MUX_SETTLE_US);
  return analogRead(SIG);
}

void readAllRaw() {
  for (uint8_t i = 0; i < 16; i++) {
    int v = readRaw(i);
#if SENSOR_REVERSED
    raw[15 - i] = v;
#else
    raw[i] = v;
#endif
  }
}

void computeScales() {
  for (uint8_t i = 0; i < 16; i++) {
    int span = cal.mx[i] - cal.mn[i];
    scaleQ10[i] = (span >= MIN_SPAN) ? ((1000UL << 10) / (uint32_t)span) : 0;
  }
}

// Reads all sensors, normalises (1000 = black), builds mask and segments.
void readLine() {
  readAllRaw();
  mask = 0; nActive = 0;
  for (uint8_t i = 0; i < 16; i++) {
    int16_t n = 0;
    if (scaleQ10[i]) {
      int32_t v = raw[i] - cal.mn[i];
      int32_t span = cal.mx[i] - cal.mn[i];
      if (v < 0) v = 0;
      if (v > span) v = span;
      n = (int16_t)(((uint32_t)v * scaleQ10[i]) >> 10);
      if (n > 1000) n = 1000;
      if (!cal.blackHigh) n = 1000 - n;
    }
    norm[i] = n;
    if (n > THRESH) { mask |= (1u << i); nActive++; }
  }
  // contiguous runs of active sensors
  nSeg = 0;
  int8_t start = -1;
  for (int8_t i = 0; i <= 16; i++) {
    bool on = (i < 16) && (mask & (1u << i));
    if (on && start < 0) start = i;
    if (!on && start >= 0) {
      if (nSeg < 8) {
        segS[nSeg] = start; segE[nSeg] = i - 1;
        segC[nSeg] = centroid(start - 1, i);   // include one neighbour each side
        nSeg++;
      }
      start = -1;
    }
  }
}

// Weighted centroid of sensors a..b (clamped), in position units (0..15000).
int32_t centroid(int8_t a, int8_t b) {
  if (a < 0) a = 0;
  if (b > 15) b = 15;
  int32_t num = 0, den = 0;
  for (int8_t i = a; i <= b; i++) { num += (int32_t)norm[i] * i * 1000L; den += norm[i]; }
  return den ? num / den : 7500;
}

int32_t windowCentroid(int8_t c, int8_t w) { return centroid(c - w, c + w); }

void setMotor(uint8_t in1, uint8_t in2, uint8_t pwm, int v) {
  if (v > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  analogWrite(pwm, v); }
  else if (v < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); analogWrite(pwm, -v); }
  else            { digitalWrite(in1, HIGH); digitalWrite(in2, HIGH); analogWrite(pwm, 0); } // brake
}

void drive(int l, int r) {
  l = (int)(l * LEFT_GAIN * vbatGain);
  r = (int)(r * RIGHT_GAIN * vbatGain);
#if INVERT_LEFT_MOTOR
  l = -l;
#endif
#if INVERT_RIGHT_MOTOR
  r = -r;
#endif
  l = constrain(l, -255, 255);
  r = constrain(r, -255, 255);
#if SWAP_MOTORS
  setMotor(AIN1, AIN2, PWMA, r);
  setMotor(BIN1, BIN2, PWMB, l);
#else
  setMotor(AIN1, AIN2, PWMA, l);
  setMotor(BIN1, BIN2, PWMB, r);
#endif
}

void updateVbat() {
#if USE_VBAT
  static uint32_t t = 0;
  static float vf = VBAT_NOMINAL;
  if (millis() - t < 20) return;
  t = millis();
  float v = analogRead(VBAT_PIN) * (5.0 / 1023.0) * VBAT_DIVIDER;
  vf += (v - vf) * 0.1;
  vbatGain = constrain(VBAT_NOMINAL / vf, 0.8, 1.3);
#endif
}

bool pressed(uint8_t pin) { return digitalRead(pin) == LOW; }

void blink(uint8_t pin, uint8_t times, uint16_t ms) {
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(pin, HIGH); delay(ms); digitalWrite(pin, LOW); delay(ms);
  }
}

// ============================== CALIBRATION =================================
bool loadCal() {
  EEPROM.get(0, cal);
  if (cal.magic != CAL_MAGIC) return false;
  computeScales();
  return true;
}

void saveCal() { cal.magic = CAL_MAGIC; EEPROM.put(0, cal); }

void sampleMinMax() {
  readAllRaw();
  for (uint8_t i = 0; i < 16; i++) {
    if (raw[i] < cal.mn[i]) cal.mn[i] = raw[i];
    if (raw[i] > cal.mx[i]) cal.mx[i] = raw[i];
  }
}

int16_t startRaw[16];      // each sensor's reading when calibration starts
#define SWEEP_DELTA 150     // "on the line" = this far past the sensor's own start value

bool onLine(uint8_t i, bool blackHigh) {
  return blackHigh ? raw[i] > startRaw[i] + SWEEP_DELTA : raw[i] < startRaw[i] - SWEEP_DELTA;
}

// Pivot `dir` until sensor `target` reaches the line, or timeout.
void sweepUntil(int8_t dir, uint8_t target, bool blackHigh, uint16_t timeoutMs) {
  const int CAL_PWM = 95;
  uint32_t t0 = millis();
  drive(-dir * CAL_PWM, dir * CAL_PWM);
  while (millis() - t0 < timeoutMs) {
    sampleMinMax();
    if (onLine(target, blackHigh) && millis() - t0 > 120) break;
  }
  drive(0, 0);
  delay(60);
}

bool calibrateAuto() {
  digitalWrite(LED1, HIGH);
  delay(400);
  readAllRaw();
  int c = (raw[7] + raw[8]) / 2;
  int e = (raw[0] + raw[1] + raw[14] + raw[15]) / 4;
  if (abs(c - e) < 80) {                      // not sitting on the line
    digitalWrite(LED1, LOW);
    blink(LED2, 6, 80);
    return false;
  }
  bool bh = c > e;
  for (uint8_t i = 0; i < 16; i++) { cal.mn[i] = 1023; cal.mx[i] = 0; startRaw[i] = raw[i]; }
  for (uint8_t i = 5; i <= 10; i++) startRaw[i] = e;   // centre sensors start on the line

  // Pivot LEFT -> line moves toward sensor 0 ; pivot RIGHT -> toward sensor 15
  sweepUntil(LEFT,  0,  bh, 2500);
  sweepUntil(RIGHT, 15, bh, 3000);
  sweepUntil(LEFT,  0,  bh, 3000);
  sweepUntil(RIGHT, 15, bh, 3000);
  // come back to centre, using the min/max just learned
  {
    uint32_t t0 = millis();
    drive(-80, 80);
    while (millis() - t0 < 2500) {
      readAllRaw();
      int m7 = (cal.mn[7] + cal.mx[7]) / 2, m8 = (cal.mn[8] + cal.mx[8]) / 2;
      bool black7 = bh ? (raw[7] > m7) : (raw[7] < m7);
      bool black8 = bh ? (raw[8] > m8) : (raw[8] < m8);
      if (black7 || black8) break;
    }
    drive(0, 0);
  }

  uint8_t bad = 0;
  for (uint8_t i = 0; i < 16; i++) if (cal.mx[i] - cal.mn[i] < MIN_SPAN) bad++;
  cal.blackHigh = bh;
  digitalWrite(LED1, LOW);
  if (bad > 2) { blink(LED2, 10, 60); return false; }
  saveCal();
  computeScales();
  calibrated = true;
  blink(LED1, 3, 100);
  return true;
}

// ============================== DECISIONS ===================================
int8_t nextDecision(uint32_t ms) {
  if (lastDecision && ms - lastDecisionMs < 300) return lastDecision;  // same junction
  char c = (decisionIdx < sizeof(PLAN) - 1) ? PLAN[decisionIdx] : 0;
  decisionIdx++;
  int8_t d = (c == 'L') ? LEFT : (c == 'R') ? RIGHT : TURN_BIAS;
  lastDecision = d; lastDecisionMs = ms;
  return d;
}

void resetControl(uint32_t ms) {
  lastPos = 7500; lastErr = 0; dFilt = 0; errFilt = 0; steerF = 0;
  hintLms = hintRms = 0; decisionSide = 0;
  loopMode = false; curveMm = 0; straightMm = 0; curveSign = 0;
  lostMs = 0; blackMm = 0;
  rampStartMs = ms;
}

// ============================== CONTROL =====================================
void runPID(int32_t pos, uint32_t ms) {
  int32_t err = pos - 7500;
  float d = (float)(err - lastErr);
  dFilt = 0.5f * dFilt + 0.5f * d;
  lastErr = err;
  errFilt += (err - errFilt) * 0.05f;

  float corr = P->kp * err + P->kd * dFilt;

  int base = P->base;
  uint32_t since = ms - rampStartMs;
  if (since < RAMP_MS) base = P->minSpeed + (long)(P->base - P->minSpeed) * since / RAMP_MS;

  long sp = base - (long)(base - P->minSpeed) * labs(err) / 7500;
  if (sp < P->minSpeed) sp = P->minSpeed;

  int l = constrain((int)(sp - corr), -P->revMax, P->maxPwm);
  int r = constrain((int)(sp + corr), -P->revMax, P->maxPwm);
  drive(l, r);
  lastSpeed = sp;

#if LOOP_DETECT
  // steering ratio: 0 = straight, ~wheelbase/diameter on a circle, + = turning left
  int sum = abs(r) + abs(l);
  float ratio = sum ? (float)(r - l) / (float)sum : 0.0f;
  steerF += (ratio - steerF) * 0.05f;
  int8_t s = (steerF > CURVE_RATIO) ? 1 : (steerF < -CURVE_RATIO) ? -1 : 0;
  uint16_t dt = LOOP_US / 1000;
  if (s != 0 && s == curveSign) {
    if (curveMm < 5000) curveMm += sp * vbatGain * (TOP_SPEED_MPS / 255.0f) * dt;
  } else {
    curveSign = s; curveMm = 0;
  }
  if (curveMm > LOOP_MM) { loopMode = true; loopOut = (curveSign > 0) ? RIGHT : LEFT; }
  if (fabs(steerF) < CURVE_RATIO / 2) {
    if (straightMm < 5000) straightMm += sp * vbatGain * (TOP_SPEED_MPS / 255.0f) * dt;
    if (straightMm > 60) loopMode = false;
  } else straightMm = 0;
#endif
}

void enterHold() {
  drive(0, 0);
  state = ST_HOLD;
  holdArmed = false; holdArmMs = 0; resumeMs = 0;
  digitalWrite(LED2, LOW);
}

void finishRun() {
  drive(-P->brakePwm, -P->brakePwm);   // kill momentum on the END block
  delay(40);
  enterHold();
}

void enterRecover(int8_t dir, uint32_t ms) {
  state = ST_RECOVER;
  recDir = dir;
  if (!lostMs) lostMs = ms;
  brakeUntil = (lastSpeed > BRAKE_ABOVE) ? ms + BRAKE_MS : ms;
}

void lineLost(uint32_t ms) {
  lostMs = ms;
  int8_t dir = 0;
  bool hl = hintLms && ms - hintLms < HINT_MS;
  bool hr = hintRms && ms - hintRms < HINT_MS;
  if (decisionSide) dir = decisionSide;
  else if (hl && hr) {
    long gap = (long)hintLms - (long)hintRms;
    if (labs(gap) < T_WINDOW_MS) dir = nextDecision(ms);   // T-junction
    else dir = (hintLms > hintRms) ? LEFT : RIGHT;          // most recent branch
  }
  else if (hl) dir = LEFT;
  else if (hr) dir = RIGHT;

#if GAP_ENABLE
  if (!dir && labs(lastErr) < GAP_ERR) {
    state = ST_GAP; gapUntil = ms + GAP_MS;
    drive(P->minSpeed + 20, P->minSpeed + 20);
    return;
  }
#endif
  if (!dir) dir = (lastErr > 0) ? LEFT : RIGHT;
  enterRecover(dir, ms);
}

void stepFollow(uint32_t ms) {
  // ---- nothing seen ----
  if (nSeg == 0) { lineLost(ms); return; }

  // ---- which segment is "our" line, and are there branches? ----
  uint8_t tk = 0;
  int32_t best = 1L << 30;
  for (uint8_t k = 0; k < nSeg; k++) {
    int32_t dd = labs(segC[k] - lastPos);
    if (dd < best) { best = dd; tk = k; }
  }
  bool brL = false, brR = false;
  for (uint8_t k = 0; k < nSeg; k++) {
    if (k == tk) continue;
    if (segC[k] > segC[tk]) brL = true; else brR = true;
  }
  uint8_t width = segE[tk] - segS[tk] + 1;
  bool wide = width >= WIDE_N;
  int8_t tci = constrain((lastPos + 500) / 1000, segS[tk], segE[tk]);
  if (wide) {
    if (segE[tk] - tci > LINE_W) brL = true;
    if (tci - segS[tk] > LINE_W) brR = true;
  }
  if (brL) hintLms = ms;
  if (brR) hintRms = ms;

  // ---- dark patch: START pad, a crossing, a T-bar or the END block ----
  // Hold course across it (no steering toward branches); the hints above
  // still record which sides it reached, for a corner or T just after it.
  if (nActive >= DARK_N) {
    if (nActive >= ALLBLACK_N && ms - runStartMs > START_IGNORE_MS) {
      // distance travelled over the dark patch, estimated from the PWM we command
      blackMm += lastSpeed * vbatGain * (TOP_SPEED_MPS / 255.0f) * (LOOP_US / 1000.0f);
      if (blackMm >= END_DIST_MM) { finishRun(); return; }
    }
    lostMs = 0;
    runPID(lastPos, ms);
    return;
  }
  blackMm = 0;

  // ---- fork or T: nothing straight ahead, branches on BOTH sides ----
  // A curved array sees a T-bar as two blobs at its sides. Use the plan/bias
  // when both blobs are far out (a T) or about equally far (a Y). Otherwise
  // (e.g. arriving at a merge) the straighter branch is followed.
  if (!decisionSide && nSeg >= 2 && best > 1200) {
    int32_t dl = 0, dr = 0;                       // nearest blob each side of our HEADING
    for (uint8_t k = 0; k < nSeg; k++) {
      int32_t dd = segC[k] - 7500;
      if (dd > 0 && (!dl || dd < dl)) dl = dd;
      if (dd <= 0 && (!dr || -dd < dr)) dr = -dd + 1;
    }
    if (dl && dr) {
      if ((dl > 3000 && dr > 3000) || labs(dl - dr) < 1200) {           // T or Y
        decisionSide = nextDecision(ms); decisionMs = ms;
      } else {                                                          // follow the straighter one
        int32_t want = (dl < dr) ? 7500 + dl : 7500 - (dr - 1);
        for (uint8_t k = 0; k < nSeg; k++) if (segC[k] == want) tk = k;
        wide = (segE[tk] - segS[tk] + 1) >= WIDE_N;
        tci = constrain((want + 500) / 1000, segS[tk], segE[tk]);
      }
    }
  }
  // ---- leaving a loop: take the first branch on the OUTSIDE ----
  if (!decisionSide && loopMode && ((loopOut == LEFT && brL && !brR) || (loopOut == RIGHT && brR && !brL))) {
    decisionSide = loopOut; decisionMs = ms;
    loopMode = false; curveMm = 0;
  }
  // ---- release the decision once the branch situation is over ----
  if (decisionSide) {
    if (nSeg >= 2 || wide) decisionMs = ms;
    else if (ms - decisionMs > DECISION_HOLD_MS) decisionSide = 0;
  }

  // ---- target position ----
  int32_t pos;
  if (decisionSide == LEFT) {
    uint8_t k = 0;
    for (uint8_t j = 1; j < nSeg; j++) if (segE[j] > segE[k]) k = j;
    pos = (segE[k] - segS[k] + 1 >= WIDE_N) ? windowCentroid(segE[k] - LINE_W / 2, LINE_W / 2 + 1) : segC[k];
  } else if (decisionSide == RIGHT) {
    uint8_t k = 0;
    for (uint8_t j = 1; j < nSeg; j++) if (segS[j] < segS[k]) k = j;
    pos = (segE[k] - segS[k] + 1 >= WIDE_N) ? windowCentroid(segS[k] + LINE_W / 2, LINE_W / 2 + 1) : segC[k];
  } else {
    pos = wide ? windowCentroid(tci, LINE_W / 2 + 1) : segC[tk];   // go straight through
  }

  lastPos = pos;
  lostMs = 0;
  runPID(pos, ms);
}

void stepGap(uint32_t ms) {
  if (nSeg > 0 && nActive < ALLBLACK_N) { state = ST_FOLLOW; lostMs = 0; stepFollow(ms); return; }
  if (ms > gapUntil) { enterRecover((lastErr > 0) ? LEFT : RIGHT, ms); return; }
  drive(P->minSpeed + 20, P->minSpeed + 20);
}

void stepRecover(uint32_t ms) {
  if (ms - lostMs > LOST_TIMEOUT_MS) { enterHold(); return; }
  if (ms < brakeUntil) { drive(-P->brakePwm, -P->brakePwm); return; }
  if ((mask & CENTER_MASK) && nActive < ALLBLACK_N) {
    // found it: hand back to the follower
    state = ST_FOLLOW;
    uint8_t k = 0; int32_t best = 1L << 30;
    for (uint8_t j = 0; j < nSeg; j++) { int32_t dd = labs(segC[j] - 7500); if (dd < best) { best = dd; k = j; } }
    lastPos = segC[k]; lastErr = lastPos - 7500; dFilt = 0;
    hintLms = hintRms = 0; decisionSide = 0; lostMs = 0;
    runPID(lastPos, ms);
    return;
  }
  drive(-recDir * P->turnPwm, recDir * P->turnPwm);   // LEFT: left wheel back, right forward
}

void stepHold(uint32_t ms) {
  drive(0, 0);
  bool flash = (ms / 300) & 1;
  digitalWrite(LED1, flash); digitalWrite(LED2, flash);
#if AUTO_RESUME
  bool clean = (nSeg == 1) && (mask & CENTER_MASK) && (segE[0] - segS[0] + 1 <= LINE_W + 2);
  if (!holdArmed) {
    // needs a period of "nothing" or "all black" (lifted / sitting on END)
    if (nSeg == 0 || nActive >= ALLBLACK_N) {
      if (!holdArmMs) holdArmMs = ms;
      if (ms - holdArmMs > RESUME_ARM_MS) holdArmed = true;
    } else holdArmMs = 0;
  } else if (clean) {
    if (!resumeMs) resumeMs = ms;
    if (ms - resumeMs > RESUME_MS) {
      digitalWrite(LED1, LOW); digitalWrite(LED2, HIGH);
      resetControl(ms);
      runStartMs = ms - START_IGNORE_MS;   // END detection live immediately
      state = ST_FOLLOW;
    }
  } else resumeMs = 0;
#endif
}

void startRun(Profile *prof) {
  P = prof;
  digitalWrite(LED1, LOW); digitalWrite(LED2, HIGH);
  delay(START_DELAY_MS);
  uint32_t ms = millis();
  resetControl(ms);
  decisionIdx = 0; lastDecision = 0;
  runStartMs = ms;
  state = ST_FOLLOW;
}

// ================================ IDLE UI ===================================
void idleUI() {
  static uint32_t t = 0;
  if (!calibrated && millis() - t > 250) {           // "please calibrate"
    t = millis();
    bool f = (t / 250) & 1;
    digitalWrite(LED1, f); digitalWrite(LED2, !f);
  }
  if (pressed(BTN_L)) {
    while (pressed(BTN_L)) delay(5);
    delay(300);
    calibrateAuto();
    digitalWrite(LED1, LOW); digitalWrite(LED2, LOW);
    return;
  }
  if (pressed(BTN_R)) {
    if (!calibrated) { while (pressed(BTN_R)) delay(5); blink(LED2, 4, 60); return; }
    uint32_t t0 = millis();
    digitalWrite(LED2, HIGH);
    while (pressed(BTN_R)) {
      if (millis() - t0 > RB_LONG_MS) digitalWrite(LED1, HIGH);   // RACE armed
      delay(5);
    }
    bool race = millis() - t0 > RB_LONG_MS;
    startRun(race ? &RACE_PROF : &SAFE_PROF);
  }
}

// ================================= SETUP ====================================
void setup() {
#if DEBUG_SERIAL
  Serial.begin(115200);
#endif
  pinMode(STDBY, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(LED1, OUTPUT); pinMode(LED2, OUTPUT);
  pinMode(BTN_L, INPUT_PULLUP); pinMode(BTN_R, INPUT_PULLUP);
  pinMode(A0, OUTPUT); pinMode(A1, OUTPUT); pinMode(A2, OUTPUT); pinMode(A3, OUTPUT);
  pinMode(MUX_EN, OUTPUT); digitalWrite(MUX_EN, LOW);   // ARC-16 enable is active LOW
  pinMode(SIG, INPUT);
  digitalWrite(STDBY, HIGH);
  drive(0, 0);

#if FAST_ADC
  ADCSRA = (ADCSRA & 0xF8) | 0x05;     // prescaler 32
#endif
#if HIGH_PWM_FREQ
  TCCR1B = (TCCR1B & 0xF8) | 0x01;     // pin 9  -> 31.4 kHz
  TCCR2B = (TCCR2B & 0xF8) | 0x01;     // pin 3  -> 31.4 kHz
#endif

  calibrated = loadCal();
  if (calibrated) blink(LED1, 3, 80);
}

// ================================== LOOP ====================================
void loop() {
  if (state == ST_IDLE) { idleUI(); return; }

  static uint32_t tLast = 0;
  uint32_t now = micros();
  if (now - tLast < LOOP_US) return;
  tLast = now;

  // practice stop
  if (pressed(BTN_L)) {
    drive(0, 0);
    while (pressed(BTN_L)) delay(5);
    digitalWrite(LED1, LOW); digitalWrite(LED2, LOW);
    state = ST_IDLE;
    return;
  }

  updateVbat();
  readLine();
  uint32_t ms = millis();

  switch (state) {
    case ST_FOLLOW:  stepFollow(ms);  break;
    case ST_GAP:     stepGap(ms);     break;
    case ST_RECOVER: stepRecover(ms); break;
    case ST_HOLD:    stepHold(ms);    break;
    default: break;
  }

#if DEBUG_SERIAL
  static uint32_t tp = 0;
  if (ms - tp > 50) {
    tp = ms;
    Serial.print(F("st=")); Serial.print(state);
    Serial.print(F(" pos=")); Serial.print(lastPos);
    Serial.print(F(" err=")); Serial.print(lastErr);
    Serial.print(F(" steer=")); Serial.print(steerF);
    Serial.print(F(" seg=")); Serial.print(nSeg);
    Serial.print(F(" act=")); Serial.print(nActive);
    Serial.print(F(" loop=")); Serial.print(loopMode);
    Serial.print(F(" dec=")); Serial.println(decisionSide);
  }
#endif
}
