// 
/*
 * 4-Wire Ultrasonic Obstacle Avoidance Car - v3
 *
 * Core fixes:
 *   1. Small-step movement instead of long blind delays
 *   2. After turn, FORCE forward before next decision (prevents spin-in-place)
 *   3. Direction inertia via lastTurn (won't reverse without good reason)
 *   4. Diagonal-front scan (60°/120°) instead of full side (45°/135°)
 *   5. Increased back-up distance so car actually clears the wall
 */

#include <Servo.h>

// ---- Motor Pins ----
const int ML1 = 2;
const int ML2 = 3;
const int MR1 = 4;
const int MR2 = 5;

// ---- Servo & Ultrasonic Pins ----
const int SERVO = 6;
const int TRIG   = 11;
const int ECHO   = 12;

Servo servo;

// ---- Distances (cm) ----
int distF = 0;
int distL = 0;   // diagonal front-left  (120°)
int distR = 0;   // diagonal front-right (60°)

// ---- Thresholds ----
const int CRITICAL  = 30;
const int WARNING   = 70;
const int SIDE_SAFE = 30;

// ---- Timing (ms) ----
const int T_FWD      = 180;
const int T_FWD_SLOW = 120;
const int T_TRN      = 100;
const int T_BCK      = 150;
const int T_BCK_TRN  = 130;
const int T_STEP     = 60;    // one small chunk of movement
const int SERVO_DLY  = 100;

// ---- Direction inertia ----
int  lastTurn    = 0;         // -1=last turned left, 0=none, 1=last turned right
bool mustGoFwd   = false;     // after turn, force forward before re-deciding
int  cycleCount  = 0;

// ================================================================
void setup() {
  Serial.begin(9600);

  pinMode(ML1, OUTPUT); pinMode(ML2, OUTPUT);
  pinMode(MR1, OUTPUT); pinMode(MR2, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);

  servo.attach(SERVO);
  servo.write(90);
  delay(500);

  Serial.println("=== 4-Wire Obstacle Avoidance v3 ===");
}

// ================================================================
float getDist() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long dur = pulseIn(ECHO, HIGH, 38000);
  if (dur == 0) return 999;
  return dur / 58.0;
}

// Diagonal front: L=120° (front-left), R=60° (front-right)
// Much more relevant than 135°/45° for forward-moving car
void scanF() { servo.write(90);  delay(SERVO_DLY); distF = getDist(); }
void scanL() { servo.write(120); delay(SERVO_DLY); distL = getDist(); }
void scanR() { servo.write(60);  delay(SERVO_DLY); distR = getDist(); }

void scanDiagonals() {
  scanL();
  servo.write(90); delay(SERVO_DLY);
  scanR();
  servo.write(90); delay(SERVO_DLY);   // return to center
}

int quickF() { return getDist(); }

// ================================================================
// Motor control
void halt() {
  digitalWrite(ML1, LOW); digitalWrite(ML2, LOW);
  digitalWrite(MR1, LOW); digitalWrite(MR2, LOW);
}
void goFwd() {
  digitalWrite(ML1, HIGH); digitalWrite(ML2, LOW);
  digitalWrite(MR1, HIGH); digitalWrite(MR2, LOW);
}
void goBck() {
  digitalWrite(ML1, LOW);  digitalWrite(ML2, HIGH);
  digitalWrite(MR1, LOW);  digitalWrite(MR2, HIGH);
}
void spinL() {
  digitalWrite(ML1, LOW);  digitalWrite(ML2, HIGH);
  digitalWrite(MR1, HIGH); digitalWrite(MR2, LOW);
}
void spinR() {
  digitalWrite(ML1, HIGH); digitalWrite(ML2, LOW);
  digitalWrite(MR1, LOW);  digitalWrite(MR2, HIGH);
}

// ================================================================
// Forward in small steps — re-checks front between each step.
// Aborts early if something appears ahead.
void fwdStepped(int totalMs) {
  int steps = totalMs / T_STEP;
  if (steps < 1) steps = 1;

  for (int i = 0; i < steps; i++) {
    int f = quickF();
    if (f < CRITICAL) {
      halt();
      return;
    }
    goFwd();
    delay(T_STEP);
  }
  halt();
}

// Turn in small chunks — re-checks front between each chunk.
// Aborts early if obstacle appears ahead.
bool turnChunked(bool turnLeft, int totalMs) {
  int chunks = totalMs / T_STEP;
  if (chunks < 1) chunks = 1;

  for (int i = 0; i < chunks; i++) {
    if (turnLeft) spinL(); else spinR();
    delay(T_STEP);
    halt();
    delay(10);

    int f = quickF();
    if (f < CRITICAL) {
      return false;
    }
  }
  return true;
}

// ================================================================
// Choose turn direction WITH INERTIA from last turn.
// Won't reverse direction unless the other side is MUCH better (20cm+).
int chooseDir(int left, int right) {
  if (lastTurn == 0) {
    return (left > right) ? -1 : 1;
  }

  if (lastTurn == -1) {
    // Last turned LEFT → need right to be 20cm+ better to switch
    if (right > left + 20) return 1;
    return -1;
  } else {
    // Last turned RIGHT → need left to be 20cm+ better to switch
    if (left > right + 20) return -1;
    return 1;
  }
}

// ================================================================
void loop() {
  distF = quickF();

  // Only scan diagonals when approaching something (or every 5th cycle)
  bool needScan = (distF < WARNING) || (cycleCount % 5 == 0);
  if (needScan) {
    scanDiagonals();
  }

  // ---- Debug ----
  Serial.print("["); Serial.print(cycleCount); Serial.print("] ");
  Serial.print("F:"); Serial.print(distF);
  if (needScan) {
    Serial.print(" L:"); Serial.print(distL);
    Serial.print(" R:"); Serial.print(distR);
  }
  Serial.print(" lastT:"); Serial.print(lastTurn);
  Serial.print(" force:"); Serial.print(mustGoFwd);
  Serial.print(" -> ");

  // ================================================================
  // AFTER-TURN: force a forward stride before any new decision.
  // This is the #1 fix for spin-in-place — the car MUST move forward
  // after turning, so it doesn't re-scan from the same spot.
  // ================================================================
  if (mustGoFwd && distF > CRITICAL) {
    Serial.println("FORCED-FWD (post-turn)");
    fwdStepped(T_FWD_SLOW);
    mustGoFwd = false;
    cycleCount++;
    return;
  }
  mustGoFwd = false;

  // ================================================================
  // 1. CRUISE: wide open ahead (>150cm)
  //    Reset inertia, just go.
  // ================================================================
  if (distF > 150) {
    Serial.println("CRUISE");
    lastTurn = 0;
    fwdStepped(T_FWD);
    cycleCount++;
    return;
  }

  // ================================================================
  // 2. CRITICAL: front blocked (<30cm)
  //    Back up properly, then turn with chunked checks.
  // ================================================================
  if (distF < CRITICAL) {
    if (!needScan) scanDiagonals();

    Serial.println("CRITICAL");

    // BACK UP — long enough to actually clear the wall
    goBck(); delay(T_BCK);
    halt();  delay(30);

    // Re-scan after backing up
    scanDiagonals();
    Serial.print("  re-scan L:"); Serial.print(distL);
    Serial.print(" R:"); Serial.println(distR);

    int dir;

    if (distL < CRITICAL && distR < CRITICAL) {
      // Both blocked → back up more, then pick the better one
      Serial.println("  TRAPPED, back more");
      goBck(); delay(T_BCK);
      halt();  delay(30);
      scanDiagonals();
      dir = (distL > distR) ? -1 : 1;
    } else if (distL < CRITICAL) {
      dir = 1;   // left blocked → must go right
    } else if (distR < CRITICAL) {
      dir = -1;  // right blocked → must go left
    } else {
      dir = chooseDir(distL, distR);
    }

    Serial.print("  turn "); Serial.println((dir == -1) ? "LEFT" : "RIGHT");
    turnChunked(dir == -1, T_BCK_TRN);
    halt();

    lastTurn  = dir;
    mustGoFwd = true;   // FORCE forward next cycle
    cycleCount++;
    return;
  }

  // ================================================================
  // 3. WARNING: front approaching (30-70cm)
  //    If a diagonal is clearly better, turn that way.
  //    Otherwise creep forward.
  // ================================================================
  if (distF < WARNING) {
    if (!needScan) scanDiagonals();

    bool leftOpen  = (distL > distF + 15);
    bool rightOpen = (distR > distF + 15);

    if (leftOpen || rightOpen) {
      int dir = chooseDir(distL, distR);

      // Don't turn into a wall
      if (dir == -1 && distL < SIDE_SAFE) dir = 1;
      if (dir == 1  && distR < SIDE_SAFE) dir = -1;

      Serial.print("WARN turn ");
      Serial.println((dir == -1) ? "LEFT" : "RIGHT");

      // Back up before turning (avoid scraping wall with front corner)
      goBck(); delay(80);
      halt();  delay(20);
      turnChunked(dir == -1, T_TRN);
      halt();

      lastTurn  = dir;
      mustGoFwd = true;
    } else {
      Serial.println("WARN slow fwd");
      fwdStepped(T_FWD_SLOW);
    }
    cycleCount++;
    return;
  }

  // ================================================================
  // 4. CLEAR (70-150cm): normal forward.
  //    Occasional tiny course correction toward the preferred side.
  // ================================================================
  if (needScan && lastTurn != 0) {
    int prefSide = (lastTurn == -1) ? distL : distR;
    int otherSide = (lastTurn == -1) ? distR : distL;

    if (prefSide > distF + 30 && prefSide > otherSide + 15) {
      Serial.print("VEER ");
      Serial.println((lastTurn == -1) ? "L" : "R");
      turnChunked(lastTurn == -1, T_TRN / 2);  // tiny correction only
      halt();
      // No mustGoFwd for tiny veers — just continue
      cycleCount++;
      return;
    }
  }

  // Default: go forward
  Serial.println("FWD");
  fwdStepped(T_FWD);
  cycleCount++;
}
