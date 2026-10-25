// ===== RoboTracer (สนามเส้นขาว พื้นดำ มี marker ข้างเส้น + มีจุดตัด) =====
// ถูก include ท้าย SPACE_S_16_Sensor.h ไม่ต้อง include เองใน sketch
//
// กติกาสนาม:
//   - marker ฝั่งขวา (ตามทิศวิ่ง) = Start / Goal   -> ผ่านอันแรก = Start, อันที่สอง = Goal
//   - marker ฝั่งซ้าย            = จุดเปลี่ยนโค้ง (Curve)
//   - จุดตัด (เส้นตัดกัน)         = วิ่งตรงผ่าน ไม่นับเป็น marker
//
// การแยกแยะ (ใช้เซนเซอร์ C[0], C[1] ข้างเส้น):
//   - เห็นแค่ฝั่ง Goal           -> Goal marker
//   - เห็นแค่ฝั่ง Curve          -> Curve marker
//   - เห็นทั้งสองฝั่ง ในการผ่านครั้งเดียว หรือ เซนเซอร์หน้าเห็นเส้นกว้าง -> จุดตัด (ไม่นับ)

#pragma once

#define MK_NONE 0
#define MK_GOAL 1
#define MK_CURVE 2

int MarkerGoalCh = 1;      // index ของ C ที่อยู่ฝั่ง Start/Goal marker (ขวา)
int MarkerCurveCh = 0;     // index ของ C ที่อยู่ฝั่ง Curve marker (ซ้าย)
int CrossSensorCount = 8;  // เซนเซอร์หน้าเห็นเส้นพร้อมกันกี่ตัวขึ้นไป = จุดตัด
int CrossIgnoreTime = 150; // ms หลังเซนเซอร์หน้าเจอจุดตัด ที่จะไม่นับ marker

unsigned char mkActive = 0, mkSeenGoal = 0, mkSeenCurve = 0, mkSeenCross = 0;
unsigned long mkStart = 0, mkLastCross = 0;

// สลับฝั่งเซนเซอร์ C ถ้าหุ่นนับ marker ผิดฝั่ง เช่น SetMarkerCh(1, 0);
void SetMarkerCh(int goalCh, int curveCh) {
  MarkerGoalCh = goalCh;
  MarkerCurveCh = curveCh;
}

// ปรับการตรวจจุดตัด SetCrossDetect(จำนวนเซนเซอร์หน้า, เวลาไม่นับ ms);
void SetCrossDetect(int count, int ignoreMs) {
  CrossSensorCount = count;
  CrossIgnoreTime = ignoreMs;
}

int CountFrontOnLine() {
  int n = 0;
  for (int i = sensor_L; i <= sensor_R; i++) {
    if (F[i] >= Ref)
      n++;
  }
  return n;
}

void ResetMarker() {
  mkActive = mkSeenGoal = mkSeenCurve = mkSeenCross = 0;
  mkLastCross = millis() - CrossIgnoreTime - 1;
}

// เรียกหลัง PID() ทุกครั้ง (F ถูกอ่านใหม่แล้วใน PID)
// คืนค่า MK_GOAL / MK_CURVE ครั้งเดียวตอนผ่าน marker พ้นแล้ว, นอกนั้นคืน MK_NONE
int CheckMarker() {
  unsigned long now = millis();
  bool cross = CountFrontOnLine() >= CrossSensorCount;
  if (cross)
    mkLastCross = now;

  ReadCalibrateC();
  bool g = C[MarkerGoalCh] >= RefC;
  bool c = C[MarkerCurveCh] >= RefC;

  if (g || c) {
    if (!mkActive) {
      mkActive = 1;
      mkStart = now;
    }
    if (g)
      mkSeenGoal = 1;
    if (c)
      mkSeenCurve = 1;
    if (cross)
      mkSeenCross = 1;
    return MK_NONE;
  }
  if (!mkActive)
    return MK_NONE;

  // ผ่าน marker พ้นแล้ว -> ตัดสินว่าเป็นอะไร
  bool isCross = (mkSeenGoal && mkSeenCurve) || mkSeenCross || (mkStart - mkLastCross < (unsigned long)CrossIgnoreTime);
  int result = MK_NONE;
  if (!isCross)
    result = mkSeenGoal ? MK_GOAL : MK_CURVE;
  mkActive = mkSeenGoal = mkSeenCurve = mkSeenCross = 0;
  return result;
}

// ผ่าน Goal แล้ว วิ่งต่อช้าๆ ตามเวลา แล้วหยุด (กติกา: ต้องหยุดภายใน 1 เมตรหลัง Goal)
void RoboTracerStop(int Speed, float Kp, float Kd, int total_time) {
  Serial.println("RoboTracer stop"); // DEBUG
  TrackTime(Speed / 2, Kp, Kd, total_time);
  MotorStop();
  BeepBeep();
}

int RampUp = 1;          // เร่งได้ทีละกี่ % ต่อ 10 ms (1 = 40->80 ใช้ 0.4 วินาที)
int RampDown = 4;        // ลดได้ทีละกี่ % ต่อ 10 ms (ต้องลดเร็วพอให้ทันก่อนเข้าโค้ง)
int RampStartSpeed = 20; // RoboTracer() เริ่มออกตัวที่ความเร็วนี้ แล้วค่อยๆ เร่งไปจน Speed

// SetSpeedRamp(เร่ง, ลด); หน่วย % ต่อ 10 ms, ใส่ค่าใหญ่ๆ เช่น 100 = เปลี่ยนทันที
void SetSpeedRamp(int up, int down) {
  RampUp = up;
  RampDown = down;
}

// SetRampStartSpeed(ความเร็วออกตัว); ใส่ค่า >= Speed = ไม่ค่อยๆ เร่ง
void SetRampStartSpeed(int spd) {
  RampStartSpeed = spd;
}

// วิ่งความเร็วเดียวจาก Start ถึง Goal แล้วหยุด
// ออกตัวที่ RampStartSpeed แล้วค่อยๆ เร่งทีละ RampUp % ต่อ 10 ms จนถึง Speed
// RoboTracer(ความเร็ว, Kp, Kd, จำนวน Goal marker ที่จะหยุด, เวลาวิ่งต่อหลัง Goal ms);
// ปกติ target_Marker = 2 (อันแรก = Start, อันที่สอง = Goal)
void RoboTracer(int Speed, float Kp, float Kd, int target_Marker, int total_time) {
  Serial.print("RoboTracer start spd=");
  Serial.println(Speed); // DEBUG
  BaseSpeed = min(RampStartSpeed, Speed);
  InitialSpeed();
  unsigned long lastRamp = millis();
  ResetMarker();
  int goal = 0;
  while (1) {
    if (BaseSpeed < Speed && millis() - lastRamp >= 10) {
      lastRamp = millis();
      BaseSpeed = min(BaseSpeed + RampUp, Speed);
      InitialSpeed(); // อัปเดต LeftBaseSpeed/RightBaseSpeed
    }
    PID(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    if (CheckMarker() == MK_GOAL) {
      goal++;
      tone(BUZZER_PIN, volumesound, 10); // ไม่ block
      Serial.print("GOAL ");
      Serial.print(goal);
      Serial.print(" t=");
      Serial.println(millis()); // DEBUG
      if (goal >= target_Marker)
        break;
    }
  }
  RoboTracerStop(Speed, Kp, Kd, total_time);
}

// วิ่งแบบแบ่งช่วง: เปลี่ยนความเร็วทุกครั้งที่ผ่าน Curve marker (หลังผ่าน Start แล้ว)
// ช่วงที่ 0 = จาก Start ถึง Curve marker แรก, ช่วงที่ 1 = ถึงอันที่สอง, ...
// ถ้า Kp/Kd เป็น NULL จะใช้ค่าจากตาราง setKpKd() ตามความเร็วของช่วงนั้น
// ถ้า Curve marker มากกว่าจำนวนช่วง จะใช้ช่วงสุดท้ายไปจนจบ
// ความเร็วไม่เปลี่ยนทันที แต่ค่อยๆ ไล่ไปหาค่าใหม่ (ดู SetSpeedRamp) Kp/Kd ไล่ตามไปด้วย กันหุ่นกระตุก
long PreBrakeTime(int seg, int spdNow, int spdNext); // อยู่ในส่วน Learn ด้านล่าง
long ExpectSegTime(int seg, int v0, int v1);
extern int CurveErrThreshold;

/* ---------- ตรวจนับ marker พลาด (ใช้ตอน RoboTracerRun) ---------- */
// เทียบเวลาจริงของแต่ละช่วงกับเวลาที่คาดจากตอนสำรวจ + ดู Error ในช่วงที่จำว่าเป็นทางตรง
// ถ้าดูแล้วนับพลาด -> โหมดปลอดภัย: วิ่งความเร็วโค้งไปจน Goal (ไม่ใช้ข้อมูลสำรวจต่อ)
int SyncEarlyPct = 50; // เจอ Curve marker ก่อน % นี้ของเวลาที่คาด = นับเกิน (ตรวจเฉพาะช่วงที่คาด >= 300 ms)
int SyncLatePct = 160; // อยู่ในช่วงเดิมนานเกิน % นี้ของเวลาที่คาด + 100 ms = พลาด marker
int SyncErrTime = 80;  // ช่วงที่จำว่าตรง แต่ |Error| เกิน CurveErrThreshold ต่อเนื่องนานเกินนี้ ms = จริงๆ อยู่ในโค้ง
#define SYNC_MIN_TIME 300
#define SYNC_LATE_SLACK 100
#define SYNC_ERR_SKIP 150 // ไม่ดู Error ช่วงแรกของช่วง ms (ยังออกจากโค้งไม่สุด)

// SetSyncCheck(% เร็วสุด, % ช้าสุด, ms Error สูง); ค่าลบ = ปิดการตรวจข้อนั้น
// ดูเวลาจริง/ที่คาดของแต่ละช่วงได้ที่ Serial Monitor ตอน RoboTracerRun (บรรทัด "S<ช่วง> <จริง>/<คาด>")
void SetSyncCheck(int earlyPct, int latePct, int errMs) {
  SyncEarlyPct = earlyPct;
  SyncLatePct = latePct;
  SyncErrTime = errMs;
}

// แกนกลางของ RoboTracerSegment / RoboTracerRun
// Idx[seg] = ช่วง seg ใช้ค่าแถวไหนของ Spd/Kp/Kd (NULL = แถว seg) -> RoboTracerRun ใช้แค่ 2 แถว (ตรง/โค้ง) ไม่เปลือง RAM
// learned = วิ่งจากผลสำรวจ (Idx = LearnSegCurve, แถว 1 = โค้ง):
//   เบรกล่วงหน้าก่อนถึง Curve marker (ดู SetBrakeMargin) ถ้าเจอ marker ก่อนก็เปลี่ยนตาม marker ตามปกติ
//   + ตรวจนับ marker พลาด (ดู SetSyncCheck)
void RoboTracerSegmentCore(const int Spd[], const float Kp[], const float Kd[], int numSeg,
                           const unsigned char Idx[], bool learned, int target_Marker, int total_time) {
#define SEG_ROW(s) (Idx ? Idx[s] : (s))
  int seg = 0;          // ช่วงที่หุ่นอยู่จริง (นับตาม Curve marker)
  int tgt = 0;          // ช่วงที่ความเร็วกำลังไล่ไปหา (เบรกล่วงหน้าแล้ว = seg + 1)
  int row = SEG_ROW(0); // แถวของ Spd/Kp/Kd ที่กำลังไล่ไปหา
  int goal = 0;
  int curveCount = 0; // Curve marker ที่นับได้ทั้งหมด (ไว้เทียบกับตอนสำรวจตอนถึง Goal)
  bool safe = false;  // โหมดปลอดภัย (นับ marker พลาด)
  long brakeAt = -1;  // ms นับจากต้นช่วง ที่จะเริ่มเบรกล่วงหน้า (-1 = รอ marker)
  long expect = 0;    // เวลาที่คาดว่าช่วงนี้จะใช้ ms (0 = ไม่ตรวจ)
  unsigned long errHighSince = 0;
  BaseSpeed = Spd[row];
  InitialSpeed();
  float kp = Kp ? Kp[row] : PID_KP;
  float kd = Kd ? Kd[row] : PID_KD;

  int spdFrom = BaseSpeed, spdTo = BaseSpeed; // ramp จากความเร็วไหน ไปความเร็วไหน
  float kpFrom = kp, kdFrom = kd;
  unsigned long lastRamp = millis();
  unsigned long segStart = millis();

  ResetMarker();
  while (1) {
    PID(LeftBaseSpeed, RightBaseSpeed, kp, kd);
    int mk = CheckMarker();
    unsigned long el = millis() - segStart; // เวลาที่อยู่ในช่วงนี้แล้ว
    int next = tgt;
    bool newSeg = false;
    const __FlashStringHelper *lost = NULL; // != NULL = นับ marker พลาด (เหตุผล)
    if (mk == MK_GOAL) {
      goal++;
      tone(BUZZER_PIN, volumesound, 30);
      Serial.print("GOAL ");
      Serial.print(goal);
      Serial.print(" t=");
      Serial.println(millis()); // DEBUG
      if (goal >= target_Marker)
        break;
      if (goal == 1)
        newSeg = true; // Start -> เริ่มจับเวลาช่วงแรก (ตรงกับตอน Learn)
    } else if (mk == MK_CURVE && goal >= 1) {
      curveCount++;
      if (learned && expect) {
        Serial.print("S");
        Serial.print(seg);
        Serial.print(" ");
        Serial.print(el);
        Serial.print("/");
        Serial.println(expect); // DEBUG เวลาจริง/ที่คาด
        if (!safe && SyncEarlyPct >= 0 && expect >= SYNC_MIN_TIME && (long)el * 100 < expect * SyncEarlyPct)
          lost = F("early marker");
      }
      if (seg < numSeg - 1) {
        seg++;
        newSeg = true;
        if (tgt < seg) // ยังไม่ได้เบรกล่วงหน้า -> เปลี่ยนตอนนี้
          next = seg;
      }
      tone(BUZZER_PIN, volumesound / 2, 20);
    } else if (brakeAt >= 0 && tgt == seg && el >= (unsigned long)brakeAt) {
      next = seg + 1; // ใกล้ถึงโค้งแล้ว -> เบรกล่วงหน้า
      brakeAt = -1;
      tone(BUZZER_PIN, volumesound / 4, 10);
    } else if (learned && !safe && goal >= 1) {
      // อยู่ในช่วงเดิมนานเกิน -> น่าจะผ่าน marker ไปแล้วโดยไม่ได้นับ
      if (SyncLatePct >= 0 && expect && (long)el * 100 > expect * SyncLatePct + SYNC_LATE_SLACK * 100L)
        lost = F("late marker");
      // จำว่าทางตรง แต่ Error สูงต่อเนื่อง -> จริงๆ อยู่ในโค้ง
      bool errHigh = SyncErrTime >= 0 && SEG_ROW(seg) == 0 && tgt == seg && el > SYNC_ERR_SKIP &&
                     abs(LastError) > CurveErrThreshold;
      if (!errHigh)
        errHighSince = 0;
      else if (!errHighSince)
        errHighSince = millis();
      else if (millis() - errHighSince > (unsigned long)SyncErrTime)
        lost = F("curve on straight");
    }

    if (newSeg) {
      segStart = millis();
      errHighSince = 0;
      brakeAt = (learned && !safe && seg < numSeg - 1) ? PreBrakeTime(seg, Spd[SEG_ROW(seg)], Spd[SEG_ROW(seg + 1)]) : -1;
      expect = learned ? ExpectSegTime(seg, BaseSpeed, Spd[SEG_ROW(seg)]) : 0;
    }

    if (lost) { // -> โหมดปลอดภัย: ความเร็วโค้งไปจน Goal
      safe = true;
      brakeAt = -1;
      row = 1;
      spdFrom = BaseSpeed;
      spdTo = Spd[row];
      kpFrom = kp;
      kdFrom = kd;
      tone(BUZZER_PIN, volumesound * 2, 300);
      Serial.print(F("SAFE MODE seg "));
      Serial.print(seg);
      Serial.print(": ");
      Serial.println(lost);
    } else if (next != tgt && !safe) {
      tgt = next;
      row = SEG_ROW(tgt);
      spdFrom = BaseSpeed; // เริ่ม ramp จากความเร็วปัจจุบัน (เผื่อ ramp เดิมยังไม่จบ)
      spdTo = Spd[row];
      kpFrom = kp;
      kdFrom = kd;
    }

    if (BaseSpeed != spdTo && millis() - lastRamp >= 10) {
      lastRamp = millis();
      if (BaseSpeed < spdTo)
        BaseSpeed = min(BaseSpeed + RampUp, spdTo);
      else
        BaseSpeed = max(BaseSpeed - RampDown, spdTo);
      InitialSpeed(); // อัปเดต LeftBaseSpeed/RightBaseSpeed (+ PID_KP/KD ตามตาราง)
      if (Kp || Kd) {
        float f = (float)(BaseSpeed - spdFrom) / (spdTo - spdFrom); // 0 -> 1
        kp = Kp ? kpFrom + (Kp[row] - kpFrom) * f : PID_KP;
        kd = Kd ? kdFrom + (Kd[row] - kdFrom) * f : PID_KD;
      } else {
        kp = PID_KP;
        kd = PID_KD;
      }
    }
  }
  RoboTracerStop(BaseSpeed, kp, kd, total_time);
  if (learned) { // เทียบจำนวน Curve marker กับตอนสำรวจ
    Serial.print(F("Curve marker "));
    Serial.print(curveCount);
    Serial.print(F(" / learn "));
    Serial.println(numSeg - 1);
    if (curveCount != numSeg - 1 || safe) {
      Serial.println(F("SYNC LOST this run"));
      tone(BUZZER_PIN, volumesound * 2, 600); // เสียงสูงยาว = รอบนี้นับพลาด
    }
  }
#undef SEG_ROW
}

void RoboTracerSegment(const int Spd[], const float Kp[], const float Kd[], int numSeg,
                       int target_Marker, int total_time) {
  RoboTracerSegmentCore(Spd, Kp, Kd, numSeg, NULL, false, target_Marker, total_time);
}

// แบบไม่ต้องใส่ numSeg: นับจำนวนช่วงจากขนาดอาร์เรย์ให้เอง (อาร์เรย์ทั้ง 3 ต้องยาวเท่ากัน)
// RoboTracerSegment(Spd, Kp, Kd, target_Marker, total_time);
template <size_t N>
void RoboTracerSegment(const int (&Spd)[N], const float (&Kp)[N], const float (&Kd)[N],
                       int target_Marker, int total_time) {
  RoboTracerSegment(Spd, Kp, Kd, N, target_Marker, total_time);
}

// ใช้ Kp/Kd จากตาราง setKpKd() -> RoboTracerSegment(Spd, target_Marker, total_time);
template <size_t N>
void RoboTracerSegment(const int (&Spd)[N], int target_Marker, int total_time) {
  RoboTracerSegment(Spd, NULL, NULL, N, target_Marker, total_time);
}

/* ================== วิ่งสำรวจรอบแรก (Learn) ================== */
// รอบแรก: RoboTracerLearn() วิ่งช้าๆ จำแต่ละช่วง (คั่นด้วย Curve marker) ว่าเป็นทางตรงหรือโค้ง
//   ดูจากค่าเฉลี่ย |Error| ของ PID ในช่วงนั้น: น้อย = ตรง, มาก = โค้ง
// รอบสอง: RoboTracerRun() สร้างตารางความเร็วจากที่จำไว้ แล้ววิ่งด้วย RoboTracerSegment()
//   ทางตรงที่ตามด้วยโค้ง จะเบรกล่วงหน้าก่อนถึง Curve marker (ดู SetBrakeMargin)
// ค่าที่จำเก็บใน RAM -> ปิดเครื่องแล้วหาย, SaveLearn() เก็บลง EEPROM แล้ว LoadLearn() โหลดกลับมาได้
#define MAX_SEG 64 // สนามโค้งถี่ marker เยอะ, 1 ช่วงใช้ RAM 7 bytes

int LearnSpeed = 0;                   // ความเร็วตอนสำรวจ ใช้ประมาณเวลาทางตรงตอนวิ่งเร็ว
int LearnSegCount = 0;                // จำนวนช่วงที่เจอ (= Curve marker + 1)
unsigned long LearnSegTime[MAX_SEG];  // เวลาของแต่ละช่วง ms
int LearnSegErr[MAX_SEG];             // ค่าเฉลี่ย |Error| ของแต่ละช่วง
unsigned char LearnSegCurve[MAX_SEG]; // 1 = โค้ง, 0 = ตรง

int CurveErrThreshold = 1500; // ค่าเฉลี่ย |Error| เกินนี้ = โค้ง
int MinStraightTime = 300;    // ทางตรงที่สั้นกว่านี้ (ms) ให้ถือเป็นโค้ง เร่งไม่ทันอยู่แล้ว

// SetCurveDetect(เกณฑ์ Error, เวลาทางตรงขั้นต่ำ ms); ดูค่าจริงได้จาก SerialLearn()
void SetCurveDetect(int errThreshold, int minStraightMs) {
  CurveErrThreshold = errThreshold;
  MinStraightTime = minStraightMs;
}

// RoboTracerLearn(ความเร็วสำรวจ, Kp, Kd, จำนวน Goal marker ที่จะหยุด, เวลาวิ่งต่อหลัง Goal ms);
void RoboTracerLearn(int Speed, float Kp, float Kd, int target_Marker, int total_time) {
  BaseSpeed = Speed;
  InitialSpeed();
  ResetMarker();
  LearnSpeed = Speed;
  LearnSegCount = 0;
  int goal = 0;
  unsigned long errSum = 0, n = 0, segStart = 0;

  while (1) {
    PID(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    if (goal >= 1) { // เก็บค่าหลังผ่าน Start แล้วเท่านั้น
      errSum += abs(LastError);
      n++;
    }
    int mk = CheckMarker();
    bool endSeg = false;
    if (mk == MK_GOAL) {
      goal++;
      tone(BUZZER_PIN, volumesound, 30);
      if (goal == 1) { // Start -> เริ่มช่วงแรก
        segStart = millis();
        errSum = n = 0;
      } else if (goal >= target_Marker) {
        endSeg = true; // Goal -> ปิดช่วงสุดท้าย
      }
    } else if (mk == MK_CURVE && goal >= 1) {
      endSeg = true;
      tone(BUZZER_PIN, volumesound / 2, 20);
    }

    if (endSeg && LearnSegCount >= MAX_SEG) {
      tone(BUZZER_PIN, volumesound * 2, 200); // ช่วงเกิน MAX_SEG -> ช่วงที่เหลือไม่ถูกจำ
      Serial.println(F("Learn: segments > MAX_SEG"));
      LearnSegCurve[MAX_SEG - 1] = 1; // ตอนวิ่งเร็ว ที่เหลือทั้งหมดใช้ช่วงสุดท้าย -> ให้เป็นความเร็วโค้ง (ปลอดภัย)
    } else if (endSeg) {
      int i = LearnSegCount++;
      LearnSegTime[i] = millis() - segStart;
      LearnSegErr[i] = n ? errSum / n : 0;
      LearnSegCurve[i] = LearnSegErr[i] > CurveErrThreshold || LearnSegTime[i] < (unsigned long)MinStraightTime;
      segStart = millis();
      errSum = n = 0;
    }
    if (goal >= target_Marker)
      break;
  }
  RoboTracerStop(Speed, Kp, Kd, total_time);
}

/* ---------- เบรกล่วงหน้าก่อนเข้าโค้ง ---------- */
// เวลาทางตรงตอนวิ่งเร็ว ประมาณจากตอนสำรวจ: เวลาสำรวจ x LearnSpeed / ความเร็วทางตรง
//   (ไม่นับช่วงเร่งออกจากโค้ง หุ่นจริงจึงช้ากว่าที่ประมาณ = เบรกเร็วไปนิด ฝั่งปลอดภัย)
// เริ่มเบรกก่อนหมดเวลานั้น = เวลาที่ RampDown ใช้ลดความเร็ว + BrakeMargin
int BrakeMargin = 0; // ms เผื่อเพิ่ม, ค่าลบ = ปิด (เปลี่ยนความเร็วตอนเจอ marker เหมือนเดิม)

// SetBrakeMargin(ms); เข้าโค้งแรงไป = เพิ่ม, ช้าลงก่อนถึงโค้งนานไป = ลด, -1 = ปิด
void SetBrakeMargin(int ms) {
  BrakeMargin = ms;
}

// เวลา ms นับจากต้นช่วง seg ที่ควรเริ่มเบรก, -1 = ไม่ต้องเบรกล่วงหน้า
long PreBrakeTime(int seg, int spdNow, int spdNext) {
  if (BrakeMargin < 0 || LearnSpeed <= 0 || seg >= LearnSegCount || spdNext >= spdNow)
    return -1;
  long t = (long)LearnSegTime[seg] * LearnSpeed / spdNow;
  t -= (long)(spdNow - spdNext) * 10 / max(RampDown, 1) + BrakeMargin;
  return max(t, 0L);
}

// เวลา ms ที่คาดว่าช่วง seg จะใช้ ถ้าเริ่มช่วงที่ความเร็ว v0 แล้วไล่ไปหา v1 (ตาม RampUp/RampDown)
//   ระยะทางของช่วง = เวลาสำรวจ x LearnSpeed, ไม่นับเบรกล่วงหน้าตอนท้ายช่วง (SyncLatePct เผื่อไว้แล้ว)
long ExpectSegTime(int seg, int v0, int v1) {
  if (seg >= LearnSegCount || LearnSpeed <= 0 || v0 <= 0 || v1 <= 0)
    return 0;
  long d = (long)LearnSegTime[seg] * LearnSpeed;
  long tr = (long)abs(v1 - v0) * 10 / max(v1 > v0 ? RampUp : RampDown, 1); // เวลาที่ ramp
  long dr = (long)(v0 + v1) * tr / 2;                                      // ระยะที่ได้ระหว่าง ramp
  // ramp ยังไม่จบก็หมดช่วงแล้ว -> ประมาณด้วยความเร็วเฉลี่ย
  if (d <= dr)
    return d * 2 / (v0 + v1);
  return tr + (d - dr) / v1;
}

// วิ่งเร็วจากที่สำรวจไว้ ใช้ Kp/Kd จากตาราง setKpKd()
// RoboTracerRun(ความเร็วทางตรง, ความเร็วโค้ง, จำนวน Goal marker ที่จะหยุด, เวลาวิ่งต่อหลัง Goal ms);
void RoboTracerRun(int SpdStraight, int SpdCurve, int target_Marker, int total_time) {
  if (LearnSegCount == 0) { // ยังไม่ได้สำรวจ
    BeepBeep();
    return;
  }
  int Spd[2] = {SpdStraight, SpdCurve}; // แถว 0 = ตรง, 1 = โค้ง (ตาม LearnSegCurve)
  RoboTracerSegmentCore(Spd, NULL, NULL, LearnSegCount, LearnSegCurve, true, target_Marker, total_time);
}

// แบบ Kp/Kd ชุดเดียวทั้งทางตรงและโค้ง (เปลี่ยนแค่ความเร็ว กันกระตุกจาก Kp/Kd เปลี่ยน)
// RoboTracerRun(ความเร็วตรง, ความเร็วโค้ง, Kp, Kd, จำนวน Goal marker, เวลาวิ่งต่อ ms);
void RoboTracerRun(int SpdStraight, int SpdCurve, float Kp, float Kd, int target_Marker, int total_time) {
  if (LearnSegCount == 0) {
    BeepBeep();
    return;
  }
  int Spd[2] = {SpdStraight, SpdCurve};
  float KpA[2] = {Kp, Kp}, KdA[2] = {Kd, Kd};
  RoboTracerSegmentCore(Spd, KpA, KdA, LearnSegCount, LearnSegCurve, true, target_Marker, total_time);
}

// แบบกำหนด Kp/Kd เอง แยกทางตรง/โค้ง
// RoboTracerRun(ความเร็วตรง, Kp ตรง, Kd ตรง, ความเร็วโค้ง, Kp โค้ง, Kd โค้ง, จำนวน Goal marker, เวลาวิ่งต่อ ms);
void RoboTracerRun(int SpdStraight, float KpS, float KdS, int SpdCurve, float KpC, float KdC,
                   int target_Marker, int total_time) {
  if (LearnSegCount == 0) {
    BeepBeep();
    return;
  }
  int Spd[2] = {SpdStraight, SpdCurve}; // แถว 0 = ตรง, 1 = โค้ง (ตาม LearnSegCurve)
  float Kp[2] = {KpS, KpC}, Kd[2] = {KdS, KdC};
  RoboTracerSegmentCore(Spd, Kp, Kd, LearnSegCount, LearnSegCurve, true, target_Marker, total_time);
}

/* ---------- เก็บผลสำรวจลง EEPROM ---------- */
// ต่อจากค่าคาลิเบรตเซนเซอร์ (0-71, flag 100-101)
#define EEPROM_LEARN_FLAG_ADDR 120
#define EEPROM_LEARN_START 121 // LearnSpeed(2) + LearnSegCount(2) + MAX_SEG x (time 4 + err 2 + curve 1)

void SaveLearn() {
  int addr = EEPROM_LEARN_START;
  EEPROM.put(addr, LearnSpeed);
  addr += sizeof(int);
  EEPROM.put(addr, LearnSegCount);
  addr += sizeof(int);
  for (int i = 0; i < LearnSegCount; i++) {
    EEPROM.put(addr, LearnSegTime[i]);
    addr += sizeof(unsigned long);
    EEPROM.put(addr, LearnSegErr[i]);
    addr += sizeof(int);
    EEPROM.update(addr++, LearnSegCurve[i]);
  }
  EEPROM.update(EEPROM_LEARN_FLAG_ADDR, 0xAA);
  Serial.println("Learn saved");
}

// คืน true ถ้าโหลดได้, false = ยังไม่เคย SaveLearn()
bool LoadLearn() {
  if (EEPROM.read(EEPROM_LEARN_FLAG_ADDR) != 0xAA) {
    Serial.println(F("EEPROM no learn data"));
    return false;
  }
  int addr = EEPROM_LEARN_START;
  EEPROM.get(addr, LearnSpeed);
  addr += sizeof(int);
  EEPROM.get(addr, LearnSegCount);
  addr += sizeof(int);
  LearnSegCount = constrain(LearnSegCount, 0, MAX_SEG);
  for (int i = 0; i < LearnSegCount; i++) {
    EEPROM.get(addr, LearnSegTime[i]);
    addr += sizeof(unsigned long);
    EEPROM.get(addr, LearnSegErr[i]);
    addr += sizeof(int);
    LearnSegCurve[i] = EEPROM.read(addr++);
  }
  Serial.println("Learn loaded");
  return true;
}

// พิมพ์ผลสำรวจออก Serial Monitor ใช้จูน SetCurveDetect() / SetBrakeMargin()
void SerialLearn() {
  Serial.print("Learn speed = ");
  Serial.print(LearnSpeed);
  Serial.print("  Segments = ");
  Serial.println(LearnSegCount);
  for (int i = 0; i < LearnSegCount; i++) {
    Serial.print("#");
    Serial.print(i);
    Serial.print("  time = ");
    Serial.print(LearnSegTime[i]);
    Serial.print(" ms  err = ");
    Serial.print(LearnSegErr[i]);
    Serial.println(LearnSegCurve[i] ? "  CURVE" : "  STRAIGHT");
  }
}

// ทดสอบ marker: เข็นหุ่นด้วยมือไปตามสนาม ดูผลที่ Serial Monitor (115200)
// ใช้เช็คว่าฝั่ง Goal/Curve ถูก และจุดตัดไม่ถูกนับ
void SerialMarker() {
  int goal = 0, curve = 0;
  ResetMarker();
  while (1) {
    ReadCalibrate();
    int mk = CheckMarker();
    if (mk == MK_GOAL) {
      goal++;
      Serial.print("GOAL  #");
      Serial.println(goal);
      Beep(30);
    } else if (mk == MK_CURVE) {
      curve++;
      Serial.print("CURVE #");
      Serial.println(curve);
      Beep(10);
    }
  }
}
