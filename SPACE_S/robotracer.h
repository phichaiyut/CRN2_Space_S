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

// วิ่งความเร็วเดียวจาก Start ถึง Goal แล้วหยุด
// RoboTracer(ความเร็ว, Kp, Kd, จำนวน Goal marker ที่จะหยุด, เวลาวิ่งต่อหลัง Goal ms);
// ปกติ target_Marker = 2 (อันแรก = Start, อันที่สอง = Goal)
void RoboTracer(int Speed, float Kp, float Kd, int target_Marker, int total_time) {
  Serial.print("RoboTracer start spd=");
  Serial.println(Speed); // DEBUG
  BaseSpeed = Speed;
  InitialSpeed();
  ResetMarker();
  int goal = 0;
  while (1) {
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
int RampUp = 1;   // เร่งได้ทีละกี่ % ต่อ 10 ms (1 = 40->80 ใช้ 0.4 วินาที)
int RampDown = 4; // ลดได้ทีละกี่ % ต่อ 10 ms (ต้องลดเร็วพอให้ทันก่อนเข้าโค้ง)

// SetSpeedRamp(เร่ง, ลด); หน่วย % ต่อ 10 ms, ใส่ค่าใหญ่ๆ เช่น 100 = เปลี่ยนทันที
void SetSpeedRamp(int up, int down) {
  RampUp = up;
  RampDown = down;
}

void RoboTracerSegment(const int Spd[], const float Kp[], const float Kd[], int numSeg,
                       int target_Marker, int total_time) {
  int seg = 0;
  int goal = 0;
  BaseSpeed = Spd[0];
  InitialSpeed();
  float kp = Kp ? Kp[0] : PID_KP;
  float kd = Kd ? Kd[0] : PID_KD;

  int spdFrom = BaseSpeed, spdTo = BaseSpeed; // ramp จากความเร็วไหน ไปความเร็วไหน
  float kpFrom = kp, kdFrom = kd;
  unsigned long lastRamp = millis();

  ResetMarker();
  while (1) {
    PID(LeftBaseSpeed, RightBaseSpeed, kp, kd);
    int mk = CheckMarker();
    if (mk == MK_GOAL) {
      goal++;
      tone(BUZZER_PIN, volumesound, 30);
      Serial.print("GOAL ");
      Serial.print(goal);
      Serial.print(" t=");
      Serial.println(millis()); // DEBUG
      if (goal >= target_Marker)
        break;
    } else if (mk == MK_CURVE && goal >= 1 && seg < numSeg - 1) {
      seg++;
      spdFrom = BaseSpeed; // เริ่ม ramp จากความเร็วปัจจุบัน (เผื่อ ramp เดิมยังไม่จบ)
      spdTo = Spd[seg];
      kpFrom = kp;
      kdFrom = kd;
      tone(BUZZER_PIN, volumesound / 2, 20);
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
        kp = Kp ? kpFrom + (Kp[seg] - kpFrom) * f : PID_KP;
        kd = Kd ? kdFrom + (Kd[seg] - kdFrom) * f : PID_KD;
      } else {
        kp = PID_KP;
        kd = PID_KD;
      }
    }
  }
  RoboTracerStop(BaseSpeed, kp, kd, total_time);
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
// ค่าที่จำเก็บใน RAM -> ปิดเครื่องแล้วหาย ต้องสำรวจใหม่
#define MAX_SEG 32

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

    if (endSeg && LearnSegCount < MAX_SEG) {
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

// วิ่งเร็วจากที่สำรวจไว้ ใช้ Kp/Kd จากตาราง setKpKd()
// RoboTracerRun(ความเร็วทางตรง, ความเร็วโค้ง, จำนวน Goal marker ที่จะหยุด, เวลาวิ่งต่อหลัง Goal ms);
void RoboTracerRun(int SpdStraight, int SpdCurve, int target_Marker, int total_time) {
  if (LearnSegCount == 0) { // ยังไม่ได้สำรวจ
    BeepBeep();
    return;
  }
  int Spd[MAX_SEG];
  for (int i = 0; i < LearnSegCount; i++)
    Spd[i] = LearnSegCurve[i] ? SpdCurve : SpdStraight;
  RoboTracerSegment(Spd, NULL, NULL, LearnSegCount, target_Marker, total_time);
}

// แบบกำหนด Kp/Kd เอง แยกทางตรง/โค้ง
// RoboTracerRun(ความเร็วตรง, Kp ตรง, Kd ตรง, ความเร็วโค้ง, Kp โค้ง, Kd โค้ง, จำนวน Goal marker, เวลาวิ่งต่อ ms);
void RoboTracerRun(int SpdStraight, float KpS, float KdS, int SpdCurve, float KpC, float KdC,
                   int target_Marker, int total_time) {
  if (LearnSegCount == 0) {
    BeepBeep();
    return;
  }
  int Spd[MAX_SEG];
  float Kp[MAX_SEG], Kd[MAX_SEG];
  for (int i = 0; i < LearnSegCount; i++) {
    Spd[i] = LearnSegCurve[i] ? SpdCurve : SpdStraight;
    Kp[i] = LearnSegCurve[i] ? KpC : KpS;
    Kd[i] = LearnSegCurve[i] ? KdC : KdS;
  }
  RoboTracerSegment(Spd, Kp, Kd, LearnSegCount, target_Marker, total_time);
}

// พิมพ์ผลสำรวจออก Serial Monitor ใช้จูน SetCurveDetect()
void SerialLearn() {
  Serial.print("Segments = ");
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
