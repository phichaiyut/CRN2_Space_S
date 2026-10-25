// ===== PID / Line Tracking =====
// ใช้ตัวแปรและฟังก์ชันจาก SPACE_S_16_Sensor.h (F, ReadCalibrate, Motor ฯลฯ)
// ถูก include จาก SPACE_S_16_Sensor.h ไม่ต้อง include เองใน sketch

#pragma once

void ReadCalibrate();                                         // ประกาศไว้ล่วงหน้า (ตัวจริงอยู่ใน SPACE_S_16_Sensor.h)
void ReadCalibrateC();                                        // ประกาศไว้ล่วงหน้า (ตัวจริงอยู่ใน SPACE_S_16_Sensor.h)
void ReadC();                                                 // ประกาศไว้ล่วงหน้า (ตัวจริงอยู่ใน SPACE_S_16_Sensor.h)
int md_sensorC(int ch);                                       // ประกาศไว้ล่วงหน้า (ตัวจริงอยู่ใน SPACE_S_16_Sensor.h)
void TrackTime(int Speed, float Kp, float Kd, int TotalTime); // ประกาศไว้ล่วงหน้า (ตัวจริงอยู่ใน SPACE_S_16_Sensor.h)

/* ================== ตารางความเร็ว / KP KD ================== */
#define SPD_10 0
#define SPD_20 1
#define SPD_30 2
#define SPD_40 3
#define SPD_50 4
#define SPD_60 5
#define SPD_70 6
#define SPD_80 7
#define SPD_90 8
#define SPD_100 9

int BaseSpeed;
float PID_KP, PID_KD;
int L[10], R[10];   // ชดเชยความเร็ว เดินหน้า
int BL[10], BR[10]; // ชดเชยความเร็ว ถอยหลัง
float KP[10], KD[10];

void setBalanceSpeed(int ch, int spdL, int spdR) {
  L[ch] = spdL;
  R[ch] = spdR;
}

void setBalanceBackSpeed(int ch, int spdL, int spdR) {
  BL[ch] = spdL;
  BR[ch] = spdR;
}

void setKpKd(int ch, float kp, float kd) {
  KP[ch] = kp;
  KD[ch] = kd;
}

void InitialSpeed() {
  if (BaseSpeed <= 10) {
    LeftBaseSpeed = BaseSpeed + L[SPD_10];
    RightBaseSpeed = BaseSpeed + R[SPD_10];
    BackLeftBaseSpeed = BaseSpeed + BL[SPD_10];
    BackRightBaseSpeed = BaseSpeed + BR[SPD_10];
    PID_KP = KP[SPD_10]; // forward PID
    PID_KD = KD[SPD_10];
  } else if (BaseSpeed <= 20) {
    LeftBaseSpeed = BaseSpeed + L[SPD_20];
    RightBaseSpeed = BaseSpeed + R[SPD_20];
    BackLeftBaseSpeed = BaseSpeed + BL[SPD_20];
    BackRightBaseSpeed = BaseSpeed + BR[SPD_20];
    PID_KP = KP[SPD_20]; // forward PID
    PID_KD = KD[SPD_20];
  } else if (BaseSpeed <= 30) {
    LeftBaseSpeed = BaseSpeed + L[SPD_30];
    RightBaseSpeed = BaseSpeed + R[SPD_30];
    BackLeftBaseSpeed = BaseSpeed + BL[SPD_30];
    BackRightBaseSpeed = BaseSpeed + BR[SPD_30];
    PID_KP = KP[SPD_30]; // forward PID
    PID_KD = KD[SPD_30];
  } else if (BaseSpeed <= 40) {
    LeftBaseSpeed = BaseSpeed + L[SPD_40];
    RightBaseSpeed = BaseSpeed + R[SPD_40];
    BackLeftBaseSpeed = BaseSpeed + BL[SPD_40];
    BackRightBaseSpeed = BaseSpeed + BR[SPD_40];
    PID_KP = KP[SPD_40]; // forward PID
    PID_KD = KD[SPD_40];
  } else if (BaseSpeed <= 50) {
    LeftBaseSpeed = BaseSpeed + L[SPD_50];
    RightBaseSpeed = BaseSpeed + R[SPD_50];
    BackLeftBaseSpeed = BaseSpeed + BL[SPD_50];
    BackRightBaseSpeed = BaseSpeed + BR[SPD_50];
    PID_KP = KP[SPD_50]; // forward PID
    PID_KD = KD[SPD_50];
  } else if (BaseSpeed <= 60) {
    LeftBaseSpeed = BaseSpeed + L[SPD_60];
    RightBaseSpeed = BaseSpeed + R[SPD_60];
    BackLeftBaseSpeed = BaseSpeed + BL[SPD_60];
    BackRightBaseSpeed = BaseSpeed + BR[SPD_60];
    PID_KP = KP[SPD_60]; // forward PID
    PID_KD = KD[SPD_60];
  } else if (BaseSpeed <= 70) {
    LeftBaseSpeed = BaseSpeed + L[SPD_70];
    RightBaseSpeed = BaseSpeed + R[SPD_70];
    BackLeftBaseSpeed = BaseSpeed + BL[SPD_70];
    BackRightBaseSpeed = BaseSpeed + BR[SPD_70];
    PID_KP = KP[SPD_70]; // forward PID
    PID_KD = KD[SPD_70];
  } else if (BaseSpeed <= 80) {
    LeftBaseSpeed = BaseSpeed + L[SPD_80];
    RightBaseSpeed = BaseSpeed + R[SPD_80];
    BackLeftBaseSpeed = BaseSpeed + BL[SPD_80];
    BackRightBaseSpeed = BaseSpeed + BR[SPD_80];
    PID_KP = KP[SPD_80]; // forward PID
    PID_KD = KD[SPD_80];
  } else if (BaseSpeed <= 90) {
    LeftBaseSpeed = BaseSpeed + L[SPD_90];
    RightBaseSpeed = BaseSpeed + R[SPD_90];
    BackLeftBaseSpeed = BaseSpeed + BL[SPD_90];
    BackRightBaseSpeed = BaseSpeed + BR[SPD_90];
    PID_KP = KP[SPD_90]; // forward PID
    PID_KD = KD[SPD_90];
  } else {
    LeftBaseSpeed = BaseSpeed + L[SPD_100];
    RightBaseSpeed = BaseSpeed + R[SPD_100];
    BackLeftBaseSpeed = BaseSpeed + BL[SPD_100];
    BackRightBaseSpeed = BaseSpeed + BR[SPD_100];
    PID_KP = KP[SPD_100]; // forward PID
    PID_KD = KD[SPD_100];
  }
}

void ModeSpdPID(int moD, int maX, int miN) {
  ModePidStatus = moD;
  MaxSpeed = maX;
  MinSpeed = miN;
}

void Dottedline(int Dot) {
  dottedline = Dot;
}

int readPosition(int Track, int noise) {
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned int sum = 0;
  static int last_value = 0;
  ReadCalibrate();
  for (i = sensor_L; i <= sensor_R; i++) {
    int values = F[i];
    if (values > Track) {
      online = 1;
    }
    if (values > noise) {
      avg += (long)(values) * (i * 1000);
      sum += values;
    }
  }
  if (!online) {
    if (dottedline) {
      return last_value;
    }
    if (last_value < (NUM_SENSORS - 1) * 1000) {
      return 0;
    } else {
      return 15000;
    }
  }
  last_value = avg / sum;
  return last_value;
}

void PID(int SpeedL, int SpeedR, float Kp, float Kd) {
  int Pos = readPosition(250, 50);
  int Error = Pos - 7500;

  int PID_Value = (Kp * Error) + (Kd * (Error - LastError));
  LastError = Error;
  int LeftPower = SpeedL + PID_Value;
  int RightPower = SpeedR - PID_Value;
  switch (ModePidStatus) {
    case 0:
      if (LeftPower > MaxSpeed)
        LeftPower = MaxSpeed;
      if (LeftPower < 0)
        LeftPower = MinSpeed;
      if (RightPower > MaxSpeed)
        RightPower = MaxSpeed;
      if (RightPower < 0)
        RightPower = MinSpeed;
      break;
    case 1:
      if (LeftPower > MaxSpeed)
        LeftPower = MaxSpeed;
      if (LeftPower < MinSpeed)
        LeftPower = MinSpeed;
      if (RightPower > MaxSpeed)
        RightPower = MaxSpeed;
      if (RightPower < MinSpeed)
        RightPower = MinSpeed;
      break;
    case 2:
      if (LeftPower > LeftBaseSpeed)
        LeftPower = LeftBaseSpeed;
      if (LeftPower < -LeftBaseSpeed)
        LeftPower = -LeftBaseSpeed;
      if (RightPower > RightBaseSpeed)
        RightPower = RightBaseSpeed;
      if (RightPower < -RightBaseSpeed)
        RightPower = -RightBaseSpeed;
      break;
    case 3:
      if (LeftPower > MaxSpeed)
        LeftPower = MaxSpeed;
      if (LeftPower < 0)
        LeftPower = -SpeedL;
      if (RightPower > MaxSpeed)
        RightPower = MaxSpeed;
      if (RightPower < 0)
        RightPower = -SpeedR;
      break;
    default:
      if (LeftPower > MaxSpeed)
        LeftPower = MaxSpeed;
      if (LeftPower < 0)
        LeftPower = 0;
      if (RightPower > MaxSpeed)
        RightPower = MaxSpeed;
      if (RightPower < 0)
        RightPower = 0;
  }
  Motor(LeftPower, RightPower);
}

void SerialPosition() {
  while (1) {
    int Pos = readPosition(250, 50);
    int Error_F = Pos - (NUM_SENSORS - 1) * 1000 / 2;
    Serial.print("Pos = ");
    Serial.print(Pos);
    Serial.print("                  Error = ");
    Serial.println(Error_F);
    delay(50);
  }
}

// RoboTracer ย้ายไปอยู่ใน robotracer.h (ต้องใช้ TrackTime ที่ประกาศทีหลัง)
