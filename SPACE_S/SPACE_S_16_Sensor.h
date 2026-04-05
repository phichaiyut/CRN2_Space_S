#include "Motor.h"
#include <light_CD74HC4067.h>
#include <EEPROM.h>


#define SWA_PIN 7
#define SWB_PIN 4
#define PRESS 0
#define NUM_SENSORS 16
/* ================== GLOBAL ================== */
// uint8_t  NUM_SENSORS = 8;          // เลือก 8 หรือ 16 ได้
uint16_t MinValue[NUM_SENSORS];
uint16_t MaxValue[NUM_SENSORS];
uint16_t F[NUM_SENSORS];

CD74HC4067 my_mux(18, 17, 16, 15);
const int signal_pin = A0;

int BUZZER_PIN = 10;

int LastError;
int Ref = 500;
int volumesound = 2700;
int Dist = 11;
uint8_t F_PIN[NUM_SENSORS] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

int sensorValues[NUM_SENSORS];
int LCh, LTurnSpdL, LTurnSpdR, TurnDelayL;
int RCh, RTurnSpdL, RTurnSpdR, TurnDelayR;
int LeftBaseSpeed, RightBaseSpeed, BackLeftBaseSpeed, BackRightBaseSpeed;
int LineColor = 0;
int MaxSpeed = 100;
int MinSpeed = -25;
int ModePidStatus = 0;
int sensor_L = 0;
int sensor_R = 15;
unsigned char dottedline = 0;  // ตรวจเส้นประ (1 = ใช่)

void RobotSetup() {
  Serial.begin(115200);
   Serial.println("Robot Ready");
  pinMode(SWA_PIN, INPUT);
  pinMode(SWB_PIN, INPUT);
  pinMode(PH1, OUTPUT);
  pinMode(EN1, OUTPUT);  
  pinMode(PH2, OUTPUT);
  pinMode(EN2, OUTPUT);
   pinMode(BUZZER_PIN, OUTPUT);
   pinMode(Dist, INPUT);
   pinMode(signal_pin, INPUT);
  tone(BUZZER_PIN, 2000, 80);
  delay(100);
  tone(BUZZER_PIN, 1000, 80);
  delay(100);
  tone(BUZZER_PIN, 2000, 150);
  delay(100);
}

void BeepBeep(){
    tone(BUZZER_PIN, 2000, 80);
  delay(100);
  tone(BUZZER_PIN, 1000, 80);
  delay(100);
  tone(BUZZER_PIN, 2000, 150);
  delay(100);

}
void Beep(int delayb) {
  tone(BUZZER_PIN, volumesound, delayb);
  delay(delayb);
  noTone(BUZZER_PIN);
}
void BZON() {
  tone(BUZZER_PIN, volumesound);  // ส่งเสียง 1 kHz
}
void BZOFF() {
  noTone(BUZZER_PIN);  // หยุดเสียง
}


void SetValueLine(int x){
  Ref = x;
}

void SetDistance(int x){
    Dist = x;
}

void SetTrackLineCh(int sl, int sr){
    sensor_L = sl;
    sensor_R = sr;
}

int analog(int __ch) {
  if(__ch==0) return analogRead(A0);
  if(__ch==1) return analogRead(A1);
  if(__ch==2) return analogRead(A2);
  if(__ch==3) return analogRead(A3);
  if(__ch==4) return analogRead(A4);
  if(__ch==5) return analogRead(A5);
  if(__ch==6) return analogRead(A6);
  if(__ch==7) return analogRead(A7);
}



int digital(int __ch) {
  if(__ch==10) return digitalRead(10);
  if(__ch==11) return digitalRead(11);
  if(__ch==12) return digitalRead(12);
  if(__ch==13) return digitalRead(13);
}

void SW_A() {
  MotorStop();
  while (1) {
    if (digitalRead(SWA_PIN) == PRESS) {
      break;
    }
  }
  delay(200);
}

void SW_B() {
  MotorStop();
  while (1) {
    if (digitalRead(SWB_PIN) == PRESS) {
      break;
    }
  }
  delay(200);
}

int SW_A_PRESS() {
  if (digitalRead(SWA_PIN) == PRESS) return PRESS;
  else {
    return 1;
  }
}

int SW_B_PRESS() {
  if (digitalRead(SWB_PIN) == PRESS) return PRESS;
  else {
    return 1;
  }
}

// void SensorValue(uint16_t minF0, uint16_t minF1, uint16_t minF2, uint16_t minF3, uint16_t minF4, uint16_t minF5, uint16_t minF6, uint16_t minF7, uint16_t maxF0, uint16_t maxF1, uint16_t maxF2, uint16_t maxF3, uint16_t maxF4, uint16_t maxF5, uint16_t maxF6, uint16_t maxF7) {
//   MinValue[0] = minF0;
//   MinValue[1] = minF1;
//   MinValue[2] = minF2;
//   MinValue[3] = minF3;
//   MinValue[4] = minF4;
//   MinValue[5] = minF5;
//   MinValue[6] = minF6;
//   MinValue[7] = minF7;

//   MaxValue[0] = maxF0;
//   MaxValue[1] = maxF1;
//   MaxValue[2] = maxF2;
//   MaxValue[3] = maxF3;
//   MaxValue[4] = maxF4;
//   MaxValue[5] = maxF5;
//   MaxValue[6] = maxF6;
//   MaxValue[7] = maxF7;
// }

void SensorValue(uint16_t minF0, uint16_t minF1, uint16_t minF2, uint16_t minF3, uint16_t minF4, uint16_t minF5, uint16_t minF6, uint16_t minF7, uint16_t minF8, uint16_t minF9, uint16_t minF10, uint16_t minF11, uint16_t minF12, uint16_t minF13, uint16_t minF14, uint16_t minF15, uint16_t maxF0, uint16_t maxF1, uint16_t maxF2, uint16_t maxF3,
                 uint16_t maxF4, uint16_t maxF5, uint16_t maxF6, uint16_t maxF7, uint16_t maxF8, uint16_t maxF9, uint16_t maxF10, uint16_t maxF11, uint16_t maxF12, uint16_t maxF13, uint16_t maxF14, uint16_t maxF15) {
  MinValue[0] = minF0;
  MinValue[1] = minF1;
  MinValue[2] = minF2;
  MinValue[3] = minF3;
  MinValue[4] = minF4;
  MinValue[5] = minF5;
  MinValue[6] = minF6;
  MinValue[7] = minF7;
  MinValue[8] = minF8;
  MinValue[9] = minF9;
  MinValue[10] = minF10;
  MinValue[11] = minF11;
  MinValue[12] = minF12;
  MinValue[13] = minF13;
  MinValue[14] = minF14;
  MinValue[15] = minF15;

  MaxValue[0] = maxF0;
  MaxValue[1] = maxF1;
  MaxValue[2] = maxF2;
  MaxValue[3] = maxF3;
  MaxValue[4] = maxF4;
  MaxValue[5] = maxF5;
  MaxValue[6] = maxF6;
  MaxValue[7] = maxF7;
  MaxValue[8] = maxF8;
  MaxValue[9] = maxF9;
  MaxValue[10] = maxF10;
  MaxValue[11] = maxF11;
  MaxValue[12] = maxF12;
  MaxValue[13] = maxF13;
  MaxValue[14] = maxF14;
  MaxValue[15] = maxF15;

}



int analog16(int channel) {
  my_mux.channel(channel);
  int val = analogRead(signal_pin);
  return val;
}

void Read() {
  for (int _sensor = 0; _sensor < NUM_SENSORS; _sensor++) {
    F[_sensor] = analog16(F_PIN[_sensor]);
  }
}



void ReadCalibrate() {
  Read();
  for (int i = 0; i < NUM_SENSORS; i++) {
    int x;
    if (LineColor == 0)
      x = map(F[i], MinValue[i], MaxValue[i], 1000, 0);
    else
      x = map(F[i], MinValue[i], MaxValue[i], 0, 1000);
    if (x < 200)   x = 0;
    if (x > 900)  x = 1000;
    if (x < 0)    x = 0;
    if (x > 1000) x = 1000;
    F[i] = x;
  }
}


void TurnSpeedLeft(int Ch, int l, int r, int de) {
  LCh = Ch;
  LTurnSpdL = l;
  LTurnSpdR = r;
  TurnDelayL = de;
}

void TurnSpeedRight(int Ch, int l, int r, int de) {
  RCh = Ch;
  RTurnSpdL = l;
  RTurnSpdR = r;
  TurnDelayR = de;
}

void TrackLineColor(int Col) {
  LineColor = Col;
}


#define EEPROM_FLAG_ADDR 100
#define EEPROM_DATA_START 0   // จุดเริ่มเก็บ Min/Max

void WriteEEPROM() {

  for (int i = 0; i < NUM_SENSORS; i++) {

    // กันค่าผิดพลาด
    if (MinValue[i] > 1023) MinValue[i] = 1023;
    if (MaxValue[i] > 1023) MaxValue[i] = 1023;
    if (MinValue[i] < 0)    MinValue[i] = 0;
    if (MaxValue[i] < 0)    MaxValue[i] = 0;

    int addr = EEPROM_DATA_START + (i * 4); // 1 sensor = 4 bytes

    EEPROM.put(addr,     (uint16_t)MinValue[i]); // 2 bytes
    EEPROM.put(addr + 2, (uint16_t)MaxValue[i]); // 2 bytes
  }

  EEPROM.update(EEPROM_FLAG_ADDR, 0xAA); // flag ว่าคาลิเบรตแล้ว
  Serial.println("Calibrate complete (uint16_t saved)");
}
void CalibrateSensor(int pauseTime, int samples) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    MinValue[i] = 1000;
    MaxValue[i] = 0;
  }
  for (int startSamp = 0; startSamp <= samples; startSamp++) {
     Read();
    for (int i = 0; i < NUM_SENSORS; i++) {
      MinValue[i] = (F[i] <= MinValue[i] ? F[i] : MinValue[i]);
      MaxValue[i] = (F[i] >= MaxValue[i] ? F[i] : MaxValue[i]);
    }
    delay(pauseTime);
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
    MinValue[i] += 10;
    MaxValue[i] -= 40;
  }
}

void ReadEEPROM() {
  if (EEPROM.read(100) != 0xAA) {
    Serial.println(F("EEPROM not calibrated"));
    return;
  }


  for (int i = 0; i < NUM_SENSORS; i++) {
    int addr = i * 4;

    EEPROM.get(addr, MinValue[i]);
    EEPROM.get(addr + 2, MaxValue[i]);

    // แสดงผลทาง Serial
   Serial.println("Load EEPROM");
  Serial.println("  ");
  Serial.print("SensorValue (");
  //MinValue
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(MinValue[i]);
    if (i < NUM_SENSORS - 1) Serial.print(",");
    else Serial.print(",");
  }
  //MaxValue
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(MaxValue[i]);
    if (i < NUM_SENSORS - 1) Serial.print(",");
    else Serial.println(");");
  }
  }

  
}

void CaliberateRobotSensor() {
  Serial.println("Press SW_B to start caribrate Front Sensor");
  SW_A();
  Serial.println("Caribrating");
  delay(500);
  CalibrateSensor(20, 200);
  delay(500);
  Serial.println("Finish");
  Serial.println("  ");
  Serial.print("SensorValue (");
  //MinValue
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(MinValue[i]);
    if (i < NUM_SENSORS - 1) Serial.print(",");
    else Serial.print(",");
  }
  //MaxValue
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(MaxValue[i]);
    if (i < NUM_SENSORS - 1) Serial.print(",");
    else Serial.println(");");
  }
  WriteEEPROM() ;
//   while (1)
//     ;
}
void SerialSensor() {
  while (1) {
  Read();
    for (int _serialF = 0; _serialF < NUM_SENSORS; _serialF++) {
      Serial.print(F[_serialF]);
      Serial.print("\t");
    }
    Serial.println("");
    delay(50);
  }
}



void SerialCalibrateSensor() {
  while (1) {
     ReadCalibrate();
    for (int _serialF = 0; _serialF < NUM_SENSORS; _serialF++) {
      Serial.print(F[_serialF]);
      Serial.print("\t");
    }
    Serial.println("");
    delay(100);
  }
}

void SW() {
  MotorStop();

  while (1) {

    if (SW_B_PRESS() == PRESS) {
      delay(20);                 // debounce
      while (SW_B_PRESS() == PRESS); // รอปล่อย
      break;                     // ออกโหมดตั้งค่า
    }

    if (SW_A_PRESS() == PRESS) {
      delay(20);                 // debounce
      BeepBeep();
      while (SW_A_PRESS() == PRESS); // รอปล่อย
      CaliberateRobotSensor();   // calibrate ครั้งเดียว
      BeepBeep();
    }
  }
}




void ModeSpdPID(int moD, int maX, int miN){
  ModePidStatus = moD;
  MaxSpeed = maX;
  MinSpeed = miN;
}




void Dottedline(int Dot){
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
    if (last_value < (NUM_SENSORS - 1) * 1000) 
    {
      return 0;
    } else {
      return 15000;
    }
  }
  last_value = avg / sum;
  return last_value;
}

void PID(int Speed, float Kp, float Kd) {
  
  int Pos = readPosition(250, 50);
   int Error = Pos - ((NUM_SENSORS - 1) * 1000 / 2);

  int PID_Value = (Kp * Error) + (Kd * (Error - LastError));
  LastError = Error;
  int LeftPower = Speed + PID_Value;
  int RightPower = Speed - PID_Value;
  switch (ModePidStatus) {
    case 0:
      if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
      if (LeftPower < 0) LeftPower = MinSpeed;
      if (RightPower > MaxSpeed) RightPower = MaxSpeed;
      if (RightPower < 0) RightPower = MinSpeed;
      break;
    case 1:
      if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
      if (LeftPower < MinSpeed) LeftPower = MinSpeed;
      if (RightPower > MaxSpeed) RightPower = MaxSpeed;
      if (RightPower < MinSpeed) RightPower = MinSpeed;
      break;
    case 2:
      if (LeftPower > LeftBaseSpeed) LeftPower = LeftBaseSpeed;
      if (LeftPower < -LeftBaseSpeed) LeftPower = -LeftBaseSpeed;
      if (RightPower > RightBaseSpeed) RightPower = RightBaseSpeed;
      if (RightPower < -RightBaseSpeed) RightPower = -RightBaseSpeed;
      break;
    case 3:
        if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
      if (LeftPower < 0) LeftPower = -Speed;
      if (RightPower > MaxSpeed) RightPower = MaxSpeed;
      if (RightPower < 0) RightPower = -Speed;
      break;
    default:
      if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
      if (LeftPower < 0) LeftPower = 0;
      if (RightPower > MaxSpeed) RightPower = MaxSpeed;
      if (RightPower < 0) RightPower = 0;
  }
  Motor(LeftPower, RightPower);
}


void TurnLeft() {
  Motor(-LTurnSpdL, LTurnSpdR);
  delay(TurnDelayL);
  while (1) {
    Motor(-LTurnSpdL, LTurnSpdR);
    ReadCalibrate();
    if (F[LCh] >= Ref) {
      MotorStop();
      break;
    }
  }
}

void TurnRight() {
  Motor(RTurnSpdL, -RTurnSpdR);
  delay(TurnDelayR);
  while (1) {
    Motor(RTurnSpdL, -RTurnSpdR);
    ReadCalibrate();
    if (F[RCh] >= Ref) {
      MotorStop();
      break;
    }
  }
}

void TrackSelect(int spd, char x) {
  if (x == 's') {
    MotorStop();
    Beep(50);
  } else if (x == 'p') {
    BZON();
    Motor(spd, spd);
    delay(5);
    while (1) {
      ReadCalibrate();
      if (F[4] < Ref && F[11] < Ref) {
        BZOFF();
        delay(50);
        break;
      }
      Motor(spd, spd);
    }
  } else if (x == 'l') {
    while (1) {
      BZON();
      Motor(spd / 2, spd / 2);
      ReadCalibrate();
      if (F[4] < Ref && F[11] < Ref) {
        BZOFF();
        break;
      }
    }
    TurnLeft();
  } else if (x == 'r') {
    while (1) {
      BZON();
      Motor(spd / 2, spd / 2);
      ReadCalibrate();
      if (F[4] < Ref && F[11] < Ref) {
        BZOFF();
        break;
      }
    }
    TurnRight();
  } else if (x == 'f') {
    Motor(spd, spd);
    Beep(30);
  }
}

void TrackCrossL(int Speed, float Kp, float Kd, char select) {
  while (1) {
    ReadCalibrate();
    if (F[4] >= Ref && F[7] >= Ref) {
      break;
    }
    PID(Speed, Kp, Kd);
  }
  TrackSelect(Speed, select);
}

void TrackCrossR(int Speed, float Kp, float Kd, char select) {
  while (1) {
    ReadCalibrate();
    if (F[8] >= Ref && F[11] >= Ref) {
      break;
    }
    PID(Speed, Kp, Kd);
  }
  TrackSelect(Speed, select);
}

void TrackCrossC(int Speed, float Kp, float Kd, char select) {
  while (1) {
    ReadCalibrate();
    if (F[5] >= Ref && F[10] >= Ref) {
      break;
    }
    PID(Speed, Kp, Kd);
  }
  TrackSelect(Speed, select);
}

void TrackCross(int Speed, float Kp, float Kd, char select) {
  while (1) {
    ReadCalibrate();
    if ((F[4] >= Ref && F[7] >= Ref) || (F[5] >= Ref && F[10] >= Ref) || (F[8] >= Ref && F[11] >= Ref)) {
      break;
    }
    PID(Speed, Kp, Kd);
  }
  TrackSelect(Speed, select);
}

void TrackTime(int Speed, float Kp, float Kd, int TotalTime) {
  unsigned long StartTime = millis();
  unsigned long EndTime = StartTime + TotalTime;
  while (millis() <= EndTime) {
    PID(Speed, Kp, Kd);
  }
}

void TrackDistance(int Speed, float Kp, float Kd) {
  while (1) {
    PID(Speed, Kp, Kd);
    if (digitalRead(Dist) == 0) {
      MotorStop();
      Beep(10);
      break;
    }
  }
}
void TrackDistanceUntilObstacle(int Speed, float Kp, float Kd) {
  while (1) {
    PID(Speed, Kp, Kd);
    if (digitalRead(Dist) == 0) {
      MotorStop();
      Beep(10);

      // รอให้วัตถุหายไป โดยเช็กทุก 100 ms
      while (digitalRead(Dist) == 0) {
        Beep(100);
        delay(50);
      }

      break;
    }
  }
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





