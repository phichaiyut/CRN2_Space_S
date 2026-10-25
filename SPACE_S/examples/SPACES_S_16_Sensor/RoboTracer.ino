void RoboTracer50() {
  RoboTracer(50, 0.012, 0.08, 2, 200);
  MotorStop();
}
void RoboTracer55() {
  RoboTracer(50, 0.014, 0.10, 2, 200);
  MotorStop();
}
void RoboTracer60() {
  RoboTracer(60, 0.014, 0.10, 2, 100);
  MotorStop();
}
void RoboTracer65() {
  RoboTracer(60, 0.016, 0.16, 2, 100);
  MotorStop();
}
void RoboTracer70() {
  RoboTracer(70, 0.016, 0.12, 2, 100);
  MotorStop();
}
void RoboTracer75() {
  RoboTracer(70, 0.018, 0.16, 2, 100);
  MotorStop();
}
void RoboTracer80() {
  RoboTracer(80, 0.018, 0.14, 2, 100);
  MotorStop();
}
void RoboTracer85() {
  RoboTracer(80, 0.022, 0.18, 2, 100);
  MotorStop();
}
void RoboTracer90() {
  RoboTracer(90, 0.020, 0.18, 2, 100);
  MotorStop();
}
void RoboTracer95() {
  RoboTracer(90, 0.026, 0.22, 2, 100);
  MotorStop();
}

void RoboTracer100() {
  RoboTracer(100, 0.028, 0.20, 2, 100);
  MotorStop();
}
void RoboTracer105() {
  RoboTracer(100, 0.020, 0.26, 2, 100);
  MotorStop();
}
// ===== สำรวจรอบแรก แล้ววิ่งเร็วจากที่จำไว้ (เบรกล่วงหน้าก่อนเข้าโค้ง) =====
// สำรวจ: วิ่งช้าๆ จำว่าแต่ละช่วงตรงหรือโค้ง แล้วเก็บลง EEPROM (ปิดเครื่องแล้วไม่หาย)
void RoboTracerLearn40() {
  RoboTracerLearn(40, 0.012, 0.08, 2, 200);
  SaveLearn();
  SerialLearn(); // ดูผลที่ Serial Monitor
  MotorStop();
}
// วิ่งเร็ว: ถ้ายังไม่ได้สำรวจในรอบนี้ จะโหลดผลสำรวจจาก EEPROM ให้
// RoboTracerRun(ความเร็วตรง, Kp ตรง, Kd ตรง, ความเร็วโค้ง, Kp โค้ง, Kd โค้ง, Goal marker, วิ่งต่อ ms);
//   Kp/Kd ชุดเดียวทั้งตรงและโค้ง: RoboTracerRun(ความเร็วตรง, ความเร็วโค้ง, Kp, Kd, Goal marker, วิ่งต่อ ms);
//     เช่น RoboTracerRun(70, 45, 0.016, 0.12, 2, 100);
//   ไม่อยากตั้ง Kp/Kd เอง ใช้ RoboTracerRun(ความเร็วตรง, ความเร็วโค้ง, Goal marker, วิ่งต่อ ms); = ใช้ค่าจาก setspeed.ino
void RoboTracerRun70() {
  if (LearnSegCount == 0)
    LoadLearn();
  RoboTracerRun(70, 0.016, 0.12, 45, 0.012, 0.08, 2, 100);
  MotorStop();
}
void RoboTracerRun80() {
  if (LearnSegCount == 0)
    LoadLearn();
  RoboTracerRun(80, 0.018, 0.14, 45, 0.012, 0.08, 2, 100);
  MotorStop();
}
void RoboTracerRun90() {
  if (LearnSegCount == 0)
    LoadLearn();
  RoboTracerRun(90, 0.020, 0.18, 50, 0.012, 0.08, 2, 100);
  MotorStop();
}

// วิ่งเร็ว แบบ Kp/Kd ชุดเดียวทั้งตรงและโค้ง (เปลี่ยนแค่ความเร็ว ถ้าแบบแยก 2 ชุดแล้วกระตุก)
// RoboTracerRun(ความเร็วตรง, ความเร็วโค้ง, Kp, Kd, Goal marker, วิ่งต่อ ms);
void RoboTracerRun70One() {
  if (LearnSegCount == 0)
    LoadLearn();
  RoboTracerRun(70, 45, 0.016, 0.12, 2, 100);
  MotorStop();
}
void RoboTracerRun80One() {
  if (LearnSegCount == 0)
    LoadLearn();
  RoboTracerRun(80, 45, 0.018, 0.14, 2, 100);
  MotorStop();
}
void RoboTracerRun90One() {
  if (LearnSegCount == 0)
    LoadLearn();
  RoboTracerRun(90, 50, 0.020, 0.18, 2, 100);
  MotorStop();
}
