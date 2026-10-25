void Mission() {
  TrackLineColor(1);  // สีของเส้น 1 = ดำ , 0 = ขาว
  TrackLineColorC(0); // สีของเส้น 1 = ดำ , 0 = ขาว
  // ModeSpdPID(0, 100, -25);  //โหมดคิดเอง
  ModeSpdPID(1, 100, -100); // // โหมด PID ปกติพื้นฐานทั่วไป
  // ModeSpdPID(2, 60, -60); // โหมดโค้งเยอะๆ แต่จะวิ่งช้า
  // ModeSpdPID(3, 100, -25); //โหมดคิดเอง หมุนตาม speed
  // ModeSpdPID(4, 100, 0); // โหมด PID ปกติพื้นฐานทั่วไป
  // Dottedline(0);  //ไม่มีเส้นประใส่ 0  >> มีเส้นประใส่ 1
  SetValueLine(500);     // ค่าแสง Track แยก
  SetValueLineC(800);    // ค่าแสง Track แยก
  SetTrackLineCh(1, 14); // เลือกเซนเซอร์ที่ต้องการ Track เส้น (0,15) (1,14) (2,13) (3,12) (4,11)
                         // // TrackTime(10, 0.002, 0.02, 10000);
                         // // TrackTime(20, 0.005, 0.05 , 10000);
                         // // TrackTime(30, 0.008, 0.08 , 10000);
                         // // TrackTime(40, 0.010, 0.10 , 10000);
                         // // TrackTime(50, 0.012, 0.12 , 10000);
                         // // TrackTime(60, 0.016 , 0.14  , 10000);
                         // // TrackTime(70, 0.020 , 0.24  , 10000);
                         // // TrackTime(80, 0.020 , 0.28  , 10000);
                         // TrackTime(90, 0.024 , 0.36  , 10000);

  // ===== สนาม RoboTracer =====
  // SerialMarker();                       // เข็นหุ่นด้วยมือ เช็คว่านับ GOAL/CURVE ถูก จุดตัดไม่ถูกนับ
  SetMarkerCh(1, 0);       // (ฝั่ง Goal, ฝั่ง Curve) ถ้านับผิดฝั่งให้สลับเป็น (1, 0)
  SetCrossDetect(10, 150); // (เซนเซอร์หน้ากี่ตัว = จุดตัด, เวลาไม่นับ marker หลังจุดตัด ms)
  SetRampStartSpeed(20);   // RoboTracer ออกตัวที่ความเร็วนี้ แล้วค่อยๆ เร่งไปจน Speed (ใส่ 100 = ไม่ค่อยๆ เร่ง)
  // RoboTracer(50, 0.030, 0.60, 2, 300);  // (Speed, Kp, Kd, Goal marker ที่ 2 = เส้นชัย, วิ่งต่อหลังเส้นชัย ms)
  // int spd[] = {50, 80, 50, 70};         // ความเร็วแต่ละช่วง (เปลี่ยนทุก Curve marker)
  // RoboTracerSegment(spd, NULL, NULL, 4, 2, 300);  // NULL = ใช้ Kp Kd จาก setspeed.ino
  // --- สำรวจรอบแรก แล้ววิ่งเร็ว (เบรกล่วงหน้าก่อนเข้าโค้ง) ---
  // RoboTracerLearn(40, 0.012, 0.08, 2, 200);  // วิ่งช้าๆ จำทางตรง/โค้ง
  // SaveLearn();                               // เก็บลง EEPROM (ครั้งหน้าใช้ LoadLearn() แทนการสำรวจ)
  // SerialLearn();                             // ดูผลสำรวจที่ Serial Monitor
  // SetBrakeMargin(50);                        // เบรกก่อนถึงโค้งเพิ่มอีกกี่ ms (-1 = ปิด)
  // RoboTracerRun(80, 45, 2, 100);             // (ความเร็วตรง, ความเร็วโค้ง, Goal marker, วิ่งต่อ ms)
  // // TrackTime(100, 0.028 , 0.36  , 10000);

  // ===== เลือกโหมดตามจำนวนครั้งที่กดปุ่ม B =====
  switch (Mode) {
    case 1:
      // RoboTracer(50, 0.012, 0.12, 2, 200);
      RoboTracer50();
      break;

    case 2:
      // RoboTracer(50, 0.012, 0.12, 2, 200);
      RoboTracer55();
      break;
    case 3:
      // RoboTracer(50, 0.012, 0.12, 2, 200);
      RoboTracer60();
      break;
    case 4:
      // RoboTracer(60, 0.016, 0.14, 2, 100);
      RoboTracer65();
      break;
    case 5:
      // RoboTracer(70, 0.020, 0.24, 2, 100);
      RoboTracer70();
      break;
    /*____________________สำรวจ + วิ่งเร็ว (ทำ 6 ก่อน)_______________________*/
    case 6:
      RoboTracerLearn40(); // สำรวจสนาม เก็บลง EEPROM
      break;
    case 7:
      RoboTracerRun70();
      break;
    case 8:
      RoboTracerRun80();
      break;
    case 9:
      RoboTracerRun90();
      break;
    /*____________________วิ่งเร็ว Kp/Kd ชุดเดียว (ทำ 6 ก่อน)_______________________*/
    case 10:
      RoboTracerRun70One();
      break;
    case 11:
      RoboTracerRun80One();
      break;
    case 12:
      RoboTracerRun90One();
      break;
    /*____________________ปรับแก้ได้_______________________*/
    case 13:
      // RoboTracer(80, 0.020, 0.24, 2, 100);
      RoboTracer75();
      break;
    case 14:
      // RoboTracer(90, 0.024, 0.36, 2, 100);
      RoboTracer80();
      break;
    case 15:
      // RoboTracer(100, 0.028, 0.36, 2, 100);
      RoboTracer85();
      break;
    case 16:
      RoboTracer90();
      break;
    case 17:
      RoboTracer95();
      break;
    case 18:
      RoboTracer100();
      break;
    case 19:
      RoboTracer105();
      break;
  }
} //__________________________ปีักกาห้ามลบ_____________________________