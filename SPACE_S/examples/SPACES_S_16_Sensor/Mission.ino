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
  SetMarkerCh(1, 0);      // (ฝั่ง Goal, ฝั่ง Curve) ถ้านับผิดฝั่งให้สลับเป็น (1, 0)
  SetCrossDetect(8, 150); // (เซนเซอร์หน้ากี่ตัว = จุดตัด, เวลาไม่นับ marker หลังจุดตัด ms)
  // RoboTracer(50, 0.030, 0.60, 2, 300);  // (Speed, Kp, Kd, Goal marker ที่ 2 = เส้นชัย, วิ่งต่อหลังเส้นชัย ms)
  // int spd[] = {50, 80, 50, 70};         // ความเร็วแต่ละช่วง (เปลี่ยนทุก Curve marker)
  // RoboTracerSegment(spd, NULL, NULL, 4, 2, 300);  // NULL = ใช้ Kp Kd จาก setspeed.ino
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
    case 6:
      // RoboTracer(80, 0.020, 0.24, 2, 100);
      RoboTracer75();
      break;
    case 7:
      // RoboTracer(90, 0.024, 0.36, 2, 100);
      RoboTracer80();
      break;
    case 8:
      // RoboTracer(100, 0.028, 0.36, 2, 100);
      RoboTracer85();
      break;
    /*____________________ปรับแก้ได้_______________________*/
    case 9:
      RoboTracer90();
      break;
    case 10:
      RoboTracer95();
      break;
    case 11:
      RoboTracer100();
      break;
    case 12:
      RoboTracer105();
      break;
  }
} //__________________________ปีักกาห้ามลบ_____________________________