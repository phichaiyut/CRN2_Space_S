void robotsetting() {
  SetTrackLineCh(0, 15);
  // SensorValue(46, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 43, 43, 757, 696, 638, 715, 725, 657, 673, 701, 701, 713, 696, 681, 679, 701, 727, 714);  //ค่าแสง
  TurnSpeedLeft(4, 25, 80, 60);   // ค่าเลี้ยวซ้าย TurnSpeedLeft(มอเตอร์ซ้าย, มอเตอร์ขวา, เวลาเลี้ยว);
  TurnSpeedRight(11, 80, 25, 60); // ค่าเลี้ยวขวา TurnSpeedRight(มอเตอร์ซ้าย, มอเตอร์ขวา, เวลาเลี้ยว);
  TrackLineColor(0);              // สีของเส้น 1 = ดำ , 0 = ขาว
  TrackLineColorC(0);             // สีของเส้น เซนเซอร์ C 1 = ดำ , 0 = ขาว
  ModeSpdPID(0, 100, -25);
  Dottedline(0); // ไม่มีเส้นประใส่ 0  >> มีเส้นประใส่ 1

  SetKpKd();          // ตั้งค่า kp kd (setspeed.ino)
  SetSpeedForward();  // setup balance เดินหน้า
  SetSpeedBackward(); // setup balance ถอยหลัง

  ReadEEPROM(); // ดึงค่าแสงที่เก็บไว้ อออกมา

  SetValueLine(500);  // ค่าแสง Track แยก
  SetValueLineC(500); // ค่าแสง Track แยก เซนเซอร์ C

  // SerialSensor();           // Serial Monitor ค่าของ Sensor
  // SerialCalibrateSensor();  // Serial Monitor ค่าของ Sensor ที่ Calibrate แล้ว
  // CaliberateRobotSensor();  // Serial Monitor Calibrate ค่าแสงหุ่นยนต์
}