void robotsetting() {
  SensorValue(46, 42, 42, 42, 42, 42, 42, 42, 757, 696, 638, 715, 725, 657, 673, 701); // ค่าแสง
  TurnSpeedLeft(2, 25, 80, 60);                                                        // ค่าเลี้ยวซ้าย TurnSpeedLeft(มอเตอร์ซ้าย, มอเตอร์ขวา, เวลาเลี้ยว);
  TurnSpeedRight(5, 80, 25, 60);                                                       // ค่าเลี้ยวขวา TurnSpeedRight(มอเตอร์ซ้าย, มอเตอร์ขวา, เวลาเลี้ยว);
  TrackLineColor(0);                                                                   // สีของเส้น 1 = ดำ , 0 = ขาว
  ModeSpdPID(0, 100, -25);
  Dottedline(0); // ไม่มีเส้นประใส่ 0  >> มีเส้นประใส่ 1

  ReadEEPROM();      // ดึงค่าแสงที่เก็บไว้ อออกมา
  SetValueLine(500); // ค่าที่ใช้ Track แยก

  // SerialSensor();           // Serial Monitor ค่าของ Sensor
  // SerialCalibrateSensor();  // Serial Monitor ค่าของ Sensor ที่ Calibrate แล้ว
  // CaliberateRobotSensor();  // Serial Monitor Calibrate ค่าแสงหุ่นยนต์
}