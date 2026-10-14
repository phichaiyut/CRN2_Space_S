void SetKpKd() { // ตั้งค่า KP KD
  // setKpKd(SPD_10, KP, KD);
  setKpKd(SPD_10, 0.002, 0.02);
  setKpKd(SPD_20, 0.005, 0.05);
  setKpKd(SPD_30, 0.008, 0.08);
  setKpKd(SPD_40, 0.010, 0.10);
  setKpKd(SPD_50, 0.012, 0.12);
  setKpKd(SPD_60, 0.016, 0.14);
  setKpKd(SPD_70, 0.020, 0.24);
  setKpKd(SPD_80, 0.020, 0.22);
  setKpKd(SPD_90, 0.024, 0.36);
  setKpKd(SPD_100, 0.028, 0.40);
}

void SetSpeedForward() { // เดินหน้า

  // setBalanceSpeed(SPD_10, ล้อซ้าย, ล้อขวา); ข้างไหนแรงกว่าไปเพิ่มข้างนั้น

  setBalanceSpeed(SPD_10, 0, 0);
  setBalanceSpeed(SPD_20, 0, 0);
  setBalanceSpeed(SPD_30, 0, 0);
  setBalanceSpeed(SPD_40, 0, 0);
  setBalanceSpeed(SPD_50, 0, 0);
  setBalanceSpeed(SPD_60, 0, 0);
  setBalanceSpeed(SPD_70, 0, 0);
  setBalanceSpeed(SPD_80, 0, 0);
  setBalanceSpeed(SPD_90, 0, 0);
  setBalanceSpeed(SPD_100, 0, 0);
}

void SetSpeedBackward() { // ถอยหลัง

  // setBalanceBackSpeed(SPD_10, ล้อซ้าย, ล้อขวา); ข้างไหนแรงกว่าไปเพิ่มข้างนั้น

  setBalanceBackSpeed(SPD_10, 0, 0);
  setBalanceBackSpeed(SPD_20, 0, 0);
  setBalanceBackSpeed(SPD_30, 0, 0);
  setBalanceBackSpeed(SPD_40, 0, 0);
  setBalanceBackSpeed(SPD_50, 0, 0);
  setBalanceBackSpeed(SPD_60, 0, 0);
  setBalanceBackSpeed(SPD_70, 0, 0);
  setBalanceBackSpeed(SPD_80, 0, 0);
  setBalanceBackSpeed(SPD_90, 0, 0);
  setBalanceBackSpeed(SPD_100, 0, 0);
}
