#define PH1 3
#define EN1 5

#define PH2 2
#define EN2 6

void Motor1(int Pow) {
  bool dir = (Pow >= 0 ? true : false);
  Pow = abs(Pow);
  Pow = (Pow * 255) / 100;
  digitalWrite(PH1, dir);
  analogWrite(EN1, Pow);
}

void Motor2(int Pow) {
  bool dir = (Pow >= 0 ? true : false);
  Pow = abs(Pow);
  Pow = (Pow * 255) / 100;
  digitalWrite(PH2, dir);
  analogWrite(EN2, Pow);
}

void Motor(int leftspeed, int rightspeed) {
  Motor1(leftspeed);
  Motor2(rightspeed);
}

void Move(int leftspeed, int rightspeed, int Movedelay) {
  Motor(leftspeed, rightspeed);
  delay(Movedelay);
}

void MotorStop() {
  Motor(0, 0);
  delay(5);
}