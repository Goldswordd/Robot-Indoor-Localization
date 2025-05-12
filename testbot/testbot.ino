// Cấu hình chân cho motor
const int leftMotorIN1 = PA6;
const int leftMotorIN2 = PA7;
const int rightMotorIN3 = PB0;
const int rightMotorIN4 = PB1;

void moveForward() {
  analogWrite(leftMotorIN1, 0);
  analogWrite(leftMotorIN2, 150);
  analogWrite(rightMotorIN3, 0);
  analogWrite(rightMotorIN4, 150);
}

void moveBackward() {
  analogWrite(leftMotorIN1, 150);
  analogWrite(leftMotorIN2, 0);
  analogWrite(rightMotorIN3, 150);
  analogWrite(rightMotorIN4, 0);
}

void turnLeft() {
  analogWrite(leftMotorIN1, 0);
  analogWrite(leftMotorIN2, 150);
  analogWrite(rightMotorIN3, 150);
  analogWrite(rightMotorIN4, 0);
}

void turnRight() {
  analogWrite(leftMotorIN1, 150);
  analogWrite(leftMotorIN2, 0);
  analogWrite(rightMotorIN3, 0);
  analogWrite(rightMotorIN4, 150);
}

void stopMotors() {
  analogWrite(leftMotorIN1, 0);
  analogWrite(leftMotorIN2, 0);
  analogWrite(rightMotorIN3, 0);
  analogWrite(rightMotorIN4, 0);
}
void setup() {
  // Cấu hình chân động cơ
  pinMode(leftMotorIN1, OUTPUT);
  pinMode(leftMotorIN2, OUTPUT);
  pinMode(rightMotorIN3, OUTPUT);
  pinMode(rightMotorIN4, OUTPUT);

  // Khởi tạo Serial`
  Serial1.begin(9600); // Sử dụng UART1 (TX PA9, RX PA10)
  stopMotors();
}

void loop() {
  if (Serial1.available() > 0) {
    char command = Serial1.read();
    
    switch(command) {
      case 'F': moveForward(); break;
      case 'B': moveBackward(); break;
      case 'L': turnLeft(); break;
      case 'R': turnRight(); break;
      case 'S': stopMotors(); break;
    }
  }
}

