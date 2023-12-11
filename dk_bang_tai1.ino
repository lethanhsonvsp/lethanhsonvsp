const int buttonPin = A0;

const int motor1PinA = 4; // Định nghĩa chân cho Động cơ 1
const int motor1PinB = 5;
const int motor1Enable = 3;

const int motor2PinA = 6; // Định nghĩa chân cho Động cơ 2
const int motor2PinB = 7;
const int motor2Enable = 9;

const int motor3PinA = 12; // Định nghĩa chân cho Động cơ 3
const int motor3PinB = 11;
const int motor3Enable = 10;
const int LedB = 2;
const int LedY = 13;

int buttonState = LOW;
unsigned long pressTime = 0;
unsigned long pressTime1 = 0;
const unsigned long duration = 3000;
int turnCount = 0;
int turnCount1 = 0;
int currentMotorIndex = 0;

const int runAllMotorsDuration = 5000;  // Thời gian chạy cả ba động cơ (18 giây)
unsigned long runAllMotorsStartTime = 0;
bool runAllMotors = false;

const int motorPins[3][3] = {
  {motor1PinA, motor1PinB, motor1Enable},
  {motor2PinA, motor2PinB, motor2Enable},
  {motor3PinA, motor3PinB, motor3Enable},
};

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  for (int i = 0; i < 3; i++) {
    pinMode(motorPins[i][0], OUTPUT);
    pinMode(motorPins[i][1], OUTPUT);
    pinMode(motorPins[i][2], OUTPUT);
    digitalWrite(motorPins[i][2], LOW); // Tắt động cơ
  }
  
  Serial.begin(9600);
  Serial.println("Hệ thống đã khởi động");
  pinMode(LedB, OUTPUT);
  pinMode(LedY, OUTPUT);
  digitalWrite(LedY, HIGH);
  digitalWrite(LedY, HIGH);
}

void loop() {
  runMotorForMedication();
  resetAllMotors();

  // Hiển thị trạng thái của LED và giá trị của biến turnCount
  Serial.print("Trạng thái LEDB: ");
  Serial.println(digitalRead(LedB));
  Serial.print("Trạng thái LEDY: ");
  Serial.println(digitalRead(LedY));
  Serial.print("Giá trị của turnCount1: ");
  Serial.println(turnCount1);
  Serial.println("--------------------------");
  delay(1000);
}

void runMotorForMedication() {
  // Thời gian chạy động cơ để phát thuốc
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    if (command == 'L' && pressTime == 0) {
      pressTime = millis();

      turnCount++;
      if (turnCount > 5) {
        currentMotorIndex = (currentMotorIndex + 1) % 3;  // Chuyển đến động cơ kế tiếp
        turnCount = 1;  // Đặt lại số lần quay
        Serial.println("Chuyển sang động cơ tiếp theo.");
      }
      controlMotor(currentMotorIndex, 100, 1);
      digitalWrite(LedB, HIGH);
      Serial.println("Nút được nhấn. Động cơ đang chạy...");
      turnCount1++;
    }
  }

  if (pressTime > 0 && millis() - pressTime >= duration) {
    controlMotor(currentMotorIndex, 0, 0);
    pressTime = 0;
    Serial.println("Động cơ đã dừng lại");
    digitalWrite(LedB, LOW);
  }

  // Thêm dòng code để nhấp nháy LedY sau khi chạy 15 lần
  if (turnCount1 == 15) {
    static unsigned long lastBlinkTime = 0;//dõi thời điểm cuối cùng khi LedY được nhấp nháy
    if (millis() - lastBlinkTime >= 2000) {
      digitalWrite(LedB, !digitalRead(LedB));  // Đảo trạng thái của LedB
      lastBlinkTime = millis();
    }

  }
}

void resetAllMotors() {
  // Thời gian chạy động cơ để reset 3 động cơ

  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'N' && pressTime == 0) {
    pressTime1 = millis();
    turnCount1 = 0;
    // Kích hoạt chạy cả ba động cơ trong 5 giây
    runAllMotors = true;
    runAllMotorsStartTime = millis();
    }
  }

  // Kiểm tra và chạy cả ba động cơ trong 5 giây
  if (runAllMotors && millis() - runAllMotorsStartTime < runAllMotorsDuration) {
    controlAllMotors(100, 1);
    digitalWrite(LedY, HIGH);
  } else {
    runAllMotors = false;  // Kết thúc chạy cả ba động cơ
    digitalWrite(LedY, LOW);
  }
}

void controlAllMotors(int speed, bool direction) {
  for (int i = 0; i < 3; i++) {
    controlMotor(i, speed, direction);
  }
}

void controlMotor(int motorIndex, int speed, bool direction) {
  if (speed > 0) {
    Serial.print("Động cơ ");
    Serial.print(motorIndex + 1);
    Serial.println(" được điều khiển.");
    
    analogWrite(motorPins[motorIndex][2], speed);
    digitalWrite(motorPins[motorIndex][0], direction);
    digitalWrite(motorPins[motorIndex][1], !direction);
  } else {
    digitalWrite(motorPins[motorIndex][2], LOW); // Tắt động cơ
  }
}
