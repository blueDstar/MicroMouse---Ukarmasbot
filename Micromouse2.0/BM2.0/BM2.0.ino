#include <Arduino.h>

// ==== Cấu hình chân động cơ ====
const uint8_t MOTOR_LEFT_DIR = 7;
const uint8_t MOTOR_RIGHT_DIR = 8;
const uint8_t MOTOR_LEFT_PWM = 9;
const uint8_t MOTOR_RIGHT_PWM = 10;

// ==== Cấu hình chân cảm biến IR ====
#define FONT_IR 11
#define DIA_IR 12
#define GOC_PHAI A0
#define TRUOC_PHAI A1
#define GOC_TRAI A2
#define TRUOC_TRAI A3

// ==== Tham số điều khiển ====
#define NO_OBJECT_THRESHOLD 50  
const unsigned long TURN_TIME_90_DEGREES = 295;

// ==== Hàm khởi động ====
void setup() {
  // IR
  pinMode(FONT_IR, OUTPUT);
  pinMode(DIA_IR, OUTPUT);
  pinMode(GOC_PHAI, INPUT);
  pinMode(GOC_TRAI, INPUT);
  pinMode(TRUOC_PHAI, INPUT);
  pinMode(TRUOC_TRAI, INPUT);

  // Motor
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  Serial.begin(9600);
  delay(1000);

  printHelp();
}

// ==== Hàm in hướng dẫn ====
void printHelp() {
  Serial.println("===== DANH SACH LENH =====");
  Serial.println("S   : Doc cam bien hong ngoai");
  Serial.println("H   : Di thang");
  Serial.println("B   : Di lui");
  Serial.println("L : Quay trai 90 do");
  Serial.println("R : Quay phai 90 do");
  Serial.println("0   : Dung dong co ngay lap tuc");
  Serial.println("?   : In lai huong dan");
  Serial.println("===========================");
}

// ==== Hàm hỗ trợ ====
float convertToVoltage(int sensorValue) {
  return (sensorValue / 1023.0) * 5.0;
}

void checkAndPrintSensor(int value, const char* pos) {
  if (value < NO_OBJECT_THRESHOLD) {
    Serial.print(pos);
    Serial.print(": No object\t");
  } else {
    float v = convertToVoltage(value);
    Serial.print(pos);
    Serial.print(": ");
    Serial.print(v);
    Serial.print(" V\t");
  }
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
  Serial.println(">>> STOP <<<");
}

// ==== Các hành động ====
void readSensors() {
  digitalWrite(FONT_IR, HIGH);
  digitalWrite(DIA_IR, HIGH);

  checkAndPrintSensor(analogRead(GOC_PHAI), "Goc Phai");
  checkAndPrintSensor(analogRead(TRUOC_PHAI), "Truoc Phai");
  checkAndPrintSensor(analogRead(GOC_TRAI), "Goc Trai");
  checkAndPrintSensor(analogRead(TRUOC_TRAI), "Truoc Trai");
  Serial.println();
}

void goStraight() {
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  analogWrite(MOTOR_LEFT_PWM, 210);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  Serial.println("Di thang");
}

void goBackward() {
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, 210);
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  Serial.println("Di lui");
}

void turnLeft90() {
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, 200);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(TURN_TIME_90_DEGREES);
  stopMotors();
  Serial.println("Hoan thanh quay trai 90 do");
}

void turnRight90() {
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  analogWrite(MOTOR_LEFT_PWM, 200);
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(TURN_TIME_90_DEGREES);
  stopMotors();
  Serial.println("Hoan thanh quay phai 90 do");
}

void spinLeft() {
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  analogWrite(MOTOR_LEFT_PWM, 210);
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  Serial.println("Spin Left");
}

void spinRight() {
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, 210);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  Serial.println("Spin Right");
}

// ==== Vòng lặp chính ====
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "S") {
      readSensors();
    } else if (cmd == "H") {
      goStraight();
    } else if (cmd == "B") {
      goBackward();
    } else if (cmd == "L") {
      turnLeft90();
    } else if (cmd == "R") {
      turnRight90();
    } else if (cmd == "0") {
      stopMotors();
    } else if (cmd == "?") {
      printHelp();
    } else {
      Serial.println("Lenh khong hop le. Goi '?' de xem huong dan.");
    }
  }
}
