#include <Arduino.h>

// Cấu hình chân
const uint8_t MOTOR_LEFT_DIR = 7;
const uint8_t MOTOR_RIGHT_DIR = 8;
const uint8_t MOTOR_LEFT_PWM = 9;
const uint8_t MOTOR_RIGHT_PWM = 10;

// Thời gian quay 90 độ (điều chỉnh theo thực tế)
const unsigned long TURN_TIME_90_DEGREES = 125;  // Đơn vị: ms (thử nghiệm để xác định giá trị chính xác)

void setup() {
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  Serial.begin(9600);  // Debug qua Serial
}

void loop() {
  turnLeft90Degrees();
  delay(3000);
}
void turnLeft90Degrees() {
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, 200);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(TURN_TIME_90_DEGREES);
  stopMotors();
  // digitalWrite(MOTOR_LEFT_DIR, HIGH);
  // analogWrite(MOTOR_LEFT_PWM, 255);
  // digitalWrite(MOTOR_RIGHT_DIR, LOW);
  // analogWrite(MOTOR_RIGHT_PWM, 233)
  // Serial.println("Hoàn thành quay trái 90 độ");
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
}
