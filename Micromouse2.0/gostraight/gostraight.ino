#include <Arduino.h>

const uint8_t MOTOR_LEFT_DIR  = 7;
const uint8_t MOTOR_RIGHT_DIR = 8;
const uint8_t MOTOR_LEFT_PWM  = 9;
const uint8_t MOTOR_RIGHT_PWM = 10;
const uint8_t ENCODER_LEFT  = 2;   // interrupt 0
const uint8_t ENCODER_RIGHT = 3;   // interrupt 1

void setup() {
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  GoStraight();
  delay(3000);
}

// ======== HÀM CHẠY THẲNG VỚI TĂNG & GIẢM TỐC ========
void GoStraight() {
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);

  int maxSpeed = 250;
  int step = 10;
  int delayTime = 50;

  Serial.println("Accelerating...");
  for (int s = 0; s <= maxSpeed; s += step) {
    analogWrite(MOTOR_LEFT_PWM, s);
    analogWrite(MOTOR_RIGHT_PWM, s);
    delay(delayTime);
  }

  Serial.println("Cruising at full speed...");
  analogWrite(MOTOR_LEFT_PWM, 240);
  analogWrite(MOTOR_RIGHT_PWM, 250);
  delay(1000);

  // ======== GIẢM TỐC DẦN ========
  gradualStop(maxSpeed, step, delayTime);
}

// ======== HÀM DỪNG TỪ TỪ ========
void gradualStop(int currentSpeed, int step, int delayTime) {
  Serial.println("Decelerating...");
  for (int s = currentSpeed; s >= 0; s -= step) {
    analogWrite(MOTOR_LEFT_PWM, s);
    analogWrite(MOTOR_RIGHT_PWM, s);
    delay(delayTime);
  }

  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
  Serial.println("Motors stopped gradually.");
}
