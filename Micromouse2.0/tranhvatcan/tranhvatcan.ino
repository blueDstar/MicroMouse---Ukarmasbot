#include <Arduino.h>

const uint8_t MOTOR_LEFT_DIR = 7;
const uint8_t MOTOR_RIGHT_DIR = 8;
const uint8_t MOTOR_LEFT_PWM = 9;
const uint8_t MOTOR_RIGHT_PWM = 10;

const uint8_t EMITTER_A = 11;
const uint8_t EMITTER_B = 12;

const uint8_t SENSOR_FRONT_LEFT  = A3;  // Trước trái
const uint8_t SENSOR_LEFT        = A2;  // Trái
const uint8_t SENSOR_RIGHT       = A1;  // Phải
const uint8_t SENSOR_FRONT_RIGHT = A0;  // Trước phải

const unsigned long TURN_LEFT_90_DEGREES = 137;
const unsigned long TURN_RIGHT_90_DEGREES = 135;


#define OBSTACLE_THRESHOLD 200  

void goStraight() {
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, 200);
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  analogWrite(MOTOR_RIGHT_PWM, 205);
  Serial.println("Di thang");
}

void goBackward() {
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  analogWrite(MOTOR_LEFT_PWM, 200);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_RIGHT_PWM, 201);
  Serial.println("Di lui");
}

void turnLeft90() {
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  analogWrite(MOTOR_LEFT_PWM, 200);
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(TURN_LEFT_90_DEGREES);
  goStraight();
  Serial.println("Hoan thanh quay trai 90 do");
}

void turnRight90() {
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, 200);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(TURN_RIGHT_90_DEGREES);

  goStraight();
  Serial.println("Hoan thanh quay phai 90 do");
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
  Serial.println("Dung xe");
}

// ==== ĐỌC CẢM BIẾN ====
int readSensor(uint8_t pin) {
  return analogRead(pin);
}

// ==== SETUP ====
void setup() {
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(EMITTER_A, OUTPUT);
  pinMode(EMITTER_B, OUTPUT);

  digitalWrite(EMITTER_A, HIGH);
  digitalWrite(EMITTER_B, HIGH);

  pinMode(SENSOR_FRONT_LEFT, INPUT);
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
  pinMode(SENSOR_FRONT_RIGHT, INPUT);

  Serial.begin(115200);
}

// ==== LOOP ====
void loop() {
  int frontLeft  = readSensor(SENSOR_FRONT_LEFT);
  int frontRight = readSensor(SENSOR_FRONT_RIGHT);
  int leftSensor = readSensor(SENSOR_LEFT);
  int rightSensor= readSensor(SENSOR_RIGHT);

  bool hasFrontObstacle = (frontLeft > OBSTACLE_THRESHOLD || frontRight > OBSTACLE_THRESHOLD);
  bool hasLeftObstacle  = (leftSensor > OBSTACLE_THRESHOLD);
  bool hasRightObstacle = (rightSensor > OBSTACLE_THRESHOLD);

  if (hasFrontObstacle) {
    stopMotors();
    delay(100);

    if (!hasLeftObstacle) {
      Serial.println("Vat can truoc, re trai...");
      turnLeft90();
    } else if (!hasRightObstacle) {
      Serial.println("Vat can truoc, re phai...");
      turnRight90();
    } else {
      Serial.println("Bi chan ca truoc va 2 ben -> lui");
      delay(500);
      turnRight90(); // lui xong quay phải
    }
  } else {
    goStraight();
  }

  delay(50); // chống quá nhạy
}
