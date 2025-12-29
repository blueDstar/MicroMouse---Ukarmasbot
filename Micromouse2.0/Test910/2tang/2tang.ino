#include <Arduino.h>

// ==== Cấu hình phần cứng ====
#define MOTOR_LEFT_DIR   7
#define MOTOR_RIGHT_DIR  8
#define MOTOR_LEFT_PWM   9
#define MOTOR_RIGHT_PWM  10

#define ENCODER_LEFT     2
#define ENCODER_RIGHT    3

// ==== Biến đếm xung ====
volatile long encoderLeftCount  = 0;
volatile long encoderRightCount = 0;

// ==== Tham số PID ====
float KpL = 1.2, KiL = 0, KdL = 0.05;   // PID bánh trái
float KpR = 2.0, KiR = 0.7, KdR = 0.0;   // PID bánh phải

// ==== Biến PID ====
float setpointLeft = 200;     // tốc độ mong muốn bánh trái (xung/giây)
float speedLeft = 0, speedRight = 0;
float prevErrLeft = 0, integralLeft = 0;
float prevErrRight = 0, integralRight = 0;

// ==== Thời gian đo tốc độ ====
unsigned long lastTime = 0;
const unsigned long interval = 100; // đo mỗi 100 ms
long prevCountLeft = 0, prevCountRight = 0;

// ==== Ngắt encoder ====
void ISR_left()  { encoderLeftCount++;  }
void ISR_right() { encoderRightCount++; }

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  pinMode(ENCODER_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), ISR_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), ISR_right, RISING);
}

// ==== Hàm PID bánh trái ====
float pidLeft(float setpoint, float measured) {
  float error = setpoint - measured;
  integralLeft += error * (interval / 1000.0);
  float derivative = (error - prevErrLeft) / (interval / 1000.0);
  float output = KpL * error + KiL * integralLeft + KdL * derivative;
  prevErrLeft = error;
  return output;
}

// ==== Hàm PID bánh phải (theo bánh trái) ====
float pidRight(float setpoint, float measured) {
  float error = setpoint - measured;
  integralRight += error * (interval / 1000.0);
  float derivative = (error - prevErrRight) / (interval / 1000.0);
  float output = KpR * error + KiR * integralRight + KdR * derivative;
  prevErrRight = error;
  return output;
}

// ==== Hàm điều khiển motor ====
void setMotorLeft(float pwmVal) {
  bool dir = pwmVal >= 0;
  digitalWrite(MOTOR_LEFT_DIR,LOW);
  analogWrite(MOTOR_LEFT_PWM, constrain(abs(pwmVal), 0, 255));
}

void setMotorRight(float pwmVal) {
  bool dir = pwmVal >= 0;
  digitalWrite(MOTOR_RIGHT_DIR,HIGH);
  analogWrite(MOTOR_RIGHT_PWM, constrain(abs(pwmVal), 0, 255));
}

// ==== Vòng lặp chính ====
void loop() {
  unsigned long now = millis();
  if (now - lastTime >= interval) {
    lastTime = now;

    // Tính tốc độ (xung/giây)
    long deltaLeft  = encoderLeftCount  - prevCountLeft;
    long deltaRight = encoderRightCount - prevCountRight;

    speedLeft  = (deltaLeft  * 1000.0) / interval;
    speedRight = (deltaRight * 1000.0) / interval;

    prevCountLeft  = encoderLeftCount;
    prevCountRight = encoderRightCount;

    // --- PID tầng 1: bánh trái ---
    float pwmLeft = pidLeft(setpointLeft, speedLeft);

    // --- PID tầng 2: bánh phải theo bánh trái ---
    float pwmRight = pidRight(speedLeft, speedRight);

    // --- Xuất PWM ---
    setMotorLeft(pwmLeft);
    setMotorRight(pwmRight);

    // --- In ra để xem trên Serial Plotter ---
    Serial.print("LeftSpeed:");
    Serial.print(speedLeft);
    Serial.print("\tRightSpeed:");
    Serial.print(speedRight);
    Serial.print("\tPWM_L:");
    Serial.print(pwmLeft);
    Serial.print("\tPWM_R:");
    Serial.println(pwmRight);
  }
}
