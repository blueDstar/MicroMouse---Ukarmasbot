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

// ==== Tham số PID (mặc định) ====
float KpL = 1.2, KiL = 0.0, KdL = 0.05;
float KpR = 2.0, KiR = 0.7, KdR = 0.0;

// ==== Biến PID ====
float setpointLeft = 200;
float speedLeft = 0, speedRight = 0;
float prevErrLeft = 0, integralLeft = 0;
float prevErrRight = 0, integralRight = 0;

// ==== Biến thời gian ====
unsigned long lastTime = 0;
const unsigned long interval = 100;
long prevCountLeft = 0, prevCountRight = 0;

// ==== Thời gian chạy PID ====
unsigned long runTime = 3000;  // mặc định 3 giây
bool isRunning = false;
unsigned long startRunTime = 0;

// ==== Ngắt encoder ====
void ISR_left()  { encoderLeftCount++;  }
void ISR_right() { encoderRightCount++; }

// ==== Hàm PID bánh trái ====
float pidLeft(float setpoint, float measured) {
  float error = setpoint - measured;
  integralLeft += error * (interval / 1000.0);
  float derivative = (error - prevErrLeft) / (interval / 1000.0);
  float output = KpL * error + KiL * integralLeft + KdL * derivative;
  prevErrLeft = error;
  return output;
}

// ==== Hàm PID bánh phải ====
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
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, constrain(abs(pwmVal), 0, 255));
}

void setMotorRight(float pwmVal) {
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  analogWrite(MOTOR_RIGHT_PWM, constrain(abs(pwmVal), 0, 255));
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
}

// ==== Hàm đọc số từ Serial ====
float readNumber(const char* label) {
  Serial.print(label);
  while (!Serial.available());
  String s = Serial.readStringUntil('\n');
  s.trim();
  return s.toFloat();
}

// ==== Cấu hình PID qua Serial ====
void configurePID() {
  Serial.println("\n=== CAI DAT PID CHO BANH TRAI ===");
  KpL = readNumber("Nhap KpL: ");
  KiL = readNumber("Nhap KiL: ");
  KdL = readNumber("Nhap KdL: ");

  Serial.println("\n=== CAI DAT PID CHO BANH PHAI ===");
  KpR = readNumber("Nhap KpR: ");
  KiR = readNumber("Nhap KiR: ");
  KdR = readNumber("Nhap KdR: ");

  runTime = (unsigned long)(readNumber("\nNhap thoi gian chay thang (giay): ") * 1000.0);

  Serial.println("\nPID da duoc cap nhat!");
  Serial.print("Trai: Kp="); Serial.print(KpL); Serial.print(", Ki="); Serial.print(KiL); Serial.print(", Kd="); Serial.println(KdL);
  Serial.print("Phai: Kp="); Serial.print(KpR); Serial.print(", Ki="); Serial.print(KiR); Serial.print(", Kd="); Serial.println(KdR);
  Serial.print("Thoi gian chay: "); Serial.print(runTime / 1000.0); Serial.println("s");
}

// ==== Setup ====
void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(ENCODER_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), ISR_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), ISR_right, RISING);

  Serial.println("===== PID Test Control =====");
  Serial.println("Nhap 'P' de cai PID moi");
  Serial.println("Nhap 'G' de chay PID voi thong so hien tai");
  Serial.println("Nhap 'S' de dung xe");
}

// ==== Vòng lặp chính ====
void loop() {
  // --- Xử lý lệnh Serial ---
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "P" || cmd == "p") {
      configurePID();
    } 
    else if (cmd == "G" || cmd == "g") {
      Serial.println("Bat dau chay PID...");
      encoderLeftCount = encoderRightCount = 0;
      prevCountLeft = prevCountRight = 0;
      prevErrLeft = prevErrRight = 0;
      integralLeft = integralRight = 0;
      startRunTime = millis();
      isRunning = true;
    } 
    else if (cmd == "S" || cmd == "s") {
      Serial.println("Dung xe ngay lap tuc!");
      isRunning = false;
      stopMotors();
    }
  }

  // --- Nếu đang chạy PID ---
  if (isRunning) {
    unsigned long now = millis();

    // Kiểm tra hết thời gian chạy
    if (now - startRunTime >= runTime) {
      isRunning = false;
      stopMotors();
      Serial.println("Het thoi gian, dung xe!");
      return;
    }

    // Chu kỳ PID
    if (now - lastTime >= interval) {
      lastTime = now;

      long deltaLeft  = encoderLeftCount  - prevCountLeft;
      long deltaRight = encoderRightCount - prevCountRight;

      speedLeft  = (deltaLeft  * 1000.0) / interval;
      speedRight = (deltaRight * 1000.0) / interval;

      prevCountLeft  = encoderLeftCount;
      prevCountRight = encoderRightCount;

      float pwmLeft = pidLeft(setpointLeft, speedLeft);
      float pwmRight = pidRight(speedLeft, speedRight);

      setMotorLeft(pwmLeft);
      setMotorRight(pwmRight);

      // In dữ liệu để debug
      Serial.print("LSPD:"); Serial.print(speedLeft);
      Serial.print("\tRSPD:"); Serial.print(speedRight);
      Serial.print("\tPWM_L:"); Serial.print(pwmLeft);
      Serial.print("\tPWM_R:"); Serial.println(pwmRight);
    }
  }
}
