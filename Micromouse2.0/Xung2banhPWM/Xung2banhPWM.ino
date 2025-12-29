#include <Arduino.h>

const uint8_t ENCODER_LEFT_CLK = 2;
const uint8_t ENCODER_RIGHT_CLK = 3;
const uint8_t MOTOR_LEFT_DIR = 7;
const uint8_t MOTOR_RIGHT_DIR = 8;
const uint8_t MOTOR_LEFT_PWM = 9;
const uint8_t MOTOR_RIGHT_PWM = 10;
const uint8_t BATTERY_PIN = A7;

// ===== Encoder & tốc độ =====
volatile long pulseCountLeft = 0;
volatile long pulseCountRight = 0;

const int PULSES_PER_REV = 20;
const int GEAR_RATIO = 30;
const float WHEEL_DIAMETER = 0.065;           
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

unsigned long lastSpeedTime = 0;
long lastPulseLeft = 0, lastPulseRight = 0;
float speedLeft = 0, speedRight = 0;

// ===== PWM hiện tại =====
int pwmLeft = 150;
int pwmRight = 150;

// ===== Serial input =====
String inputString = "";
bool commandReady = false;

// ===== Encoder ISR =====
void encoderLeftISR() { pulseCountLeft++; }
void encoderRightISR() { pulseCountRight++; }

// ===== Motor control =====
void setMotorSpeed(int left, int right) {
  pwmLeft = constrain(left, 0, 255);
  pwmRight = constrain(right, 0, 255);

  digitalWrite(MOTOR_LEFT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, pwmLeft);

  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  analogWrite(MOTOR_RIGHT_PWM, pwmRight);
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
}

// ===== Battery =====
float readBatteryVoltage() {
  int adcValue = analogRead(BATTERY_PIN);
  float voltage = adcValue * (5.0 / 1023.0) * 4.0; // chia áp 30k-10k
  return voltage;
}

int batteryPercent(float voltage) {
  int percent = map((int)(voltage * 100), 650, 840, 0, 100);
  return constrain(percent, 0, 100);
}

// ===== Velocity calc =====
void updateWheelSpeed() {
  unsigned long now = millis();
  if (now - lastSpeedTime >= 500) {
    long dL = pulseCountLeft - lastPulseLeft;
    long dR = pulseCountRight - lastPulseRight;

    float revL = (float)dL / (PULSES_PER_REV * GEAR_RATIO);
    float revR = (float)dR / (PULSES_PER_REV * GEAR_RATIO);

    speedLeft = (revL * WHEEL_CIRCUMFERENCE) / ((now - lastSpeedTime) / 1000.0);
    speedRight = (revR * WHEEL_CIRCUMFERENCE) / ((now - lastSpeedTime) / 1000.0);

    lastPulseLeft = pulseCountLeft;
    lastPulseRight = pulseCountRight;
    lastSpeedTime = now;
  }
}

// ===== Serial input =====
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) commandReady = true;
    } else {
      inputString += inChar;
    }
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(9600);

  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  pinMode(ENCODER_LEFT_CLK, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_CLK, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_CLK), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_CLK), encoderRightISR, RISING);

  Serial.println("=== HE THONG DA SAN SANG ===");
  Serial.println("Lenh: PWML [x], PWMR [x], ENDL, ENDR, VL, VR, PIN, STOP, START, RESET");
  Serial.println("Them so sau PWML/PWMR de thay doi gia tri (VD: PWML 180)");
  Serial.println("--------------------------------");

  setMotorSpeed(pwmLeft, pwmRight);  // xe chạy luôn
}

// ===== Loop =====
void loop() {
  updateWheelSpeed();

  if (commandReady) {
    inputString.trim();
    String cmd = inputString;
    inputString = "";
    commandReady = false;

    cmd.toUpperCase();

    // --- PWML ---
    if (cmd.startsWith("PWML")) {
      if (cmd.length() > 4) {
        int val = cmd.substring(4).toInt();
        pwmLeft = constrain(val, 0, 255);
        setMotorSpeed(pwmLeft, pwmRight);
        Serial.print("PWM trai moi: "); Serial.println(pwmLeft);
      } else {
        Serial.print("PWM trai hien tai: "); Serial.println(pwmLeft);
      }
    }
    // --- PWMR ---
    else if (cmd.startsWith("PWMR")) {
      if (cmd.length() > 4) {
        int val = cmd.substring(4).toInt();
        pwmRight = constrain(val, 0, 255);
        setMotorSpeed(pwmLeft, pwmRight);
        Serial.print("PWM phai moi: "); Serial.println(pwmRight);
      } else {
        Serial.print("PWM phai hien tai: "); Serial.println(pwmRight);
      }
    }
    // --- Encoder ---
    else if (cmd == "ENDL") {
      Serial.print("So vong quay trai: ");
      Serial.println((float)pulseCountLeft / (PULSES_PER_REV * GEAR_RATIO), 3);
    } 
    else if (cmd == "ENDR") {
      Serial.print("So vong quay phai: ");
      Serial.println((float)pulseCountRight / (PULSES_PER_REV * GEAR_RATIO), 3);
    }
    // --- Velocity ---
    else if (cmd == "VL") {
      Serial.print("Van toc trai: "); Serial.print(speedLeft, 3); Serial.println(" m/s");
    }
    else if (cmd == "VR") {
      Serial.print("Van toc phai: "); Serial.print(speedRight, 3); Serial.println(" m/s");
    }
    // --- Pin ---
    else if (cmd == "PIN") {
      float v = readBatteryVoltage();
      Serial.print("Pin: "); Serial.print(v, 2);
      Serial.print("V | "); Serial.print(batteryPercent(v)); Serial.println("%");
    }
    // --- STOP ---
    else if (cmd == "STOP") {
      stopMotors();
      Serial.println(">> Da dung dong co");
    }
    // --- START ---
    else if (cmd == "START") {
      setMotorSpeed(pwmLeft, pwmRight);
      Serial.println(">> Xe da chay lai");
    }
    else {
      Serial.println("Lenh khong hop le!");
    }
  }
}
