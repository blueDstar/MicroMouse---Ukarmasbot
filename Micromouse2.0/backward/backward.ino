#include <Arduino.h>

const uint8_t MOTOR_LEFT_DIR  = 7;
const uint8_t MOTOR_RIGHT_DIR = 8;
const uint8_t MOTOR_LEFT_PWM  = 9;
const uint8_t MOTOR_RIGHT_PWM = 10;
const uint8_t ENCODER_LEFT  = 2;   // interrupt 0
const uint8_t ENCODER_RIGHT = 3;   // interrupt 1

// ======== HÀM KHỞI TẠO ========
void setup() {
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  Serial.begin(9600);  // Debug qua Serial
}

// ======== VÒNG LẶP CHÍNH ========
void loop() {
  Backward();
  delay(3000);
}

// ======== HÀM CHẠY LÙI ========
void Backward() {
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  analogWrite(MOTOR_LEFT_PWM, 250);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_RIGHT_PWM, 245);

  delay(1000); // chạy 1 giây

  // Giảm tốc từ từ trước khi dừng
  gradualStop();
}

// ======== HÀM DỪNG TỪ TỪ ========
void gradualStop() {
  int currentSpeed = 250;   // tốc độ hiện tại
  int step = 10;            // giảm mỗi lần 10 đơn vị
  int delayTime = 50;       // thời gian giữa mỗi bước giảm (ms)

  for (int s = currentSpeed; s >= 0; s -= step) {
    analogWrite(MOTOR_LEFT_PWM, s);
    analogWrite(MOTOR_RIGHT_PWM, s);
    delay(delayTime);
  }

  // đảm bảo dừng hẳn
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
  Serial.println("Motors stopped gradually.");
}
