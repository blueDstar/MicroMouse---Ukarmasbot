#define font_ir 11
#define dia_ir 12
#define goc_phai A0
#define truoc_phai A1
#define goc_trai A2
#define truoc_trai A3

// Ngưỡng để xác định không có vật cản (tùy chỉnh theo cảm biến thực tế)
#define NO_OBJECT_THRESHOLD 50  // Giá trị ADC nhỏ hơn 50 xem như không có vật cản

void setup() {
  Serial.begin(115200);
  pinMode(font_ir, OUTPUT);
  pinMode(dia_ir, OUTPUT);
  pinMode(goc_phai, INPUT);
  pinMode(goc_trai, INPUT);
  pinMode(truoc_phai, INPUT);
  pinMode(truoc_trai, INPUT);
}

void checkAndPrintSensorValue(int sensorValue, const char* position) {
  if (sensorValue < NO_OBJECT_THRESHOLD) {
    Serial.print(position);
    Serial.print(": No object\t");
  } else {
    Serial.print(position);
    Serial.print(": ");
    Serial.print(sensorValue);   // In trực tiếp giá trị ADC (0-1023)
    Serial.print("\t");
  }
}

void loop() {
  digitalWrite(font_ir, HIGH);  // Bật LED phát hồng ngoại phía trước
  digitalWrite(dia_ir, HIGH);   // Bật LED phát hồng ngoại chéo

  // Đọc giá trị cảm biến và hiển thị kết quả
  checkAndPrintSensorValue(analogRead(goc_phai), "Phai");
  checkAndPrintSensorValue(analogRead(truoc_phai), "Truoc Phai");
  checkAndPrintSensorValue(analogRead(goc_trai), "Trai");
  checkAndPrintSensorValue(analogRead(truoc_trai), "Truoc Trai");

  Serial.println();  // Xuống dòng sau khi in hết các giá trị

  delay(100);  // Thêm một khoảng trễ nhỏ để dễ đọc dữ liệu
}
