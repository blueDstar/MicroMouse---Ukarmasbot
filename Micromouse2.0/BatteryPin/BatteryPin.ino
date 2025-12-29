// Chân analog
const int BATTERY_VOLTS = A7;

// Điện áp tham chiếu của Arduino (5V nếu dùng Nano)
const float VREF = 5.0;

// Hệ số chia áp (VD: R1=10k, R2=10k => scale = 2.0)
const float DIVIDER_RATIO = 2.0;

// Thông số pin Li-ion 1 cell
const float BATTERY_MIN = 3.2;  // Điện áp cạn
const float BATTERY_MAX = 4.2;  // Điện áp đầy

void setup() {
  Serial.begin(115200);
}

void loop() {
  int raw = analogRead(BATTERY_VOLTS);

  // Chuyển đổi ADC -> Điện áp thực tế pin
  float voltage = (raw / 1023.0) * VREF * DIVIDER_RATIO;

  // Tính % dung lượng pin
  float percent = (voltage - BATTERY_MIN) / (BATTERY_MAX - BATTERY_MIN) * 100.0;
  if (percent > 100) percent = 100;
  if (percent < 0) percent = 0;

  // Xuất ra Plotter
  Serial.print("Voltage:");
  Serial.print(voltage);
  Serial.print("\t");

  Serial.print("Percent:");
  Serial.println(percent);

  delay(500);  // Cập nhật mỗi 0.5s
}
