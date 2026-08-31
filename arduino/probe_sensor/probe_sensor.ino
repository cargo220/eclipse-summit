// TARS probe angle sensor reader.
// Sensor: SERA SME360AP-05DP-XY, analog output option.
//
// Wiring:
//   Sensor VCC -> Arduino 5V
//   Sensor OUT -> Arduino A1
//   Sensor GND -> Arduino GND
//
// Serial output format:
//   raw,voltage,angle_deg,arduino_state
//
// Notes:
// - Arduino Uno/Nano ADC is 10-bit: 0..1023.
// - The sensor is a 0..360 degree absolute encoder.
// - Direction and zero offset must be calibrated after mechanical assembly.

const int PROBE_PIN = A1;

const unsigned long SERIAL_BAUD = 115200;
const unsigned long SAMPLE_PERIOD_MS = 50;  // 20 Hz

const float ADC_MAX = 1023.0;       // Change if using a board with different ADC resolution.
const float ADC_REF_VOLTAGE = 5.0;  // Arduino analog reference voltage.

// Calibration values. Adjust after measuring the real installed sensor.
const int RAW_MIN = 0;
const int RAW_MAX = 876;

// Angle offset in degrees. Use serial command 'z' to print a suggested value.
float zero_offset_deg = 0.0;

// Set to -1.0 if angle direction is reversed after assembly.
const float ANGLE_DIRECTION = 1.0;

unsigned long last_sample_ms = 0;

float readAveragedRaw(int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(PROBE_PIN);
    delayMicroseconds(300);
  }
  return (float)sum / samples;
}

float rawToAngle(float raw) {
  float normalized = (raw - RAW_MIN) / (float)(RAW_MAX - RAW_MIN);
  normalized = constrain(normalized, 0.0, 1.0);

  float angle = normalized * 360.0;
  angle = (angle * ANGLE_DIRECTION) - zero_offset_deg;

  while (angle < 0.0) {
    angle += 360.0;
  }
  while (angle >= 360.0) {
    angle -= 360.0;
  }
  return angle;
}

void printSample() {
  float raw = readAveragedRaw(16);
  float voltage = raw * ADC_REF_VOLTAGE / ADC_MAX;
  float angle_deg = rawToAngle(raw);

  Serial.print(raw, 1);
  Serial.print(',');
  Serial.print(voltage, 3);
  Serial.print(',');
  Serial.print(angle_deg, 2);
  Serial.print(',');
  Serial.println("OK");
}

void handleSerialCommand() {
  if (!Serial.available()) {
    return;
  }

  char command = Serial.read();
  if (command == 'z' || command == 'Z') {
    float raw = readAveragedRaw(64);
    float suggested_offset = (raw - RAW_MIN) / (float)(RAW_MAX - RAW_MIN) * 360.0;
    Serial.print("# suggested_zero_offset_deg=");
    Serial.println(suggested_offset, 2);
  } else if (command == 'h' || command == 'H') {
    Serial.println("# commands: z=print suggested zero offset, h=help");
    Serial.println("# output: raw,voltage,angle_deg,arduino_state");
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  analogReadResolution(10);  // UNO R4 default is 14-bit; keep 10-bit for RAW_MAX=876.
  pinMode(PROBE_PIN, INPUT);
  delay(1000);

  Serial.println("# TARS probe sensor reader");
  Serial.println("# output: raw,voltage,angle_deg,arduino_state");
}

void loop() {
  handleSerialCommand();

  unsigned long now = millis();
  if (now - last_sample_ms >= SAMPLE_PERIOD_MS) {
    last_sample_ms = now;
    printSample();
  }
}
