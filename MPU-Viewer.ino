#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <EEPROM.h>

Adafruit_MPU6050 mpu;

// Calibration offsets
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
bool isCalibrated = false;

// Complementary filter variables
float roll = 0, pitch = 0;
const float alpha = 0.98; // Complementary filter weight (0.98 for gyro, 0.02 for accel)

// Moving average filter variables
const int WINDOW_SIZE = 5;
float accelXBuffer[WINDOW_SIZE], accelYBuffer[WINDOW_SIZE], accelZBuffer[WINDOW_SIZE];
float gyroXBuffer[WINDOW_SIZE], gyroYBuffer[WINDOW_SIZE], gyroZBuffer[WINDOW_SIZE];
int bufferIndex = 0;

// EEPROM addresses for storing calibration data
#define EEPROM_CALIB_FLAG 0
#define EEPROM_ACCEL_X 4
#define EEPROM_ACCEL_Y 8
#define EEPROM_ACCEL_Z 12
#define EEPROM_GYRO_X 16
#define EEPROM_GYRO_Y 20
#define EEPROM_GYRO_Z 24

void loadCalibration() {
  // Check if calibration data exists in EEPROM
  if (EEPROM.read(EEPROM_CALIB_FLAG) == 1) {
    accelOffsetX = EEPROM.get(EEPROM_ACCEL_X, accelOffsetX);
    accelOffsetY = EEPROM.get(EEPROM_ACCEL_Y, accelOffsetY);
    accelOffsetZ = EEPROM.get(EEPROM_ACCEL_Z, accelOffsetZ);
    gyroOffsetX = EEPROM.get(EEPROM_GYRO_X, gyroOffsetX);
    gyroOffsetY = EEPROM.get(EEPROM_GYRO_Y, gyroOffsetY);
    gyroOffsetZ = EEPROM.get(EEPROM_GYRO_Z, gyroOffsetZ);
    isCalibrated = true;
    Serial.println("Loaded calibration from EEPROM");
  }
}

void saveCalibration() {
  EEPROM.write(EEPROM_CALIB_FLAG, 1);
  EEPROM.put(EEPROM_ACCEL_X, accelOffsetX);
  EEPROM.put(EEPROM_ACCEL_Y, accelOffsetY);
  EEPROM.put(EEPROM_ACCEL_Z, accelOffsetZ);
  EEPROM.put(EEPROM_GYRO_X, gyroOffsetX);
  EEPROM.put(EEPROM_GYRO_Y, gyroOffsetY);
  EEPROM.put(EEPROM_GYRO_Z, gyroOffsetZ);
  Serial.println("Saved calibration to EEPROM");

}

void calibrateIMU() {
  const int numSamples = 1000;
  float accelXSum = 0, accelYSum = 0, accelZSum = 0;
  float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;

  Serial.println("Calibrating IMU... Keep sensor stationary.");
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accelXSum += a.acceleration.x;
    accelYSum += a.acceleration.y;
    accelZSum += a.acceleration.z - 9.8; // Adjust for gravity (Z up)
    gyroXSum += g.gyro.x;
    gyroYSum += g.gyro.y;
    gyroZSum += g.gyro.z;
    delay(2);
  }

  accelOffsetX = accelXSum / numSamples;
  accelOffsetY = accelYSum / numSamples;
  accelOffsetZ = accelZSum / numSamples;
  gyroOffsetX = gyroXSum / numSamples;
  gyroOffsetY = gyroYSum / numSamples;
  gyroOffsetZ = gyroZSum / numSamples;

  saveCalibration();
  isCalibrated = true;

  Serial.println("Calibration complete!");
  Serial.print("Accel Offsets (X, Y, Z): ");
  Serial.print(accelOffsetX, 2); Serial.print(", ");
  Serial.print(accelOffsetY, 2); Serial.print(", ");
  Serial.print(accelOffsetZ, 2); Serial.println(" m/s^2");
  Serial.print("Gyro Offsets (X, Y, Z): ");
  Serial.print(gyroOffsetX, 2); Serial.print(", ");
  Serial.print(gyroOffsetY, 2); Serial.print(", ");
  Serial.print(gyroOffsetZ, 2); Serial.println(" rad/s");
}

float movingAverage(float* buffer, float newValue) {
  buffer[bufferIndex % WINDOW_SIZE] = newValue;
  float sum = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    sum += buffer[i];
  }
  return sum / WINDOW_SIZE;
}

bool checkCalibrationNeeded() {
  // Check if sensor is stationary by comparing variance over a few samples
  const int checkSamples = 50;
  float accelXSum = 0, accelYSum = 0, accelZSum = 0;
  float accelXSumSq = 0, accelYSumSq = 0, accelZSumSq = 0;

  for (int i = 0; i < checkSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accelXSum += a.acceleration.x;
    accelYSum += a.acceleration.y;
    accelZSum += a.acceleration.z;
    accelXSumSq += a.acceleration.x * a.acceleration.x;
    accelYSumSq += a.acceleration.y * a.acceleration.y;
    accelZSumSq += a.acceleration.z * a.acceleration.z;
    delay(2);
  }

  float varianceX = (accelXSumSq / checkSamples) - pow(accelXSum / checkSamples, 2);
  float varianceY = (accelYSumSq / checkSamples) - pow(accelYSum / checkSamples, 2);
  float varianceZ = (accelZSumSq / checkSamples) - pow(accelZSum / checkSamples, 2);

  // If variance is low, sensor is likely stationary
  const float varianceThreshold = 0.01;
  return (varianceX < varianceThreshold && varianceY < varianceThreshold && varianceZ < varianceThreshold);
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Adafruit MPU6050 Advanced Demo!");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize moving average buffers
  for (int i = 0; i < WINDOW_SIZE; i++) {
    accelXBuffer[i] = accelYBuffer[i] = accelZBuffer[i] = 0;
    gyroXBuffer[i] = gyroYBuffer[i] = gyroZBuffer[i] = 0;
  }

  // Load calibration from EEPROM
  loadCalibration();

  // Perform calibration if not loaded or invalid
  if (!isCalibrated) {
    calibrateIMU();
  }

  Serial.println("Setup complete!");
}





void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  float dt = (lastUpdate > 0) ? (now - lastUpdate) / 1000.0 : 0;
  lastUpdate = now;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (isnan(a.acceleration.x) || isnan(a.acceleration.y) || isnan(a.acceleration.z) ||
      isnan(g.gyro.x) || isnan(g.gyro.y) || isnan(g.gyro.z) || 
      isnan(temp.temperature)) {
    Serial.println("Error: Invalid sensor data");
    delay(100);
    return;
  }

  // Apply calibration offsets
  float accelX = a.acceleration.x - accelOffsetX;
  float accelY = a.acceleration.y - accelOffsetY;
  float accelZ = a.acceleration.z - accelOffsetZ;
  float gyroX = g.gyro.x - gyroOffsetX;
  float gyroY = g.gyro.y - gyroOffsetY;
  float gyroZ = g.gyro.z - gyroOffsetZ;

  // Apply moving average filter
  accelX = movingAverage(accelXBuffer, accelX);
  accelY = movingAverage(accelYBuffer, accelY);
  accelZ = movingAverage(accelZBuffer, accelZ);
  gyroX = movingAverage(gyroXBuffer, gyroX);
  gyroY = movingAverage(gyroYBuffer, gyroY);
  gyroZ = movingAverage(gyroZBuffer, gyroZ);
  bufferIndex++;

  // Complementary filter for orientation
  float accelRoll = atan2(accelY, accelZ) * 180 / PI;
  float accelPitch = atan(-accelX / sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;
  roll = alpha * (roll + gyroX * dt * 180 / PI) + (1 - alpha) * accelRoll;
  pitch = alpha * (pitch + gyroY * dt * 180 / PI) + (1 - alpha) * accelPitch;

  // Check if recalibration is needed (every 5 minutes)
  static unsigned long lastCalibrationCheck = 0;
  if (now - lastCalibrationCheck > 300000) {
    if (checkCalibrationNeeded()) {
      Serial.println("Sensor appears stationary. Recalibrating...");
      calibrateIMU();
    }
    lastCalibrationCheck = now;
  }



  // Print accelerometer data in the format expected by Python
  Serial.print("Acceleration X: ");
  Serial.print(accelX, 2);
  Serial.print(", Y: ");
  Serial.print(accelY, 2);
  Serial.print(", Z: ");
  Serial.print(accelZ, 2);
  Serial.println();

  // Print gyroscope data rad/s
  Serial.print("Rotation X: ");
  Serial.print(gyroX, 2);
  Serial.print(", Y: ");
  Serial.print(gyroY, 2);
  Serial.print(", Z: ");
  Serial.print(gyroZ, 2);
  Serial.println();

  // Print gyroscope data °/s
  Serial.print("Rotation X: ");
  Serial.print(gyroX * 180 / PI, 2);
  Serial.print(", Y: ");
  Serial.print(gyroY * 180 / PI, 2);
  Serial.print(", Z: ");
  Serial.print(gyroZ * 180 / PI, 2);
  Serial.println();


  
  Serial.print("Estimated Roll: ");
  Serial.print(roll, 2);
  Serial.println(" °");
  
  Serial.print("Estimated Pitch: ");
  Serial.print(pitch, 2);
  Serial.println(" °");

  // Print temperature
  Serial.print("Temperature: ");
  Serial.print(temp.temperature, 2);
  Serial.println();

  Serial.println("---");

  delay(700); // Update every 700ms
}