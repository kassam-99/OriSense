#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Motion Tracking Variables
float velocity = 0.0;
float sessionDistance = 0.0;
float totalDistance = 0.0;
float accelBiasX = 0.0;

unsigned long lastTime = 0;

// Configuration Constants
const float ACCEL_THRESHOLD = 100.0;        // Dead zone in cm/s^2
const float MAX_ACCEL = 300.0;             // Clamp extreme values
const float DRIFT_DISTANCE_LIMIT = 100.0;  // Maximum distance drift
const float STILL_THRESHOLD = 0.5;         // Acceptable variance for stillness
const int CALIBRATION_SAMPLES = 100;
const float VELOCITY_DECAY = 0.98;         // Reduce slow drift
const float STOP_THRESHOLD = 0.05;          // Velocity cutoff (cm/s)
int stillCounter = 0;
const int STILL_COUNT_REQUIRED = 3; // adjust if needed



void calibrateSensor() {
  Serial.println("Calibrating MPU6050...");
  float sum = 0.0, sumSq = 0.0;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sum += a.acceleration.x;
    sumSq += a.acceleration.x * a.acceleration.x;
    delay(10);
  }

  accelBiasX = sum / CALIBRATION_SAMPLES;
  float variance = (sumSq / CALIBRATION_SAMPLES) - (accelBiasX * accelBiasX);

  Serial.print("Bias: "); Serial.println(accelBiasX, 4);
  Serial.print("Variance: "); Serial.println(variance, 4);

  if (variance > STILL_THRESHOLD) {
    Serial.println("Calibration failed. Please ensure the sensor is still.");
    while (1);

  } 
  
  else {
    Serial.println("Calibration successful.");
  }

}





void setup() {


  Serial.begin(115200);
  while (!Serial);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found! Check wiring.");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);  // Strong filtering

  delay(1000);
  calibrateSensor();

  lastTime = millis();

}





void loop() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // Convert to cm/s^2 and adjust bias
  float accelX = (a.acceleration.x - accelBiasX) * 100.0;

  

  // Clamp and dead zone
  if (abs(accelX) > MAX_ACCEL) accelX = 0.0;
  if (abs(accelX) < ACCEL_THRESHOLD) accelX = 0.0;


  // Detect stillness
  if (accelX == 0.0) {
    stillCounter++;
  } else {
    stillCounter = 0;  // reset if movement resumes
  }


  if (stillCounter >= STILL_COUNT_REQUIRED) {
    velocity = 0.0;
  } else {
    // Integrate to velocity
    velocity += accelX * dt;
    velocity *= VELOCITY_DECAY;

    // Stop very small velocities (reduce drift)
    if (abs(velocity) < STOP_THRESHOLD) {
      velocity = 0.0;
    }
  }

  // Only integrate distance if velocity is non-zero
  if (velocity != 0.0) {
    sessionDistance += (velocity * dt);
    totalDistance += abs(velocity * dt);

    Serial.print("AccelX (cm/s^2): ");
    Serial.print(accelX, 2);
    Serial.print("\tVelocity (cm/s): ");
    Serial.print(velocity, 2);
    Serial.print("\tSessionDist (cm): ");
    Serial.print(sessionDistance, 2);
    Serial.print("\tTotalDist (cm): ");
    Serial.println(totalDistance, 2);   


  }

  delay(100);  // 10Hz sampling
}










