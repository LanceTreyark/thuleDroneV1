#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <ITG3200.h>  // Include ITG3200 library

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
ITG3200 gyro;  // ITG3200 instance

float roll = 0, pitch = 0, yaw = 0;
unsigned long prevTime = 0;
float gyroXBias = 0, gyroYBias = 0, gyroZBias = 0;

void calibrateGyro() {
  int numSamples = 500;
  for (int i = 0; i < numSamples; i++) {
    int16_t gx, gy, gz;
    gyro.getRotation(&gx, &gy, &gz);
    gyroXBias += gx;
    gyroYBias += gy;
    gyroZBias += gz;
    delay(5);
  }
  gyroXBias /= numSamples;
  gyroYBias /= numSamples;
  gyroZBias /= numSamples;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  if (!accel.begin()) {
    Serial.println("No ADXL345 detected!");
    while (1);
  }

  gyro.initialize();
  
  if (!gyro.testConnection()) {
    Serial.println("No ITG3200 detected!");
    while (1);
  }

  // Perform gyro calibration
  calibrateGyro();
  Serial.println("IMU Initialized");
}

void loop() {
  // Get current time
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0; // Time in seconds
  prevTime = currentTime;

  // Read accelerometer
  sensors_event_t accelEvent;
  accel.getEvent(&accelEvent);

  // Calculate pitch and roll from accelerometer
  float accelRoll = atan2(accelEvent.acceleration.y, accelEvent.acceleration.z) * 180 / PI;
  float accelPitch = atan2(-accelEvent.acceleration.x, sqrt(accelEvent.acceleration.y * accelEvent.acceleration.y +
                                                            accelEvent.acceleration.z * accelEvent.acceleration.z)) * 180 / PI;

  // Read gyroscope
  int16_t gyroX, gyroY, gyroZ;
  gyro.getRotation(&gyroX, &gyroY, &gyroZ);

  // Apply bias compensation
  gyroX -= gyroXBias;
  gyroY -= gyroYBias;
  gyroZ -= gyroZBias;

  // Convert gyro readings to degrees per second
  float gyroRollRate = gyroX * 1.0 / 14.375; // Scale by sensitivity
  float gyroPitchRate = gyroY * 1.0 / 14.375;
  float gyroYawRate = gyroZ * 1.0 / 14.375; // Gyroscope angular velocity around Z-axis for yaw

  // Complementary filter for roll, pitch, and yaw
  float alpha = 0.98;
  roll = alpha * (roll + gyroRollRate * dt) + (1 - alpha) * accelRoll;
  pitch = alpha * (pitch + gyroPitchRate * dt) + (1 - alpha) * accelPitch;
  yaw += gyroYawRate * dt;  // Integrate angular velocity to get yaw

  // Print results
  Serial.print("Roll: "); Serial.print(roll); Serial.print(" ");
  Serial.print("Pitch: "); Serial.print(pitch); Serial.print(" ");
  Serial.print("Yaw: "); Serial.println(yaw);

  delay(10);
}
