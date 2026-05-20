#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const int PRINT_INTERVAL = 30; // ~33Hz for smooth plotting
const int LOOP_INTERVAL  = 10;
const float C = 0.98; // Put this back to 0.98!
const float DT = (float)LOOP_INTERVAL / 1000.0; 
static float tiltx = 0.0;

unsigned long lastPrint = 0;
unsigned long loopTimer = 0; // Added a timer for the math

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); 

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip! Check your wiring.");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
}

void loop() {
  // 1. Math runs strictly every 10ms
  if (millis() - loopTimer >= LOOP_INTERVAL) {
    loopTimer = millis();

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // 1. Accel looks at X and Z for forward/backward tilt
    float accel_tilt = atan2(a.acceleration.x, a.acceleration.z) * 180.0 / PI;

    // 2. Gyro looks at Y for rotation around the wheel axle
    float gyro_tilt = -g.gyro.y * 180.0 / PI; 

    // 3. The Perfect Blend (C = 0.98)
    tiltx = (1.0 - C) * accel_tilt + C * (tiltx + gyro_tilt * DT);
  }

  // 2. Plotter prints strictly every 30ms
  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();
    Serial.print("Tilt_Value:");
    Serial.println(tiltx, 4); 
  }
}
