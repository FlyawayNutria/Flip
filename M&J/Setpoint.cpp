#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const int PRINT_INTERVAL = 30;
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for serial monitor

  Serial.println("MPU6050 Center Calibration Script");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip! Check your wiring.");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  Serial.println("Successful Connection");
  delay(2000);
}

void loop() {
  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //Change to get radians
    float tiltx = atan2(a.acceleration.x, a.acceleration.z)*180.0/PI;
    
    // Formatted cleanly for the Arduino Serial Plotter
    Serial.print("Tilt_Value:");
    Serial.println(tiltx, 4); // Prints with 4 decimal places for precision
  }
}
