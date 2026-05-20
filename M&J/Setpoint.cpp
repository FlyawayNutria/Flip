#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const int PRINT_INTERVAL = 30; // ~33Hz for smooth plotting
const int LOOP_INTERVAL     = 10;
const float C = 0.9;
const float DT = (float)LOOP_INTERVAL / 1000.0; 
static float tiltx = 0.0;
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for serial monitor

  Serial.println("MPU6050 Center Calibration Script");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip! Check your wiring.");
    while (1) delay(10);
  }

  // Match the exact configurations used in your main code
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  Serial.println("--- Calibration Instructions ---");
  Serial.println("1. Open the Arduino IDE Serial Plotter (Ctrl+Shift+L / Cmd+Shift+L).");
  Serial.println("2. Hold the robot by hand at its perfect, stable balance point.");
  Serial.println("3. Note the average baseline value of 'Tilt_Value'.");
  Serial.println("4. Copy that number into your main script's 'targetAngle' variable.");
  Serial.println("--------------------------------");
  delay(2000);
}

void loop() {
  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // This replicates your exact math from the main script
    float accel_tilt = atan2(a.acceleration.x, a.acceleration.z)*180.0/PI;
    float gyro_tilt = g.gyro.x * 180.0/PI;
    
    tiltx = (1.0-C)*accel_tilt + C*(tiltx + gyro_tilt * DT);
    
    // Formatted cleanly for the Arduino Serial Plotter
    Serial.print("Tilt_Value:");
    Serial.println(tiltx, 4); // Prints with 4 decimal places for precision
  }
}
