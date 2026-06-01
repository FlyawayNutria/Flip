#pragma once

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

extern Adafruit_MPU6050 mpu;

inline void calibrateGyro() {
    const int NUM_SAMPLES = 2000;

    float GYRO_BIAS_X = 0.0f;
    float GYRO_BIAS_Y = 0.0f;
    float GYRO_BIAS_Z = 0.0f;

    sensors_event_t a, g, temp;

    Serial.println();
    Serial.println("Gyro Calibration - keep bot still");

    delay(3000);

    for (int i = 0; i < NUM_SAMPLES; i++) {
        mpu.getEvent(&a, &g, &temp);

        GYRO_BIAS_X += g.gyro.x;
        GYRO_BIAS_Y += g.gyro.y;
        GYRO_BIAS_Z += g.gyro.z;

        delay(2);
    }

    GYRO_BIAS_X /= NUM_SAMPLES;
    GYRO_BIAS_Y /= NUM_SAMPLES;
    GYRO_BIAS_Z /= NUM_SAMPLES;

    Serial.println();
    Serial.println("Calibration complete.");
    Serial.printf("GYRO_BIAS_X = %.6f\n", GYRO_BIAS_X);
    Serial.printf("GYRO_BIAS_Y = %.6f\n", GYRO_BIAS_Y);
    Serial.printf("GYRO_BIAS_Z = %.6f\n", GYRO_BIAS_Z);
}
