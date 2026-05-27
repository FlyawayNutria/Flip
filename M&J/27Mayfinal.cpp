#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include "secrets.h"
#include <webpage.h>

#include <WiFi.h>
#include "esp_wpa2.h"
#include <WebServer.h>

struct Vec3 {
  float x;
  float y;
  float z;
};

Vec3 matMul(const float R[3][3], Vec3 v) {
  Vec3 out;

  out.x = R[0][0] * v.x + R[0][1] * v.y + R[0][2] * v.z;
  out.y = R[1][0] * v.x + R[1][1] * v.y + R[1][2] * v.z;
  out.z = R[2][0] * v.x + R[2][1] * v.y + R[2][2] * v.z;

  return out;
}

float R_sensor_to_robot[3][3] = {
  { 1.0, 0.0, 0.0 },
  { 0.0, 1.0, 0.0 },
  { 0.0, 0.0, 1.0 }
};

// Manually measured gyro bias, in rad/s.
// These are subtracted from the raw gyro readings.
const float GYRO_BIAS_X = -0.029064;
const float GYRO_BIAS_Y = -0.012810;
const float GYRO_BIAS_Z = -0.022884;

const char* ssid = WIFI_SSID;
const char* username = WIFI_USER;
const char* password = WIFI_PASS;

const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15;

const int ADC_CS_PIN        = 5;
const int ADC_SCK_PIN       = 18;
const int ADC_MISO_PIN      = 19;
const int ADC_MOSI_PIN      = 23;

const int TOGGLE_PIN        = 32;

const int PRINT_INTERVAL      = 5000;
const int LOOP_INTERVAL       = 10;
const int OUTER_LOOP_INTERVAL = 50;
const int STEPPER_INTERVAL_US = 20;

const float VREF = 4.096;

const float C = 0.99;
const float DT = (float)LOOP_INTERVAL / 1000.0;

// Velocity damping for the extra acceleration-to-speed integrator
const float Kv = 5.0;
float Kp = 24.0;
float Ki = 0.00;
float Kd = 0.6;
float setpoint = 88.0;

float tiltx = 0.0;
float target_accel = 0.0;
bool robot_active = false;

float error_old = 0.0;
float error_integral = 0.0;
float integrated_velocity = 0.0;

const float MAX_INTEGRAL = 50.0;
const float MAX_ACCEL = 1000.0;
const float MAX_SPEED = 40.0;

float drive_position = 0.0;
float drive_velocity = 0.0;
float target_drive_velocity = 0.0;
float MAX_DRIVE_VELOCITY = 15.0;

const float K_POS = 0.015;   // position correction
const float K_VEL = 0.20;    // velocity correction
const float MAX_SETPOINT_SHIFT = 4.0;  // degrees

// Dynamic control variables
float target_setpoint = 0.0;
float steering_offset = 0.0;
const float MAX_DRIVE_ANGLE = 2.0;
const float TURN_SPEED = 4.5;
float commanded_setpoint = 0.0;
float current_setpoint = 0.0;
bool is_holding = false; //Need this to tell when bot is still or not

//Make a perfect 90 degree turn (for maze)
float current_heading = 0.0;   //Absolute angle (degrees)
float target_heading = 0.0;    //Desired angle
bool is_turning = false;       //Flag
float K_YAW = 0.07;      //Proportional gain for turning (tune ts) //0.07 and 0.02 works
float K_DAMP = 0.02; //Derivative gain for turning

float yawX = 0.0;
float yawRate = 0.0;
float alpha = 0.98;
float pitch = 0.0;
float roll = 0.0;
float yaw = 0.0;

float current_target_velocity = 0.0;
const float VELOCITY_RAMP_RATE = 30.0; //20

ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;
WebServer server(80);

step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

String getWebPage() {
  String html = FPSTR(WEBPAGE_HTML);
  
  html.replace("%KP%", String(Kp));
  html.replace("%KI%", String(Ki));
  html.replace("%KD%", String(Kd));
  html.replace("%SP%", String(setpoint));
  html.replace("%TKP%", String(K_YAW));
  html.replace("%TKD%", String(K_DAMP));
  html.replace("%STATUS%", robot_active ? "<span style='color:green;'>ARMED</span>" : "<span style='color:red;'>STANDBY</span>");

  return html;
}

bool TimerHandler(void * timerNo) {
  step1.runStepper();
  step2.runStepper();

  return true;
}

uint16_t readADC(uint8_t channel) {
  uint8_t tx0 = 0x06 | (channel >> 2);
  uint8_t tx1 = (channel & 0x03) << 6;

  digitalWrite(ADC_CS_PIN, LOW);

  SPI.transfer(tx0);
  uint8_t rx0 = SPI.transfer(tx1);
  uint8_t rx1 = SPI.transfer(0x00);

  digitalWrite(ADC_CS_PIN, HIGH);

  uint16_t result = ((rx0 & 0x0F) << 8) | rx1;
  return result;
}

void setup()
{
  Serial.begin(115200);

  pinMode(TOGGLE_PIN, OUTPUT);
  pinMode(STEPPER_EN_PIN, OUTPUT);
  pinMode(ADC_CS_PIN, OUTPUT);

  digitalWrite(STEPPER_EN_PIN, HIGH);   // disabled during setup
  digitalWrite(ADC_CS_PIN, HIGH);

  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }

  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  //Connect to Imperial-WPA
  Serial.print("Connecting to WPA2 Enterprise WiFi");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)WIFI_USER,strlen(WIFI_USER));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)WIFI_USER,strlen(WIFI_USER));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)WIFI_PASS,strlen(WIFI_PASS));
  esp_wifi_sta_wpa2_ent_enable();
  WiFi.begin(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println("\n--- WIFI CONNECTED ---");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", []() {
    server.send(200, "text/html", getWebPage());
  });

  server.on("/data", []() {
    String json = "{";
    json += "\"tilt\":" + String(tiltx, 2);
    json += ",\"set\":" + String(setpoint, 2);
    json += ",\"active\":" + String(robot_active ? "true" : "false");
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/update", []() {
    if (server.hasArg("p")) Kp = server.arg("p").toFloat();
    if (server.hasArg("i")) Ki = server.arg("i").toFloat();
    if (server.hasArg("d")) Kd = server.arg("d").toFloat();
    if (server.hasArg("t")) setpoint = server.arg("t").toFloat();
	if (server.hasArg("tkp")) K_YAW = server.arg("tkp").toFloat();
	if (server.hasArg("tkd")) K_DAMP = server.arg("tkd").toFloat();

    error_integral = 0.0;
    Serial.printf("Web Update | Kp: %.2f | Ki: %.2f | Kd: %.2f | Setpoint: %.2f | TKP: %.2f | TKD: %.2f\n", Kp, Ki, Kd, setpoint, K_YAW, K_DAMP);

    server.send(200, "text/html", getWebPage());
  });

  server.on("/control", []() {
    if (server.hasArg("dir")) {
      String dir = server.arg("dir");
      Serial.println(dir);
      if (dir == "F") { target_drive_velocity = MAX_DRIVE_VELOCITY; steering_offset = 0.0; is_turning = false; }
      else if (dir == "B") { target_drive_velocity = -MAX_DRIVE_VELOCITY; steering_offset = 0.0; is_turning = false; }
      else if (dir == "L") { target_drive_velocity = 0.0; steering_offset = -TURN_SPEED; is_turning = false; }
      else if (dir == "R") { target_drive_velocity = 0.0; steering_offset = TURN_SPEED; is_turning = false; }
      else if (dir == "T1" && !is_turning) { target_heading = current_heading - 90.0; is_turning = true; }//Turn 90 degrees CW
      else if (dir == "T2" && !is_turning) { target_heading = current_heading + 90.0; is_turning = true; } //Turn 90 degrees ACW
      else {
        target_drive_velocity = 0.0;
        if (!is_turning) { steering_offset = 0.0; }
      } //Do nothing
    }
    server.send(200, "text/plain", "OK");
  });


  server.on("/stop", []() {
    robot_active = false;
    digitalWrite(STEPPER_EN_PIN, HIGH);

    integrated_velocity = 0.0;
    error_integral = 0.0;
    drive_position = 0.0;
    drive_velocity = 0.0;

    step1.setTargetSpeedRad(0.0);
    step2.setTargetSpeedRad(0.0);

    Serial.println("Web Emergency Stop");

    server.send(200, "text/html", getWebPage());
  });

  server.begin();

  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
  }

  Serial.println("Initialised Interrupt for Stepper");

  step1.setAccelerationRad(2000.0);
  step2.setAccelerationRad(2000.0);

  digitalWrite(STEPPER_EN_PIN, LOW);    // enable driver after setup
}

void loop()
{
  static unsigned long printTimer = 0;
  static unsigned long loopTimer = 0;

  static unsigned long outerLoopTimer = 0;
  static float dynamic_lean = 0.0;

  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Raw MPU readings
    Vec3 accelRaw = {
      a.acceleration.x,
      a.acceleration.y,
      a.acceleration.z
    };

    Vec3 gyroRaw = {
      g.gyro.x - GYRO_BIAS_X,
      g.gyro.y - GYRO_BIAS_Y,
      g.gyro.z - GYRO_BIAS_Z
    };

    // Correct raw sensor frame into robot frame
    Vec3 accelRobot = matMul(R_sensor_to_robot, accelRaw);
    Vec3 gyroRobot  = matMul(R_sensor_to_robot, gyroRaw);

	/*
    //Estimate angles - NOT MY CODE
    float accPitch = atan2(a.acceleration.x, a.acceleration.z) * 180 / PI;
    float accRoll  = atan2(-a.acceleration.y, a.acceleration.x) * 180 / PI;
    // Complementary filter: combine gyro and accelerometer
    pitch = alpha * (pitch + g.gyro.y * DT * 180 / PI) + (1 - alpha) * accPitch;
    roll  = alpha * (roll  + g.gyro.z * DT * 180 / PI) + (1 - alpha) * accRoll;
    yaw += g.gyro.x * DT * 180 / PI; // Yaw only from gyro → will drift

    yawX = gyroRobot.x; */

    // Use corrected robot-frame axes
    float accel_tilt = atan2(accelRobot.x, accelRobot.z) * 180.0 / PI;
    float gyro_tilt  = -gyroRobot.y * 180.0 / PI;

    // Complementary filter
    tiltx = (1.0 - C) * accel_tilt + C * (tiltx + gyro_tilt * DT);

    //Track yaw for the 90 degree turn (x is the yaw axis)
    yawRate = gyroRobot.x * (180.0 / PI);
    if (abs(yawRate) < 0.5) { //ignore microjitters
      yawRate = 0.0;
    }
    current_heading += yawRate * DT;
    if (is_turning) {
        float heading_error = target_heading - current_heading;
		while (heading_error > 180.0) heading_error -= 360.0;
		while (heading_error < -180.0) heading_error += 360.0;
        steering_offset = -(K_YAW * heading_error) + (K_DAMP * yawRate); //PD controller
        steering_offset = constrain(steering_offset, -TURN_SPEED, TURN_SPEED);

        if (abs(heading_error) < 1.0 && abs(yawRate) < 5.0) { //Stop turning if within 1 degree of target and rotation is slow
            is_turning = false;
            steering_offset = 0.0;
            current_heading = target_heading;
        }
    }

    // Estimate how far the robot has driven, using commanded wheel speed.
    drive_velocity = integrated_velocity;

    if (target_drive_velocity != 0.0) {
        is_holding = false;
    } else if (abs(drive_velocity) < 1.0 && !is_holding) { //Robot has braked to a near stop, and no longer controlling
        is_holding = true; //Lock position now
    }

    if (!is_holding) {
        drive_position = 0.0;
    } else {
        drive_position += drive_velocity * DT;
    }

    if (millis() > outerLoopTimer) {
        outerLoopTimer += OUTER_LOOP_INTERVAL;

        //Ramping the speed slowly
        float max_step = VELOCITY_RAMP_RATE * (OUTER_LOOP_INTERVAL/1000.0);
        if (current_target_velocity < target_drive_velocity) {
            current_target_velocity = min(current_target_velocity + max_step, target_drive_velocity); //Add a step or become target
        } else if (current_target_velocity > target_drive_velocity) {
            current_target_velocity = max(current_target_velocity - max_step, target_drive_velocity);
        }
        float velocity_error = drive_velocity - current_target_velocity;
        dynamic_lean = K_VEL * velocity_error;

        //float velocity_error = drive_velocity - target_drive_velocity;
        dynamic_lean = K_VEL * velocity_error;

        if (is_holding) {
           dynamic_lean += K_POS * drive_position;
        }

        dynamic_lean = constrain(dynamic_lean, -MAX_SETPOINT_SHIFT, MAX_SETPOINT_SHIFT);
    }

    float balance_setpoint = setpoint + dynamic_lean;
    float error = balance_setpoint - tiltx;

    // Fall safety
    if (abs(error) > 30.0 && robot_active) {
      robot_active = false;
      digitalWrite(STEPPER_EN_PIN, HIGH);

      integrated_velocity = 0.0;
      error_integral = 0.0;
      error_old = 0.0;
      drive_position = 0.0;
      drive_velocity = 0.0;

      step1.setTargetSpeedRad(0.0);
      step2.setTargetSpeedRad(0.0);
    }

    // Auto-arm when close to setpoint
    if (!robot_active) {
      if (abs(error) < 0.5) {
        robot_active = true;
        digitalWrite(STEPPER_EN_PIN, LOW);

        integrated_velocity = 0.0;
        error_integral = 0.0;
        error_old = error;
        drive_position = 0.0;
        drive_velocity = 0.0;
        is_turning = false;
        target_heading = current_heading;
        steering_offset = 0.0;

        Serial.println("Bot armed");
      } else {
        step1.setTargetSpeedRad(0.0);
        step2.setTargetSpeedRad(0.0);
      }
    }

    // Balance controller
    if (robot_active) {
      error_integral += error * DT;
      error_integral = constrain(error_integral, -MAX_INTEGRAL, MAX_INTEGRAL);

      // Since error = setpoint - tiltx
      float error_derivative = -gyro_tilt;

      // PID gives acceleration, Kv damps the integrated velocity
      target_accel = Kp * error + Ki * error_integral + Kd * error_derivative - Kv * integrated_velocity;

      target_accel = constrain(target_accel, -MAX_ACCEL, MAX_ACCEL);

      integrated_velocity += target_accel * DT;
      integrated_velocity = constrain(integrated_velocity, -MAX_SPEED, MAX_SPEED);

      step1.setTargetSpeedRad(-integrated_velocity - steering_offset);
      step2.setTargetSpeedRad(integrated_velocity - steering_offset);

      error_old = error;
    }
  }

  server.handleClient();

  // Debug print
  if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;

    uint16_t rawAdc = readADC(0);
    float voltage = (rawAdc * VREF) / 4095.0;
    //Serial.println(yawRate);
    //Serial.printf("Yawrate %.2f | heading %.2f\n", yawRate, current_heading);
    //Serial.printf("Tilt: %.2f | Set: %.2f | Vel: %.2f | Accel: %.2f | Active: %d | BattADC: %.2fV\n",tiltx, setpoint, integrated_velocity, target_accel, robot_active, voltage);
  }
}
