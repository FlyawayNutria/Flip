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
#include <HTTPClient.h> //Send IP to google sheet
#include "calibrate.h" //Calibrate the MPU6050

struct Vec3 { //3x1 vector
	float x;
	float y;
	float z;
};

//Multiplies a 3x3 matrix by a 3x1 vector
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
float GYRO_BIAS_X = -0.029064;
float GYRO_BIAS_Y = -0.012810;
float GYRO_BIAS_Z = -0.022884;

const char* ssid = WIFI_SSID;
const char* username = WIFI_USER;
const char* password = WIFI_PASS;
const char* url = SCRIPT_URL; //Send IP to google sheet

//Given pins
const int STEPPER1_DIR_PIN  = 16; //D1
const int STEPPER1_STEP_PIN = 17; //S1 
const int STEPPER2_DIR_PIN  = 4; //D2
const int STEPPER2_STEP_PIN = 14; //S2
const int STEPPER_EN_PIN    = 27; //EN

const int ADC_CS_PIN        = 5;
const int ADC_SCK_PIN       = 18;
const int ADC_MISO_PIN      = 19;
const int ADC_MOSI_PIN      = 23;
const int TOGGLE_PIN        = 32;

//Interval constants
const int PRINT_INTERVAL      = 50;
const int LOOP_INTERVAL       = 10;
const int STEPPER_INTERVAL_US = 20;
const int OUTER_LOOP_INTERVAL = 50;

//Filter constants
const float C = 0.99;
const float DT = (float)LOOP_INTERVAL / 1000.0;

//Variables for inner loop balancing
float Kp = 12.0;
float Ki = 0.00;
float Kd = 2;
float setpoint = 87.0;
float tiltx = 0.0;
float target_accel = 0.0;
bool robot_active = false;

float error_old = 0.0;
float error_integral = 0.0;
float integrated_velocity = 0.0;

const float MAX_INTEGRAL = 50.0;
const float MAX_ACCEL = 1000.0;
const float MAX_SPEED = 40.0;

//Make a perfect 90 degree turn (for maze)
float steering_offset = 0.0;
const float TURN_SPEED = 3.0; //Max turn speed
float current_heading = 0.0; //Absolute angle (degrees)
float target_heading = 0.0; //Desired angle
bool is_turning = false; //Flag
float K_YAW = 0.06; //Proportional gain for turning (tune ts) //0.07 and 0.02 works
float K_DAMP = 0.01; //Derivative gain for turning
float yawRate = 0.0;

//Dynamic control variables
float dynamic_tilt = 0.0;
float vKp = 0.2;
float vKi = 0.0;
float vKd = 0.005;
float prev_drive_velo = 0.0;
bool velo_derivative = false;
float target_pos = 0.0; //rad
const float oDT = (float)OUTER_LOOP_INTERVAL/1000.0;
const float MAX_TILT = 5.0; //5 degrees
float target_drive_velocity = 0.0;
float MAX_DRIVE_VELOCITY = 5.0;
bool driving = false;
const float brake_threshold = 0.15;

void uploadIP() { //Send IP to google sheet
	HTTPClient http;
	http.begin(url);
	http.addHeader("Content-Type", "application/json");
	String payload = "{\"device\":\"balancebot\"," "\"ip\":\"" + WiFi.localIP().toString() + "\"}";
	int code = http.POST(payload);
	Serial.printf("Upload result: %d\n", code);
	http.end();
}

ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;
WebServer server(80);

step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

float getPos() {
    return (step1.getPositionRad() - step2.getPositionRad()) / 2.0;
}

void clearDrive() {
    target_drive_velocity = 0.0;
    driving = false;
    target_pos = getPos();
	prev_drive_velo = 0.0;
	velo_derivative = false;
}

String getWebPage() { //Pulls the webpage from webpage.h, and replaces placeholders
	String html = FPSTR(WEBPAGE_HTML);

	html.replace("%KP%", String(Kp));
	html.replace("%KI%", String(Ki));
	html.replace("%KD%", String(Kd));
	html.replace("%SP%", String(setpoint));
	html.replace("%TKP%", String(K_YAW, 3));
	html.replace("%TKD%", String(K_DAMP, 3));
	html.replace("%VKP%", String(vKp));
	html.replace("%VKI%", String(vKi));
	html.replace("%VKD%", String(vKd, 3));

	return html;
}

bool TimerHandler(void * timerNo) {
	step1.runStepper();
	step2.runStepper();
	return true;
}

//Used for closed loop turning
void startSpotTurn(float deltaDeg) {
	clearDrive();
	error_integral = 0.0;
	steering_offset = 0.0;
	target_heading = current_heading + deltaDeg;
	is_turning = true;
}

void resetBot() {
	robot_active = false;
	digitalWrite(STEPPER_EN_PIN, HIGH);
	
	//Inner loop balancing
	integrated_velocity = 0.0;
	error_integral = 0.0;
	error_old = 0.0;
	target_accel = 0.0;

	//Turning and movement
	steering_offset = 0.0;
	target_drive_velocity = 0.0;
	is_turning = false;
	yawRate = 0.0;
	clearDrive();

	step1.setTargetSpeedRad(0.0);
	step2.setTargetSpeedRad(0.0);
}

void setup() {
	Serial.begin(115200); //Serial monitor rate
	//Set pinmodes
	pinMode(TOGGLE_PIN, OUTPUT);
	pinMode(STEPPER_EN_PIN, OUTPUT);
	pinMode(ADC_CS_PIN, OUTPUT);
	digitalWrite(STEPPER_EN_PIN, HIGH); // disabled during setup
	digitalWrite(ADC_CS_PIN, HIGH);

	SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN);

	//Connect to IMU
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
	Serial.print("IP Address: http://");
	Serial.println(WiFi.localIP());
	uploadIP();

	server.on("/", []() {
		server.send(200, "text/html", getWebPage());
	});

	server.on("/data", []() {
		String json = "{";
		json += "\"tilt\":" + String(tiltx, 2);
		json += ",\"set\":" + String(setpoint, 2);
		json += ",\"active\":" + String(robot_active ? "true" : "false");
		json += ",\"turning\":" + String(is_turning ? "true" : "false");
		json += "}";
		server.send(200, "application/json", json);
	});

	server.on("/update", []() {
		if (server.hasArg("p")) Kp = server.arg("p").toFloat();
		if (server.hasArg("i")) Ki = server.arg("i").toFloat();
		if (server.hasArg("d")) Kd = server.arg("d").toFloat();
		if (server.hasArg("t")) setpoint = server.arg("t").toFloat();
		if (server.hasArg("vp")) vKp = server.arg("vp").toFloat();
		if (server.hasArg("vi")) vKi = server.arg("vi").toFloat();
		if (server.hasArg("vd")) vKd = server.arg("vd").toFloat();
		if (server.hasArg("tkp")) K_YAW = server.arg("tkp").toFloat();
		if (server.hasArg("tkd")) K_DAMP = server.arg("tkd").toFloat();

		error_integral = 0.0;
		Serial.printf("Web Update | Kp: %.2f | Ki: %.2f | Kd: %.2f | Setpoint: %.2f | TKP: %.3f | TKD: %.3f | VKP: %.2f | VKI: %.2f | VKD: %.2f\n", Kp, Ki, Kd, setpoint, K_YAW, K_DAMP, vKp, vKi, vKd);

		server.send(200, "text/html", getWebPage());
	});

	server.on("/control", []() {
		if (server.hasArg("dir")) {
			String dir = server.arg("dir");
			Serial.println(dir);
			if (dir == "F") { target_drive_velocity = MAX_DRIVE_VELOCITY; steering_offset = 0.0; is_turning = false; driving = true; }
			else if (dir == "B") { target_drive_velocity = -MAX_DRIVE_VELOCITY; steering_offset = 0.0; is_turning = false; driving = true; }
			else if (dir == "L") { clearDrive(); steering_offset = TURN_SPEED; is_turning = false; }
			else if (dir == "R") { clearDrive(); steering_offset = -TURN_SPEED; is_turning = false; }
			else if (dir == "T1" && !is_turning) { startSpotTurn(90.0); }//target_heading = current_heading - 90.0; is_turning = true; }//Turn 90 degrees CW
			else if (dir == "T2" && !is_turning) { startSpotTurn(-90.0); }//target_heading = current_heading + 90.0; is_turning = true; } //Turn 90 degrees ACW
			else {
                if (driving) {
                    driving = false;
                    target_drive_velocity = 0.0;
                } else { clearDrive();}
			        if (!is_turning) { steering_offset = 0.0; }
			} //Do nothing
		}
		server.send(200, "text/plain", "OK");
	});

	server.on("/calibrate", []() {
		Serial.println("Web gyro calibration");
		resetBot();
		calibrateGyro();
		server.send(200, "text/plain", "Calibration Complete");
	});

	server.on("/stop", []() {
		resetBot();
		Serial.println("Web Stop");
		server.send(200, "text/html", getWebPage());
	});

	server.begin();

	if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
		Serial.println("Failed to start stepper interrupt");
		while (1) delay(10);
	}

	Serial.println("Initialised Interrupt for Stepper");

	step1.setAccelerationRad(1000.0);
	step2.setAccelerationRad(1000.0);

	digitalWrite(STEPPER_EN_PIN, LOW); // enable driver after setup
}

void loop() {
	static unsigned long printTimer = 0;
	static unsigned long loopTimer = 0;
	static unsigned long outerLoopTimer = 0;

	//Outer loop dynamic control - ver2, turning is good
	if (millis() > outerLoopTimer) {
		outerLoopTimer += OUTER_LOOP_INTERVAL;

		float leftSpeed = step1.getSpeedRad();
		float rightSpeed = step2.getSpeedRad();
		float drive_velocity = (rightSpeed - leftSpeed) * 0.5;

		float drive_accel = 0.0;
		if (!is_turning && velo_derivative) {
			drive_accel = (drive_velocity - prev_drive_velo) / oDT;
		}
		velo_derivative = !is_turning;
		prev_drive_velo = drive_velocity;

		if (is_turning || !driving) {
			dynamic_tilt = 0.0;
		} else {
			float velocity_error = target_drive_velocity - drive_velocity;
			float new_dynamic_tilt = (vKp * velocity_error) - (vKd * drive_accel);
			dynamic_tilt = constrain(new_dynamic_tilt, -MAX_TILT, MAX_TILT);
		}
	}
	
	/*
	//Outer loop ver1 
	if (millis() > outerLoopTimer) {
		outerLoopTimer += OUTER_LOOP_INTERVAL;

		if (is_turning) {
			dynamic_tilt = 0.0;
			prev_drive_velo = 0.0;
			velo_derivative = false;
			return;
		}

		float leftSpeed = step1.getSpeedRad();
		float rightSpeed = step2.getSpeedRad();
		float drive_velocity = (rightSpeed - leftSpeed) * 0.5;

		if (!driving) {
			target_drive_velocity = 0.0;
			prev_drive_velo = drive_velocity;
			velo_derivative = true;
			dynamic_tilt = 0.0;
			return;
		}

		//Drive mode (F/B is pressed)
		float velocity_error = target_drive_velocity - drive_velocity;
		float drive_accel = 0.0;
		if (velo_derivative) {
			drive_accel = (drive_velocity - prev_drive_velo) /oDT;
		} else {
			velo_derivative = true;
		}
		prev_drive_velo = drive_velocity;
		dynamic_tilt = (vKp * velocity_error) - (vKd * drive_accel);
		dynamic_tilt = constrain(dynamic_tilt, -MAX_TILT, MAX_TILT);
	} */

	//Inner loop balancing
	if (millis() > loopTimer) {
		loopTimer += LOOP_INTERVAL;

		sensors_event_t a, g, temp;
		mpu.getEvent(&a, &g, &temp);

		// Raw MPU readings
		Vec3 accelRaw = { a.acceleration.x, a.acceleration.y, a.acceleration.z };
		Vec3 gyroRaw = { g.gyro.x - GYRO_BIAS_X, g.gyro.y - GYRO_BIAS_Y,g.gyro.z - GYRO_BIAS_Z };
		Vec3 accelRobot = matMul(R_sensor_to_robot, accelRaw); //Correct for sensor calibration
		Vec3 gyroRobot  = matMul(R_sensor_to_robot, gyroRaw);

		// Use corrected robot-frame axes
		float accel_tilt = atan2(accelRobot.x, accelRobot.z) * 180.0 / PI;
		float gyro_tilt  = -gyroRobot.y * 180.0 / PI;

		tiltx = (1.0 - C) * accel_tilt + C * (tiltx + gyro_tilt * DT); //Complementary filter

		//Track yaw for the 90 degree turn (x is the yaw axis)
		yawRate = gyroRobot.x * (180.0 / PI);
		current_heading -= yawRate * DT;
		if (is_turning) {
			float heading_error = target_heading - current_heading;
			while (heading_error > 180.0) heading_error -= 360.0;
			while (heading_error < -180.0) heading_error += 360.0;
			steering_offset = -(K_YAW * heading_error) - (K_DAMP * yawRate); //PD controller //Here might have to flip signs
            steering_offset = constrain(steering_offset, -TURN_SPEED, TURN_SPEED);

			//if (abs(heading_error) < 2.0 && abs(yawRate) < 7.0) { //Stop turning if within 2 degree of target and rotation is slow
			if (abs(heading_error) < 2.0) { //Force exit 
				is_turning = false;
				steering_offset = 0.0;
				current_heading = target_heading;
			}
		}

		float error = (setpoint + dynamic_tilt) - tiltx;

		// Fall safety
		if (abs(error) > 30.0 && robot_active) {
			resetBot();
		}

		// Auto-arm when close to setpoint
		if (!robot_active) {
			if (abs(error) < 0.5) {
                resetBot();
                robot_active = true; //Overwrite these after the reset
                digitalWrite(STEPPER_EN_PIN, LOW);
                error_old = error;
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
			float error_derivative = -gyro_tilt; //might be a sign error?

			// PID gives acceleration
			target_accel = Kp * error + Ki * error_integral + Kd * error_derivative;
			target_accel = constrain(target_accel, -MAX_ACCEL, MAX_ACCEL);

			integrated_velocity += target_accel * DT;
			integrated_velocity = constrain(integrated_velocity, -MAX_SPEED, MAX_SPEED);

			step1.setTargetSpeedRad(-integrated_velocity - steering_offset);
			step2.setTargetSpeedRad(integrated_velocity - steering_offset);

			error_old = error;
		}
	}

	static unsigned long serverTimer = 0;
	if (millis() - serverTimer >= 20) {
		serverTimer = millis();
		server.handleClient();
	}

	//Debug print
	if (millis() > printTimer) {
		printTimer += PRINT_INTERVAL;
		//Print something to serial
	}
}
