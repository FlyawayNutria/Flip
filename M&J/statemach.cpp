#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include "secrets.h"
#include <webpage.h>

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h> //Send IP to google sheet
#include <Preferences.h>
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

Preferences prefs;
float accel_pitch_correction_deg = 0.0;

void applyAccelPitchCorrection(float correction_deg) {
	float r = correction_deg * PI / 180.0;
	float c = cos(r);
	float s = sin(r);

	R_sensor_to_robot[0][0] = c;
	R_sensor_to_robot[0][1] = 0.0;
	R_sensor_to_robot[0][2] = s;
	R_sensor_to_robot[1][0] = 0.0;
	R_sensor_to_robot[1][1] = 1.0;
	R_sensor_to_robot[1][2] = 0.0;
	R_sensor_to_robot[2][0] = -s;
	R_sensor_to_robot[2][1] = 0.0;
	R_sensor_to_robot[2][2] = c;
}

void saveAccelCalibration() {
	prefs.begin("accelcal", false);
	prefs.putFloat("pitch_deg", accel_pitch_correction_deg);
	prefs.end();
}

void loadAccelCalibration() {
	prefs.begin("accelcal", true);
	accel_pitch_correction_deg = prefs.getFloat("pitch_deg", 0.0);
	prefs.end();
	applyAccelPitchCorrection(accel_pitch_correction_deg);
	Serial.printf("Accel correction loaded: %.4f deg\n", accel_pitch_correction_deg);
}

// Manually measured gyro bias, in rad/s.
// These are subtracted from the raw gyro readings.  
float GYRO_BIAS_X = -0.029064;
float GYRO_BIAS_Y = -0.012810;
float GYRO_BIAS_Z = -0.022884;

const char* ssid = WIFI_SSID;
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
float Kv = 0.0;
float Kp = 24.0;
float Ki = 0.00;
float Kd = 5.0;

float moveKv = 5.0;
float moveKp = 24.0;
float moveKi = 0.00;
float moveKd = 0.6;

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
float v_error_integral = 0.0;
const float oDT = (float)OUTER_LOOP_INTERVAL/1000.0;
const float MAX_TILT = 5.0; //5 degrees
float target_drive_velocity = 0.0;
float MAX_DRIVE_VELOCITY = 5.0;
enum RobotState {
	STATE_STATIONARY,
	STATE_DRIVING,
	STATE_BRAKING,
	STATE_TURNING
};

RobotState robot_state = STATE_STATIONARY;
const float brake_threshold = 0.15;

bool accel_cal_active = false;
unsigned long accel_cal_start_ms = 0;
unsigned long accel_cal_count = 0;
double accel_cal_sum = 0.0;
const unsigned long ACCEL_CAL_DURATION_MS = 5000;
const unsigned long ACCEL_CAL_WARMUP_MS = 500;
const float MAX_ACCEL_CAL_DELTA_DEG = 8.0;

void setRobotState(RobotState new_state) {
	if (robot_state != new_state) {
		robot_state = new_state;
		error_integral = 0.0;
		v_error_integral = 0.0;
	}
}

void clearVelocityOuterLoop() {
    target_drive_velocity = 0.0;
    dynamic_tilt = 0.0;
    v_error_integral = 0.0;
}

void enterStationaryState() {
    clearVelocityOuterLoop();
    steering_offset = 0.0;
    is_turning = false;
    setRobotState(STATE_STATIONARY);
}

void startAccelCalibration() {
	accel_cal_active = true;
	accel_cal_start_ms = millis();
	accel_cal_count = 0;
	accel_cal_sum = 0.0;
}

void updateAccelCalibration(float accel_tilt) {
	if (!accel_cal_active) return;

	if (!robot_active || robot_state != STATE_STATIONARY) {
		accel_cal_active = false;
		Serial.println("Accel calibration cancelled");
		return;
	}

	unsigned long elapsed = millis() - accel_cal_start_ms;

	if (elapsed >= ACCEL_CAL_WARMUP_MS) {
		accel_cal_sum += accel_tilt;
		accel_cal_count++;
	}

	if (elapsed >= ACCEL_CAL_DURATION_MS) {
		if (accel_cal_count > 0) {
			float avg_accel_tilt = accel_cal_sum / accel_cal_count;
			float delta = setpoint - avg_accel_tilt;

			if (fabsf(delta) <= MAX_ACCEL_CAL_DELTA_DEG) {
				accel_pitch_correction_deg += delta;
				applyAccelPitchCorrection(accel_pitch_correction_deg);
				saveAccelCalibration();
				tiltx += delta;
				Serial.printf("Accel calibration saved | avg: %.4f | delta: %.4f | correction: %.4f\n", avg_accel_tilt, delta, accel_pitch_correction_deg);
			} else {
				Serial.printf("Accel calibration rejected | avg: %.4f | delta: %.4f\n", avg_accel_tilt, delta);
			}
		}

		accel_cal_active = false;
	}
}

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

String getWebPage() { //Pulls the webpage from webpage.h, and replaces placeholders
	String html = FPSTR(WEBPAGE_HTML);

	html.replace("%KP%", String(Kp));
	html.replace("%KI%", String(Ki));
	html.replace("%KD%", String(Kd));
	html.replace("%SP%", String(setpoint));
	html.replace("%KV%", String(Kv));
	html.replace("%TKP%", String(K_YAW));
	html.replace("%TKD%", String(K_DAMP));
	html.replace("%VKP%", String(vKp));
	html.replace("%VKI%", String(vKi));
	html.replace("%MKP%", String(moveKp));
	html.replace("%MKI%", String(moveKi));
	html.replace("%MKD%", String(moveKd));
	html.replace("%MKV%", String(moveKv));

	return html;
}

bool TimerHandler(void * timerNo) {
	step1.runStepper();
	step2.runStepper();
	return true;
}

//Used for closed loop turning
void startSpotTurn(float deltaDeg) {
	clearVelocityOuterLoop();
	error_integral = 0.0;
	steering_offset = 0.0;
	target_heading = current_heading + deltaDeg;
	is_turning = true;
	setRobotState(STATE_TURNING);
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
	accel_cal_active = false;
	steering_offset = 0.0;
	target_drive_velocity = 0.0;
	is_turning = false;
	yawRate = 0.0;
	enterStationaryState();

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
	loadAccelCalibration();

	//Connect to WiFi
	Serial.print("Connecting to WiFi");
	WiFi.disconnect(true);
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
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
		json += ",\"state\":" + String((int)robot_state);
		json += ",\"accelCalibrating\":" + String(accel_cal_active ? "true" : "false");
		json += ",\"accelCorrectionDeg\":" + String(accel_pitch_correction_deg, 4);
		json += "}";
		server.send(200, "application/json", json);
	});

	server.on("/update", []() {
		if (server.hasArg("p")) Kp = server.arg("p").toFloat();
		if (server.hasArg("i")) Ki = server.arg("i").toFloat();
		if (server.hasArg("d")) Kd = server.arg("d").toFloat();
		if (server.hasArg("t")) setpoint = server.arg("t").toFloat();
		if (server.hasArg("v")) Kv = server.arg("v").toFloat();
		if (server.hasArg("mp")) moveKp = server.arg("mp").toFloat();
		if (server.hasArg("mi")) moveKi = server.arg("mi").toFloat();
		if (server.hasArg("md")) moveKd = server.arg("md").toFloat();
		if (server.hasArg("mv")) moveKv = server.arg("mv").toFloat();
		if (server.hasArg("vp")) vKp = server.arg("vp").toFloat();
		if (server.hasArg("vi")) vKi = server.arg("vi").toFloat();
		if (server.hasArg("tkp")) K_YAW = server.arg("tkp").toFloat();
		if (server.hasArg("tkd")) K_DAMP = server.arg("tkd").toFloat();

		error_integral = 0.0;
		v_error_integral = 0.0;
		Serial.printf("Web Update | Static Kp: %.2f | Static Ki: %.2f | Static Kd: %.2f | Static Kv: %.2f | Move Kp: %.2f | Move Ki: %.2f | Move Kd: %.2f | Move Kv: %.2f | Setpoint: %.2f | TKP: %.2f | TKD: %.2f | VKP: %.2f | VKI: %.2f\n", Kp, Ki, Kd, Kv, moveKp, moveKi, moveKd, moveKv, setpoint, K_YAW, K_DAMP, vKp, vKi);

		server.send(200, "text/html", getWebPage());
	});

	server.on("/control", []() {
		if (server.hasArg("dir")) {
			String dir = server.arg("dir");
			Serial.println(dir);
			if (dir == "F") { target_drive_velocity = MAX_DRIVE_VELOCITY; steering_offset = 0.0; is_turning = false; setRobotState(STATE_DRIVING); }
			else if (dir == "B") { target_drive_velocity = -MAX_DRIVE_VELOCITY; steering_offset = 0.0; is_turning = false; setRobotState(STATE_DRIVING); }
			else if (dir == "L") { clearVelocityOuterLoop(); steering_offset = TURN_SPEED; is_turning = false; setRobotState(STATE_TURNING); }
			else if (dir == "R") { clearVelocityOuterLoop(); steering_offset = -TURN_SPEED; is_turning = false; setRobotState(STATE_TURNING); }
			else if (dir == "T1" && !is_turning) { startSpotTurn(90.0); }//target_heading = current_heading - 90.0; is_turning = true; }//Turn 90 degrees CW
			else if (dir == "T2" && !is_turning) { startSpotTurn(-90.0); }//target_heading = current_heading + 90.0; is_turning = true; } //Turn 90 degrees ACW
			else {
                if (robot_state == STATE_DRIVING) {
                    setRobotState(STATE_BRAKING);
                    target_drive_velocity = 0.0;
                    v_error_integral = 0.0;
                } else if (!is_turning) { enterStationaryState(); }
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

	server.on("/calibrate_accel", []() {
		if (!robot_active || robot_state != STATE_STATIONARY) {
			server.send(409, "text/plain", "Robot must be self-balancing and stationary");
			return;
		}
		startAccelCalibration();
		server.send(200, "text/plain", "Accel calibration started");
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

	//Outer loop dynamic control
	if (millis() > outerLoopTimer) {
		outerLoopTimer += OUTER_LOOP_INTERVAL;

		if (robot_state == STATE_TURNING) {
			clearVelocityOuterLoop();
		} else {
			float leftSpeed = step1.getSpeedRad();
			float rightSpeed = step2.getSpeedRad();
			float drive_velocity = (leftSpeed - rightSpeed) * 0.5;

            bool outerLoopActive = false;
            if (robot_state == STATE_DRIVING) {
                outerLoopActive = true;
            } else if (robot_state == STATE_BRAKING) {
                target_drive_velocity = 0.0;
                if (abs(drive_velocity) < brake_threshold) {
                    enterStationaryState();
                } else {
                    outerLoopActive = true;
                }
            }

            if (outerLoopActive) {

                float velocity_error = target_drive_velocity - drive_velocity;
                v_error_integral += velocity_error * oDT;
                v_error_integral = constrain(v_error_integral, -MAX_INTEGRAL, MAX_INTEGRAL);
                dynamic_tilt = vKp * velocity_error + vKi * v_error_integral;
                dynamic_tilt = constrain(dynamic_tilt, -MAX_TILT, MAX_TILT);
            } else {
                clearVelocityOuterLoop();
            }    
		}
	}

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

		updateAccelCalibration(accel_tilt);

		tiltx = (1.0 - C) * accel_tilt + C * (tiltx + gyro_tilt * DT); //Complementary filter

		//Track yaw for the 90 degree turn (x is the yaw axis)
		yawRate = gyroRobot.x * (180.0 / PI);
		if (abs(yawRate) < 0.5) { //ignore microjitters in turning
			yawRate = 0.0;
		}
		current_heading += yawRate * DT;
		if (is_turning) {
			float heading_error = target_heading - current_heading;
			while (heading_error > 180.0) heading_error -= 360.0;
			while (heading_error < -180.0) heading_error += 360.0;
			steering_offset = -(K_YAW * heading_error) - (K_DAMP * yawRate); //PD controller //Changed second term to negative from positive
			steering_offset = constrain(steering_offset, -TURN_SPEED, TURN_SPEED);

			if (abs(heading_error) < 2.0 && abs(yawRate) < 5.0) { //Stop turning if within 2 degree of target and rotation is slow
				is_turning = false;
				steering_offset = 0.0;
				current_heading = target_heading;
				setRobotState(STATE_STATIONARY);
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
			float error_derivative = -gyro_tilt;

			float activeKp = Kp;
			float activeKi = Ki;
			float activeKd = Kd;
			float activeKv = Kv;

			if (robot_state != STATE_STATIONARY) {
				activeKp = moveKp;
				activeKi = moveKi;
				activeKd = moveKd;
				activeKv = moveKv;
			}

			// PID gives acceleration, Kv damps the integrated velocity
			target_accel = activeKp * error + activeKi * error_integral + activeKd * error_derivative - activeKv * integrated_velocity;

			target_accel = constrain(target_accel, -MAX_ACCEL, MAX_ACCEL);

			integrated_velocity += target_accel * DT;
			integrated_velocity = constrain(integrated_velocity, -MAX_SPEED, MAX_SPEED);

			step1.setTargetSpeedRad(-integrated_velocity - steering_offset);
			step2.setTargetSpeedRad(integrated_velocity - steering_offset);

			error_old = error;
		}
	}

	server.handleClient();

	//Debug print
	if (millis() > printTimer) {
		printTimer += PRINT_INTERVAL;
		//Print something to serial
	}
}
