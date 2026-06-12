#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include "secrets.h"
#include "calibrate.h" //Calibrate the MPU6050

struct Vec3 { float x; float y; float z; }; //3x1 vector

//Multiplies a 3x3 matrix by a 3x1 vector
Vec3 matMul(const float R[3][3], Vec3 v) { Vec3 out; out.x = R[0][0] * v.x + R[0][1] * v.y + R[0][2] * v.z; out.y = R[1][0] * v.x + R[1][1] * v.y + R[1][2] * v.z; out.z = R[2][0] * v.x + R[2][1] * v.y + R[2][2] * v.z; return out; }
float R_sensor_to_robot[3][3] = { { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 } };

// Manually measured gyro bias, in rad/s, subtracted from GYRO readings. 
float GYRO_BIAS_X = -0.029064;
float GYRO_BIAS_Y = -0.012810;
float GYRO_BIAS_Z = -0.022884;

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
const int TOGGLE_PIN        = 13; //oscilloscope pin i think

//Interval constants
const int PRINT_INTERVAL      = 50;
const int LOOP_INTERVAL       = 10;
const int STEPPER_INTERVAL_US = 20;
const int OUTER_LOOP_INTERVAL = 50;

//Filter constants
const float C = 0.99;
const float DT = (float)LOOP_INTERVAL / 1000.0;
const float oDT = (float)OUTER_LOOP_INTERVAL / 1000.0; //outerloop dt

//Variables for inner loop balancing
float Kp = 12.0;
const float Ki = 0.00;
float Kd = 2;
float setpoint = 87.0;
float tiltx = 0.0;
float target_accel = 0.0;
bool robot_active = false;
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
float K_DAMP = -0.01; //D gain for turning, for some reason minus is smoother
float yawRate = 0.0;

//Dynamic control variables
float dynamic_tilt = 0.0;
float vKp = 0.2;
float vKi = 0.0;
float MAX_TILT = 5.0; //5 degrees
float target_drive_velocity = 0.0;
float MAX_DRIVE_VELOCITY = 5.0;
bool driving = false;
float Kveer = 0.15;
float velo_integral = 0.0;

enum botMode {
	MANUAL,
	LINE_FOLLOW
};

botMode current = MANUAL;

//Line following constants
const int NUM_SENSORS = 3;
const int SENSOR_PINS[NUM_SENSORS] = {32, 33, 34};
int rawLeft = 0;
int rawCenter = 0;  
int rawRight  = 0;  
float LINE_F_SPEED = 0.4; //Line forward speed
float LINE_T_SPEED = 0.45; //Line turn speed
const int TAPE_THRESHOLD         = 2000;  // Stricter general tape detection edge
const int CENTER_LOCK_THRESHOLD  = 1500;  // Deep center requirement to stop rotating

enum LineState {
    LF_FORWARD,
    LF_ALIGN_LEFT,
    LF_ALIGN_RIGHT,
    LF_RECOVERY
};

LineState lfState = LF_FORWARD;

unsigned long lf_timer = 0;
const int LF_SETTLE_MS = 60;

void updateLineFollowing() {

    rawLeft   = analogRead(SENSOR_PINS[0]);
    rawCenter = analogRead(SENSOR_PINS[1]);
    rawRight  = analogRead(SENSOR_PINS[2]);

    switch (lfState) {

        //Forward
        case LF_FORWARD: {

            target_drive_velocity = LINE_F_SPEED;
            steering_offset = 0.0;
            driving = true;

            //if line centered go straight
            if (rawCenter < CENTER_LOCK_THRESHOLD) {
                break;
            }

            //if lost centre pick a direction to search
            if (rawLeft < TAPE_THRESHOLD && rawRight >= TAPE_THRESHOLD) {
                lfState = LF_ALIGN_LEFT;
                lf_timer = millis();
            }
            else if (rawRight < TAPE_THRESHOLD && rawLeft >= TAPE_THRESHOLD) {
                lfState = LF_ALIGN_RIGHT;
                lf_timer = millis();
            }
            else {
                lfState = LF_RECOVERY;
            }

            break;
        }

        //Search left
        case LF_ALIGN_LEFT: {

            target_drive_velocity = 0.3;        // slow crawl
            steering_offset = +LINE_T_SPEED;     // turn left
            driving = true;

            // wait a bit for robot to respond physically
            if (millis() - lf_timer < LF_SETTLE_MS) break;

            if (rawCenter < CENTER_LOCK_THRESHOLD) {
                lfState = LF_FORWARD;
            }

            break;
        }

        //Search right
        case LF_ALIGN_RIGHT: {

            target_drive_velocity = 0.3;
            steering_offset = -LINE_T_SPEED;
            driving = true;

            if (millis() - lf_timer < LF_SETTLE_MS) break;

            if (rawCenter < CENTER_LOCK_THRESHOLD) {
                lfState = LF_FORWARD;
            }

            break;
        }

        //Recovery mode
        case LF_RECOVERY: {

            target_drive_velocity = 0.0;
            steering_offset = 0.0;

            // choose direction to search
            if (rawLeft < rawRight) {
                lfState = LF_ALIGN_LEFT;
            } else {
                lfState = LF_ALIGN_RIGHT;
            }

            lf_timer = millis();
            break;
        }
    }

    is_turning = false; //Doesn't use closed loop turning
}

//Power and Battery consumption
const float VREF = 4.096;
float channelVoltages[3] = {0.0, 0.0, 0.0};
static uint8_t currentChannel = 0;
uint16_t readADC(uint8_t channel) {
  channel = channel & 0x07; 
  uint8_t txByte0 = 0x06 | (channel >> 2);  
  uint8_t txByte1 = (channel & 0x03) << 6;  
  digitalWrite(ADC_CS_PIN, LOW); 
  SPI.transfer(txByte0);                    
  uint8_t rx0 = SPI.transfer(txByte1);      
  uint8_t rx1 = SPI.transfer(0x00);         
  digitalWrite(ADC_CS_PIN, HIGH); 
  uint16_t result = ((rx0 & 0x0F) << 8) | rx1; 
  return result;
}

ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;

step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

float getPos() { return (step1.getPositionRad() - step2.getPositionRad()) / 2.0; } //Unused in final codebase

void clearDrive() {
    target_drive_velocity = 0.0;
    driving = false;
	//idk about this one but
	integrated_velocity = 0.0;
}

bool TimerHandler(void * timerNo) { step1.runStepper(); step2.runStepper(); return true; }

//Used for closed loop turning
void startSpotTurn(float deltaDeg) {
	clearDrive();
	integrated_velocity = 0.0; //Just added this
	error_integral = 0.0;
	steering_offset = 0.0;
	velo_integral = 0.0;
	target_heading = current_heading + deltaDeg;
	is_turning = true;
}

void resetBot() {
	robot_active = false;
	digitalWrite(STEPPER_EN_PIN, HIGH);
	integrated_velocity = 0.0; //Inner loop
	error_integral = 0.0;
	target_accel = 0.0;
	steering_offset = 0.0; //Turning and movement
	target_drive_velocity = 0.0;
	velo_integral = 0.0;
	dynamic_tilt = 0.0;
	is_turning = false;
	yawRate = 0.0;
	step1.setTargetSpeedRad(0.0);
	step2.setTargetSpeedRad(0.0);
}

// Helper to extract values from incoming key-pair configurations over Serial
float parseValue(String data, String key) {
    int pos = data.indexOf(key + "=");
    if (pos == -1) return 0.0;
    int start = pos + key.length() + 1;
    int end = data.indexOf(",", start);
    if (end == -1) end = data.length();
    return data.substring(start, end).toFloat();
}

void sendFullSetup() {
    // Send all initial metrics back cleanly inside a structural payload
    Serial.print("{\"p\":"); Serial.print(Kp);
    Serial.print(",\"d\":"); Serial.print(Kd);
    Serial.print(",\"t\":"); Serial.print(setpoint);
    Serial.print(",\"tkp\":"); Serial.print(K_YAW, 3);
    Serial.print(",\"tkd\":"); Serial.print(K_DAMP, 3);
    Serial.print(",\"vp\":"); Serial.print(vKp);
    Serial.print(",\"vi\":"); Serial.print(vKi, 3);
    Serial.print(",\"kt\":"); Serial.print(Kveer);
    Serial.print(",\"tilt\":"); Serial.print(MAX_TILT);
    Serial.print(",\"lff\":"); Serial.print(LINE_F_SPEED);
    Serial.print(",\"lft\":"); Serial.print(LINE_T_SPEED);
    Serial.println(",\"init\":true}");
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

	//Initialise IR array for line following
	analogReadResolution(12);
	for (int i = 0; i < NUM_SENSORS; i++) {
		pinMode(SENSOR_PINS[i], INPUT);
		analogSetPinAttenuation(SENSOR_PINS[i], ADC_11db);
	}

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
    if (Serial.available() > 0) { //Read from serial
        String input = Serial.readStringUntil('\n');
        input.trim(); // Remove any extra whitespace

        if (input == "GET_SETUP") { //initial handshake with rpi to set PID values
            sendFullSetup();
        } else if (input.startsWith("MODE:")) {
            String mStr = input.substring(5);
            clearDrive();
            steering_offset = 0.0;
			integrated_velocity = 0.0;
			velo_integral = 0.0;
            if (mStr == "MANUAL") current = MANUAL;
            else if (mStr == "LINE") current = LINE_FOLLOW;
        } else if (input.startsWith("CMD:")) {
            String dir = input.substring(4);
            if (current == MANUAL) {
                if (dir == "F") { target_drive_velocity = MAX_DRIVE_VELOCITY; steering_offset = 0.0; is_turning = false; driving = true; }
                else if (dir == "B") { target_drive_velocity = -MAX_DRIVE_VELOCITY; steering_offset = 0.0; is_turning = false; driving = true; }
                else if (dir == "L") { clearDrive(); steering_offset = TURN_SPEED; is_turning = false; }
                else if (dir == "R") { clearDrive(); steering_offset = -TURN_SPEED; is_turning = false; }
                else if (dir == "T1" && !is_turning) { startSpotTurn(90.0); }
                else if (dir == "T2" && !is_turning) { startSpotTurn(-90.0); }
                else if (dir == "FL") { target_drive_velocity = MAX_DRIVE_VELOCITY; steering_offset = TURN_SPEED * Kveer; is_turning = false; driving = true; }
                else if (dir == "FR") { target_drive_velocity = MAX_DRIVE_VELOCITY; steering_offset = -TURN_SPEED * Kveer; is_turning = false; driving = true; }
                else if (dir == "CAL") { resetBot(); calibrateGyro(); }
                else if (dir == "STOP" || dir == "S") { clearDrive(); steering_offset = 0.0; }//resetBot(); }
                else {
                    target_drive_velocity = 0.0;
                    driving = false;
                    integrated_velocity = 0.0;
                    velo_integral = 0.0;
                    if (!is_turning) {
                        steering_offset = 0.0;
                    }
			    }
            }
        } else if (input.startsWith("CFG:")) { //Update PID values via webpage
            String cfg = input.substring(4);
            Kp = parseValue(cfg, "p");
            Kd = parseValue(cfg, "d");
            setpoint = parseValue(cfg, "t");
            K_YAW = parseValue(cfg, "tkp");
            K_DAMP = parseValue(cfg, "tkd");
            vKp = parseValue(cfg, "vp");
            vKi = parseValue(cfg, "vi");
            Kveer = parseValue(cfg, "kt");
            MAX_TILT = parseValue(cfg, "tilt");
            LINE_F_SPEED = parseValue(cfg, "lff");
            LINE_T_SPEED = parseValue(cfg, "lft");
            error_integral = 0.0;
        }
    }
	
    static unsigned long telemetryTimer = 0;
	static unsigned long loopTimer = 0;
	static unsigned long outerLoopTimer = 0;

	//Outer loop dynamic control - ver2, turning is good
	if (millis() > outerLoopTimer) {
		outerLoopTimer += OUTER_LOOP_INTERVAL;

		if (robot_active) {
			switch (current) { //Check what state we are in. 
				case MANUAL: //Takes commands from the user
					break;
				case LINE_FOLLOW: //The IR sensors give the commands
					updateLineFollowing();
					break;
			}
		}

		float leftSpeed = step1.getSpeedRad();
		float rightSpeed = step2.getSpeedRad();
		float drive_velocity = (leftSpeed - rightSpeed) * 0.5; //Try left-right

		if (is_turning) {
			dynamic_tilt = 0.0;
			velo_integral = 0.0;
		} else {
			float velocity_error = target_drive_velocity - drive_velocity;
			velo_integral += velocity_error * oDT;
			velo_integral = constrain(velo_integral, -MAX_INTEGRAL, MAX_INTEGRAL);
			dynamic_tilt = (vKp * velocity_error) + (vKi * velo_integral);
			dynamic_tilt = constrain(dynamic_tilt, -MAX_TILT, MAX_TILT);
		}
	}

	//Inner loop balancing
	if (millis() > loopTimer) {
		loopTimer += LOOP_INTERVAL;

        uint16_t rawADC = readADC(currentChannel);
        channelVoltages[currentChannel] = (rawADC * VREF) / 4095.0;
        currentChannel++;
        if (currentChannel >= 3) { currentChannel = 0; }

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

			if (abs(heading_error) < 2.0) { //Stop if turn within 2 deg
				is_turning = false;
				steering_offset = 0.0;
				integrated_velocity = 0.0; //Try to stop moving after the turn
				current_heading = target_heading;
			}
		}

		float error = (setpoint + dynamic_tilt) - tiltx;

		// Fall safety
		if (abs(error) > 30.0 && robot_active) { resetBot(); }

		// Auto-arm when close to setpoint
		if (!robot_active) {
			if (abs(error) < 0.5) {
                resetBot();
                robot_active = true; //Overwrite these after the reset
                digitalWrite(STEPPER_EN_PIN, LOW);
                Serial.println("Bot armed");
			} else {
                step1.setTargetSpeedRad(0.0);
                step2.setTargetSpeedRad(0.0);
			}
		}

		//Inner loop balance controller
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
		}
	}


	if (millis() - telemetryTimer >= 200) { //Sends updates to PI 5 times per second
        telemetryTimer = millis();
		Serial.print("{\"tilt\":"); Serial.print(tiltx, 2);
        Serial.print(",\"set\":"); Serial.print(setpoint, 2);
        Serial.print(",\"active\":"); Serial.print(robot_active ? "true" : "false");
        Serial.print(",\"turning\":"); Serial.print(is_turning ? "true" : "false");
        Serial.print(",\"mode\":"); Serial.print((int)current);
        Serial.print(",\"v0\":"); Serial.print(channelVoltages[0], 4);
        Serial.print(",\"v1\":"); Serial.print(channelVoltages[1], 4);
        Serial.print(",\"v2\":"); Serial.print(channelVoltages[2], 4);
        Serial.println("}");
	}
}
