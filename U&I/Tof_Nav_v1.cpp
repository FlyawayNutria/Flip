#include <Arduino.h>           // not required in Arduino IDE
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <TimerInterrupt_Generic.h>
#include <step.h>


// Stepper motor control pins
const int STEPPER1_STEP_PIN = 17;   // Motor 1 step pulse
const int STEPPER1_DIR_PIN  = 16;   // Motor 1 direction
const int STEPPER2_STEP_PIN = 14;   // Motor 2 step pulse
const int STEPPER2_DIR_PIN  = 4;    // Motor 2 direction
const int STEPPER_EN_PIN    = 15;   // Enable

const int TOGGLE_PIN        = 32;

// VL53L0X Time-of-Flight sensor pins.
const int TOF_FRONT_XSHUT   = 27;
const int TOF_LEFT_XSHUT    = 26;
const int TOF_RIGHT_XSHUT   = 33;

const int I2C_SDA_PIN       = 21;
const int I2C_SCL_PIN       = 22;


//motor directions
const float M1_FWD =  1.0;   //left wheel
const float M2_FWD = -1.0;   //right wheel

const float DRIVE_SPEED     = 8.0;   // Normal cruise speed (forward/backward)
const float SLOW_SPEED      = 4.0;   // Reduced speed for gentle steering corrections
const float TURN_SPEED      = 5.0;   // Speed of each wheel during a point turn

const float MOTOR_ACCEL     = 10.0;
const int   TURN_DURATION_MS = 400;
const int   BACKUP_DURATION_MS = 300;


//< reverse immediately
const uint16_t OBSTACLE_CLOSE_MM     = 100;

//> clear path
const uint16_t OBSTACLE_FAR_MM       = 250;

//< steer
const uint16_t WALL_FOLLOW_MM        = 150;

//> path
const uint16_t PATH_WIDTH_MM         = 200;

//> junction
const uint16_t JUNCTION_TRIGGER_MM   = 300;

//> open space
const uint16_t MAZE_EXIT_MM          = 500;

const int JUNCTION_COOLDOWN_MS       = 1000;
const int JUNCTION_SETTLE_MS         = 300;

const int MAX_PATH_MEMORY            = 100;


const int STEPPER_INTERVAL_US = 20;
const int NAV_LOOP_INTERVAL_MS = 50;
const int PRINT_INTERVAL_MS    = 500;


ESP32Timer ITimer(3);

step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

Adafruit_VL53L0X tofFront = Adafruit_VL53L0X();
Adafruit_VL53L0X tofLeft  = Adafruit_VL53L0X();
Adafruit_VL53L0X tofRight = Adafruit_VL53L0X();


char pathMemory[MAX_PATH_MEMORY];   
int  pathIndex     = 0;             
bool solvingMaze   = true;         
bool returnJourney = false;         

bool          atJunction   = false; 
unsigned long junctionTime = 0;     


bool IRAM_ATTR TimerHandler(void* timerNo)
{
  static bool toggle = false;

  step1.runStepper();    // Advance motor 1 pulse state machine by one ISR tick
  step2.runStepper();    // Advance motor 2 pulse state machine by one ISR tick

  // Toggle the diagnostic pin so ISR timing can be verified on an oscilloscope
  digitalWrite(TOGGLE_PIN, toggle);
  toggle = !toggle;

  return true;
}


void moveForward()
{
  step1.setTargetSpeedRad( M1_FWD * DRIVE_SPEED);
  step2.setTargetSpeedRad( M2_FWD * DRIVE_SPEED);
}


void moveBackward(int duration = BACKUP_DURATION_MS)
{
  step1.setTargetSpeedRad(-M1_FWD * DRIVE_SPEED);   // Negate FWD to go backward
  step2.setTargetSpeedRad(-M2_FWD * DRIVE_SPEED);
  delay(duration);    // Motors continue stepping via ISR during this delay
  stopMotors();
}


void turnLeft(int duration = TURN_DURATION_MS)
{
  step1.setTargetSpeedRad(-M1_FWD * TURN_SPEED);   // Left wheel: backward
  step2.setTargetSpeedRad( M2_FWD * TURN_SPEED);   // Right wheel: forward
  delay(duration);
  stopMotors();
}


void turnRight(int duration = TURN_DURATION_MS)
{
  step1.setTargetSpeedRad( M1_FWD * TURN_SPEED);   // Left wheel: forward
  step2.setTargetSpeedRad(-M2_FWD * TURN_SPEED);   // Right wheel: backward
  delay(duration);
  stopMotors();
}


void stopMotors()
{
  step1.setTargetSpeedRad(0.0);
  step2.setTargetSpeedRad(0.0);
}


void smoothLeft()
{
  step1.setTargetSpeedRad( M1_FWD * SLOW_SPEED);    // Left wheel slowed
  step2.setTargetSpeedRad( M2_FWD * DRIVE_SPEED);   // Right wheel full
}


void smoothRight()
{
  step1.setTargetSpeedRad( M1_FWD * DRIVE_SPEED);   // Left wheel full
  step2.setTargetSpeedRad( M2_FWD * SLOW_SPEED);    // Right wheel slowed
}


void initializeTofSensor(Adafruit_VL53L0X &sensor, uint8_t xshutPin, uint8_t address)
{
  digitalWrite(xshutPin, HIGH);  // Release this sensor from reset
  delay(20);                     // Allow sensor boot time (datasheet: 1.2 ms min)

  if (!sensor.begin(address)) {
    Serial.print("ERROR: ToF sensor on XSHUT pin ");
    Serial.print(xshutPin);
    Serial.println(" failed. Check wiring. Halting.");
    while (1);   // Stop everything — a missing sensor makes navigation unsafe
  }

  Serial.print("ToF ready: XSHUT pin ");
  Serial.print(xshutPin);
  Serial.print(" → I2C address 0x");
  Serial.println(address, HEX);
}


uint16_t readTofDistance(Adafruit_VL53L0X &sensor)
{
  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);   // false = don't print debug to Serial

  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;      // Valid reading
  } else {
    return 9999;                         // Invalid — treat as "nothing detected"
  }
}


bool checkJunction(uint16_t leftDist, uint16_t rightDist)
{
  return (leftDist > JUNCTION_TRIGGER_MM || rightDist > JUNCTION_TRIGGER_MM);
}


void recordTurn(char direction)
{
  if (pathIndex < MAX_PATH_MEMORY - 1) {
    pathMemory[pathIndex++] = direction;
    Serial.print("Path recorded: ");
    Serial.print(direction);
    Serial.print("  (step ");
    Serial.print(pathIndex);
    Serial.println(")");
  } else {
    Serial.println("WARNING: Path memory full — cannot record further turns.");
  }
}


void simplifyPath()
{
  Serial.println("(Path simplification not yet implemented)");
}


void executeReturnJourney()
{
  Serial.println("--- Return journey starting ---");

  //180° spin (two 90° turns)
  turnLeft(TURN_DURATION_MS * 2);
  delay(500);

  for (int i = pathIndex - 1; i >= 0; i--) {
    Serial.print("Return step ");
    Serial.print(i);
    Serial.print(" (original: ");
    Serial.print(pathMemory[i]);
    Serial.println(")");

    unsigned long segmentStart = millis();
    while (millis() - segmentStart < 2000) {
      uint16_t frontDist = readTofDistance(tofFront);
      uint16_t leftDist  = readTofDistance(tofLeft);
      uint16_t rightDist = readTofDistance(tofRight);

      if (frontDist < OBSTACLE_CLOSE_MM) {
        moveBackward();   
        break;
      }


      if      (leftDist  < WALL_FOLLOW_MM) smoothRight();
      else if (rightDist < WALL_FOLLOW_MM) smoothLeft();
      else                                 moveForward();

      delay(NAV_LOOP_INTERVAL_MS);
    }

    stopMotors();
    delay(500);

    if      (pathMemory[i] == 'L') turnRight();
    else if (pathMemory[i] == 'R') turnLeft();
  }

  Serial.println("--- Return journey complete. Robot stopped. ---");
  while (1) {
    stopMotors();   
    delay(1000);
  }
}

void solveMaze(uint16_t frontDist, uint16_t leftDist, uint16_t rightDist)
{
  //print sesnor reading for debug
  Serial.print("F:");  Serial.print(frontDist);
  Serial.print("  L:"); Serial.print(leftDist);
  Serial.print("  R:"); Serial.print(rightDist);
  Serial.println(" mm");

  bool junctionDetected = checkJunction(leftDist, rightDist);

  if (junctionDetected && !atJunction) {

    atJunction   = true;
    junctionTime = millis();

    stopMotors();
    delay(JUNCTION_SETTLE_MS);  

    Serial.println("Junction detected — deciding direction:");

    //rght-hand rule priority: right → straight → left → 180°
    if (rightDist > PATH_WIDTH_MM) {
      Serial.println("  → Turning RIGHT");
      recordTurn('R');
      turnRight();

    } else if (frontDist > OBSTACLE_FAR_MM) {
      Serial.println("  → Going STRAIGHT");
      recordTurn('S');
      moveForward();

    } else if (leftDist > PATH_WIDTH_MM) {
      Serial.println("  → Turning LEFT");
      recordTurn('L');
      turnLeft();

    } else {
      Serial.println("  → Dead end — turning 180°");
      recordTurn('B');
      turnLeft(TURN_DURATION_MS * 2);   // Two 90° turns = 180°
    }

    delay(JUNCTION_SETTLE_MS);   // Short pause after turn before resuming navigation

  } else if (atJunction && millis() - junctionTime > JUNCTION_COOLDOWN_MS) {

    atJunction = false;

  } else if (frontDist < OBSTACLE_CLOSE_MM) {

    Serial.println("Obstacle! Reversing.");
    moveBackward();   // Uses BACKUP_DURATION_MS from configuration
    delay(100);       // Brief pause before next navigation decision

  } else {
    if      (leftDist  < WALL_FOLLOW_MM) smoothRight();  // Drifting left — correct right
    else if (rightDist < WALL_FOLLOW_MM) smoothLeft();   // Drifting right — correct left
    else                                 moveForward();   // Centred — cruise forward
  }

  if (leftDist > MAZE_EXIT_MM && rightDist > MAZE_EXIT_MM && frontDist > MAZE_EXIT_MM) {
    Serial.println("Maze exit detected! Stopping to prepare return journey...");
    stopMotors();
    delay(2000);
    simplifyPath();
    solvingMaze   = false;
    returnJourney = true;
  }
}



void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Robot initialising ===");

  // Diagnostic toggle pin for oscilloscope
  pinMode(TOGGLE_PIN, OUTPUT);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  pinMode(TOF_FRONT_XSHUT, OUTPUT);
  pinMode(TOF_LEFT_XSHUT,  OUTPUT);
  pinMode(TOF_RIGHT_XSHUT, OUTPUT);

  digitalWrite(TOF_FRONT_XSHUT, LOW);   
  digitalWrite(TOF_LEFT_XSHUT,  LOW);
  digitalWrite(TOF_RIGHT_XSHUT, LOW);
  delay(10);

  initializeTofSensor(tofFront, TOF_FRONT_XSHUT, 0x30);
  initializeTofSensor(tofLeft,  TOF_LEFT_XSHUT,  0x31);
  initializeTofSensor(tofRight, TOF_RIGHT_XSHUT, 0x32);

  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("ERROR: Could not start stepper timer. Halting.");
    while (1) delay(10);
  }
  Serial.println("Stepper interrupt running.");

  step1.setAccelerationRad(MOTOR_ACCEL);
  step2.setAccelerationRad(MOTOR_ACCEL);

  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, LOW);

  for (int i = 0; i < MAX_PATH_MEMORY; i++) pathMemory[i] = ' ';

  Serial.println("=== Initialisation complete — maze run starting ===");
}



void loop()
{
  static unsigned long navTimer   = 0;   
  static unsigned long printTimer = 0;   

  //navigation
  if (millis() > navTimer) {
    navTimer += NAV_LOOP_INTERVAL_MS;

    uint16_t frontDist = readTofDistance(tofFront);
    uint16_t leftDist  = readTofDistance(tofLeft);
    uint16_t rightDist = readTofDistance(tofRight);

    if (solvingMaze) {
      solveMaze(frontDist, leftDist, rightDist);
    } else if (returnJourney) {
      executeReturnJourney();
    }
  }

  //motor status print
  if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL_MS;
    Serial.print("Motor speeds —  M1: ");
    Serial.print(step1.getSpeedRad(), 2);
    Serial.print(" rad/s   M2: ");
    Serial.print(step2.getSpeedRad(), 2);
    Serial.println(" rad/s");
  }
}
