// =============================================================================
// robot_movement.ino
// Robot movement control - Arduino IDE version
//
// Provides forward, backward, and turn functions over Serial.
// Send commands via Serial Monitor at 115200 baud:
//   'w' = forward
//   's' = backward
//   'a' = turn left
//   'd' = turn right
//   'x' = stop
//
// Setup for Arduino IDE:
//   1. Install ESP32 board support:
//      File → Preferences → Additional Board Manager URLs, add:
//      https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
//      Then: Tools → Board → Boards Manager → search "esp32" → install
//   2. Select board: Tools → Board → ESP32 Arduino → ESP32 Dev Module
//   3. Install libraries via Sketch → Include Library → Manage Libraries:
//      - Search "ESP32TimerInterrupt" by Khoi Hoang → install
//   4. Copy step.h into the same folder as this .ino file
//      (the Arduino IDE automatically includes files in the same sketch folder)
//   5. In Serial Monitor, set line ending to "No line ending" or "Newline",
//      baud rate to 115200
//
// NOTE: Unlike the PlatformIO version, there is NO #include <Arduino.h> here.
//       The Arduino IDE adds it automatically. Adding it manually causes errors.
// =============================================================================

// #include <Arduino.h>  ← DO NOT include this in the Arduino IDE
#include <TimerInterrupt_Generic.h>
#include <step.h>

// -----------------------------------------------------------------------------
// Pin definitions
// -----------------------------------------------------------------------------
const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15;

// Diagnostic pin - toggles every ISR call, useful for oscilloscope timing checks
const int TOGGLE_PIN        = 32;

// -----------------------------------------------------------------------------
// Timing constants
// -----------------------------------------------------------------------------
const int STEPPER_INTERVAL_US = 20;   // ISR fires every 20 µs to update steppers
const int LOOP_INTERVAL       = 10;   // Control loop runs every 10 ms
const int PRINT_INTERVAL      = 500;  // Serial status printed every 500 ms

// -----------------------------------------------------------------------------
// Movement speed constants (radians/second)
// Adjust these to suit your robot's physical dimensions and desired behaviour
// -----------------------------------------------------------------------------
const float DRIVE_SPEED = 10.0;  // Speed used when driving forward or backward
const float TURN_SPEED  = 5.0;   // Speed used for each wheel when turning
                                  // Turning works by driving wheels in opposite
                                  // directions, so effective turn rate is 2x this

// Motor acceleration (rad/s²)
// Higher = snappier response but more mechanical stress
const float MOTOR_ACCEL = 10.0;

// -----------------------------------------------------------------------------
// Global objects
// -----------------------------------------------------------------------------

// Hardware timer 3 on the ESP32 drives the stepper ISR.
// Timers 0 and 1 are used by the WiFi/BT stack; timer 2 by the Arduino tone()
// function. Timer 3 is safest to use for custom ISRs on this platform.
ESP32Timer ITimer(3);

// Motor objects. The step library handles step pulse generation and acceleration
// internally; we just set a target speed and call runStepper() in the ISR.
step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

// -----------------------------------------------------------------------------
// Interrupt Service Routine
// Called every STEPPER_INTERVAL_US microseconds by the hardware timer.
//
// IMPORTANT: Do not add floating point calculations, Serial prints, or any
// slow operations here. The ESP32 FPU is not available in ISR context.
//
// IRAM_ATTR places this function in internal RAM rather than flash, which is
// required for reliable ISR execution on the ESP32. Both IDEs support this.
// -----------------------------------------------------------------------------
bool IRAM_ATTR TimerHandler(void* timerNo)
{
  static bool toggle = false;

  step1.runStepper();
  step2.runStepper();

  // Toggle diagnostic pin so you can verify ISR timing on an oscilloscope.
  // At STEPPER_INTERVAL_US = 20 µs you should see a 25 kHz square wave.
  digitalWrite(TOGGLE_PIN, toggle);
  toggle = !toggle;

  return true;
}

// -----------------------------------------------------------------------------
// Movement functions
//
// Motor 2 is always negated relative to Motor 1 because the two motors face
// opposite directions on the chassis. Without this, one motor would fight the
// other when driving straight.
// -----------------------------------------------------------------------------

// Both wheels forward at DRIVE_SPEED
void moveForward()
{
  step1.setTargetSpeedRad(DRIVE_SPEED);
  step2.setTargetSpeedRad(-DRIVE_SPEED);   // Negated: facing opposite direction
}

// Both wheels backward at DRIVE_SPEED
void moveBackward()
{
  step1.setTargetSpeedRad(-DRIVE_SPEED);
  step2.setTargetSpeedRad(DRIVE_SPEED);    // Negated: facing opposite direction
}

// Left wheel backward, right wheel forward → robot turns left on the spot
void turnLeft()
{
  step1.setTargetSpeedRad(-TURN_SPEED);
  step2.setTargetSpeedRad(-TURN_SPEED);   // Both same sign here because
                                           // the negation cancels for a turn
}

// Left wheel forward, right wheel backward → robot turns right on the spot
void turnRight()
{
  step1.setTargetSpeedRad(TURN_SPEED);
  step2.setTargetSpeedRad(TURN_SPEED);
}

// Bring both motors to rest
void stopMotors()
{
  step1.setTargetSpeedRad(0.0);
  step2.setTargetSpeedRad(0.0);
}

// -----------------------------------------------------------------------------
// setup()
// -----------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  Serial.println("Robot movement controller starting...");

  pinMode(TOGGLE_PIN, OUTPUT);

  // Attach the stepper ISR to the hardware timer
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("ERROR: Failed to start stepper interrupt. Halting.");
    while (1) delay(10);
  }
  Serial.println("Stepper interrupt started.");

  // Set motor acceleration
  step1.setAccelerationRad(MOTOR_ACCEL);
  step2.setAccelerationRad(MOTOR_ACCEL);

  // Enable motor drivers (EN pin is active LOW on this PCB)
  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, LOW);

  Serial.println("Ready. Commands: w=forward, s=backward, a=left, d=right, x=stop");
}

// -----------------------------------------------------------------------------
// loop()
// -----------------------------------------------------------------------------
void loop()
{
  static unsigned long printTimer = 0;
  static unsigned long loopTimer  = 0;
  static char currentCommand      = 'x';   // Tracks the last received command

  // --- Control loop: runs every LOOP_INTERVAL ms ---
  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;

    // Read a command from Serial if one is available
    if (Serial.available() > 0) {
      char cmd = Serial.read();

      // Ignore whitespace and newline characters from the Serial Monitor
      if (cmd != '\n' && cmd != '\r' && cmd != ' ') {
        currentCommand = cmd;
      }
    }

    // Execute the current command every loop iteration.
    // Calling setTargetSpeedRad() repeatedly is safe; the step library only
    // updates if the value has changed.
    switch (currentCommand) {
      case 'w': moveForward();  break;
      case 's': moveBackward(); break;
      case 'a': turnLeft();     break;
      case 'd': turnRight();    break;
      case 'x': stopMotors();   break;
      default:  stopMotors();   break;   // Unknown command: fail safe to stop
    }
  }

  // --- Status print: runs every PRINT_INTERVAL ms ---
  if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;

    Serial.print("CMD:");
    Serial.print(currentCommand);
    Serial.print("  M1 speed(rad/s):");
    Serial.print(step1.getSpeedRad(), 2);
    Serial.print("  M2 speed(rad/s):");
    Serial.println(step2.getSpeedRad(), 2);
  }
}
