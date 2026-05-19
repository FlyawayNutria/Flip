#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

// The Stepper pins
const int STEPPER1_DIR_PIN  = 16; //D1 = 16
const int STEPPER1_STEP_PIN = 17; //S1 = 17
const int STEPPER2_DIR_PIN  = 4; // D2 = 4
const int STEPPER2_STEP_PIN = 14; // S2 = 14
const int STEPPER_EN_PIN    = 15; //EN = 15

//ADC pins
const int ADC_CS_PIN        = 5; // 
const int ADC_SCK_PIN       = 18; // 
const int ADC_MISO_PIN      = 19; // 
const int ADC_MOSI_PIN      = 23; //

// Diagnostic pin for oscilloscope
const int TOGGLE_PIN        = 32;

const int PRINT_INTERVAL    = 500;
const int LOOP_INTERVAL     = 10;
const int STEPPER_INTERVAL_US = 20;

const float kx = 20.0;
const float VREF = 4.096;

//Global objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu; 

step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

//Interrupt Service Routine for motor update
bool TimerHandler(void * timerNo)
{
  static bool toggle = false;

  step1.runStepper();
  step2.runStepper();

  digitalWrite(TOGGLE_PIN, toggle);  
  toggle = !toggle;
  return true;
}

uint16_t readADC(uint8_t channel) {
  // Lowercase to avoid conflicting with ESP32 internal macros
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

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
  }
  Serial.println("Initialised Interrupt for Stepper");

  step1.setAccelerationRad(10.0);
  step2.setAccelerationRad(10.0);

  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, false);

  // Set up ADC and SPI (Fixed pin configurations)
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN);
}

void loop()
{
  static unsigned long printTimer = 0;  
  static unsigned long loopTimer = 0;   
  static float tiltx = 0.0;             
  
  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    tiltx = atan2(a.acceleration.x, a.acceleration.z)*180.0/PI;
    

    step1.setTargetSpeedRad(tiltx * kx);
    step2.setTargetSpeedRad(-tiltx * kx);
  }
  
  if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;
    
    // Extracted out of Serial.print to protect SPI timing stability
    uint16_t rawAdc = readADC(0);
    float voltage = (rawAdc * VREF) / 4095.0;

    Serial.print(tiltx * 1000);
    Serial.print(' ');
    Serial.print(step1.getSpeedRad());
    Serial.print(' ');
    Serial.print(voltage);
    Serial.println();
  }
}

/*
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const int PRINT_INTERVAL = 30; // ~33Hz for smooth plotting
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
    //float tiltx = a.acceleration.z / 9.67;
    float tiltx = atan2(a.acceleration.x, a.acceleration.z)*180.0/PI;
    
    // Formatted cleanly for the Arduino Serial Plotter
    Serial.print("Tilt_Value:");
    Serial.println(tiltx, 4); // Prints with 4 decimal places for precision
  }
}
  */
