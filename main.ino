#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_VL53L1X.h"
#include <Adafruit_BME280.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include <PID_v1.h>

#define SERVOMIN  210 // This is the 'minimum' pulse length count (out of 4096) original 100
#define SERVOMAX   300 // This is the 'maximum' pulse length count (out of 4096)  original 650
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define IRQ_PIN 2
#define XSHUT_PIN 3

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Servo ESC;     // create servo object to control the ESC
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BME280 bme;

int pulse_width;  // value from the analog pin
bool setupDone = false;

// our servo # counter and offsets
uint8_t servonum = 0;
int offsets[4]; // Declare an array for servo values

// Yaw control variables
double SetpointYaw, InputYaw, OutputYaw;
double KpYaw=4, KiYaw=0.2, KdYaw=0.5;
PID PID_Yaw(&InputYaw, &OutputYaw, &SetpointYaw, KpYaw, KiYaw, KdYaw, DIRECT);

// Pitch control variables
double SetpointPitch, InputPitch, OutputPitch;
double KpPitch=4, KiPitch=0.2, KdPitch=0.5;
PID PID_Pitch(&InputPitch, &OutputPitch, &SetpointPitch, KpPitch, KiPitch, KdPitch, DIRECT);


void setup() {
  Serial.begin(115200);

  while (!Serial) delay(10);  
  Serial.println("ROCKET");

  // Attaching the ESC and LED pins
  ESC.attach(22,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Setting up PWM
  pwm.begin();
  pwm.setOscillatorFrequency(25000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  // Setting up sensors
  Wire.begin();

  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  if (!bme.begin(0x76)) 
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  delay(1000);

  // Servo angle offsets
  offsets[0] = -15;
  offsets[1] = 5;
  offsets[2] = 0;
  offsets[3] = 0;

  // Initialize the PID objects
  PID_Yaw.SetMode(AUTOMATIC);
  PID_Yaw.SetSampleTime(10); // Adjust based on your system's needs
  PID_Yaw.SetOutputLimits(0, 100); // Adjust based on your thrust control system

  PID_Pitch.SetMode(AUTOMATIC);
  PID_Pitch.SetSampleTime(10); // Adjust based on your system's needs
  PID_Pitch.SetOutputLimits(0, 100); // Example limits for servo angle adjustments

  delay(10);
}

void calibrate_and_start() {
  ESC.write(180); //setting to full throttle/max pulse_width
  delay(5000); //should beep two times 

  ESC.write(0); //setting to zero throttle
  delay(5000); //should beep five times
}

void setVaneAngle(uint8_t n, double percent) {
    // where 0 is fully left, 100 is fully right 
    int offset = offsets[n];
    int pulselen =  210 + 0.9 * percent; // normalizing the range of servo between 210-300
    pwm.setPWM(servonum+n, 0, pulselen+offset);
}

void adjustYaw(double percent) {
  setVaneAngle(0, percent);
  setVaneAngle(2, percent);
}

void adjustPitch(double percent) {
  setVaneAngle(1, percent);
  setVaneAngle(3, percent);
}

 
void loop() {
  while (setupDone == false) {
    calibrate_and_start();
    setupDone = true;
    digitalWrite(LED_BUILTIN,HIGH);
    delay(3000);
    digitalWrite(LED_BUILTIN,LOW);
  }

  // for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
  //   pwm.setPWM(servonum, 0, pulselen-15);
  //   pwm.setPWM(servonum+1, 0, pulselen+5);
  //   pwm.setPWM(servonum+2, 0, pulselen);
  //   pwm.setPWM(servonum+3, 0, pulselen);
  //   delay(10);
  // }

  // //delay(500);
  // for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen++) {
  //   pwm.setPWM(servonum, 0, pulselen-15);
  //   pwm.setPWM(servonum+1, 0, pulselen+5);
  //   pwm.setPWM(servonum+2, 0, pulselen);
  //   pwm.setPWM(servonum+3, 0, pulselen);
  //   delay(10);
  // }

  // setServoAngle(0, 0);
  // setServoAngle(2, 0);

  ESC.write(50);
  delay(1000);

  // setServoAngle(0, 100);
  // setServoAngle(2, 100);

  // ESC.write(50); 

  // adjustYaw(0);
  // delay(1000);
  // adjustYaw(50);
  // delay(1000);
  // adjustYaw(100);
  // delay(1000);
  // adjustYaw(50);
  // delay(1000);

  // adjustPitch(0);
  // delay(1000);
  // adjustPitch(50);
  // delay(1000);
  // adjustPitch(100);
  // delay(1000);
  // adjustPitch(50);
  // delay(1000);

  // Defining setpoints
  // int SetpointAltitude = 0; 
  // int SetpointYaw = 0;
  // int SetpointPitch = 0;

  // float InputYaw, InputPitch;


  // while(1) {

  //     InputAltitude = readAltitude(); // Implement this function to read from your barometer
  //     readOrientation(InputYaw, InputPitch); // This updates InputYaw and InputPitch

  //     // Compute PID outputs
  //     PID_Yaw.Compute();
  //     PID_Pitch.Compute();
      
  //      // Adjust thrust and servos based on PID outputs
  //     // adjustThrust(OutputThrust);
  //     // adjustServos(OutputServo);
  //     adjustYaw(OutputYaw);
  //     adjustPitch(OutputPitch);

  //     delay(10);
  // }

}

// Functions to read sensors, adjust actuators, and define desired states
double readAltitude() {
  // Read and return altitude from barometer
  return 0.0; // Placeholder
}

void readOrientation(float &yaw, float &pitch) {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  yaw = euler.x();    // Heading
  pitch = euler.z();  // Pitch
}


void adjustThrust(double thrust) {
  // Adjust rocket thrust based on PID output
  // Placeholder: Implement your thrust control logic here
}

void adjustServos(double servoAngle) {
  // Adjust servo angles based on PID output
  // Placeholder: Implement your servo control logic here
}


