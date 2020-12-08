#include "Servo.h"
#include "MsTimer2.h"

volatile Servo ESC;

#define setSpeed(DRIVE) ESC.writeMicroseconds((DRIVE)+1000)
#define Kp(E) ((Kp*(E))/1000)
#define Kd(E) ((Kd*(E))/1000)
#define Ki(E) ((Ki*(E))/1000)

volatile int16_t target;
volatile int16_t Kp;
volatile int16_t Kd;
volatile int16_t Ki;
volatile int16_t offset;

uint32_t times[128];

void updatePidFast(void)
{
  volatile static int16_t old_error = 0;
  volatile static int16_t acc_error = 0;
  int16_t error = target-analogRead(A0);
  acc_error += error;
  int16_t new_drive = Kp(error) + Kd(error-old_error) + Ki(acc_error) + offset;
  setSpeed(new_drive);
  old_error = error;
}

typedef struct PID_t {
  float   input;      // Input to controller (requested value)
  float   offset;
  float   Ki;         // Integral gain
  float   Kp;         // Proportional gain
  float   Kd;         // Derivative gain
  float   dt;         // Period between updates (in seconds)
  float   old_error;  // Last error value
  float   iState;     // Integrator state (accumulated error)
} PID;
volatile PID controller;  // PID controller for the system
volatile float pitch;     // Measured pitch
volatile int drive;       // Drive signal value fed to ESC

volatile bool updatedPID; // Flag to indicate whenever controller is updated

volatile float filterBuffer[2];  // Array for moving average filter
volatile float filteredVal;                // Current filtered valued
volatile int index;                        // Current index of filterBuffer

bool running = false;

float filter(float value) {
  // Remove oldest value from moving average
  filteredVal -= filterBuffer[index] / 2;

  // Add new value to buffer and incrememnt index
  filterBuffer[index++] = value;

  // Add new value to moving average
  filteredVal += value / 2;

  // Prevent index out of bounds errors
  index %= 2;

  return filteredVal;
}

void setSpeed0(Servo *ESC, int drive) {
  // Scale drive signal to ESC's range of accepted values
  int us = map(drive, 0, 1000, 1000, 2000); //Scale drive signal to ESC's accepted range
  ESC->writeMicroseconds(us);
}

void updatePidSlow(void)
{
  // P, I, & D terms
  float pTerm, iTerm, dTerm;

  // Measure and filter rotary sensor value
  pitch = filter((-0.3656 * analogRead(0)) + 185.64);
  
  // Controller error is difference between input and current state
  float error = controller.input - pitch;

  // Calculate the proportional term
  pTerm = controller.Kp * error;

  // Calculate the integral state with appropriate min/max constraints
  controller.iState += error * controller.dt;
  controller.iState = constrain(controller.iState, 3/controller.Ki, 3/controller.Ki);

  // Calculate the integral term
  iTerm  = controller.Ki * controller.iState;

  // Calculate the derivative term
  dTerm  = controller.Kd * ((error - controller.old_error)/controller.dt);

  // Update the dState of the controller
  controller.old_error = error;

  // Add PID terms to get new drive signal (0-1000 scale)
  drive = controller.offset + pTerm + iTerm + dTerm; // Add OFFSET to PID correction

  // Send new drive signal to ESC
  setSpeed0(&ESC, drive);

  // Set updatedPID flag
  updatedPID = true;
}

void flushFn(void)
{
  volatile int a = 0;
  volatile int b = 1;
  volatile int c = 2;
  volatile int d = 3;
  volatile int e = 4;
  volatile int f = 5;
  volatile int g = 6;
  volatile int h = 7;
  volatile int result = a+b+c+d+e+f+g+h;
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  updatePidFast();
  updatePidSlow();
  Serial.begin(115200);
  for(uint8_t i=0; i<128; ++i)
  {
    uint32_t t0 = micros();
    times[i] = micros()-t0;
  }
  Serial.println("Reference times:");
  for(uint8_t i=0; i<128; ++i)
    Serial.println(times[i]);
  for(uint8_t i=0; i<128; ++i)
  {
    uint32_t t0 = micros();
    updatePidFast();
    times[i] = micros()-t0;
    for(uint8_t j=0; j<128; ++j)
      flushFn();
  }
  Serial.println("Fast times:");
  for(uint8_t i=0; i<128; ++i)
    Serial.println(times[i]);
  for(uint8_t i=0; i<128; ++i)
  {
    uint32_t t0 = micros();
    updatePidSlow();
    times[i] = micros()-t0;
    for(uint8_t j=0; j<128; ++j)
      flushFn();
  }
  Serial.println("Slow times:");
  for(uint8_t i=0; i<128; ++i)
    Serial.println(times[i]);
  Serial.println("Done");
  while(1);
}
