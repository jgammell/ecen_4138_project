#include "Servo.h"
#include "MsTimer2.h"
#include <math.h>

#define ESC_PIN (6)
#define DRIVE_PIN (10)
#define SENSOR_PIN (A0)

#define READY_CMD ('1')
#define CHECK_AVAILABLE_CMD ('2')
#define SET_BASE_CMD ('3')
#define SET_FREQ_CMD ('4')
#define SET_AMPLITUDE_CMD ('5')
#define SET_DURATION_CMD ('6')
#define COMPUTE_LUT_CMD ('7')
#define BEGIN_CMD ('8')
#define SET_KP_CMD ('9')
#define SET_KD_CMD ('A')
#define SET_KI_CMD ('B')
#define SET_OFFSET_CMD ('C')
#define ERROR_CMD ('e')
#define ACK   ('a')
#define NACK  ('n')

// Experiment parameters
#define SAMPLES_PER_PERIOD (64)
int16_t signal[SAMPLES_PER_PERIOD];
int16_t datapoints[4*SAMPLES_PER_PERIOD];
int16_t base;
int16_t frequency;
uint32_t sample_period_us;
int16_t amplitude;
uint8_t periods_measure;
uint8_t periods_wait;

// PID parameters
int16_t target;
int16_t Kp;
int16_t Kd;
int16_t Ki;
int16_t offset;

Servo ESC;

#define setSpeed(DRIVE) ESC.writeMicroseconds((DRIVE)+1000)
#define Kp(E) ((Kp*(E))/1000)
#define Kd(E) ((Kd*(E))/1000)
#define Ki(E) ((Ki*(E))/1000)

void updatePid(void)
{
  static int16_t old_error = 0;
  static int16_t acc_error = 0;
  int16_t error = target-analogRead(SENSOR_PIN);
  acc_error += error;
  int16_t new_drive = Kp(error) + Kd(error-old_error) + Ki(acc_error) + offset;
  setSpeed(new_drive);
  old_error = error;
}

char readCmd(void)
{
  while(!Serial.available());
  char cmd = Serial.read();
  while(!Serial.available());
  if(Serial.read() != '\n')
    cmd = ERROR_CMD;
  return cmd;
}

void setup() 
{
  target = 0;
  Kp = 0;
  Kd = 0;
  Ki = 0;
  offset = 0;
  Serial.begin(115200);
  ESC.attach(ESC_PIN);
  pinMode(DRIVE_PIN, OUTPUT);
  digitalWrite(DRIVE_PIN, LOW);
  delay(500);
  setSpeed(0);
  digitalWrite(DRIVE_PIN, HIGH);
  delay(2500);
  MsTimer2::stop();
  MsTimer2::set(20, updatePid);
  while(Serial.available() && Serial.read());
  Serial.println(READY_CMD);
  while(readCmd() != READY_CMD);
}

void loop() 
{
  char cmd = readCmd();
  switch(cmd)
  {
    case CHECK_AVAILABLE_CMD:
      Serial.println(ACK);
      break;
    case SET_BASE_CMD:
      Serial.println(ACK);
      while(!Serial.available());
      base = ((int16_t) Serial.read()) << 8;
      while(!Serial.available());
      base |= (int16_t) Serial.read();
      while(!Serial.available());
      if(Serial.read() != '\n')
      {
        Serial.println("Invalid base command");
        break;
      }
      Serial.println(base);
      Serial.println(ACK);
      break;
    case SET_FREQ_CMD:
      Serial.println(ACK);
      while(!Serial.available());
      frequency = ((int16_t)(uint8_t) Serial.read()) << 8;
      while(!Serial.available());
      frequency |= (int16_t)(uint8_t) Serial.read();
      while(!Serial.available());
      if(Serial.read() != '\n')
      {
        Serial.println("Invalid frequency command");
        break;
      }
      Serial.println(frequency);
      sample_period_us = 10*1000000UL/(frequency*SAMPLES_PER_PERIOD);
      Serial.println(ACK);
      break;
    case SET_AMPLITUDE_CMD:
      Serial.println(ACK);
      while(!Serial.available());
      amplitude = ((int16_t) Serial.read()) << 8;
      while(!Serial.available());
      amplitude |= (int16_t) Serial.read();
      while(!Serial.available());
      if(Serial.read() != '\n')
      {
        Serial.println("Invalid amplitude command");
        break;
      }
      Serial.println(amplitude);
      Serial.println(ACK);
      break;
    case SET_DURATION_CMD:
      Serial.println(ACK);
      while(!Serial.available());
      periods_measure = (uint8_t) Serial.read();
      while(!Serial.available());
      periods_wait = (uint8_t) Serial.read();
      while(!Serial.available());
      if(Serial.read() != '\n')
      {
        Serial.println("Invalid duration command");
        break;
      }
      Serial.println(periods_measure);
      Serial.println(periods_wait);
      Serial.println(ACK);
      break;
    case SET_KP_CMD:
      Serial.println(ACK);
      while(!Serial.available());
      Kp = ((int16_t) Serial.read())<<8;
      while(!Serial.available());
      Kp |= ((int16_t) Serial.read());
      while(!Serial.available());
      if(Serial.read() != '\n')
      {
        Serial.println("Invalid Kp command");
        break;
      }
      Serial.println(Kp);
      Serial.println(ACK);
      break;
    case SET_KD_CMD:
      Serial.println(ACK);
      while(!Serial.available());
      Kd = ((int16_t) Serial.read())<<8;
      while(!Serial.available());
      Kd |= (int16_t) Serial.read();
      while(!Serial.available());
      if(Serial.read() != '\n')
      {
        Serial.println("Invalid Kd command");
        break;
      }
      Serial.println(Kd);
      Serial.println(ACK);
      break;
    case SET_KI_CMD:
      Serial.println(ACK);
      while(!Serial.available());
      Ki = ((int16_t) Serial.read())<<8;
      while(!Serial.available());
      Ki |= (int16_t) Serial.read();
      while(!Serial.available());
      if(Serial.read() != '\n')
      {
        Serial.println("Invalid Ki command");
        break;
      }
      Serial.println(Ki);
      Serial.println(ACK);
      break;
    case SET_OFFSET_CMD:
      Serial.println(ACK);
      while(!Serial.available());
      offset = ((int16_t) Serial.read()) << 8;
      while(!Serial.available());
      offset |= (int16_t) Serial.read();
      while(!Serial.available());
      if(Serial.read() != '\n')
      {
        Serial.print("Invalid offset command");
        break;
      }
      Serial.println(offset);
      Serial.println(ACK);
      break;
    case COMPUTE_LUT_CMD:
      Serial.println(ACK);
      for(int16_t i=0; i<SAMPLES_PER_PERIOD; ++i)
      {
        signal[i] = amplitude*sin(2*M_PI*((float)i/SAMPLES_PER_PERIOD))+base;
        Serial.println(signal[i]);
      }
      Serial.println(ACK);
      break;
    case BEGIN_CMD:
      Serial.println(ACK);
      target = base;
      setSpeed(offset);
      delay(5000);
      MsTimer2::start();
      for(uint8_t i=0; i<periods_wait; ++i)
      {
        for(uint8_t j=0; j<SAMPLES_PER_PERIOD; ++j)
        {
          uint32_t t0 = micros();
          target = signal[j];
          if(micros()-t0 >= sample_period_us)
          {
            Serial.println(NACK);
            break;
          }
          while(micros()-t0 < sample_period_us);
        }
      }
      for(uint8_t i=0; i<periods_measure; ++i)
      {
        for(uint8_t j=0; j<SAMPLES_PER_PERIOD; ++j)
        {
          uint32_t t0 = micros();
          target = signal[j];
          datapoints[j+SAMPLES_PER_PERIOD*i] = analogRead(SENSOR_PIN);
          if(micros()-t0 >= sample_period_us)
          {
            Serial.println(NACK);
            break;
          }
          while(micros()-t0 < sample_period_us);
        }
      }
      MsTimer2::stop();
      setSpeed(0);
      Serial.println(ACK);
      for(uint16_t i=0; i<SAMPLES_PER_PERIOD*periods_measure; ++i)
        Serial.println(datapoints[i]);
      Serial.println(ACK);
      break;
    default:
      Serial.print("Invalid command received: "); Serial.println(cmd);
      break;
  }
}
