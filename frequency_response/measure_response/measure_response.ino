#include "Servo.h"
#include "MsTimer2.h"
#include <math.h>
#include <stdint.h>
#include <arduino-timer.h>

#define ESC_PIN    (6)
#define DRIVE_PIN  (10)
#define SENSOR_PIN (A0)

bool progressState(void *);

#define Nm (5)
#define NUM_SAMPLES (100)
int16_t  SIN_LUT[NUM_SAMPLES/4];
inline int16_t sin_lut(uint16_t x)
{
  if(x < (NUM_SAMPLES/4))
    return SIN_LUT[x];
  else if(x < (NUM_SAMPLES/2))
    return SIN_LUT[(NUM_SAMPLES/2) - x - 1];
  else if(x < (3*NUM_SAMPLES/4))
    return -1*SIN_LUT[x - (NUM_SAMPLES/2) - 1];
  else
    return -1*SIN_LUT[NUM_SAMPLES-x - 1];
}

typedef enum
{
  idle,
  settling,
  measuring
} state_enum;

int16_t center;
int16_t amplitude;
uint8_t Ns;
uint16_t f;
state_enum state;

Servo ESC;
Timer<10, micros> timer;

void setup() 
{
  center = 0;
  amplitude = 0;
  Ns = 0;
  f = 0;
  state = idle;
  
  // initializations based on original code
  Serial.begin(115200);
  ESC.attach(ESC_PIN);
  pinMode(DRIVE_PIN, OUTPUT);
  digitalWrite(DRIVE_PIN, LOW);

  // clear serial buffer
  while(Serial.available() && Serial.read());
  Serial.println("r"); // Indicate readiness
}

void loop()
{
  if(Serial.available())
  {
    Serial.read();
    Serial.println("A");
    while(!Serial.available());
    center = Serial.parseInt();
    Serial.read();
    while(!Serial.available());
    amplitude = Serial.parseInt();
    Serial.read();
    while(!Serial.available());
    Ns = Serial.parseInt();
    Serial.read();
    while(!Serial.available());
    f = Serial.parseInt();
    Serial.read();
      // create lookup table for sin
    for(uint16_t i = 0; i < NUM_SAMPLES/4; ++i)
      SIN_LUT[i] = (int16_t) amplitude*sin(i*(2*M_PI/NUM_SAMPLES))+center;
    Serial.println("Running trial with the following parameters:");
    Serial.print("\tCenter: "); Serial.print(center); Serial.print('\n');
    Serial.print("\tAmplitude: "); Serial.print(amplitude); Serial.print('\n');
    Serial.print("\tSettling periods: "); Serial.print(Ns); Serial.print('\n');
    Serial.print("\tFrequency: "); Serial.print(f); Serial.print('\n');
    Serial.print("Sin lookup table:\n");
    for(uint16_t i=0; i<=NUM_SAMPLES; ++i)
      Serial.println(sin_lut(i));
    Serial.println("Done");
    digitalWrite(DRIVE_PIN, LOW);
    delay(500);
    digitalWrite(DRIVE_PIN, HIGH);
    timer.every(1000000UL/(NUM_SAMPLES*f), progressState);
  }
}

uint8_t N;
uint16_t i;
uint8_t measurements[NUM_SAMPLES*Nm];
uint16_t measurements_idx;
bool progressState(void * arg)
{
  (void) arg;
  switch(state)
  {
    case idle:
      N = Ns;
      i = 0;
      state = settling;
      break;
    case settling:
      if(i < NUM_SAMPLES)
      {
        ESC.writeMicroseconds(map(sin_lut(i), 0, 1000, 1000, 2000));
        i += 1;
      }
      else
      {
        N -= 1;
        if(N == 0)
        {
          N = Nm;
          i = 0;
          measurements_idx = 0;
          state = measuring;
        }
      }
      break;
    case measuring:
      measurements[measurements_idx] = analogRead(SENSOR_PIN);
      measurements_idx += 1;
      if(i < NUM_SAMPLES)
      {
        ESC.writeMicroseconds(map(sin_lut(i), 0, 1000, 1000, 2000));
        i += 1;
      }
      else
      {
        N -= 1;
        if(N == 0)
        {
          ESC.writeMicroseconds(map(0, 0, 1000, 1000, 2000));
          state = idle;
          timer.cancel();
          for(uint16_t i=0; i<NUM_SAMPLES*Nm; ++i)
            Serial.println(measurements[i]);
        }
      }
      break;
  }
  return true;
}
