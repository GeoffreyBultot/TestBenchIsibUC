#include <Wire.h>
#include <TimerOne.h>                           // Header file for TimerOne library
#include <Adafruit_ADS1015.h>

#define TIMER_US 1000                         // 1mS set timer duration in microseconds 
#define TICK_COUNTS 20                          // 2S worth of timer ticks

Adafruit_ADS1115 ads(0x48); // ads 1

const byte interruptPin = 2;
volatile byte state = LOW;
int countInterrupt = 0;
int speedCount = 0;

volatile bool in_long_isr = false;              // True if in long interrupt


void externalSpeedIsr()
{
  countInterrupt++;
}

void timerIsr()
{
  speedCount = countInterrupt;
  countInterrupt =0;
}



void setup(void)
{
  Serial.begin(9600);
  
  ads.begin();
  ads.setGain(GAIN_ONE);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV

  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), externalSpeedIsr, RISING);

  Timer1.initialize(TIMER_US);                  // Initialise timer 1
  Timer1.attachInterrupt( timerIsr );           // attach the ISR routine here
}
int16_t data;
String inputString;
boolean stringComplete;
void loop(void)
{
  data = ads.readADC_Differential_0_1(); // Differential read from ads 1 pin A0/A1
  if(data<0)
    data *= -1;
  if (Serial.available())
  {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inputString.length() == 8) {
      stringComplete = true;
    }
  }
  
  if(stringComplete)
  {
    noInterrupts();
    Serial.write(0xC5);
    Serial.write(0x08);
    Serial.write( (data>>8)&0xff); //High  part of differential voltage
    Serial.write( (data&0xff) ); //Low part of differential voltage
  
    Serial.write( (speedCount>>16)&0xff ); //8MSB of countSpeed
    Serial.write( (speedCount>>8)&0xff ); //8mil of countSpeed
    Serial.write( (speedCount)&0xff ); //8LSB of countSpeed
    
    Serial.write(0xC5);
    interrupts();
    stringComplete = false;
    inputString = "";
  }
  
  
  //Serial.write(countInterrupt);  
}

