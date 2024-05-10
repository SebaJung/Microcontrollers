
/*
  Line sensors with ammendity LCD screen
  By: Carlos, Jon, Sebastian
  Written: March 22, 2024
  Edited: May 10, 2024
  I/O Pins
  A0: Left Line Sensor
  A1: Right Line Sensor
  A2: Center Line Sensor
  A3: E PIN
  A4: RS PIN
  A5:
  D0:
  D1:
  D2: Right Wheel Encoder
  D3: Right Forward PWM - OCR2B
  D4: Left Motor Control
  D5: Left Forward PWM - OC0B
  D6: Left Reverse PWM - OC0A
  D7: Right Motor Control
  D8: LCD PIN DB4
  D9: LCD PIN DB5
  D10: LCD PIN DB6
  D11: Right Reverse PWM - OC2A
  D12: Left Wheel Encoder
  D13: LCD PIN DB7
*/

#include "hd44780.h"                // including the lcd library

void setup() {

  DDRD = 0xF8;                      // output pin for the PWM signals of motor control and trigger pin
  DDRB = 0x08;                      // output pin for the PWM signals of motor control

  PORTB |= 0x10;                    // enables internal pull-up on left encoder
  PORTD |= 0x04;                    // enables internal pull-up on right encoder

  cli();                            // clears the interrupt enable flag on SREG

  TCCR0A = 0xA1;                    // Using phase-correct PWM, clear on A, clear on B
  TCCR0B = 0x01;                    // with prescale value of 1

  TCCR2A = 0xA1;                    // Using phase-correct PWM, clear on A, clear on B
  TCCR2B = 0x01;                    // with prescale value of 1

  ADCSRA = 0xCF;                    // auto trigger disable, prescale of 128 ADC enabled and start conversion bit on
  ADCSRB = 0x00;                    // not in free-running mode
  ADMUX = 0x40;                     // AVCC mode, 10-bit precision, analog bit A0 originally

  PCICR = 0x01;                     // activates the pin change interrupt
  PCMSK0 = 0x10;                    // pins D12 and enables interrupts

  EICRA = 0x02;                     // pin D2 and triggers on falling edge
  EIMSK = 0x01;                     // enables interrupts on D2

  lcd_init();                       // lcd startup routine
  sei();                            // sets the interrupt enable flag on SREG
  Serial.begin(9600);
  PORTD |= 0x90;                    // enables the motor control
}

volatile unsigned int  sregValue;
volatile unsigned int  leftWheel = 0;
volatile unsigned int  rightWheel = 0;
volatile unsigned int  leftSensor = 0;
volatile unsigned int  centerSensor = 0;
volatile unsigned int  rightSensor = 0;

void loop() {
  lcd_clrscr();                   // clears the content of the screen

  unsigned int avgCount = average(leftWheel, rightWheel);    // average value of the toggles between both the wheels
  unsigned long distance = (avgCount * 105L) / 100;

  //section below relating to line sensor:
  if ((centerSensor >= 760) && (rightSensor <= 700) && (leftSensor <= 700)) {
    OCR0A = 0;
    OCR0B = 200;                      // sends the left motor forward
    OCR2A = 0;
    OCR2B = 200;                      // sends the right motor forward
  }
  else if ((rightSensor > leftSensor))  {
    OCR0A = 0;
    OCR0B = 220;
    OCR2B = 0;
    OCR2A = 60;                      //left reverse signal half speed of right
  } else if (leftSensor > rightSensor) {
    OCR0A = 60;                      // right reverse signal half speed of left
    OCR0B = 0;
    OCR2A = 0;
    OCR2B = 220;
  }
  if (distance >= 4300) {
    OCR0A = 0;
    OCR0B = 0;
    OCR2A = 0;
    OCR2B = 0;
  }
  Serial.print(leftSensor);
  Serial.print('\t');
  Serial.print(centerSensor);
  Serial.print('\t');
  Serial.print(rightSensor);
  Serial.println();

  lcd_puts("Odometer:");
  char charbufferDistance[10];
  ltoa((distance / 10), charbufferDistance, 10);
  lcd_goto(0x0A);
  lcd_puts(charbufferDistance);
  lcd_puts(" cm");
  _delay_ms(50);
}

unsigned int average(unsigned int leftWheel, unsigned int rightWheel) {
  return ((leftWheel + rightWheel) / 2);
}

ISR(PCINT0_vect) {                // left wheel encoder
  if ((PINB & 0x10) == 0)
    leftWheel++;
}

ISR(INT0_vect) {                  // right wheel encoder
  if ((PIND & 0x04) == 0)
    rightWheel++;
}

ISR(ADC_vect) {
  /* Depending on which ADMUX value is set, it
      will store the value of that ADC register
      onto that sensor's variable
  */

  sregValue = SREG;
  if (ADMUX == 0x40) {               // A0 left sensor
    leftSensor = ADC;
    ADMUX = 0x41;
  } else if (ADMUX == 0x41) {        // A1 right sensor
    rightSensor = ADC + 110;
    ADMUX = 0x42;
  } else if (ADMUX == 0x42) {        // A2 center sensor
    centerSensor = ADC;
    ADMUX = 0x40;
  }
  SREG = sregValue;
  ADCSRA |= 0x40;                    // start a new conversion
}
