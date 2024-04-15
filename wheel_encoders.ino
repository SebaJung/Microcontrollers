/*
  Wheel Encoders
  By: Carlos Perez
  Written: March 22, 2024
  Edited: March 22, 2024
  I/O Pins
  A0: Left Line Sensor
  A1: Center Line Sensor
  A2: Right Line Sensor
  A3:
  A4: Right whisker
  A5: Left whisker
  D0:
  D1:
  D2:
  D3: Right Forward PWM - OCR2B
  D4: Left Motor Control
  D5: Left Forward PWM - OC0B
  D6: Left Reverse PWM - OC0A
  D7: Right Motor Control
  D8:
  D9:
  D10:
  D11: Right Reverse PWM - OC2A
  D12: Left Wheel Encoder
  D13: Right Wheel Encoder
*/

volatile unsigned char button = 1;
volatile unsigned int leftWheel = 0;
volatile unsigned int rightWheel = 0;

void setup() {

  cli();          // clears the interrupt enable on SREG

  PCICR = 0x01;   // activates the pin change interrupt on PORT B
  PCMSK0 = 0x30;  // pins D12 and D13 and enables interrupts

  sei();          // sets the interrupt enable on SREG

  PORTB |= 0x30;  // enables internal pull-up on wheel encoders

  Serial.begin(9600); // serial monitor start up command w baud rate of 9600
}

void loop() {
  unsigned int avg = average(leftWheel, rightWheel);    // average value of the toggles between both the wheels
  //Serial.println(avg);
}

unsigned int average(unsigned int leftWheel, unsigned int rightWheel)
{
  return ((leftWheel + rightWheel) / 2);
}

ISR(PCINT0_vect)
{
  if ((button = PINB & 0x10) == 0) {
    leftWheel++;
    //rightWheel++;
  } else if ((button = PINB & 0x20) == 0)
    rightWheel++;
  /*else if ((button = PINB & 0x30) == 2)
    leftWheel;
  */
  Serial.println(rightWheel);
}
