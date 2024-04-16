/*
  Wheel Encoders
  By: Carlos, John, Sebastian
  Written: April 9, 2024
  Edited: April 16, 2024
  I/O Pins
  A0: Left Line Sensor
  A1: Center Line Sensor
  A2: Right Line Sensor
  A3:
  A4: Right whisker
  A5: Left whisker
  D0:
  D1:
  D2: Right Wheel Encoder
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
  D13: Trigger pin
*/

volatile unsigned char button = 1;
volatile unsigned int leftWheel = 0;
volatile unsigned int rightWheel = 0;

void setup() {

  cli();          // clears the interrupt enable on SREG

  PCICR = 0x03;   // activates the pin change interrupt on PORT B
  PCMSK0 = 0x10;  // pins D12 and enables interrupts
  PCMSK1 = 0x07;  // pins A0-A2 and enables interrupts

  EICRA = 0x02;   // pin D2 and triggers on falling edge
  EIMSK = 0x01;   // enables interrupts on D2

  TCCR0A = 0xA1;  // Using phase-correct PWM, clear on A, clear on B
  TCCR0B = 0x01;  // with prescale value of 1

  TCCR2A = 0xA1;  // Using phase-correct PWM, clear on A, clear on B
  TCCR2B = 0x01;  // with prescale value of 1


  sei();          // sets the interrupt enable on SREG

  PORTB |= 0x10;  // enables internal pull-up on left encoder
  PORTD |= 0x04;  // enables internal pull-up on right encoder

  Serial.begin(9600); // serial monitor start up command w baud rate of 9600
}

void loop() {
  unsigned int avgCount = average(leftWheel, rightWheel);    // average value of the toggles between both the wheels
  unsigned int distance = (avgCount * 105L) / 100;
  
  Serial.print(avg);
  Serial.print('\t');
  Serial.print(leftWheel);
  Serial.print('\t');
  Serial.print(rightWheel);
  Serial.print('\t');
  Serial.print(distance);
  Serial.print('\n');

}

unsigned int average(unsigned int leftWheel, unsigned int rightWheel)
{
  return ((leftWheel + rightWheel) / 2);
  
}

ISR(PCINT0_vect)      // left wheel encoder
{
  if ((button = PINB & 0x10) == 0)
    leftWheel++;
}

ISR(INT0_vect)      // right wheel encoder
{
  if ((button = PIND & 0x04) == 0)
    rightWheel++;
}
