/*
  MUX Sensors
  By: Carlos Perez
  Written: March 22, 2024
  Edited: March 22, 2024
  I/O Pins
  A0: 
  A1: 
  A2: 
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
  D12: 
  D13: 
*/

void setup() {

  DDRD = 0xF8;              // output pin for the PWM signals of motor control and trigger pin

  DDRB = 0x08;              // output pin for the PWM signals of motor control


  cli();          // clears the interrupt enable on SREG

  TCCR0A = 0xA1;  // Using phase-correct PWM, clear on A, clear on B
  TCCR0B = 0x01;  // with prescale value of 1


  TCCR2A = 0xA1;  // Using phase-correct PWM, clear on A, clear on B
  TCCR2B = 0x01;  // with prescale value of 1


  PCICR = 0x02;   // activates the pin change interrupt
  PCMSK1 = 0x30;  // on pins A4 and A5 and enables interrupts

  sei();          // sets the interrupt enable on SREG


  OCR0A = 0;        // initially has the car driving forward
  OCR0B = 150;
  OCR2A = 0;
  OCR2B = 150;
}

volatile unsigned char button = 0;
volatile unsigned char middle = 0;
void loop() {
  // enables the motor control
  PORTD |= 0x90;

  switch (middle) {
    case (0):             // nothing is hit
      goForward();
      break;
    case (1):             // right whisker is hit
      rightWhisker();
      break;
    case (2):             // left whisker is hit
      leftWhisker();
      break;
    case (3):             // both whiskers are hit
      rightTurn();
      break;
  }
}

void goForward() {    // if nothing is hit...
  OCR0A = 0;
  OCR0B = 150;        // sends the left motor forward
  OCR2A = 0;
  OCR2B = 150;        // sends the right motor forward
}

void rightWhisker() { // if right bumper hits...
  OCR0A = 150;                // left reverse signal
  OCR0B = 0;
  OCR2A = 80;                // right reverse signal half speed of left
  OCR2B = 0;
  _delay_ms(500);              // for 500ms then not reverse anymore
  OCR0A = 0;
  OCR0B = 150;                // go forward with both wheels at same speed
  OCR2B = 150;
}

void leftWhisker() { // if left bumper is hit...
  OCR0A = 80;        //left reverse signal half speed of right
  OCR0B = 0;
  OCR2B = 0;
  OCR2A = 150;        // right reverse signal
  _delay_ms(500);      // 500ms and then go forward
  OCR2A = 0;
  OCR2B = 150;        // go forward with both signals at same speed
  OCR0B = 150;
}

void rightTurn() {
  OCR2A = 150;        // spin "90 degrees" and go forward
  OCR2B = 0;
  OCR0B = 150;
  OCR0A = 0;
  _delay_ms(750);
  OCR2A = 0;          // go forward
  OCR2B = 150;
  OCR0B = 150;
  OCR0A = 0;
  goOtherWay();     // function call to alternate case
}

void goOtherWay() {   // alternate case if the wall is still hit face on...
  while (middle == 1) { // while the right bumper is still hit...
    OCR2A = 0;        // execute rotate "180 degrees" action
    OCR2B = 150;
    OCR0B = 0;        // forward right, reverse left
    OCR0A = 150;
    _delay_ms(1500);
  }
  OCR2A = 0;          // go forward otherwise
  OCR2B = 150;        
  OCR0B = 150;
  OCR0A = 0;
}

ISR(PCINT1_vect)
{
  _delay_ms(50);            // allows for the data to register first and then it can be read
  button = (PINC & 0x30);
  if (button == 0x00)       // nothing hit
    middle = 0;
  else if (button == 0x10)  // right hit
    middle = 1;
  else if (button == 0x20)  // left hit
    middle = 2;
  else if (button == 0x30)  // both hit
    middle = 3;
}
