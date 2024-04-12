/*
  MUX Sensors
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
  D2: TRIGGER PIN
  D3: Right Forward PWM - OCR2B
  D4: Left Motor Control
  D5: Left Forward PWM - OC0B
  D6: Left Reverse PWM - OC0A
  D7: Right Motor Control
  D8: ICU from ultrasonic sensor
  D9:
  D10:
  D11: Right Reverse PWM - OC2A
  D12: Left Wheel Encoder
  D13: Right Wheel Encoder
*/

void setup() {

  // output pin for the PWM signals of motor control and trigger pin
  DDRD = 0xFC;

  // output pin for the PWM signals of motor control
  DDRB = 0x08;

  // clears the interrupt enable on SREG
  cli();

  TCCR0A = 0xA1;  // Using phase-correct PWM, clear on A, clear on B
  TCCR0B = 0x01;  // with prescale value of 1


  TCCR2A = 0xA1;  // Using phase-correct PWM, clear on A, clear on B
  TCCR2B = 0x01;  // with prescale value of 1


  PCICR = 0x02;   // activates the pin change interrupt
  PCMSK1 = 0x30;  // on pins A4 and A5



  // sets the interrupt enable on SREG
  sei();

  OCR0A = 0;
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
    case (0):
      goForward();
      break;
    case (1):
      rightWhisker();
      break;
    case (2):
      leftWhisker();
      break;
    case (3):
      rightTurn();
      break;
  }
}

void goForward() {
  OCR0A = 0;
  OCR0B = 150;
  OCR2A = 0;
  OCR2B = 150;
}

void rightWhisker() {
  OCR0A = 150;                // if right bumper hits, reverse backwards
  OCR0B = 0;                // if right bumper hits, dont forward
  OCR2A = 80;                // if right bumper hits, dont reverse
  OCR2B = 0;                // if right bumper hits, dont forward
  _delay_ms(500);              // for 50ms then not reverse anymore
  OCR0A = 0;
  OCR0B = 150;
  OCR2B = 150;
}

void leftWhisker() {
  OCR0A = 80;        // if left bumper is hit, dont reverse
  OCR0B = 0;        // if left bumper is hit, dont forward for
  OCR2B = 0;        // if left bumper is hit, dont forward for
  OCR2A = 150;        // if left bumper is hit, reverse for
  _delay_ms(500);      // 50ms and then go forward
  OCR2A = 0;
  OCR2B = 150;
  OCR0B = 150;
}

void rightTurn() {
  OCR2A = 150;        // rotate 90 degrees and go forward
  OCR2B = 0;
  OCR0B = 150;
  OCR0A = 0;
  _delay_ms(750);
  OCR2A = 0;          // go forward
  OCR2B = 120;
  OCR0B = 150;
  OCR0A = 0;
  goOtherWay();
}

void goOtherWay() {
  while (middle == 1) {
    OCR2A = 150;        // rotate 180 degrees and go forward
    OCR2B = 0;
    OCR0B = 150;
    OCR0A = 0;
    _delay_ms(1500);
  }
}

ISR(PCINT1_vect)
{
  _delay_ms(50);
  button = (PINC & 0x30);
  if (button == 0x00)       // if no bumper closed, dont reverse
    middle = 0;
  else if (button == 0x10)
    middle = 1;
  else if (button == 0x20)
    middle = 2;
  else if (button == 0x30)
    middle = 3;

}
