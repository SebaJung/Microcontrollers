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

volatile unsigned int distance = 0;
volatile unsigned long count = 0;
volatile unsigned long capt[2];

void setup() {

  // clears the interrupt enable on SREG
  cli();

  // output pin for the PWM signals of motor control and trigger pin
  DDRD = 0xFC;

  // output pin for the PWM signals of motor control
  DDRB = 0x08;

  TCCR0A = 0xA1;  // Using phase-correct PWM, clear on A, clear on B
  TCCR0B = 0x01;  // with prescale value of 1
  TIMSK0 = 0x06;

  TCCR2A = 0xA1;  // Using phase-correct PWM, clear on A, clear on B
  TCCR2B = 0x01;  // with prescale value of 1
  TIMSK2 = 0x06;

  PCICR = 0x02;   // activates the pin change interrupt
  PCMSK1 = 0x30;  // on pins A4 and A5

  // sets the interrupt enable on SREG
  sei();

  // start the motors by triggering the forward PWM
  OCR0B = 255;
  OCR0A = 0;
  OCR2B = 255;
  OCR2A = 0;

}

volatile unsigned char button = 1;

void loop() {
  // enables the motor control
  PORTD |= 0x90;
}


ISR(TIMER0_COMPA_vect)      // reverse left PWM
{
  if (button == 0x00)       // if no bumper closed, dont reverse
    OCR0A = 0;

  else if (button == 0x01) {
    OCR0A = 125;                // if right bumper hits, reverse backwards
    _delay_ms(50);              // for 50ms then not reverse anymore
    OCR0A = 0;
  }

  else if (button == 0x20)
    OCR0A = 0;        // if left bumper is hit, dont reverse
}

ISR(TIMER0_COMPB_vect)    // forward left PWM
{
  if (button == 0x00)       // if no bumper closed, go forward
    OCR0B = 250;

  else if (button == 0x01) {
    OCR0B = 0;                // if right bumper hits, dont forward
    _delay_ms(50);              // for 50ms then forard
    OCR0B = 250;
  }

  else if (button == 0x20) {
    OCR0B = 0;        // if left bumper is hit, dont forward for
    _delay_ms(50);      // 50ms and then go forward
    OCR0B = 250;

  }
}

  ISR(TIMER2_COMPA_vect)      // reverse right PWM
  {
    if (button == 0x00)       // if no bumper closed, dont reverse 
      OCR2A = 0;

    else if (button == 0x01) {
      OCR2A = 0;                // if right bumper hits, dont reverse
    }

    else if (button == 0x20) {
      OCR2A = 125;        // if left bumper is hit, reverse for
      _delay_ms(50);      // 50ms and then go forward
      OCR2A = 0;

    }
  }
    ISR(TIMER2_COMPB_vect)      // forward right PWM
{
  if (button == 0x00)       // if no bumper closed, go forward 
    OCR2B = 250;

  else if (button == 0x01) {
    OCR2B = 0;                // if right bumper hits, dont forward
    _delay_ms(50);              // for 50ms then forward
    OCR2B = 250;
  }

  else if (button == 0x20) {
    OCR2B = 0;        // if left bumper is hit, dont forward for
    _delay_ms(50);      // 50ms and then go forward
    OCR2B = 250;

  }
}
    ISR(PCINT1_vect)
    {
      unsigned char i = 0;
      if (i == 0)
        (button = PINC & 0xCF);
      i ^= 1;
    }
