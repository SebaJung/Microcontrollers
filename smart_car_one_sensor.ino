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
  A4:
  A5:
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

volatile unsigned long count = 0;
volatile unsigned long capt[2];

void setup() {

  // output pin for the PWM signals of motor control and trigger pin
  DDRD = 0xFC;

  // output pin for the PWM signals of motor control
  DDRB = 0X08;

  // output pin for the trigger pin of ultrasonic sensor
  DDRC = 0X08;

  // enables the motor control
  PORTD |= 0X90;
  // clears the interrupt enable on SREG
  cli();

  TCCR0A = 0xA1;  // Using phase-correct PWM, clear on A, clear on B
  TCCR0B = 0x01;  // with prescale value of 1

  TCCR2A = 0XA1;  // Using phase-correct PWM, clear on A, clear on B
  TCCR2B = 0X01;  // with prescale value of 1


  TCCR1A = 0X00;          // configure input capture unit register
  TCCR1B = 0XC4;          // normal mode for everything
  TIMSK1 = 0X21;          // input capture enabled w/ prescale of 256
  // input capture enabled and output disabled

  ADCSRA = 0XEF;           // ADC registers for line sensors
  ADCSRB = 0X00;          // configure using AVCC, pin 4, ADLAR = 1
  ADMUX = 0X64;           // w/ ADIF bit OFF

  EICRA = 0X02;           // call interrupt on rising edge
  EIMSK = 0X01;           // trigger on pin D2

  // sets the interrupt enable on SREG
  sei();
  // start the motors by triggering the forward PWM
  OCR0B = 255;
  OCR0A = 0;
  OCR2B = 255;
  OCR2A = 0;

  Serial.begin(9600);
}


void loop() {

  // declare variables for the periods
  unsigned long ticksBetween = 0;
  static unsigned long tHigh = 0;

  // send a 10 us delay to the echo pin
  PORTC ^= 0X08;
  _delay_us(10);
  PORTC ^= 0X08;

  cli();
  if (capt[1] > capt[0]) {
    ticksBetween = capt[1] - capt[0];
    tHigh = (ticksBetween << 4);
  }
  else {
    ticksBetween = capt[0] - capt[1];
    tHigh = (ticksBetween << 4);
  }
  sei();

  unsigned int distance = (tHigh * 17182L) / 10000;      // gets the whole number of distance
  Serial.println(distance);
  if (distance > 25) {
    OCR0B = 255;
    OCR2B = 255;
  } else if (distance > 15) {
    OCR0B = 125;
    OCR2B = 125;
  } else if (distance > 5) {
    OCR0B = 80;
    OCR2B = 80;
  }
  else {
    OCR0B = 125;
    OCR2B = 0;
    OCR0A = 0;
    OCR2A = 125;
  }

  _delay_ms(120);         // reduce flickering
}


ISR(TIMER1_CAPT_vect)
{
  static unsigned char x = 0;               // variable used to change between conditions
  if (!x)                                   // if x is not equal to its value
  {
    capt[0] = count + ICR1;
    TCCR1B &= 0XBF;                         // clears the ICES1 bit and captures input on falling edge
  } else
  {
    capt[1] = count + ICR1;
    TCCR1B |= 0X40;                         // sets the ICES1 bit and captures input on rising edge
  }
  x ^= 1;                                   // toggling x to change the condition of the statement
}

ISR(TIMER1_OVF_vect)          // keeps running total of the ticks elasped even after overflow
{
  count += 65536;
}

ISR(TIMER0_COMPA_vect)
{

}

