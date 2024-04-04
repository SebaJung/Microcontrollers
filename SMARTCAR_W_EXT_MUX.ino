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
  A4:
  A5:
  D0:
  D1:
  D2: TRIGGER PIN
  D3:
  D4:
  D5:
  D6:
  D7:
  D8: ICU from ultrasonic sensors
  D9:
  D10:
  D11:
  D12:
  D13:
*/

volatile unsigned int 
volatile unsigned long count = 0;
volatile unsigned long capt[2];

void setup() {

  // output pin for the PWM signals and toggle switches
  DDRD = 0x40;

  // clears the interrupt enable on SREG
  cli();

  TCCR0A = 0x81;  // Using phase-correct PWM, clear on A
  TCCR0B = 0x01;  // with prescale value of 1

  TCCR2A = 0X81;  // Using phase-correct PWM, clear on A
  TCCR2B = 0X01;  // with presclae value of 1

  // configure input capture unit register                      // start of ultrasonic configurations
  // normal mode for everything
  // input capture enabled w/ prescale of 256
  // input capture enabled and output disabled
  TCCR1A = 0X00;
  TCCR1B = 0XC4;
  TIMSK1 = 0X21;

  ADCSRA = 0XEF;
  ADCSRB = 0X00;
  ADMUX = 0X64;

  // trigger pin set as an output pin
  DDRD = 0X04;                                                  // end of ultrasonic configurations

  // sets the interrupt enable on SREG
  sei();

  Serial.begin(9600);

}


void loop() {

  // declare variables for the periods
  unsigned long ticksBetween = 0;
  static unsigned long tHigh = 0;

  // send a 10 us delay to the echo pin
  PORTD ^= 0X04;
  _delay_us(10);
  PORTD ^= 0X04;

  cli();
  if (capt[1] > capt[0])
    ticksBetween = capt[1] - capt[0];
  else (capt[0] > capt[1])
    ticksBetween = capt[0] - capt[1];
  sei();
  unsigned int distance = ticksBetween
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

  PORTC ^= 0X10;          // toggles pin A4
  a = ;
  PORTC ^= 0X20;          // toggles pin A5
  b = ;
  PORTC ^= 0X30;          // toggles pin A4 and A5
  c = ;
}

ISR(ADC_vect)
{

}






/*  // for tenth's place precision
  unsigned char tenth = distanceMM % 10;

  // for the whole number place
  unsigned int wholeNum = distanceMM / 10;
*/
