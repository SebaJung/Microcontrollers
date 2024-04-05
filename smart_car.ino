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

volatile unsigned int sensorArr[3];
volatile unsigned long count = 0;
volatile unsigned long capt[2];

void setup() {

  // clears the interrupt enable on SREG
  cli();

  // configuring ADC registers to have AVCC as voltage reference,
  //

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
  unsigned long ticksBetween[3] = {0};
  static unsigned long tHigh[3] = {0};

  // send a 10 us delay to the echo pin
  PORTD ^= 0X04;
  _delay_us(10);
  PORTD ^= 0X04;

  cli();            // changes the ADMUX configurations for left sensor
  ADMUX = 0x63;
  sensorArr[0] = (PINC & 0X08);
  if (capt[1] > capt[0])
    ticksBetween[0] = capt[1] - capt[0];
  sei();
  /*
    cli();            // changes the ADMUX configurations for front sensor
    ADMUX = 0x64;
    sensorArr[1] = (PINC & 0X10);
    if (capt[1] > capt[0])
      ticksBetween[1] = capt[1] - capt[0];
    sei();

    cli();            // changes the ADMUX configurations for right sensor
    ADMUX = 0x65;
    sensorArr[2] = (PINC & 0X20);
    if (capt[1] > capt[0])
      ticksBetween[2] = capt[1] - capt[0];
    sei();
  */
  unsigned long distance[3] = {0};

  for (unsigned char i = 0; i < 3; i++) {
    tHigh[i] = (ticksBetween[i] << 4);
    distance[i] = (tHigh[i] * 17182L) / 10000;     // deleted a power of 10
    Serial.println(distance[0]);                               // because of no need for tenth place accuracy
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

ISR(TIMER1_OVF_vect)          // counts the
{
  count += 65536;
}

ISR(ADC_vect)
{

}








/*  // for tenth's place precision
  unsigned char tenth = distanceMM % 10;

  // for the whole number place
  unsigned int wholeNum = distanceMM / 10;
*/