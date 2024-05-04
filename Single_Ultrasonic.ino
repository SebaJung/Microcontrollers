/*
  Using whiskers and a single ultrasonic sensor
  By: Carlos, John, Sebastian
  Written: May 3, 2024
  Edited: May 3, 2024
  I/O Pins
  A0:
  A1:
  A2:
  A3:
  A4: Right whisker
  A5: Left whisker
  D0:
  D1:
  D2:  Right Wheel Encoder
  D3:  Right Forward PWM - OCR2B
  D4:  Left Motor Control
  D5:  Left Forward PWM - OC0B
  D6:  Left Reverse PWM - OC0A
  D7:  Right Motor Control
  D8:  ICU from ultrasonic sensor
  D9:  Ultrasonic Trig
  D10: 
  D11: Right Reverse PWM - OC2A
  D12: Left Wheel Encoder
  D13: TRIGGER PIN
*/

volatile unsigned long count = 0;
volatile unsigned long captArr[2];

volatile unsigned int  leftWheel = 0;
volatile unsigned int  rightWheel = 0;

volatile unsigned char middle = 0;
volatile unsigned char sregValue;

void setup(){
  DDRD = 0xF8;            				// output pin for the PWM signals of motor control, input for right encoder
  DDRB = 0x28;            				// output pin for the PWM signals of motor control, input for left encoder

  cli();                  				// clears the interrupt enable on SREG

  TCCR0A = 0xA1;         				  // Using phase-correct PWM, clear on A, clear on B
  TCCR0B = 0x01;          				// with prescale value of 1
  TIMSK0 = 0x00;          				// Disable interrupt on compare match A & B

  TCCR1A = 0x00;          				// configure input capture unit register
  TCCR1B = 0xC4;          				// normal mode for everything
  TIMSK1 = 0x21;          				// input capture enabled w/ prescale of 256

  TCCR2A = 0xA1;          				// Using phase-correct PWM, clear on A, clear on B
  TCCR2B = 0x01;          				// with prescale value of 1
  TIMSK2 = 0x00;          				// Disable interrupt on compare match A & B

  PCICR  = 0x03;           				// activates the pin change interrupt on port B
  PCMSK0 = 0x10;          				// port C and enables interrupts (left encoder)
  PCMSK1 = 0x30;          				// changes pins A4 and A5

  EICRA  = 0x01;           				// pin D2 and triggers on value change
  EIMSK  = 0x01;           				// enables interrupts on D2 (right encoder)
  
  // WDT config for ammendment

  sei();                  				// sets the interrupt enable on SREG
  
  PORTD |= 0x90;          				// enables the motor control
  Serial.begin(9600);
}

void loop(){

  PORTB |= 0x20;                        // send a 10 us pulse from the trigger pin D13
  _delay_us(10);
  PORTB &= 0xDF;                        // turns it off after 10 us

  unsigned long ticksBetween = 0;       // declare variables for the periods
  static unsigned long tHigh = 0;

  cli();
  if (captArr[1] > captArr[0]) {        // calculates the period high and stores it in tHigh
    ticksBetween = captArr[1] - captArr[0];
    tHigh = (ticksBetween << 4);        // bitshifting multiplies by 16 bc of I/O prescaler
  }
  sei();

  unsigned int ultraDistance = (tHigh * 17182L) / 1000000;     // gets the whole number of distance
  unsigned int avgCountWheel = average(leftWheel, rightWheel);      // average value of the toggles between both the wheels
  unsigned int wheelDistance = (avgCountWheel * 105L) / 100;

  if(ultraDistance <= 15)
    spinThatShit();
  
  switch (middle) {
    case (0):                   	  // go the fuck forwards
      OCR0A = 0;
      OCR0B = 195;
      OCR2A = 0;
      OCR2B = 150;
      break;
    case (1):                       // right whisker is hit
      OCR0A = 150;
      OCR0B = 0;
      OCR2A = 100;
      OCR2B = 0;
      break;
    case (2):                       // left whisker is hit
      OCR0A = 100;
      OCR0B = 0;
      OCR2A = 150;
      OCR2B = 0;
      break;
  }
  Serial.print(ultraDistance);
  Serial.print('\t');
  Serial.print(wheelDistance);
  Serial.print('\n');
}

void spinThatShit(){
  OCR0A = 150;
  OCR0B = 0;
  OCR2A = 0;
  OCR2B = 150;
}

unsigned int average(unsigned int leftWheel, unsigned int rightWheel){
  return ((leftWheel + rightWheel) / 2);
}

ISR(TIMER1_CAPT_vect){              	// INPUT CAPTURE INTERRUPT
  static unsigned char x = 0;       	// variable used to change between conditions
  sregValue = SREG;
  if (!x) {                         	// if x is not equal to its value
    captArr[0] = count + ICR1;
    TCCR1B &= 0xBF;                 	// sets the ICES1 bit and captures input on rising edge
  }
  else {
    captArr[1] = count + ICR1;
    TCCR1B |= 0x40;                		// clears the ICES1 bit and captures input on falling edge
  }
  x ^= 1;                           	// toggling x to change the condition of the statement
  SREG = sregValue;
}

ISR(TIMER1_OVF_vect){               	// keeps running total of the ticks elasped even after overflow
  count += 65536;
}

ISR(PCINT0_vect){                  		// LEFT WHEEL ENCODER INTERRUPT
  //sregValue = SREG;
  if ((PINB & 0x10) == 0)
    leftWheel++;
  //SREG = sregValue;
}

ISR(INT0_vect){                    		// RIGHT WHEEL ENCODER INTERRUPT
  //sregValue = SREG;
  if ((PIND & 0x04) == 0)
    rightWheel++;
  //SREG = sregValue;
}

ISR(PCINT1_vect){                   	// INTERRUPT FOR THE WHISKERS
  sregValue = SREG;
  if ((PINC & 0x30) == 0x20)        	// left hit
    middle = 2;
  else if ((PINC & 0x30) == 0x10)   	// right hit
    middle = 1;
  else                              	// nothing hit
    middle = 0;
  SREG = sregValue;
}
