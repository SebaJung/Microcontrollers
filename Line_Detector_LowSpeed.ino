/*
  Barrier Demo With Whiskers and line sensors
  By: Carlos, Jon, Sebastian
  Written: March 22, 2024
  Edited: April 18, 2024
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
  ADCSRB = 0x00;                    // free-running mode
  ADMUX = 0x40;                     // AVCC mode, 10-bit precision, analog bit A0 originally

  PCICR = 0x01;                     // activates the pin change interrupt
  PCMSK0 = 0x10;                    // pins D12 and enables interrupts

  EICRA = 0x02;                     // pin D2 and triggers on falling edge
  EIMSK = 0x01;                     // enables interrupts on D2

  sei();                            // sets the interrupt enable flag on SREG

  Serial.begin(9600);               // serial monitor start up command w baud rate of 9600
}

volatile unsigned char button = 0;
volatile unsigned char middle = 0;
volatile unsigned int leftWheel = 0;
volatile unsigned int rightWheel = 0;
volatile unsigned int leftSensor = 0;
volatile unsigned int centerSensor = 0;
volatile unsigned int rightSensor = 0;


void loop() {
  PORTD |= 0x90;                    // enables the motor control

  unsigned int avgCount = average(leftWheel, rightWheel);    // average value of the toggles between both the wheels
  unsigned int distance = (avgCount * 105L) / 100;
  /*
    Serial.print(avgCount);
    Serial.print('\t');
    Serial.print(leftWheel);
    Serial.print('\t');
    Serial.print(rightWheel);
    Serial.print('\t');
    Serial.print(distance);
    Serial.print('\n');
  */

  //section below relating to line sensor:

  if ((centerSensor >= 800) && (((leftSensor < 700) || (rightSensor < 700)) || (distance < 1800))) {

    goForward();

    if ((leftSensor >= 700) && (rightSensor < 700))
      leftDetect();                   // Increase speed of left wheel(or reduce speed of right wheel)
    // to encourage the car to yaw right OCR0B = 200 or OCR2B = 100

    else if ((leftSensor < 700) && (rightSensor >= 700))
      rightDetect();                  // Increase speed of right wheel(or reduce speed of left wheel)
    // to encourage the car to yaw left OCR2B = 200 or OCR0B = 100
  }
  else if ((centerSensor < 800) && (distance >= 1800) && (leftSensor < 700) && (rightSensor < 700)) {
    OCR0B = 0;
    OCR2B = 0;
  }

  Serial.print(leftSensor);
  Serial.print('\t');
  Serial.print(centerSensor);
  Serial.print('\t');
  Serial.print(rightSensor);
  Serial.print('\t');
  Serial.print(distance);
  Serial.print('\n');
}

// Do we want to use a circular buffer?
unsigned int average(unsigned int leftWheel, unsigned int rightWheel) {
  return ((leftWheel + rightWheel) / 2);
}

void goForward() {                  // if nothing is hit...
  OCR0A = 0;
  OCR0B = 150;                      // sends the left motor forward
  OCR2A = 0;
  OCR2B = 150;                      // sends the right motor forward
}

void leftDetect() {             // if left sensor off tape...
  OCR0A = 0;
  OCR0B = 0;
  OCR2A = 0;                    // right reverse signal half speed of left
  OCR2B = 150;
}

void rightDetect() {            // if left bumper is hit...

  OCR0A = 0;                    //left reverse signal half speed of right
  OCR0B = 150;
  OCR2B = 0;
  OCR2A = 0;                  // right reverse signal
}

ISR(PCINT0_vect) {                // left wheel encoder
  if ((button = PINB & 0x10) == 0)
    leftWheel++;
}

ISR(INT0_vect) {                  // right wheel encoder
  if ((button = PIND & 0x04) == 0)
    rightWheel++;
}

ISR(ADC_vect) {
  /* Depending on which ADMUX value is set, it
      will store the value of that ADC register
      onto that sensor's variable
  */

  if (ADMUX == 0x40) {         // A0 left sensor
    leftSensor = ADC;
    ADMUX = 0x41;
  } else if (ADMUX == 0x41) {        // A1 center sensor
    centerSensor = ADC;
    ADMUX = 0x42;
  } else if (ADMUX == 0x42) {        // A2 right sensor
    rightSensor = ADC;
    ADMUX = 0x40;
  }


  ADCSRA |= 0x40;               // start a new conversion
}
