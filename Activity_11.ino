/*
  Activity 11: H-Bridge Motor Driver
  By: Carlos, Jon, Sebastian
  Written: March 28, 2024
  Edited: March 28, 2024
  I/O Pins
  A0: Potentiometer Wiper pin
  A1:
  A2:
  A3:
  A4:
  A5:
  D0:
  D1:
  D2:
  D3:
  D4:
  D5: 
  D6: PWM backward signal
  D7:
  D8:
  D9:
  D10:
  D11:
  D12:
  D13:
*/


void setup() {
  // output pin for the PWM signals and toggle switches
  DDRD = 0x40;

  // configure timer/counter register 1
  cli();
  TCCR0A = 0x81;  // Using phase-correct PWM, clear on A
  TCCR0B = 0x01;  // with prescale value of 1

  // ADIF disabled
  ADCSRA = 0xEF;
  ADCSRB = 0x00;
  ADMUX = 0x60;   // 8-bit precision mode, AVCC, analog A0
  sei();

}

void loop() {
}

ISR(ADC_vect)
{
  OCR0A = ((178L*ADCH)>>8) + 77;
}
