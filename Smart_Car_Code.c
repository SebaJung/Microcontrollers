/*--------------------------
Smart Car Code
  By: Sebastian, Jon, Carlos 
  Written: 03 April, 2024
  I/O Pins
  A0:	Left Line Sensor
  A1:	Center Line Sensor
  A2:	Right Line Sensor
  A3:	Ultrasonic Trig
  A4:	
  A5:	
  D0:	
  D1:	
  D2:	Ultrasonic Trig
  D3:	Right Forward (RF) - OC2B
  D4:	Left Control (LE)
  D5:	Left Forward (LF) - OC0B
  D6:	Left Reverse (LR) - OC0A
  D7:	Right Control (RE)
  D8:	ICR
  D9:	
  D10: 	
  D11:	Right Reverse (RR) - OC2A
  D12: 	Left Encoder
  D13: 	Right Encoder
  
  1. Run motors forwards
  2. Run motors reverse
  3. Rotate clockwise
  4. Rotate counterclockwise
--------------------------*/

void setup(){
	// Phase correct frequency of 25kHz
	cli();
	TCCR0A = 0xA1;				// Phase correct, Max = Top
	TCCR0B = 0x01;				// N = 1
	TCCR2A = 0xA1;				// 
	TCCR2B = 0x01;				//
	sei();
	
	DDRD = 0xFC;				// Output: RE, LR, LF, LE, RF
	DDRB = 0x08;				// Output: RR
}

void loop(){
	
	
}
