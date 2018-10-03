//Joseph Rondinelli
//LAB Section 03
boolean Flag = false;
void setup() {
  Serial.begin(9600);
  DDRB = 0b00001000; //PB3
  TCCR2A = 0b01000010; //OCR2A, toggle, CTC
  TCCR2B = 0b00000111; //1:1024,
  OCR2A = 125;

  DDRD = 0b00010000; //PD4
  TCCR1A = 0b00000000; // CTC, No OCS PINS
  TCCR1B = 0b00001101; //1:1024, CTC
  OCR1A = 46875; //3sec CTC so 6sec period

  cli(); //disable global interrupts
  TIMSK1 = 0b00000010; //enable interrupt on TIMER1 COMPA
  sei(); //enable global interrupts

}

void loop() {
	//DO NOTHING
}

ISR(TIMER1_COMPA_vect) {
  //Serial.print("CompA ");
  Flag = !(Flag);
  //Serial.println(Flag);
  if (Flag) {
    PORTD |= 0x10; //Turn ON PD4
  }
  else {
    PORTD &= 0b11101111; //Turn OFF PD4
  }
}
