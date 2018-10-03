//Lab7 Section 4
//Joseph Rondinelli
//4/6/17

#define dimBlue      0x00000F
#define dimRed       0x0F0000
#define dimGreen     0x000F00
#define dimRedBlue   0x0F000F
#define dimGreenBlue 0x000F0F
#define brightWhite  0x111111


enum directionState_t {STOP, FORWARD, STOP2, CLOCKWISE};
//void stopWheels(int rate), goForward(int rate) ,  goBackward(int rate), goClockwise(int rate), goCounterclockwise(int rate) ;

int state = STOP, prevState = -1;
int stateTimer = 0, rate = 120, timeStampStartOfLoopMs = 0;
boolean isNewState = true;


void setup() {
  Serial.begin(115200);
  Serial.println(F("Testing motor A and B using Timer0 in fast PWM mode 3."));
  Serial.println(F("Requires external 9V battery pack."));

  configureTimer0RegisterForPWMtoDriveMotor();


  DDRB |= 0b00011000;

}

//====================================================================================================

void configureTimer0RegisterForPWMtoDriveMotor() {
  TCCR0A = 0b10100011;
  //TCCR0B not needed since WGM02 is a default 0 and the prescaler is default as well
  OCR0A  = 0;
  OCR0B  = 0;

  DDRD  |= 0b01100000; //OCR0A and OCR0B pins set as outputs

  DDRB |= 0b00000111;
  DDRD |= 0b11100000;
  PORTD |= 0b00000000;
  PORTD &= 0b01111111;
  PORTB |= 0b00000101;
  PORTB &= 0b11111101;
}

//====================================================================================================
//****660 for left turn
void loop() {
  //timeStampStartOfLoopMs = millis(); // mark the time
  isNewState = (state != prevState);
  prevState = state;

  switch (state) {

    case STOP:
      if (isNewState) {
        Serial.println("STOP");
        stateTimer = 0;
        stopWheels(0);
      }
      stateTimer++;
      if (stateTimer >= 500)
      {
        state = FORWARD;
      }
      break;

    case FORWARD:
      if (isNewState) {
        stateTimer = 0;
        goForward(120);
        Serial.println("FOWARD");
      }
      stateTimer++;
      if (stateTimer >= 1500) {
        state = STOP2;
      }

      break;

     case STOP2:
      if (isNewState) {
        Serial.println("STOP2");
        stateTimer = 0;
        stopWheels(0);
      }
      stateTimer++;
      if (stateTimer >= 500)
      {
        state = CLOCKWISE;
      }
      break;

    case CLOCKWISE:
      if (isNewState) {
        stateTimer = 0;
        Serial.println("RIGHT");
        turnClockwise(90);
      }
      stateTimer++;
      if (stateTimer >= 1100)
      {
        state = STOP;
      }
      break;

    default: state = STOP;
  } // switch (state)

  // Wait until a full 10msec has passed before next reading
  delay(1); // - (millis() - timeStampStartOfLoopMs));

}

//====================================================================================================

void goForward(int rate) {
  PORTD |= 0b00000000;
  PORTD &= 0b01111111; //left forward  (PB0 = 1, PD7 = 0)
  PORTB |= 0b00000011; //right forward (PB1 = 1, PB2 = 0)
  PORTB &= 0b11111011;

  OCR0A  = rate+2; //set motor rates
  OCR0B  = rate;

  display_color_on_RGB_led(dimGreen);
}

void goBackward(int rate) {
  PORTD |= 0b10000000;
  PORTD &= 0b11111111; //left backward  (PB0 = 0, PD7 = 1)
  PORTB |= 0b00000100; //right backward (PB1 = 0, PB2 = 1)
  PORTB &= 0b11111100;

  OCR0A  = rate + 30; //set motor rates
  OCR0B  = rate;

  display_color_on_RGB_led(brightWhite);
}
void turnClockwise(int rate) {
  PORTD |= 0b10000000;
  PORTD &= 0b11111111; //left forward   (PB0 = 0, PD7 = 1)
  PORTB |= 0b00000010; //right backward (PB1 = 1, PB2 = 0)
  PORTB &= 0b11111010;

  OCR0A  = rate + 30; //set motor rates
  OCR0B  = rate;

  display_color_on_RGB_led(dimGreenBlue);
}
void turnCounterclockwise(int rate) {
  PORTD |= 0b00000000;
  PORTD &= 0b01111111; //left backward (PB0 = 1, PD7 = 0)
  PORTB |= 0b00000101; //right forward (PB1 = 0, PB2 = 1)
  PORTB &= 0b11111101;

  OCR0A  = rate + 30; //set motor rates
  OCR0B  = rate;

  display_color_on_RGB_led(dimRedBlue);
}
void stopWheels(int rate) {
  PORTD |= 0b00000000;
  PORTD &= 0b01111111; //left stop  (PB0 = 1, PD7 = 0)
  PORTB |= 0b00000011; //right stop (PB1 = 1, PB2 = 0)
  PORTB &= 0b11111011;

  OCR0A  = rate + 30; //set motor rates
  OCR0B  = rate;

  display_color_on_RGB_led(dimRed);
}

//====================================================================================================

void display_color_on_RGB_led(unsigned long color) {
  unsigned long bitmask = 0UL; // UL unsigned long literal (forces compiler to use long data type)
  unsigned long masked_color_result = 0UL;

  //digitalWrite(LED_CLOCK_PIN,LOW); //start with clock low.
  PORTB &= 0b11110111;

  for (int i = 23; i >= 0; i--) { // clock out one data bit at a time, starting with the MSB first
    bitmask = (1UL << i); // build bit mask. Note must use "1UL" unsigned long literal, not "1"
    masked_color_result = color & bitmask; // reveals just one bit of color at time
    boolean data_bit = !(masked_color_result == 0); // this is the bit of data to be clocked out.
    //digitalWrite(LED_DATA_PIN,data_bit);
    if (data_bit) {
      PORTB |= 0b00010000;
    }
    else {
      PORTB &= 0b11101111;
    }

    //digitalWrite(LED_CLOCK_PIN,HIGH);
    PORTB |= 0b00001000;
    //digitalWrite(LED_CLOCK_PIN,LOW);
    PORTB &= 0b11110111;

  }

  //digitalWrite(LED_CLOCK_PIN,HIGH);
  PORTB |= 0b00001000;
  //digitalWrite(LED_DATA_PIN,LOW);
  PORTB &= 0b11101111;
  delayMicroseconds(1); // after writing data to LED driver, must hold clock line
  // high for 1 ms to latch color data in led shift register.
  //digitalWrite(LED_CLOCK_PIN,LOW);
}//display_color_on_RGB_led()

