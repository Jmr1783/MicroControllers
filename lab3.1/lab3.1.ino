#define SW1_PIN 5
#define LED_CLOCK_PIN 11
#define LED_DATA_PIN 12
#define QTR_SIG_PIN A3
#define QTR_5V_PIN A4
#define QTR_GND_PIN A5
#define MSEC_SAMPLE 1

enum {LED_OFF, BLINK_GON, BLINK_GOFF, BLINK_R, BLINK_GR, BLINK_RATE};

boolean isSwPressed, prevIsSwPressed, isSwJustReleased, isSwJustPressed, isSwChange;
int state = LED_OFF, prevState = !state;
int stateTimer, adcQTR;
boolean isNewState;

void setup() {
  pinMode(SW1_PIN, INPUT_PULLUP);//  pinMode(SW1_PIN, INPUT); won't work

  pinMode(LED_CLOCK_PIN, OUTPUT);  digitalWrite(LED_CLOCK_PIN, LOW);
  pinMode(LED_DATA_PIN, OUTPUT);  digitalWrite(LED_DATA_PIN, LOW);

  pinMode(QTR_SIG_PIN, INPUT);
  pinMode(QTR_5V_PIN, OUTPUT);   digitalWrite(QTR_5V_PIN, HIGH);
  pinMode(QTR_GND_PIN, OUTPUT);  digitalWrite(QTR_GND_PIN, LOW);
   
  Serial.begin(9600);
  Serial.println(F("Lab 2 Complex State Machine"));
} // setup()

// ADD loop() HERE – IT IS GIVEN ON THE NEXT PAGE

//****************************************************************************
void redOn(void) {
  PORTB = PORTB |(1 << 0);  // sets Uno dig_8, PORTB.0, pin to 1 (HIGH)
  display_color_on_RGB_led(0xFF0000);
}

//****************************************************************************
void greenOn(void) {

  unsigned long start_time_microseconds,end_time_microseconds;
  start_time_microseconds=micros();
  
  display_color_on_RGB_led(0x00FF00);

  end_time_microseconds=micros();
  Serial.print("Displaying the green color took ");
  Serial.print(end_time_microseconds-start_time_microseconds);
  Serial.println(" microseconds ");

}

//****************************************************************************
void allOff(void) {
  display_color_on_RGB_led(0x000000);
}

void loop() {
  prevIsSwPressed = isSwPressed;
  isSwPressed = !digitalRead(SW1_PIN);
  isSwJustPressed = (isSwPressed && !prevIsSwPressed);  // switch edge detection
  isSwJustReleased = (!isSwPressed && prevIsSwPressed);    
  isSwChange = (isSwJustReleased || isSwJustPressed);
  
  isNewState = (state != prevState);
  prevState = state;
  
  switch (state) {
 
    case LED_OFF:
      if (isNewState) Serial.println("LED_OFF");
      allOff();
      if (isSwJustReleased) state = BLINK_G;
    break;
    
    case BLINK_GON:
      if (isNewState) {
      stateTimer = 0;
      Serial.println("BLINK_GON");
      }
      stateTimer++;
      if (stateTimer < 250) greenOn();
      else {state = BLINK_GOFF;} 
      //else allOff();
      //if (stateTimer >= 1000) stateTimer = 0;
        //state = BLINK_GOFF; 
      }
    break;

     case BLINK_GOFF:
      if (isNewState) {
      stateTimer = 0;
      Serial.println("BLINK_GOFF");
      }
      stateTimer++;
      allOff();
      //if (stateTimer < 250) greenOn();
      //else allOff();
      if (stateTimer >= 1000) stateTimer = 0;
      if (isSwJustReleased) {
        allOff();
        state = BLINK_R; 
      }
    break;

  // ADD BLINK_R STATE HERE. ************
      case BLINK_R:
      if (isNewState) {
      stateTimer = 0;
      Serial.println("BLINK_R");
      }
      stateTimer++;
      if (stateTimer < 250) redOn();
      else allOff();
      if (stateTimer >= 1000) stateTimer = 0;
      if (isSwJustReleased) {
        allOff();
        state = BLINK_GR; }

        break;


    case BLINK_GR:
      if (isNewState) {
        stateTimer = 0;
        Serial.println("BLINK_GR");
      }
      stateTimer++;
      if (stateTimer < 500) redOn();
      else greenOn();
      if (stateTimer >= 1000) stateTimer = 0;
      if (isSwJustReleased) {
        allOff();
        state = BLINK_RATE;
      }
    break;

    case BLINK_RATE:
      if (isNewState) {
        stateTimer = 0;
        Serial.println("BLINK_RATE");
      }
      stateTimer++;
      adcQTR = analogRead(QTR_SIG_PIN);
      if (stateTimer < adcQTR/2) redOn();
      else greenOn();
      if (stateTimer >= adcQTR) stateTimer = 0;
      if (isSwJustReleased) {
        allOff();
        state = LED_OFF;
      }
    break; 
      
    default: state = LED_OFF;
  } // switch (state)

  delay(MSEC_SAMPLE);

  
} // loop()

//-----------------------------------------
void display_color_on_RGB_led(unsigned long color) {
  unsigned long bitmask=0UL; // UL unsigned long literal (forces compiler to use long data type)
  unsigned long masked_color_result=0UL;

  PORTB &= 0b11110111;
  
  //digitalWrite(LED_CLOCK_PIN,LOW); //start with clock low.
  
  for(int i=23; i>=0; i--) { // clock out one data bit at a time, starting with the MSB first
    bitmask= (1UL<<i);    // build bit mask. Note must use "1UL" unsigned long literal, not "1"
    masked_color_result = color & bitmask; // reveals just one bit of color at time
    boolean data_bit=!(masked_color_result==0); // this is the bit of data to be clocked out.

    if (data_bit == 1){
      PORTB |= ( data_bit << 4);
    }
    else{
      PORTB &= (data_bit << 4);
    }
    
  
    //digitalWrite(LED_DATA_PIN,data_bit);

    
    
    PORTB |= 0b00001000;
    //digitalWrite(LED_CLOCK_PIN,HIGH);  

    PORTB &= 0b11110111;

    //digitalWrite(LED_CLOCK_PIN,LOW);  
  }
  PORTB |= 0b00001000;
  //digitalWrite(LED_CLOCK_PIN,HIGH);  
  delay(1); // after writing data to LED driver, must hold clock line  
            // high for 1 ms to latch color data in led shift register.
}//display_color_on_RGB_led()

