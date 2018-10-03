//Lab8 Section 1
//Joseph Rondinelli
//4/6/17

#define dimBlue      0x00000F
#define dimRed       0x0F0000
#define dimGreen     0x000F00
#define dimRedBlue   0x0F000F
#define dimGreenBlue 0x000F0F
#define brightWhite  0x111111

int MotorABpwmOffset = 0, MotorCDpwmOffset = 0;

#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu; // declare a variable called mpu of datatype MPU6050
//unsigned long timeStampStartOfLoopMs = 0;
float timeStepS = 0.01;
float pitch, roll, yaw = 0.0f; // pitch, roll and yaw values
Vector normalizedGyroDPS; //stores the three gyroscope readings XYZ in degrees per second (DPS)
int tempYaw = 0;
void display_color_on_RGB_led(unsigned long color);
enum directionState_t {CalibratePWM, STOP, FORWARD, STOP2, CLOCKWISE};

int state = CalibratePWM, prevState = -1;
int stateTimer = 0, rate = 40, setMotorPWM = 0;
boolean isNewState = true;


void setup() {
  Serial.begin(115200);

  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring.");
    delay(1000);
  }


  DDRB |= 0b00011000;

  configureTimer0RegisterForPWMtoDriveMotor();


}

//====================================================================================================

void configureTimer0RegisterForPWMtoDriveMotor() {
  Serial.println("Starting Init");
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


  Serial.println("Finished");
  Serial.println("Starting Gyro Calibration");
  mpu.calibrateGyro(); // Calibrate gyroscope- must be done with sensor not moving.
  Serial.println("Finished");
  Serial.println("Setting Threshold");
  mpu.setThreshold(1);// sets level below which changes in gyro readings are ignored.
  // helps to reduce noise. 1 = one standard deviation. Range is 0 to 3.
  Serial.println("Finished");
}

//====================================================================================================
void getGyroStatus() {
  //Serial.println("gryo code");
  normalizedGyroDPS = mpu.readNormalizeGyro(); // Read normalized values
  // Calculate Pitch, Roll and Yaw
  pitch = pitch + normalizedGyroDPS.YAxis * timeStepS;
  roll = roll + normalizedGyroDPS.XAxis * timeStepS;
  yaw = yaw + normalizedGyroDPS.ZAxis * timeStepS;
  //  Serial.print(pitch);
  //  Serial.print(" ");
  //  Serial.print(roll);
  //  Serial.print(" ");
  Serial.println(yaw);
  // Wait until a full timeStepS has passed before next reading


}
//===========================================================================================
unsigned int getPWM(boolean motorSelect) {
  setMotorPWM = 0;
  normalizedGyroDPS = {0,0,0}; 

  if (motorSelect) { //right motor on
    PORTD |= 0b00000000;
    PORTD &= 0b01111111; //left stop  (PB0 = 0, PD7 = 0)
    PORTB |= 0b00000010; //right forward (PB1 = 1, PB2 = 0)
    PORTB &= 0b11111011;

    while (!(abs(normalizedGyroDPS.ZAxis) >= 2) && (setMotorPWM <= 255)) {
      normalizedGyroDPS = mpu.readNormalizeGyro(); // Read normalized values
      setMotorPWM++;
      OCR0A = setMotorPWM;
      delay(20);
    }


  }//Right Calb
  else { //left motor on
    PORTD |= 0b00000000;
    PORTD &= 0b01111111; //left forward  (PB0 = 1, PD7 = 0)
    PORTB |= 0b00000001; //right Stop (PB1 = 0, PB2 = 0)
    PORTB &= 0b11111011;

    while (!(abs(normalizedGyroDPS.ZAxis) >= 2) && (setMotorPWM <= 255)) {
      normalizedGyroDPS = mpu.readNormalizeGyro(); // Read normalized values
      setMotorPWM++;
      OCR0B = setMotorPWM;
      delay(20);
    }

  }//Left Calb

  if (setMotorPWM >= 255){
	  Serial.println("Error: Calibration Failed");
	  while(1){display_color_on_RGB_led(dimRed);delay(100);display_color_on_RGB_led(dimBlue);delay(100);}
	}
  
  OCR0A = 0;
  OCR0B = 0;
  return setMotorPWM;
}
//===========================================================================================
void loop() {
  //timeStampStartOfLoopMs = millis(); // mark the time
  isNewState = (state != prevState);
  prevState = state;

  getGyroStatus();
  
  switch (state) {

    case CalibratePWM:
      //entry housekeeping
      if (isNewState) {
        //find_motorstiction_using_Gyro();
        MotorABpwmOffset = getPWM(true); //Right Motor
        Serial.print("Motor Phase offset is "); Serial.println(MotorABpwmOffset);
        //display_color_on_RGB_led(DGREEN);
        delay(500);
        MotorCDpwmOffset = getPWM(false); //Left Motor
        Serial.print("Motor Phase offset is "); Serial.println(MotorCDpwmOffset);
        //display_color_on_RGB_led(MED_BLUE_COLOR);
        delay(500);
      }
      //state business

      //exit housekeeping
      state = STOP;
      break;


    case STOP:
      if (isNewState) {
        Serial.println("STOP");
        stateTimer = 0;
        stopWheels();
      }
      stateTimer++;
      if (stateTimer >= 50)
      {
        state = FORWARD;
      }
      break;
	 
	 
	
    case FORWARD:
      if (isNewState) {
		    tempYaw = yaw;
        Serial.println("FOWARD");
      }
        goForward(120,(3.0 * (tempYaw - yaw)));
        state = FORWARD;
      break;

    /* case STOP2:
      if (isNewState) {
        Serial.println("STOP2");
        stateTimer = 0;
        stopWheels();
        //mpu.calibrateGyro();
      }
      stateTimer++;
      if (stateTimer >= 50)
      {
        state = CLOCKWISE;
      }
      break;

    case CLOCKWISE:
      if (isNewState) {
        Serial.println("RIGHT");
        turnCounterclockwise(120);
        yawTemp = yaw;
      }

      if (yaw >= (yawTemp + 60)) {
        state = STOP;
      }
      break; */

    default: state = STOP;
  } // switch (state)

  // Wait until a full 10msec has passed before next reading
  delay(10); // - (millis() - timeStampStartOfLoopMs));

}

//====================================================================================================

void goForward(int rate, int steering) {
  PORTD |= 0b00000000;
  PORTD &= 0b01111111; //left forward  (PB0 = 1, PD7 = 0)
  PORTB |= 0b00000011; //right forward (PB1 = 1, PB2 = 0)
  PORTB &= 0b11111011;

  int A = MotorABpwmOffset + rate - steering;
  int B = MotorCDpwmOffset + rate + steering;
  
  if(A>255){
    A = 255;
  }
  else if(A<0){
    A = 0;
  }

  if(B>255){
    B = 255;
  }
  else if(B<0){
    B = 0;  
  }
  
  OCR0A = A; OCR0B =B;
  display_color_on_RGB_led(dimGreen);
  
}

void goBackward(int rate) {
  PORTD |= 0b10000000;
  PORTD &= 0b11111111; //left backward  (PB0 = 0, PD7 = 1)
  PORTB |= 0b00000100; //right backward (PB1 = 0, PB2 = 1)
  PORTB &= 0b11111100;

  OCR0A  = MotorABpwmOffset + rate; //set motor rates
  OCR0B  = MotorCDpwmOffset + rate;

  display_color_on_RGB_led(brightWhite);
}
void turnClockwise(int rate) {
  PORTD |= 0b10000000;
  PORTD &= 0b11111111; //left forward   (PB0 = 0, PD7 = 1)
  PORTB |= 0b00000010; //right backward (PB1 = 1, PB2 = 0)
  PORTB &= 0b11111010;

  OCR0A  = MotorABpwmOffset + rate; //set motor rates
  OCR0B  = MotorCDpwmOffset + rate;

  display_color_on_RGB_led(dimGreenBlue);
}
void turnCounterclockwise(int rate) {
  PORTD |= 0b00000000;
  PORTD &= 0b01111111; //left backward (PB0 = 1, PD7 = 0)
  PORTB |= 0b00000101; //right forward (PB1 = 0, PB2 = 1)
  PORTB &= 0b11111101;

  OCR0A  = MotorABpwmOffset + rate; //set motor rates
  OCR0B  = MotorCDpwmOffset + rate;

  display_color_on_RGB_led(dimRedBlue);
}
void stopWheels() {
  PORTD |= 0b00000000;
  PORTD &= 0b01111111; //left stop  (PB0 = 1, PD7 = 0)
  PORTB |= 0b00000011; //right stop (PB1 = 1, PB2 = 0)
  PORTB &= 0b11111011;

  OCR0A  = 0; //set motor rates
  OCR0B  = 0;

  display_color_on_RGB_led(dimRed);
}

//====================================================================================================

void display_color_on_RGB_led(unsigned long color){
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

