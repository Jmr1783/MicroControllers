/* Lab6_I2C_gyro_serial_plotter.ino
 Written by: Clark Hochgraf, revised: Mar 18, 2017 
 Description: ++ PLEASE READ ALL COMMENTS ++
       Demonstrates calculation of roll, pitch and yaw angles using a gyroscope sensor. 
       The gyro outputs the rate of rotation in degrees per second.
       The gyro signal is the integrated (multipled by sample interval) to get degrees.
       Hardware: uses the MPU-6050 6-axis accelerometer and gyro
       Software: uses library for MPU6050 
*/
#include <Wire.h>
#include <MPU6050.h>

#define LED_CLOCK_PIN 11
#define LED_DATA_PIN 12
#define DIMBLUE_COLOR 0x00000F
#define DIMRED_COLOR 0x0F0000
#define DIMGREEN_COLOR 0x000F00


MPU6050 mpu; // declare a variable called mpu of datatype MPU6050
unsigned long timeStampStartOfLoopMs = 0;
float timeStepS = 0.01;
float pitch,roll,yaw = 0.0f; // pitch, roll and yaw values
Vector normalizedGyroDPS; //stores the three gyroscope readings XYZ in degrees per second (DPS)
//volatile boolean newDataFlag = false;
volatile bool newDataFlag=false;  // boolean flag to indicate that timer1 overflow has occurred
unsigned long startMicroseconds,elapsedMicroseconds;  // use for profiling time for certain tasks

//==============================================================================

void setup() {
  Serial.begin(115200);
  // Initialize MPU6050 to have full scale range of 2000 degrees per second
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring.");
    delay(1000);
  }

  DDRD  |= 0b01000000;
  DDRB  |= 0b00011110;
  PORTB &= 0b11100011;
  PORTB |= 0b00000010;
  

  TCCR0A = 0b10000011;
  TCCR0B = 0b00000001;
  OCR0A  = 130;


  TCCR1A = 0b00000010;
  TCCR1B = 0b00011100;
  ICR1   = 624;
  
  cli();
  TIMSK1 = 0b00000001;
  TIFR1  = 0b00000001;
  sei();
  
  mpu.calibrateGyro(); // Calibrate gyroscope- must be done with sensor not moving.
  mpu.setThreshold(1);// sets level below which changes in gyro readings are ignored.
                 // helps to reduce noise. 1 = one standard deviation. Range is 0 to 3.
} // setup

//==============================================================================
  void loop() {
  while (!newDataFlag) {}; // stay stuck here until new data arrives, then run loop
                            // this will occur every 10 millisecond
  startMicroseconds=micros(); // mark time at start of main loop code
  normalizedGyroDPS = mpu.readNormalizeGyro();
  // Calculate Pitch, Roll and Yaw
  pitch = pitch + normalizedGyroDPS.YAxis * timeStepS;
  roll = roll + normalizedGyroDPS.XAxis * timeStepS;
  yaw = yaw + normalizedGyroDPS.ZAxis * timeStepS;
  Serial.print(pitch);
  Serial.print(F(" "));
  Serial.print(roll);
  Serial.print(F(" "));
  Serial.println(yaw);


    display_color_on_RGB_led(DIMBLUE_COLOR); // default is dim blue color
    OCR0A=90; // set motor speed off
  if (yaw>5) {
    display_color_on_RGB_led(DIMRED_COLOR);
    OCR0A=150; // set motor speed slow
    digitalWrite(10, LOW);  //motor direction pins
    digitalWrite(9, HIGH);   //motor direction pins
  }
  if (yaw<-5) {
    display_color_on_RGB_led(DIMGREEN_COLOR);
    OCR0A=200; // set motor speed slow
    digitalWrite(10, HIGH);  //motor direction pins
    digitalWrite(9, LOW);   //motor direction pins
  }
  newDataFlag=false;
 } //loop


void display_color_on_RGB_led(unsigned long color) {
  unsigned long bitmask=0UL; // UL unsigned long literal (forces compiler to use long data type)
  unsigned long masked_color_result=0UL;
  
  //digitalWrite(LED_CLOCK_PIN,LOW); //start with clock low.
  PORTB &= 0b11110111;
 
  for(int i=23; i>=0; i--) { // clock out one data bit at a time, starting with the MSB first
    bitmask= (1UL<<i);    // build bit mask. Note must use "1UL" unsigned long literal, not "1"
    masked_color_result = color & bitmask; // reveals just one bit of color at time
    boolean data_bit=!(masked_color_result==0); // this is the bit of data to be clocked out.
    //digitalWrite(LED_DATA_PIN,data_bit);
    if(data_bit) {
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
  for(int i=23; i>=0; i--) { // clock out one data bit at a time, starting with the MSB first
    bitmask= (1UL<<i);    // build bit mask. Note must use "1UL" unsigned long literal, not "1"
    masked_color_result = color & bitmask; // reveals just one bit of color at time
    boolean data_bit=!(masked_color_result==0); // this is the bit of data to be clocked out.
    //digitalWrite(LED_DATA_PIN,data_bit);
    if(data_bit) {
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
  delayMicroseconds(501); // after writing data to LED driver, must hold clock line  
            // high for 1 ms to latch color data in led shift register.
  //digitalWrite(LED_CLOCK_PIN,LOW); 
}//display_color_on_RGB_led()

ISR (TIMER1_OVF_vect) {
  newDataFlag = true;
}


