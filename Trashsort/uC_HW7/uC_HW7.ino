/*
    Joseph Rondinelli H#7
    This was co-produced by Liam N.
*/

#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu; // declare a variable called mpu of datatype MPU6050
//unsigned long timeStampStartOfLoopMs = 0;
float timeStepS = 0.01;
float pitch,roll,yaw = 0.0f; // pitch, roll and yaw values
Vector normalizedGyroDPS; //stores the three gyroscope readings XYZ in degrees per second (DPS)
int yawTemp = 0;

unsigned long servoValue = 0;

void setup() {
  Serial.begin(115200);

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring.");
    delay(1000);
  }

  mpu.calibrateGyro();
  mpu.setThreshold(1);
  DDRB = 0x02; 
  TCCR1A=0xB0;  //10110000 COM1A clear on compare match, COM1B set on compare match
  TCCR1B=0x13;  //00010011 mode 9: phase and frequency correct, top = ICR1A, 64 prescaler
  ICR1=0x9C4;   //2500 
}

void loop() {
  normalizedGyroDPS = mpu.readNormalizeGyro(); // Read normalized values
  // Calculate Pitch, Roll and Yaw
  //pitch = pitch + normalizedGyroDPS.YAxis * timeStepS;
  //roll = roll + normalizedGyroDPS.XAxis * timeStepS;
  yaw = yaw + normalizedGyroDPS.ZAxis * timeStepS;
  Serial.print(yaw);  

  OCR1A = int(abs(188 + (yaw * 1.39))); // Linearization of yaw to PWM value range of .5ms to 2.5ms
  Serial.print(" , ");
  Serial.println(OCR1A);
  delay(10);
}

