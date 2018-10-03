// Lab10_section1_IRremoteYDR_r1.ino
// Clark Hochgraf
// April 24, 2017

// Description: Controls a YDR robot using an IR remote. 

#include <IRremote.h>  
#include <Wire.h>
#include <MPU6050.h>

/* IR remote control codes - define symbols UP / DOWN to corresponding IR codes 0xFF629D */
#define UP  0xFF629D
#define DOWN 0xFFA857
#define LEFT 0xFF22DD
#define RIGHT 0xFFC23D
#define CENTER 0xFF02FD
#define ONE 0xFF6897
#define TWO 0xFF9867
#define THREE 0xFFB04F
#define FOUR 0xFF30CF
#define REPEATCODE 0xFFFFFFFF
#define SHOWCODES false  // use for debug to show raw codes from IR remote

/*
  case 0xFF629D: Serial.println(" FORWARD"); break;
  case 0xFF22DD: Serial.println(" LEFT");    break;
  case 0xFF02FD: Serial.println(" -OK-");    break;
  case 0xFFC23D: Serial.println(" RIGHT");   break;
  case 0xFFA857: Serial.println(" REVERSE"); break;
  case 0xFF6897: Serial.println(" 1");    break;
  case 0xFF9867: Serial.println(" 2");    break;
  case 0xFFB04F: Serial.println(" 3");    break;
  case 0xFF30CF: Serial.println(" 4");    break;
  case 0xFF18E7: Serial.println(" 5");    break;
  case 0xFF7A85: Serial.println(" 6");    break;
  case 0xFF10EF: Serial.println(" 7");    break;
  case 0xFF38C7: Serial.println(" 8");    break;
  case 0xFF5AA5: Serial.println(" 9");    break;
  case 0xFF42BD: Serial.println(" *");    break;
  case 0xFF4AB5: Serial.println(" 0");    break;
  case 0xFF52AD: Serial.println(" #");    break;
  case 0xFFFFFFFF: Serial.println(" REPEAT");break;
*/
#define IR_RECV_PIN A3 // connect IR remote receiver to this pin

long previousIRcommand = 0;
IRrecv irrecv(IR_RECV_PIN);
decode_results IR_command_received; //create a variable named IR_command_received of type decode_results

// RGB Led data and clock pins on the arduino
#define LED_DATA_PIN 12
#define LED_CLOCK_PIN 11

/* define RGB LED colors */
#define DIM_WHITE_COLOR 0x1F1F1F
#define DIM_GREEN_COLOR 0x001F00
#define DIM_RED_COLOR 0x1F0000
#define DIM_BLUE_COLOR 0x00001F
#define DIM_RED_BLUE_COLOR 0x1F001F
#define DIM_RED_GREEN_COLOR 0x1F1F00
#define DIM_GREEN_BLUE_COLOR 0x001F1F
#define MED_BLUE_COLOR 0x0000FF

MPU6050 mpu; //create a variable named mpu of type MPU6050
#define MPU_INT_PIN 2  // pin is required for code to operate, but does not show up here in code.
#define MPU_SDA_PIN A4 // pin is required for code to operate, but does not show up here in code.
#define MPU_SCK_PIN A5 // pin is required for code to operate, but does not show up here in code.


//motor driver pins
const int AB_mtr_INA_PIN = 10;//10
const int AB_mtr_INB_PIN = 9;//9
const int CD_mtr_INC_PIN = 8;
const int CD_mtr_IND_PIN = 7;

const int PWM_AB_PIN = 6;
const int PWM_CD_PIN = 5;

const int SR04_TRIG_PIN = 4; //pin which triggers ultrasonic sound
const int SR04_ECHO_PIN = 3;//pin which measures time to receive echo using pulseIn()
const int MaxDistanceTimeout = 9000; // Max distance = Timeout in usec/ 29 /2 = 9000/29/2=155 cm

int MotorABpwmOffset = 0;
int MotorCDpwmOffset = 0;

int current_Angle = 0;

int distanceInCm() {
  //sending the signal, starting with LOW for a clean signal
  digitalWrite(SR04_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SR04_TRIG_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(SR04_TRIG_PIN, LOW);
  int distance = pulseIn(SR04_ECHO_PIN, HIGH, MaxDistanceTimeout) / 29 / 2; 
  // timeout after 9000 microseconds (150 cm)
  if (distance == 0) distance = MaxDistanceTimeout / 29 / 2; 
  // if no echo, assume object is at farthest distance
  return distance;
  // if pulse
} //distanceInCm()



// Timers
unsigned long timerInMsec = 0;
float timeStepS = 0.01;
float yaw = 0;
int robot_commanded_heading = 0;

enum directionState_type {FORWARD, BACKWARD, LEFT_TURN, RIGHT_TURN, STOPPED, CALIBRATE_MOTOR_STICTION, 
                          SCAN_FOR_NEAREST_OBJECT, TURN_TO_NEAREST_OBJECT, FOLLOW_AT_15_CM
                         }; // declare a new type of data (directionState_type), with enumerated values
directionState_type directionState = CALIBRATE_MOTOR_STICTION; // create a variable named directionState
directionState_type previousDirectionState = STOPPED;
boolean isNewState = true;



void setup() {
  Serial.begin(115200);
  Serial.println("I'm alive - Robot with IR remote control");

//  pinMode(IR_RECV_PIN, INPUT_PULLUP); 
//  irrecv.enableIRIn(); // Start the receiver
//  delay(1000);
//
//  initialize_MPU6050();
//  stripInit();
//  configureMotorPins();
//  display_color_on_RGB_led(DIM_BLUE_COLOR);
//  delay(1000);

  pinMode(SR04_TRIG_PIN, OUTPUT);
  pinMode(SR04_ECHO_PIN, INPUT);

  
  Serial.println("done with setup");
} //setup

//================================================================
void loop() {
    
  Serial.print("Distance reading in cm : ");
  Serial.println(distanceInCm());
  delay(100);

  
  timerInMsec = millis(); // Mark time at start of loop
  Vector normalizedGyroDPS = mpu.readNormalizeGyro(); // Read normalized values
  yaw = yaw + normalizedGyroDPS.ZAxis * timeStepS; // Calculate Yaw
  get_IR_command_if_any();
  
  isNewState = (directionState != previousDirectionState); // if state != prevState, then isNewState



  switch (directionState) {
    case FORWARD:
      //entry housekeeping
      if (isNewState) {
        robot_commanded_heading = yaw;
        Serial.print(F("new state is FORWARD, \tcommanded heading is "));
        Serial.println(robot_commanded_heading);
        display_color_on_RGB_led(DIM_GREEN_COLOR);
      }
      //state business
      robot_goes_forward_at_given_yaw_at_speed(robot_commanded_heading, 15);
      //exit housekeeping
      break;

    case LEFT_TURN:
      //entry housekeeping
      if (isNewState) {
        robot_commanded_heading = yaw;
        Serial.print(F("new state is LEFT_TURN, \tcommanded heading is ")); Serial.println(robot_commanded_heading);
        display_color_on_RGB_led(DIM_GREEN_BLUE_COLOR);
      }
      //state business
      robot_commanded_heading =+5;
      robot_turns_to_heading(robot_commanded_heading);
      //exit housekeeping


      break;

    case RIGHT_TURN:
    current_Angle = 0;
      //entry housekeeping
      if (isNewState) {
        robot_commanded_heading = yaw - current_Angle;
        Serial.print(F("new state is RIGHT_TURN,\tcommanded heading is "));
        Serial.println(robot_commanded_heading);
        display_color_on_RGB_led(DIM_RED_GREEN_COLOR);
      }
      //state business
        current_Angle =+ 1;
        robot_commanded_heading = yaw - current_Angle;
        robot_turns_to_heading(robot_commanded_heading);
      //exit housekeeping
      delay(10);
      break;

    case BACKWARD:
      //entry housekeeping
      if (isNewState) {
        robot_commanded_heading = yaw;
        Serial.print(F("new state is BACKWARD, \tcommanded heading is "));
        Serial.println(robot_commanded_heading);
        display_color_on_RGB_led(DIM_RED_BLUE_COLOR);
      }
      //state business
      robot_goes_backward_at_given_yaw_at_speed(robot_commanded_heading, 15);
      //exit housekeeping
      break;

    case STOPPED:
      //entry housekeeping
      if (isNewState) {
        robot_commanded_heading = yaw;
        Serial.print(F("new state is STOPPED, \tcommanded heading is ")); Serial.println(robot_commanded_heading);
        display_color_on_RGB_led(DIM_RED_COLOR);
      }
      //state business
      robot_stop();

      //exit housekeeping
      break;

    case CALIBRATE_MOTOR_STICTION:
      //entry housekeeping
      if (isNewState) {
        //find_motorstiction_using_Gyro();
        MotorABpwmOffset = find_motorstiction_using_gyro(AB_mtr_INA_PIN, AB_mtr_INB_PIN, PWM_AB_PIN);
        Serial.print("Motor Phase offset is "); Serial.println(MotorABpwmOffset);
        display_color_on_RGB_led(DIM_GREEN_COLOR);
        delay(500);
        MotorCDpwmOffset = find_motorstiction_using_gyro(CD_mtr_INC_PIN, CD_mtr_IND_PIN, PWM_CD_PIN);
        Serial.print("Motor Phase offset is "); Serial.println(MotorCDpwmOffset);
        display_color_on_RGB_led(MED_BLUE_COLOR);
        delay(500);
      }
      //state business

      //exit housekeeping
      directionState = STOPPED;
      break;
      
    case SCAN_FOR_NEAREST_OBJECT:
      //entry housekeeping
      if (isNewState) {
        robot_commanded_heading = yaw;
        Serial.print(F("new state is SCAN_FOR_NEAREST_OBJECT, \tcommanded heading is "));

      }
      //state business

      //exit housekeeping
      break;

    case TURN_TO_NEAREST_OBJECT:
      //entry housekeeping
      if (isNewState) {
        Serial.print("new state is TURN_TO_NEAREST_OBJECT, \tcommanded heading is ");
        display_color_on_RGB_led(DIM_RED_BLUE_COLOR);
      }
      //state business
      //exit housekeeping
      break;

    case FOLLOW_AT_15_CM:
      //entry housekeeping
      if (isNewState) {
        Serial.print(F("new state is FOLLOW_AT_15_CM, \tcommanded heading is "));
        display_color_on_RGB_led(DIM_GREEN_COLOR);
      }
      //state business
       //exit housekeeping
      break;

    default:
      break;

  }//switch

  previousDirectionState =  directionState;

  // Wait to full timeStepS period, note that we have to check that any state has not lasted too long
  // or else the delay will be negative, which will cause bad things to happen.
  // using "delay((timeStepS * 1000) - (millis() - timerInMsec));" without checking the sign will cause tears.
  long safeDelay = (timeStepS * 1000) - (millis() - timerInMsec);
  if (safeDelay > 0) delay(safeDelay); else Serial.println("Main Loop Time overflow - inaccurate calcs");

} // loop()

//================================================================

void robot_turns_to_heading(float yaw_heading) {
  if (yaw < (yaw_heading - 2)) {
    turn_counterclockwise(1 + 0.15 * ((yaw_heading + 2) - yaw));
  }
  else if (yaw > (yaw_heading + 2)) {
    turn_clockwise(1 + 0.15 * (yaw - (yaw_heading + 2)));
  }
  else {
    robot_stop();
  }
}
//================================================================

void get_IR_command_if_any()
{
  if (irrecv.decode(&IR_command_received)) { // takes up to 6 ms to run

    if (SHOWCODES) Serial.println(IR_command_received.value, HEX);
    if (IR_command_received.value == REPEATCODE) { // key held down, repeat last command
      IR_command_received.value = previousIRcommand;
      if (SHOWCODES) {
        Serial.print("repeating command IR_command_received.value ");
        Serial.println(IR_command_received.value, HEX);
      }
    }
    if (IR_command_received.value == LEFT)     directionState = LEFT_TURN;
    if (IR_command_received.value == CENTER)   directionState = STOPPED;
    if (IR_command_received.value == RIGHT)    directionState = RIGHT_TURN;
    if (IR_command_received.value == UP)       directionState = FORWARD;
    if (IR_command_received.value == DOWN)     directionState = BACKWARD;
    if (IR_command_received.value == ONE)      directionState = SCAN_FOR_NEAREST_OBJECT;
    if (IR_command_received.value == TWO)      directionState = TURN_TO_NEAREST_OBJECT;
    if (IR_command_received.value == THREE)    directionState = CALIBRATE_MOTOR_STICTION;

    previousIRcommand = IR_command_received.value;
    //Serial.println(directionState);
    irrecv.resume(); // Receive the next value
    //Serial.print("elapsed time "); Serial.println(millis()-starttime);
  }
}
//================================================================

void robot_goes_forward_at_given_yaw_at_speed(float yaw_heading, int rate) {
  go_forward(rate, 3.0 * (yaw_heading - yaw)); // rate, steering amount
  // change the gain term (3.0) if robot moves too quickly or slowly.
}

//================================================================
void robot_goes_backward_at_given_yaw_at_speed(float yaw_heading, int rate) {
  go_backward(rate, 3.0 * (yaw_heading - yaw)); // rate, steering amount
  // change the gain term (3.0) if robot moves too quickly or slowly.
}

//==============================================================
void turn_clockwise(int rate) {
  digitalWrite(AB_mtr_INA_PIN, HIGH);
  digitalWrite(AB_mtr_INB_PIN, LOW);
  digitalWrite(CD_mtr_INC_PIN, LOW);
  digitalWrite(CD_mtr_IND_PIN, HIGH);
  analogWrite(PWM_AB_PIN, constrain(MotorABpwmOffset + rate, 0, 255));
  analogWrite(PWM_CD_PIN, constrain(MotorCDpwmOffset + rate, 0, 255));
}

//==============================================================
void go_forward(int rate, int steering) {
  digitalWrite(AB_mtr_INA_PIN, HIGH);
  digitalWrite(AB_mtr_INB_PIN, LOW);
  digitalWrite(CD_mtr_INC_PIN, HIGH);
  digitalWrite(CD_mtr_IND_PIN, LOW);
  analogWrite(PWM_AB_PIN, constrain(MotorABpwmOffset + rate - steering, 0, 255));
  analogWrite(PWM_CD_PIN, constrain(MotorCDpwmOffset + rate + steering, 0, 255));
}

//==============================================================
void go_backward(int rate, int steering) {
  digitalWrite(AB_mtr_INA_PIN, LOW);
  digitalWrite(AB_mtr_INB_PIN, HIGH);
  digitalWrite(CD_mtr_INC_PIN, LOW);
  digitalWrite(CD_mtr_IND_PIN, HIGH);
  analogWrite(PWM_AB_PIN, constrain(MotorABpwmOffset + rate + steering, 0, 255));
  analogWrite(PWM_CD_PIN, constrain(MotorCDpwmOffset + rate - steering, 0, 255));
}

//==============================================================
void turn_counterclockwise(int rate) {
  digitalWrite(AB_mtr_INA_PIN, LOW);
  digitalWrite(AB_mtr_INB_PIN, HIGH);
  digitalWrite(CD_mtr_INC_PIN, HIGH);
  digitalWrite(CD_mtr_IND_PIN, LOW);
  analogWrite(PWM_AB_PIN, constrain(MotorABpwmOffset + rate, 0, 255));
  analogWrite(PWM_CD_PIN, constrain(MotorCDpwmOffset + rate, 0, 255));
}
//==============================================================
void robot_stop() {
  digitalWrite(AB_mtr_INA_PIN, LOW);
  digitalWrite(AB_mtr_INB_PIN, LOW);
  digitalWrite(CD_mtr_INC_PIN, LOW);
  digitalWrite(CD_mtr_IND_PIN, LOW);
  analogWrite(PWM_AB_PIN, 0);
  analogWrite(PWM_CD_PIN, 0);
}

//==============================================================
void initialize_MPU6050() {
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro(); // Calibrate gyroscope. The calibration must be at rest.
  mpu.setThreshold(3); // Set threshold sensivity. Default 3. To disable, set to 0.
}

//==============================================================
int find_motorstiction_using_gyro(int phase1Pin, int phase2Pin, int EnPWMPin) {
  // ramp up PWM command for motor until rotation begins, then record
  // the required PWM value as a stiction offset.

  boolean calibrating_motor_stiction = true;
  int pwm_value_sent_to_motor = 50;
  while (calibrating_motor_stiction) {
    pwm_value_sent_to_motor++;
    digitalWrite(phase1Pin, HIGH);
    digitalWrite(phase2Pin, LOW);
    analogWrite(EnPWMPin, constrain(pwm_value_sent_to_motor, 0, 255)); // send pwm value to motor
    Vector normalizedGyroDPS = mpu.readNormalizeGyro();
    Serial.print(F("Rotation speed = ")); Serial.print(normalizedGyroDPS.ZAxis);
    Serial.print(F("  PWM value = ")); Serial.println(pwm_value_sent_to_motor);
    delay(20); //was 5
    if (abs(normalizedGyroDPS.ZAxis) > 2) calibrating_motor_stiction = false; // if robot moves stop calibration
    if (pwm_value_sent_to_motor > 250) //failed calibration
    { Serial.print(F("Calibration failed. Check if battery is connected/switched on."));
      analogWrite(EnPWMPin, 0); // send pwm value to motor
      while (1) {
        display_color_on_RGB_led(DIM_RED_COLOR);
        delay(100);
        display_color_on_RGB_led(DIM_BLUE_COLOR);
        delay(100);
      }
    }
  }
  analogWrite(EnPWMPin, 0); // stop motor
  delay(1000);
  return pwm_value_sent_to_motor-2;
}

//==============================================================
// Initialize the pins used to control the WS2801 driver
void stripInit(void) {
  pinMode(LED_DATA_PIN, OUTPUT);
  pinMode(LED_CLOCK_PIN, OUTPUT);
  digitalWrite(LED_DATA_PIN, LOW);
  digitalWrite(LED_CLOCK_PIN, LOW);
}

//==============================================================
void configureMotorPins() {
  // configure motor pins
  pinMode(AB_mtr_INA_PIN, OUTPUT);
  pinMode(AB_mtr_INB_PIN, OUTPUT);
  pinMode(CD_mtr_INC_PIN, OUTPUT);
  pinMode(CD_mtr_IND_PIN, OUTPUT);
  pinMode(PWM_AB_PIN, OUTPUT);
  pinMode(PWM_CD_PIN, OUTPUT);
}
// =======================================================================
void display_color_on_RGB_led(unsigned long color) {
  unsigned long bitmask = 0UL; // UL unsigned long literal (forces compiler to use long data type)
  unsigned long masked_color_result = 0UL;
  digitalWrite(LED_CLOCK_PIN, LOW); //start with clock low.
  for (int i = 23; i >= 0; i--) { // clock out one data bit at a time, starting with the MSB first
    bitmask = (1UL << i); // build bit mask. Note must use "1UL" unsigned long literal, not "1"
    masked_color_result = color & bitmask; // reveals just one bit of color at time
    boolean data_bit = !(masked_color_result == 0); // this is the bit of data to be clocked out.
    digitalWrite(LED_DATA_PIN, data_bit);
    digitalWrite(LED_CLOCK_PIN, HIGH);
    digitalWrite(LED_CLOCK_PIN, LOW);
  }
  digitalWrite(LED_CLOCK_PIN, HIGH);
  delayMicroseconds(501); // after writing data to LED driver, must hold clock line
  // high for 500 microseconds to latch color data in led shift register.
} //display_color_on_RGB_led()

