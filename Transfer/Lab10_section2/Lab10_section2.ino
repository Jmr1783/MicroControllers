// Lab10section2_testSR04.ino
// Clark Hochgraf
// April 24, 2017

// Description: Tests SR04 distance sensor

const int SR04_TRIG_PIN = 4; //pin which triggers ultrasonic sound
const int SR04_ECHO_PIN = 3;//pin which measures time to receive echo using pulseIn()
const int MaxDistanceTimeout = 9000; // Max distance = Timeout in usec/ 29 /2 = 9000/29/2=155 cm


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


void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
  Serial.println("I'm alive - SR04 distance sensor");
  pinMode(SR04_TRIG_PIN, OUTPUT);
  pinMode(SR04_ECHO_PIN, INPUT);
  Serial.println("done with setup");

}

void loop() {
  // put your main code here, to run repeatedly:
Serial.print("Distance reading in cm : ");
  Serial.println(distanceInCm());
  delay(100);

}
