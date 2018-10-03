const int TRIGA_PIN = 2,ECHOA_PIN = 3,TRIGB_PIN = 4,ECHOB_PIN = 5, PIZO_PIN =6;
const int MaxDistanceTimeout = 9000;
const int sizeOfDistArray = 19; // for storing distance measurements
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  DDRD = 0b00000101;
  DDRB = 0b00000010;
  TCCR1A = 0b01000000;
  TCCR1B = 0b00001010;
  OCR1A = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  int dist = distanceInCm(TRIGA_PIN,ECHOA_PIN);
   if((dist < 60)&&(dist >= 40)){OCR1A=5000;}
   else if((dist < 40)&&(dist >= 30)){OCR1A=1000;}
   else if((dist < 30)&&(dist >= 20)){OCR1A=800;}
   else if((dist < 20)&&(dist >= 0)){OCR1A=500;}
   else{OCR1A=7000;}
   delay(10);
  Serial.println(dist);
  //Serial.println(distanceInCm(TRIGB_PIN,ECHOB_PIN));
}

int distanceInCm(int Trig,int Echo) {
  //sending the signal, starting with LOW for a clean signal
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(Trig, LOW);
  int distance = pulseIn(Echo, HIGH, MaxDistanceTimeout) / 29 / 2; 
  // timeout after 9000 microseconds (150 cm)
  if (distance == 0) distance = MaxDistanceTimeout / 29 / 2; 
  // if no echo, assume object is at farthest distance
  return distance;
  // if pulse
} //distanceInCm()


