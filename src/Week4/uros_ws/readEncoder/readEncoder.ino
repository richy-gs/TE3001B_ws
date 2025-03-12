#define ENCODER_A 19
#define ENCODER_B 21

volatile long motorPosition = 0;

void updateMotorPosition(){
 if(digitalRead(ENCODER_B) != digitalRead(ENCODER_A)){
    motorPosition++;
 } else {
  motorPosition--;
 }
}

void setup() {
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateMotorPosition, CHANGE);
}

void loop() {
  Serial.println(motorPosition);

}


// 885
// 917
// 893
// 887

// PROM() = 895