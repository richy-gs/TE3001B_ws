#include <ESP32Servo.h>

Servo myservo; 
int pos = 0;   
int servoPin = 23;

void setup() {
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(50);    // standard 50 hz servo
	myservo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
  Serial.begin(9600);
}

void loop() {
  
  int sensorValue = analogRead(34);
  Serial.println(sensorValue);
  delay(10);  //

  int val = map(sensorValue, 0, 4095, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  // Serial.println(val);
  myservo.write(val);                  // set the servo position according to the scaled value
  delay(10);                          // wait for the servo to get there
}