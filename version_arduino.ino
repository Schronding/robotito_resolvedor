#include <Servo.h>

const int servoPin = 6; // It seems that the servo might create some noise in 
// the ultrasonic sensor, as it seems that it overstimates the distance by 
// about half to 1 cm. 
const int trigPin = 9;
const int echoPin = 10; 

float duration; // variable to store pulse duration
float distanceCM; // variable to store distance in CM 
float distanceIN; // variable to store distance in IN

Servo myServo; // create servo object to control the servo

void setup(){
  Serial.begin(9600); // it seems that this serves the purpose of allowing us
  // to check the distance for debugging purposes
  Serial.println("Setup completo. Iniciando loop...");
  pinMode(trigPin, OUTPUT); // the purpose of this is to send the signal, 
  // probably the PMW as a signal to send the ultrasonic wave
  pinMode(echoPin, INPUT); // this serves the opposite purpose, it needs to 
  // know where it receives the signals, so it acts as the input of the bounce
  // of the ultrasonic wave. I wonder if both could be reverted and still work
  // My intuition is that they should. 
  myServo.attach(servoPin); // attatches servo on servoPin to the servo object
}

void loop(){
  digitalWrite(trigPin, LOW); // As there are a lot of ultrasonic signals 
  // everywhere, the ultrasonic sends a very specific 8 pattern pulse, so it
  // knows when and when is not the signal the own sensor sent. This digital
  // write command starts with a clean signal. 
  delayMicroseconds(2); 
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10); // It seems that in some way, this longer delay 
  // (compared to the previous 2 microseconds) makes so the signal is actually
  // sent
  digitalWrite(trigPin, LOW); // return pulse duration in microseconds. If
  // set to HIGH, pulseIn() waits for the pin to go from LOW to HIGH, stops
  // timing when pin goes back to LOW
  duration = pulseIn(echoPin, HIGH); // convert m/s in to cm/s
  // As the speed of sound is 343 meter/second = 0.034 cm/microsecond
  distanceCM = (duration * 0.034) / 2; // The division by two is necessary
  // because for the signal to be sent and received, it has to bounce, so it 
  // effectively traveled twice the distance that the one we're interested in
  distanceIN = distanceCM / 2.54; 
  Serial.print("Distance: ");
  Serial.print(distanceCM); 
  Serial.print(" cm | ");
  Serial.print(distanceIN);
  Serial.println(" in");

  //if (distanceCM <= 5 && distanceCM > 0){
    //myServo.write(0); // This should turn the Servo head to the left to check
    // if there is any wall for us to travel by
    //delay(1000); 
//    myServo.write(90);
//    delay(500); 
//    myServo.write(180); // I don't know precisely how I would put the servo
    // to rotate both to left and right... I could calculate the distance from
    // both turns to decide if I am going to follow a BFS or DFS
//    delay(500);
  }
  delay(60); // This delay allows for the sensor to work properly, as the wait 
  // eliminates residual echos and other debris 

  // map distance in inches to servo position (degrees)
  int servoPos = map(distanceCM, 2, 30, 0, 180);
  // rotate servo according to distance
  myServo.write(servoPos);
}