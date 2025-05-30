#include <Servo.h>


// -- Pines del sensor ultrasonico y servo -- //
const int servoPin = 6; // It seems that the servo might create some noise in 
// the ultrasonic sensor, as it seems that it overstimates the distance by 
// about half to 1 cm. 
const int trigPin = 9;
const int echoPin = 10; 

// -- Pines del controlador L298N -- // 
// Motor izquierdo
const int ENA = 3;
const int IN1 = 4;
const int IN2 = 5; 

// Motor derecho
const int ENB = 11;
const int IN3 = 12;
const int IN4 = 13; 

// Variables globales
float duration; // variable to store pulse duration
float distanceCM; // variable to store distance in CM 
float distanceIN; // variable to store distance in IN
Servo myServo; // create servo object to control the servo

// -- Constantes de movimiento --
const int VELOCIDAD_AVANCE = 150; // Velocidad para avanzar (0-255)
const int VELOCIDAD_GIRO = 120; // Velocidad para girar (0-255)
const int TIEMPO_GIRO_90_GRADOS = 750; // En milisegundos


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
  myServo.write(90); // Para que inicie mirando al frente

  // Configuracion del L298N como salidas
  pinMode(ENA, OUTPUT); 
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(ENB, OUTPUT); 
  pinMode(IN3, OUTPUT); 
  pinMode(IN4, OUTPUT); 

  //Serial.println("Probando motores...");
  //moverAtras(120); delay(1000);
  //detenerMotores(); delay(500);

  //moverAdelante(120); delay(1000);
  //detenerMotores(); delay(500);

  //girarIzquierda(120); delay(500); // Giro corto
  //detenerMotores(); delay(500);

  //girarDerecha(120); delay(500); // Giro corto
  //detenerMotores();
  //Serial.println("Prueba de motores finalizada.");
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
  delay(500);

  if (distanceCM <= 5 && distanceCM > 0){
    //myServo.write(0); // This should turn the Servo head to the left to check
    // if there is any wall for us to travel by
    //delay(1000); 
    myServo.write(90);
    delay(500); 
    myServo.write(180); // I don't know precisely how I would put the servo
    // to rotate both to left and right... I could calculate the distance from
    // both turns to decide if I am going to follow a BFS or DFS
    delay(500);
  }
  delay(60); // This delay allows for the sensor to work properly, as the wait 
  // eliminates residual echos and other debris 

  // map distance in inches to servo position (degrees)
  //int servoPos = map(distanceCM, 2, 30, 0, 180);
  // rotate servo according to distance
  //myServo.write(servoPos);
  girarIzquierda(120); 
}

void moverAtras(int velocidad){
  // Motor izquierdo adelante
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, velocidad);
  // Motor derecho adelante 
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, velocidad);
  Serial.println("Moviendo Adelante");
}

void moverAdelante(int velocidad) {
  // Motor Izquierdo Atrás
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, velocidad);
  // Motor Derecho Atrás
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, velocidad);
  Serial.println("Moviendo Atrás");
}

void girarIzquierda(int velocidad) {
  // Motor Izquierdo Adelante
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, velocidad);
  // Motor Derecho Atrás (para un giro sobre su eje)
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, velocidad);
  Serial.println("Girando Derecha");
}

void girarDerecha(int velocidad) {
  // Motor Izquierdo Atrás
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, velocidad);
  // Motor Derecho Adelante
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, velocidad);
  Serial.println("Girando Izquierda");
}

void detenerMotores() {
  // Motor Izquierdo Frenado
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW); 
  analogWrite(ENA, 0);   
  // Motor Derecho Frenado
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
  Serial.println("Motores Detenidos");
}

