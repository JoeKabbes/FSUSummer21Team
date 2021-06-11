//www.elegoo.com - OBSTACLE AVOIDANCE

#include <Servo.h>  //servo library
Servo myservo;      // create servo object to control servo

int Echo = A4;  
int Trig = A5; 

#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
#define SPEED 150
int rightDistance = 0, leftDistance = 0, middleDistance = 0;

void forward(int carSpeed){ //, int runTime){ 
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  //delay(runTime);
  Serial.println("Forward");
}

void back(int carSpeed, int runTime) {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(runTime);
  Serial.println("Back");
}

void left(int carSpeed, int runTime) {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  delay(runTime);
  Serial.println("Left");
}

void right(int carSpeed, int runTime) {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(runTime);
  Serial.println("Right");
}

void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  Serial.println("Stop!");
} 

//Ultrasonic distance measurement Sub function
int Distance_test(int degreeTurn) {
  myservo.write(degreeTurn);  //set servo position according to scaled value
  delay(150); 
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);   
  float Fdistance = pulseIn(Echo, HIGH);  
  Fdistance= Fdistance / 58;       
  return (int)Fdistance;
}  

void setup() { 
  myservo.attach(3);  // attach servo on pin 3 to servo object
  Serial.begin(9600);     
  pinMode(Echo, INPUT);    
  pinMode(Trig, OUTPUT);  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  stop();
} 

void loop() { 
    middleDistance = Distance_test(90);

    if(middleDistance <= 20) {     
      stop();
   
      rightDistance = Distance_test(0);
      leftDistance = Distance_test(180);
      if(rightDistance > leftDistance) {
        right(SPEED, 360);
      }
      else if(rightDistance < leftDistance) {
        left(SPEED, 360);
      }
      else if((rightDistance <= 20) || (leftDistance <= 20)) {
        back(SPEED, 180);
      }
      else {
        forward(SPEED);
      }
    }  
    else {
        forward(SPEED);
    }                     
}
