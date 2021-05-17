//FSU JPL Summer Team - Object Avoidance 

#include <Servo.h>  //servo library
Servo myservo;      // create servo object to control servo
#include <arduino-timer.h>  // Timer library

Timer<> timer;  // create TImer object

int Echo = A4;  
int Trig = A5; 

#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
#define SPEED 150
#define SLOWSPEED 95
#define SERVODELAY 500

#define RIGHT 0
#define RIGHT45 1
#define FRONT 2
#define LEFT45 3
#define LEFT 4

int rightDistance = 0, leftDistance = 0, middleDistance = 0;
int right45Distance = 0, left45Distance = 0;

int angles[] = {0, 45, 90, 135, 180};
int turnTimes[] = {360, 180, 0, 180, 360};
int distances[5];

void forward(int carSpeed){ 
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
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
int Distance_test(int degreeTurn) {
  return Distance_test(degreeTurn, 350);
}

//Ultrasonic distance measurement Sub function
int Distance_test(int degreeTurn, int delayTime) {
  myservo.write(degreeTurn);  //set servo position according to scaled value
  delay(delayTime); 
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);   
  float Fdistance = pulseIn(Echo, HIGH);  
  Fdistance= Fdistance / 58;       
  return (int)Fdistance;
}  

void setServo(int degreeTurn, int delayTime) {
  myservo.write(degreeTurn);  //set servo position according to scaled value
  delay(delayTime);
}

bool stopCallback() {
  stop();
  return false;
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
  timer.tick();
  
    rightDistance = Distance_test(0, SERVODELAY);
    right45Distance = Distance_test(45, SERVODELAY);
    middleDistance = Distance_test(90, SERVODELAY);
    left45Distance = Distance_test(135, SERVODELAY);
    leftDistance = Distance_test(180, SERVODELAY);

    if(middleDistance <= 20 || rightDistance <= 20 || right45Distance <= 20 || left45Distance <= 20 || leftDistance <= 20) {     
      stop();
   
      if(rightDistance > leftDistance) {
        right(SPEED, 360);
      }
      else if(rightDistance < leftDistance) {
        left(SPEED, 360);
      }
      else if((rightDistance <= 20) || (leftDistance <= 20)) {
        back(SLOWSPEED, 180);
      }
      else {
        forward(SPEED);
      }
    }  
    else {
        forward(SPEED);
    }                     
}
