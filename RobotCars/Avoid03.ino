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

#define BACKTIME 200

#define RIGHT 0
#define RIGHT45 1
#define FRONT 2
#define LEFT45 3
#define LEFT 4

#define STOPPED 0
#define STOPPING 1
#define FORWARD 2
#define BACK 3
#define BACKING 4
#define TURNLEFT 5
#define TURNLEFT45 6
#define TURNRIGHT 7
#define TURNRIGHT45 8
#define TURNING 9

int state = 0;

int rightDistance = 0, leftDistance = 0, middleDistance = 0;
int right45Distance = 0, left45Distance = 0;

int angles[] = {0, 45, 90, 135, 180};
int turnTimes[] = {360, 180, 0, 180, 360};
int distances[5];

bool stopCallback(void *argument) {
  stop();
  state = STOPPED;
  return false;
}

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
  timer.in(runTime, stopCallback);
  Serial.println("Back");
}

void left(int carSpeed, int runTime) {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  timer.in(runTime, stopCallback);
  Serial.println("Left");
}

void right(int carSpeed, int runTime) {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  timer.in(runTime, stopCallback);
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

void scan() {
  for(int i = 0; i < 5; i++) {
    distances[i] = Distance_test(angles[i], 400);
  }
}

int closest() {
  int lowvalue = 9999;
  for (int i = 0; i < 5; i++) {
    if (distances[i] != 0 && distances[i] < lowvalue)
      lowvalue = distances[i];
  }
  return lowvalue;
}

int closestDirection() {
  int lowvalue = 9999;
  int index;
  for (int i = 0; i < 5; i++) {
    if (distances[i] != 0 && distances[i] < lowvalue)
      lowvalue = distances[i];
      index = i;
  }
  return index;
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
  int scanDirection;
  timer.tick();
  scan();
  
  switch (state) {
    case STOPPED:
      timer.cancel();
      if ( closest() > 20 ) {
        state = FORWARD;
        forward(SPEED);
      } else
      {
        scanDirection = closestDirection();
        switch (scanDirection) {
          case RIGHT:
          case RIGHT45:
            left(SPEED, turnTimes[scanDirection]);
            state = TURNING;
            break;
          case LEFT:
          case LEFT45:
            right(SPEED, turnTimes[scanDirection]);
            state = TURNING;
            break;
          case FRONT:
            state = BACK;
            break;
        }
      }
      break;
      
    case STOPPING:
      break;
     
    case FORWARD:
      if ( closest() < 20 ) {
        stop();
        state = STOPPED;
      }
      break;
      
    case BACK:
      back(SPEED, BACKTIME);
      state = BACKING;
      break;
      
    case BACKING:
      break;

    case TURNING:
      break;
  }

  
                       
}
