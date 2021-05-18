//FSU JPL Summer Team - Object Avoidance 
#define MAXDISTANCE 150
#define MINDISTANCE 30

#include <NewPing.h>
NewPing sonar(A5, A4, MAXDISTANCE);  // sonar object

#include <Servo.h>  //servo library
Servo myservo;      // create servo object to control servo
#include <arduino-timer.h>  // Timer library

Timer<> timer;  // create TImer object for movement
Timer<> ledBlinker;  // Timer object for LED

int Echo = A4;  
int Trig = A5; 

#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
#define SPEED 150
#define SLOWSPEED 105
#define SERVODELAY 400

#define BACKTIME 500

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
#define TURNSTOP 10
#define STARTFORWARD 11
#define BACKSTOP 12

int turnIndex = 2;

int state = STOPPED;
bool wasBacking = false;

int angles[] = {0, 45, 90, 135, 180};
int turnTimes[] = {600, 400, 0, 400, 600};
int distances[5];
int scanCount = 0;
int LEDcount = 0;

//  Callback for forward movement
bool stopCallback(void *argument) {
  stop();
  state = STOPPED;
  return false;
}

// Callback for turn movement
bool turnCallback(void *argument) {
  stop();
  state = TURNSTOP;
  return false;
}

// Callback for back movement
bool backCallback(void *argument) {
  stop();
  state = BACKSTOP;
  return false;
}

// Callback for blink LED debug
bool toggle_led(void *) {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // toggle the LED
  LEDcount++;
  if (LEDcount >= 20 ) return false;
  return true; // repeat? true
}

// Set forward movement speed
// IN: carSpeed (PWM value)
// RETURN void
//
void forwardSpeed(int carSpeed) {
  analogWrite(ENA, carSpeed+5);
  analogWrite(ENB, carSpeed);
}

// Start forward movement
// IN:  carSpeed PWM
//      MS motor run time
// RETURN void
//
void forward(int carSpeed, int runTime){ 
  analogWrite(ENA, carSpeed+5);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  timer.in(runTime, stopCallback);
  //Serial.println("Forward");
}

// Start back movement
// IN:  carSpeed PWM
//      MS motor run time
// RETURN void
//
void back(int carSpeed, int runTime) {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  timer.in(runTime, backCallback);
  //Serial.println("Back");
}

// Start left movement
// IN:  carSpeed PWM
//      MS motor run time
// RETURN void
//
void left(int carSpeed, int runTime) {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  timer.in(runTime, turnCallback);
  //Serial.println("Left");
}

// Start right movement
// IN:  carSpeed PWM
//      MS motor run time
// RETURN void
//
void right(int carSpeed, int runTime) {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  timer.in(runTime, turnCallback);
  //Serial.println("Right");
}

// Stop car motion
// IN: none
// RETURN void
//
void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  //Serial.println("Stop!");
} 

//  Get distance at given angle with default servo delay
//  IN: degree for servo 0-180
//  Override of Distanst_test( degree, delay)
//  RETURN  distance
//
int Distance_test(int degreeTurn) {
  return Distance_test(degreeTurn, SERVODELAY);
}

//  Ultrasonic distance measurement
//  IN: degrees
//      servo delay
//  RETURN  distance
//
int Distance_test(int degreeTurn, int delayTime) {
  setServo(degreeTurn, delayTime);  //set servo position according to scaled value
  return sonar.convert_cm(sonar.ping_median());  
}  

//  Compute travel delay time in MS
//  IN: power (PWM)
//      cm distance
//  RETURN: ms delaytime
//
int delayTime(int power, int cm) {
  if (cm == 0) cm = MAXDISTANCE;
  return (int)((float)cm *10 / (0.0028*(float)power + 0.00595));
}

//  Set servo position
//  IN: degrees
//      servo delay
//  RETURN void
//
void setServo(int degreeTurn, int delayTime) {
  myservo.write(degreeTurn);  //set servo position according to scaled value
  delay(delayTime);
}

// Scan distances
//  IN: right limit
//      left limit
//      servo delay
//  RETURN void (sets distances array)
//
void scan(int right, int left, int delayTime) {
  int distance;
  for(int i = 0; i <= 4; i++) {
    distances[i] = MAXDISTANCE;
  }
  for(int i = right; i <= left; i++) {
    distance = Distance_test(angles[i], delayTime);
    if (distance > 0 ) {
      distances[i] =  distance;
    } else{
      distances[i] = MAXDISTANCE;
    }
  }
}

// Return furthest distance
// IN:  Right limit
//      Left limit
// RETURN:  furthest distance in CM
//
int farthest(int right, int left) {
  int highvalue = 0;
  for (int i = right; i <= left; i++) {
    //Serial.println(distances[i]);
    if (distances[i] > 0 && distances[i] < highvalue)
      highvalue = distances[i];
  }
  if (highvalue == 0) return MAXDISTANCE;
  return highvalue;
}

// Return direction index for furthest distance
// IN:  Right limit
//      Left limit
// RETURN:  index of furthest direction R to L
//
int farthestDirection(int right, int left) {
  int highvalue = 0;
  int index = 9;
  for (int i = right; i <= left; i++) {
    //Serial.println(distances[i]);
    if (distances[i] > 0 && distances[i] < highvalue) {
      highvalue = distances[i];
      index = i;
    }
  }
  return index;
}

// Get closest distance
// IN: none
// Override closest(R,L)
// RETURN: closest distance
//
int closest() {
  return closest(RIGHT, LEFT);
}

// Return closest distance
// IN:  Right limit
//      Left limit
// RETURN:  closest distance in CM
//
int closest(int right, int left) {
  int lowvalue = MAXDISTANCE;
  for (int i = right; i <= left; i++) {
    //Serial.println(distances[i]);
    if (distances[i] > 0 && distances[i] < lowvalue)
      lowvalue = distances[i];
  }
  return lowvalue;
}

// Return direction index for closest distance
// IN:  none (scan full range R-L
//  Override for closestDirection (right, left)
// RETURN:  index of closest direction R to L
//
int closestDirection() {
  return closestDirection(RIGHT, LEFT);
}

// Return direction index for closest distance
// IN:  Right limit
//      Left limit
// RETURN:  index of closest direction R to L
//
int closestDirection(int right, int left) {
  int lowvalue = MAXDISTANCE;
  int index = 9;
  for (int i = right; i <= left; i++) {
    if (distances[i] < lowvalue)
      lowvalue = distances[i];
      index = i;
  }
  return index;
}

// Blink onboard LED
// Use ledBlinker timer to actuate
//
void blinkLED() { 
  LEDcount = 0;
  ledBlinker.cancel();
  ledBlinker.in(100, toggle_led);
}

// Arduino SETUP logic
//
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
int lastState = STOPPED;

// Arduino LOOP
//
void loop() { 
  char buffer[80];
  int scanDirection;
  int delayMS;
  int distance;
  timer.tick();
   
  switch (state) {
    
    // Robot is STOPPED
    case STOPPED:
      timer.cancel();
      Serial.println("Stopped");
      scan(RIGHT, LEFT, SERVODELAY);
      //blinkLED();
      distance = closest();
      Serial.println(distance);
      if (distance < 20) {
        state = BACK;
        break;
      }
      
      if ( distances[FRONT] > MINDISTANCE ) { 
        state = STARTFORWARD;
        break;
      } else
      {
        scanDirection = closestDirection();
        if (scanDirection == 9) {
          state = STARTFORWARD;
          break;
        }
        
        switch (scanDirection) {
          case RIGHT:
            state = TURNLEFT45;
            turnIndex = 1;
            break;
          case RIGHT45:
            state = TURNLEFT;
            turnIndex = 0;
            break;
          case LEFT:
            state = TURNRIGHT45;
            turnIndex = 3;
            break;
          case LEFT45:
            state = TURNRIGHT;
            turnIndex = 4;
            break;
          case FRONT:
            
            sprintf(buffer, "BCK: %d", closest());
            Serial.println(buffer);
            state = TURNRIGHT;
            break;
        }
      }
      break;

    // Begin Left turn
    case TURNLEFT:
      lastState = state;
      left(SPEED, turnTimes[turnIndex]);
      break;

    // Begin Left 45 turn
    case TURNLEFT45:
      lastState = state;
      left(SPEED, turnTimes[turnIndex]);
      break;

    //Begin Right 45 turn
    case TURNRIGHT45:
      lastState = state;
      right(SPEED, turnTimes[turnIndex]);
      break;

    // Begin Right turn
    case TURNRIGHT:
      lastState = state;
      right(SPEED, turnTimes[turnIndex]);
      break;

    // Handle STOP for all turns
    case TURNSTOP:
      timer.cancel();
      scan(RIGHT, LEFT, 350);
      //blinkLED();
      if (closest(RIGHT45, LEFT45) > MINDISTANCE) {
        state = STARTFORWARD;
        break;
      }
      state = STOPPED;
      switch (lastState) {
        case LEFT:
          state = TURNLEFT45;
          break;
        case LEFT45:
          state = TURNLEFT45;
          break;
        case RIGHT45:
          state = TURNRIGHT45;
          break;
        case RIGHT:
          state = TURNRIGHT45;
          break;
      }
      
      break;

    // State to handle inertia
    case STOPPING:
      break;

    // Begin forward motion
    case STARTFORWARD:
      wasBacking = false;
      setServo(90, 350);
      scan(FRONT, FRONT, 0);
      distance = closest()-MINDISTANCE;
      delayMS = delayTime(SPEED, distance);
      sprintf(buffer, "DMS: %d RNG: %d", delayMS, distance);
      Serial.println(buffer);
      scanCount = 0;
      forward(SPEED, delayMS);
      state = FORWARD;
      break;

   // Moving Forward
    case FORWARD:
    // Scan 45 degrees R/L every N times
      if ((scanCount % 8) == 0) {
        scan(RIGHT45, LEFT45, 200);
      } else {
        scan(FRONT, FRONT, 0);
      }
      scanCount++;
      
      sprintf(buffer, "FWD: %d", closest());
      Serial.println(buffer);
      distance = closest(RIGHT45, LEFT45);
      if ( distance < 60) {
        forwardSpeed(SLOWSPEED);
      }
      
      if ( distance < MINDISTANCE ) {
        Serial.println(distance);
        stop();
        state = STOPPED;
      }
      break;

    // Begin Beck movement
    case BACK:
      back(SLOWSPEED, BACKTIME);
      state = BACKING;
      wasBacking = true;
      break;

    // Moving back
    case BACKING:
      break;

    // STOP state for back movement
    case BACKSTOP:
      scan(RIGHT, LEFT, SERVODELAY);
      if (distances[RIGHT] > MINDISTANCE) {
        state = TURNRIGHT;
      } else if (distances[LEFT] > MINDISTANCE ){
        state = TURNLEFT;
      } else
        state = BACK;
      break;

    // State for all turns active
    case TURNING:
      break;
  }
                       
}
