//FSU JPL Summer Team - Object Avoidance 
#define MAXDISTANCE 150
#define MINDISTANCE 30
#define RUNDISTANCE 60
#define BOOST 30
#define OFFSET 15

#include <IRremote.h>

////////// IR REMOTE CODES //////////
#define FWD 3108437760  // FORWARD
#define STP 1286666973  // STOP
#define UNKNOWN_F 3108437760  // FORWARD
#define UNKNOWN_B 2747854299  // BACK
#define UNKNOWN_L 1386468383  // LEFT
#define UNKNOWN_R 553536955   // RIGHT
#define UNKNOWN_S 3208707840  // STOP

#define IR_RECEIVE_PIN  12

decode_results results;

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

#define BACKTIME 500

#define STOP 0
#define STOPPED 1
#define STOPPING 2
#define STARTFORWARD 3
#define FORWARD 4
#define CONTINUEFORWARD 5
#define BACK 6
#define BACKING 7
#define BACKSTOP 8
#define TURNLEFT 9
#define TURNLEFT45 10
#define TURNRIGHT 11
#define TURNRIGHT45 12
#define TURNING 13
#define TURNSTOP 14
#define HARDSTOP 99

int turnIndex = 2;

int state = HARDSTOP;
bool wasBacking = false;

#define RIGHT   0
#define RIGHT7  1
#define RIGHT15 2
#define RIGHT30 3
#define RIGHT45 4
#define RIGHT60 5
#define RIGHT75 6
#define FRONT   7
#define LEFT75  8
#define LEFT60  9
#define LEFT45  10
#define LEFT30  11
#define LEFT15  12
#define LEFT7   13
#define LEFT    14
#define MAXANGLES 15

#define SERVODELAY 400
#define SERVOINTERDELAY 60
#define PINGREADS 3

int angles[] = {0, 7, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 173, 180};

int turnTimes[] = {540, 300, 0, 300, 540};
int distances[MAXANGLES];
int scanCount = 0;
int LEDcount = 0;

//  Callback for forward movement
bool stopCallback(void *argument) {
  state = STOP;
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
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed+BOOST);
}

// Start forward movement
// IN:  carSpeed PWM
//      MS motor run time
// RETURN void
//
void forward(int carSpeed, int runTime){ 
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed+BOOST);
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

//  Ultrasonic distance measurement
//  IN: degrees
//      servo delay
//  RETURN  distance
//
int Distance_test() {
  return sonar.convert_cm(sonar.ping_median(PINGREADS));  
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
  static int lastAngle = 999;

  if (lastAngle == degreeTurn) return;
  myservo.write(degreeTurn);  //set servo position according to scaled value
  lastAngle = degreeTurn;
  delay(delayTime);
  //irReceive();
}

// Scan distances
//  IN: right limit
//      left limit
//      servo delay
//  RETURN void (sets distances array)
//
void scan(int right, int left, int delayTime) {
  int distance;
  bool init = true;
  int servoDelay;
  static int lastAngle = 999;
  int servoIndex;
  char buffer[80];
  bool reverse = false;
  
  for(int i = 0; i < MAXANGLES; i++) {
    distances[i] = MAXDISTANCE;
  }

 if (lastAngle == angles[left]) {
    reverse = true;
  }
  
  for(int i = right; i <= left; i++) {
    if (init) {
      servoDelay = delayTime;
      init = false;
    }
    else {
      servoDelay = SERVOINTERDELAY;
    }
      
    servoIndex = i;
    if (reverse) {
      servoIndex = left-(i-right);
    }
    sprintf(buffer, "R %d L %d Inx: %d Sts %d", right, left, servoIndex, reverse);
    Serial.println(buffer);
    
    setServo(angles[servoIndex], servoDelay);  //set servo position according to scaled value
    lastAngle = angles[servoIndex];
    distance = Distance_test();
    if (distance > 0 ) {
      distances[servoIndex] =  distance;
    } else{
      distances[servoIndex] = MAXDISTANCE;
    }
  }
}

//  Return pose angle
//  IN: none (distances scanned)
//  RETURN: pose angle
//
int poseAngle() {
  int minValue = 999;
  int minIndex = 99;
  int rightIndex;
  int leftIndex;
  int sumAngles;
  bool rightEdge = false;
  bool leftEdge = false;
  char buffer[80];
  
  for (int i= 0; i < MAXANGLES; i++) {
    if (distances[i] < minValue) {
      minValue = distances[i];
      minIndex = i;
    }
  }
  // have min and index
  // work R & L of index
  rightIndex = leftIndex = minIndex;
  while (true) {
    rightIndex = rightIndex-1;
    if (rightIndex < 0) {
      rightIndex = 0;
      rightEdge = true;
    }

    leftIndex = leftIndex+1;
    if (leftIndex >= MAXANGLES) {
      leftIndex = MAXANGLES-1;
      leftEdge = true;
    }

    if (distances[rightIndex] > minValue * 1.15 ) {
      rightEdge = true;
      rightIndex++;
    }
    if (distances[leftIndex] > minValue * 1.15 ) {
      leftEdge = true;
      leftIndex--;
    }
    // exit loop when both sides on mimima are found
    if (rightEdge && leftEdge) break;
  }
  //  Now average the angles and return
  sumAngles = 0;
  for (int i = rightIndex; i <= leftIndex; i++ ) {
    sumAngles += angles[i];
  }
//  sprintf(buffer, "R %d L %d SUM: %d C: %d", rightIndex, leftIndex, sumAngles, leftIndex-rightIndex+1);
//  Serial.println(buffer);
  return (int)((float)sumAngles/(leftIndex-rightIndex+1));
  
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
    if (distances[i] > 0 && distances[i] > highvalue)
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
  int index = 99;
  for (int i = right; i <= left; i++) {
    //Serial.println(distances[i]);
    if (distances[i] > 0 && distances[i] > highvalue) {
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
  int index = 99;
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

void irReceive() {
  unsigned long val;
  if (IrReceiver.decode()){ 
    val = IrReceiver.decodedIRData.decodedRawData ;
    IrReceiver.resume();
    if (val == 0) return;
    switch(val){
      case FWD:
      //case UNKNOWN_F: 
        state = STOPPED; 
        break;
//      case STP: 
//      case UNKNOWN_S: 
//        state = HARDSTOP; 
//        Serial.println("IR stop");
//        break;
      default: break;
    }
  }
}

void dumpDistances() {
  char buffer[120];
  sprintf (buffer, "(R) %d %d %d %d (R45) %d %d %d (F) %d %d %d (L45) %d %d %d %d (L) %d", 
    distances[0], distances[1], distances[2], distances[3],  distances[4],   distances[5],  distances[6],
    distances[7], distances[8], distances[9], distances[10], distances[11],  distances[12], distances[13],
    distances[14]);
  Serial.println(buffer);
}

// Arduino SETUP logic
//
void setup() { 
  myservo.attach(3);  // attach servo on pin 3 to servo object
  IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK); // Start the receiver
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
  state = HARDSTOP;
} 
int lastState = STOPPED;

// Arduino LOOP
//
void loop() { 
  char buffer[80];
  int scanDirection;
  int delayMS;
  int turnDelay;
  int closeDistance;
  int closeDirection;
  int furthestDirection;
  int carAngle;

  // Invoke the timer tick
  timer.tick();
  irReceive();
   
  switch (state) {

    case HARDSTOP:
      break;
      
    // Stop car
    case STOP:
      stop();
      state = STOPPED;
      break;
      
    // State to handle inertia
    case STOPPING:
      break;

    // Robot is STOPPED
    case STOPPED:
      timer.cancel();
      Serial.println("Stopped");
      scan(RIGHT, LEFT, SERVODELAY);
      dumpDistances();
      //if (state != STOPPED) break;

      if (closest() >= RUNDISTANCE) {
        Serial.println(closest());
        state = STARTFORWARD;
        break;
      }
        
      carAngle = poseAngle();
      sprintf(buffer, "Pose: %d", carAngle);
      Serial.println(buffer);
      if (carAngle >= 0 && carAngle < 45 )
        state = TURNLEFT45;
      if (carAngle >= 45 && carAngle < 90 )
        state = TURNLEFT;
      if (carAngle >= 90 && carAngle < 135 )
        state = TURNRIGHT;
      if (carAngle >= 135  && carAngle <= 180)
        state = TURNRIGHT45;

      if (farthest(RIGHT45, LEFT45) < MINDISTANCE) {
        state = BACK;
        break;
      }
      break;

    // Begin Left turn
    case TURNLEFT:
      lastState = state;
      state = TURNING;
      turnDelay = turnTimes[0];
      left(SPEED, turnDelay);
      break;

    // Begin Left 45 turn
    case TURNLEFT45:
      lastState = state;
      state = TURNING;
      turnDelay = turnTimes[1];
      left(SPEED, turnDelay);
      break;

    //Begin Right 45 turn
    case TURNRIGHT45:
      lastState = state;
      state = TURNING;
      turnDelay = turnTimes[3];
      right(SPEED, turnDelay);
      break;

    // Begin Right turn
    case TURNRIGHT:
      lastState = state;
      state = TURNING;
      turnDelay = turnTimes[4];
      right(SPEED, turnDelay);
      break;

    // State for all turns active
    case TURNING:
      break;
                       
    // Handle STOP for all turns
    case TURNSTOP:
      timer.cancel();
      scan(RIGHT, LEFT, 350);
      if (state != TURNSTOP) break;
      
      blinkLED();
      if (closest(RIGHT60, LEFT60) > MINDISTANCE) {
        state = STARTFORWARD;
        break;
      }
      state = STOPPED;
      break;

    // Begin forward motion
    case STARTFORWARD:
      timer.cancel();
      wasBacking = false;
      setServo(90, 350);
      scan(RIGHT, LEFT, SERVODELAY);
      closeDistance = closest(RIGHT45, LEFT45)-MINDISTANCE-OFFSET;
      delayMS = delayTime(SPEED, closeDistance);
      sprintf(buffer, "DMS: %d RNG: %d", delayMS, closeDistance);
      Serial.println(buffer);
      scanCount = 0;
      forward(SPEED, delayMS);
      state = FORWARD;
      break;

   // Continue forward
    case CONTINUEFORWARD:
      timer.cancel();
      delayMS = delayTime(SPEED, closeDistance-MINDISTANCE-OFFSET);
      forward(SPEED, delayMS);
      state = FORWARD;
      break;
      
   // Moving Forward
    case FORWARD:
    // Scan 45 degrees R/L every N times
      if ((scanCount % 24) == 0) {
          scan(RIGHT, LEFT, 200);
          if (state != FORWARD) break;
          closeDistance = closest(RIGHT, LEFT);
      } else if ((scanCount % 8) == 0) {
          scan(RIGHT60, LEFT60, 200);
          if (state != FORWARD) break;
          closeDistance = closest(RIGHT60, LEFT60);
      } else {
        scan(FRONT, FRONT, 0);
        if (state != FORWARD) break;
        closeDistance = distances[FRONT];
      }
      scanCount++;
      
      sprintf(buffer, "FWD: %d", closeDistance);
      Serial.println(buffer);
      
      if ( closeDistance < RUNDISTANCE) {
        forwardSpeed(SLOWSPEED);
      }
      if (closeDistance >= RUNDISTANCE) {
        state = CONTINUEFORWARD;
      }
      
      if ( closeDistance < MINDISTANCE ) {
        Serial.println(closeDistance);
        state = STOP;
      }
      break;

    // Begin Back movement
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
      if (state != BACKSTOP) break;
      if (farthest(RIGHT,LEFT) < MINDISTANCE) {
        state = BACK;
        break;
      }
      furthestDirection = farthestDirection(RIGHT, LEFT);
      switch (furthestDirection) {
        case LEFT:
        case LEFT30:
          state = TURNLEFT;
          break;
        //case LEFT45:
        case LEFT60:
          state = TURNLEFT45;
          break;
        case FRONT:
          state = STARTFORWARD;
          break;
        //case RIGHT45:
        case RIGHT60:
          state = TURNRIGHT45;
          break;
        case RIGHT:
        case RIGHT30:
          state = TURNRIGHT;
          break;
        default:
          state = STARTFORWARD;
          break;
      }
      break;
  }
}
