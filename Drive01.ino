/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/
//    The direction of the car's movement
//  ENA   ENB   IN1   IN2   IN3   IN4   Description  
//  HIGH  HIGH  HIGH  LOW   LOW   HIGH  Car is running forward
//  HIGH  HIGH  LOW   HIGH  HIGH  LOW   Car is running back
//  HIGH  HIGH  LOW   HIGH  LOW   HIGH  Car is turning left
//  HIGH  HIGH  HIGH  LOW   HIGH  LOW   Car is turning right
//  HIGH  HIGH  LOW   LOW   LOW   LOW   Car is stopped
//  HIGH  HIGH  HIGH  HIGH  HIGH  HIGH  Car is stopped
//  LOW   LOW   N/A   N/A   N/A   N/A   Car is stopped

//define L298n module IO Pin
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

void forward(int leftPwr,int rightPwr, float time){ 
  analogWrite(ENA,leftPwr); //enable L298n A channel
  analogWrite(ENB,rightPwr); //enable L298n B channel
  digitalWrite(IN1,HIGH); //set IN1 hight level
  digitalWrite(IN2,LOW);  //set IN2 low level
  digitalWrite(IN3,LOW);  //set IN3 low level
  digitalWrite(IN4,HIGH); //set IN4 hight level
  Serial.println("Forward");//send message to serial monitor
  delay(time);
}

void back(int speedVal, int delayms){
  analogWrite(ENA,speedVal);
  analogWrite(ENB,speedVal);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  Serial.println("Back");
  delay(delayms);
}

void left(int speedVal, int deg){
  int timr = (5.425 * deg) + 61.473;
  analogWrite(ENA,speedVal);
  analogWrite(ENB,speedVal);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH); 
  Serial.println("Left");
  delay(timr);
}

void right(int speedVal, int delayms){
  analogWrite(ENA,speedVal);
  analogWrite(ENB,speedVal);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  Serial.println("Right");
  delay(delayms);
}
void stopCar(int delayms){ 
  digitalWrite(ENA,LOW); //enable L298n A channel
  digitalWrite(ENB,LOW); //enable L298n B channel
  digitalWrite(IN1,LOW); //set IN1 hight level
  digitalWrite(IN2,LOW);  //set IN2 low level
  digitalWrite(IN3,LOW);  //set IN3 low level
  digitalWrite(IN4,LOW); //set IN4 hight level
  Serial.println("Stop");//send message to serial monitor
  delay(delayms);
}
void runTime(int pwr,int rPwr, int dist){
    float spd = .0028 * pwr + .0595;
    float t = (dist/spd) * 1000;
    forward(pwr,rPwr, t);
   
 
}
//before execute loop() function, 
//setup() function will execute first and only execute once
void setup() {
  Serial.begin(9600);//open serial and set the baud rate
  pinMode(IN1,OUTPUT);//before using IO pin, pin mode must be set first 
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
}

//Repeat execution
void loop() {
//  runTime(200,190,3.5);
//  forward(100, 100, 2000);  //go forward
//  back(200, 500);     //go back
  left(150, 30);     //turning left
//  right(180,375);     //turning right
 stopCar(3000);
 left(150, 30);
 stopCar(3000);
 left(150, 30);
 stopCar(3000);
 left(150, 45);
 stopCar(3000);
 left(150, 45);
 stopCar(3000);
 left(150,180);
 stopCar(3000);
}
