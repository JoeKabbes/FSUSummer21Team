//www.elegoo.com - SPEED CONTROL

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

void setup() {
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
}

void loop() {
  //go forward
  digitalWrite(IN1,HIGH); 
  digitalWrite(IN2,LOW);  
  digitalWrite(IN3,LOW);  
  digitalWrite(IN4,HIGH);
  //reduce the speed
  for(int i = 255; i >= 0; i--){ 
    analogWrite(ENB,i);
    analogWrite(ENA,i);
    delay(20);
  }

  //stop
  analogWrite(ENB,0); //speed = 0
  analogWrite(ENA,0);  
  delay(1000);

  //running back
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  //accelerate
  for(int i = 0; i <= 255; i++){ 
    analogWrite(ENB,i);
    analogWrite(ENA,i);
    delay(20);
  } 
  
  //stop
  digitalWrite(ENB,LOW); //Motor is off 
  digitalWrite(ENA,LOW);  
  delay(5000);   
}
