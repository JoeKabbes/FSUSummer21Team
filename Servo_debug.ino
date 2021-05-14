//www.elegoo.com
#include <Servo.h>
Servo myservo;

void setup(){
  myservo.attach(8);
  //myservo.write(90);// move servos to center position -> 90°
} 
void loop(){
    for(int x=0; x < 3; x++) {
    myservo.write(0);
    delay(1000);
    myservo.write(90);
    delay(1000);
    myservo.write(180);
    delay(1000);
  }
  while(true);
}
