#include <Servo.h>
#define SERVO1_PIN 3
#define SERVO2_PIN 4
#define STRIPLINE1 5
#define STRIPLINE2 6


Servo servo1;
Servo servo2;

int pos = 0;

void setup() {
   servo1.attach(SERVO1_PIN);
   servo2.attach(SERVO2_PIN);
   pinMode(STRIPLINE1, OUTPUT);
   pinMode(STRIPLINE2, OUTPUT);
   digitalWrite(STRIPLINE1, HIGH);
   digitalWrite(STRIPLINE2, HIGH);
  
}
void loop() {
    pos += 20;
    if(pos > 180){
      pos =  140;
    } 
    
    digitalWrite(STRIPLINE1, LOW);
    digitalWrite(STRIPLINE2, LOW);
 
    delay(1000);
   
    servo1.write(pos);
    servo2.write(pos);
    digitalWrite(STRIPLINE1, HIGH);
    digitalWrite(STRIPLINE2, HIGH);
 
    delay(1000);
  
}
