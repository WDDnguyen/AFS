#include <Servo.h>

#define SERVO1_PIN 3
#define SERVO2_PIN 4
Servo servo1;
Servo servo2;

int pos = 0;

void setup() {
   servo1.attach(SERVO1_PIN);
   servo2.attach(SERVO2_PIN);
  
}
void loop() {
  for(pos = 0; pos <= 180; pos +=1){
    servo1.write(pos);
    servo2.write(pos);
    delay(15);   
  }
  
  for(pos = 180; pos >= 0; pos -= 1){
    servo1.write(pos);
    servo2.write(pos);
    delay(15);
  }
  
  
}
