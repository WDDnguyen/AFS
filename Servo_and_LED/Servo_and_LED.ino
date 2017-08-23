#define STRIPLINE1 3
#define STRIPLINE2 4

int pos = 0;

void setup() {
   pinMode(STRIPLINE1, OUTPUT);
   pinMode(STRIPLINE2, OUTPUT);
  
}
void loop() {
    digitalWrite(STRIPLINE1, LOW);
    digitalWrite(STRIPLINE2, LOW);
    delay(5000);
    digitalWrite(STRIPLINE1, HIGH);
    digitalWrite(STRIPLINE2, HIGH);
    delay(5000);
  
}
