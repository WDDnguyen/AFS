int relayPin1 = 2;                 // IN1 connected to digital pin 7
int relayPin2 = 3;                 // IN2 connected to digital pin 8

void setup()
{
  pinMode(relayPin1, OUTPUT);      // sets the digital pin as output
  pinMode(relayPin2, OUTPUT);      // sets the digital pin as output
  digitalWrite(relayPin1, HIGH);        // Prevents relays from starting up engaged
  digitalWrite(relayPin2, HIGH);        // Prevents relays from starting up engaged
}

void loop()
{
  digitalWrite(relayPin1, LOW);   // energizes the relay and lights the LED
  digitalWrite(relayPin2, LOW);   // energizes the relay and lights the LED
  delay(1000);                  // waits for a second
  digitalWrite(relayPin1, HIGH);    // de-energizes the relay and LED is off
  digitalWrite(relayPin2, HIGH);    // de-energizes the relay and LED is off
  delay(1000);                  // waits for a second
}
