int relayPin1 = 7;                 // Relay 1 IN1 connected to digital pin 7
int relayPin2 = 8;                 // Relay 1 IN2 connected to digital pin 8
int relayPin3 = 9;                 // Relay 2 IN1 connected to digital pin 9
int relayPin4 = 10;                // Relay 2 IN2 connected to digital pin 10


void setup()
{
  pinMode(relayPin1, OUTPUT);           // sets the digital pin as output
  pinMode(relayPin2, OUTPUT);           // sets the digital pin as output
  pinMode(relayPin3, OUTPUT);
  pinMode(relayPin4, OUTPUT);
  
  digitalWrite(relayPin1, HIGH);        // Prevents relays from starting up engaged
  digitalWrite(relayPin2, HIGH);        // Prevents relays from starting up engaged
  digitalWrite(relayPin3, HIGH);        // Prevents relays from starting up engaged
  digitalWrite(relayPin4, HIGH);        // Prevents relays from starting up engaged
}

void loop()
{
  digitalWrite(relayPin1, LOW);   // energizes the relay and lights the LED
  digitalWrite(relayPin2, LOW);   // energizes the relay and lights the LED
  digitalWrite(relayPin3, LOW);   // energizes the relay and lights the LED
  digitalWrite(relayPin4, LOW);   // energizes the relay and lights the LED
  delay(1000);                  // waits for a second
  digitalWrite(relayPin1, HIGH);    // de-energizes the relay and LED is off
  digitalWrite(relayPin2, HIGH);    // de-energizes the relay and LED is off
  digitalWrite(relayPin3, HIGH);    // de-energizes the relay and LED is off
  digitalWrite(relayPin4, HIGH);    // de-energizes the relay and LED is off
  delay(1000);                  // waits for a second
}
