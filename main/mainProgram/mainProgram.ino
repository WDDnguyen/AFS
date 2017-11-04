//Libraries
#include "dht.h"
#include "TimeLib.h"
#include "Time.h"

dht DHT;
//Constants
#define DHT22_PIN       2   // Temperature and Humidity Sensor
#define LED_ROW_1       3   // First row of each LED Strip
#define LED_ROW_2       4   // Second row of each LED Strip
#define PIPE_SERVO_1    5
#define PIPE_SERVO_2    6
#define RELAY1_IN_1     7   // Mix tank pump        
#define RELAY1_IN_2     8   // Heater
#define RELAY2_IN_1     9   // Fan
#define RELAY2_IN_2     10  // Dripping tank pump
#define AIR_PUMP        11
#define SOLUTION_PUMP   12
#define WATER_PUMP      13

#define SPROUT_PERIOD 7
#define GROWTH_PERIOD 25

//Variables
float hum;          //Stores humidity value
float temp;         //Stores temperature value
int chk;            //Read sensor value
String humidity;    // display humidity
String temperature; // display temperature

int sproutFinalDayFromStart;
int growthFinalDayFromStart;


void setup()
{
  Serial.begin(9600);
  sproutFinalDayFromStart = day() + SPROUT_PERIOD ;
  growthFinalDayFromStart = day() + SPROUT_PERIOD + GROWTH_PERIOD ;
  
  pinMode(LED_ROW_1, OUTPUT);
  pinMode(LED_ROW_2, OUTPUT);
  pinMode(PIPE_SERVO_1, OUTPUT);
  pinMode(PIPE_SERVO_2, OUTPUT);
  pinMode(RELAY1_IN_1, OUTPUT);
  pinMode(RELAY1_IN_2, OUTPUT);
  pinMode(RELAY2_IN_1, OUTPUT);
  pinMode(RELAY2_IN_2, OUTPUT);
  pinMode(AIR_PUMP, OUTPUT);
  pinMode(SOLUTION_PUMP, OUTPUT);
  pinMode(WATER_PUMP, OUTPUT);
}

void pollTempSensor() {
  //Temperature and Humidity Sampling
  chk = DHT.read22(DHT22_PIN);
  hum = DHT.humidity;
  temp = DHT.temperature;
  //Print temp and humidity values to serial monitor
  humidity = String(hum);
  temperature = String(temp);
  Serial.print("Humidity: " + humidity + " %, Temperature : " + temperature + " Celsius \n");
  //delay(2000); //Delay 2 sec.
}

void setLEDRows() {
  digitalWrite(LED_ROW_1, HIGH);
  digitalWrite(LED_ROW_2, HIGH);
}

void disableLEDRows() {
  digitalWrite(LED_ROW_1, LOW);
  digitalWrite(LED_ROW_2, LOW);
}

void setAirPump() {
  digitalWrite(AIR_PUMP, HIGH);
}
void disableAirPump() {
  digitalWrite(AIR_PUMP, LOW);
}

void setWaterPump() {
  digitalWrite(WATER_PUMP, HIGH);
}

void disableWaterPump() {
  digitalWrite(WATER_PUMP, LOW);
}

void setSolutionPump() {
  digitalWrite(SOLUTION_PUMP, HIGH);
}

void disableSolutionPump() {
  digitalWrite(SOLUTION_PUMP, LOW);
}

void setPipeMotors() {
  digitalWrite(PIPE_SERVO_1, HIGH);
  digitalWrite(PIPE_SERVO_2, HIGH);
}

void disablePipeMotors() {
  digitalWrite(PIPE_SERVO_1, LOW);
  digitalWrite(PIPE_SERVO_2, LOW);
}

void setRelays() {
  digitalWrite(RELAY1_IN_1, HIGH);
  digitalWrite(RELAY1_IN_2, HIGH);
  digitalWrite(RELAY2_IN_1, HIGH);
  digitalWrite(RELAY2_IN_2, HIGH);
}

void disableRelays() {
  digitalWrite(RELAY1_IN_1, LOW);
  digitalWrite(RELAY1_IN_2, LOW);
  digitalWrite(RELAY2_IN_1, LOW);
  digitalWrite(RELAY2_IN_2, LOW);
}

void loop()
{
  Serial.println(sproutFinalDayFromStart);
  Serial.println(growthFinalDayFromStart);
  digitalClockDisplay();
  delay(1000);
  /*pollTempSensor();
    setLEDRows();
    setWaterPump();
    setSolutionPump();
    setAirPump();
    setPipeMotors();
    setRelays();
    delay(1000);
    disablePipeMotors();
    disableLEDRows();
    disableWaterPump();
    disableSolutionPump();
    disableAirPump();
    disableRelays();
    delay(1000);
  */
}



void digitalClockDisplay() {
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


