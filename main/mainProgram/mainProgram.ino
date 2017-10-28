//Libraries
#include "dht.h"
#include "MovingAverageFilter.h"
dht DHT;
MovingAverageFilter movingAverageFilter(15);

//Constants, on schematic, pin - 1
#define PH_SENSOR       0
#define DHT22_PIN       2   // Temperature and Humidity Sensor
#define LED_ROW_1       3   // First row of each LED Strip
#define LED_ROW_2       4   // Second row of each LED Strip
#define PIPE_SERVO_1    5    
#define PIPE_SERVO_2    6     
#define RELAY1_IN_1     7   // Mix tank pump        
#define RELAY1_IN_2     8   // Fan
#define RELAY2_IN_1     9   // Dripping tank pump
#define RELAY2_IN_2     10  // Heater
#define AIR_PUMP        11  
#define SOLUTION_PUMP   12
#define WATER_PUMP      13


//Variables
float hum;          //Stores humidity value
float temp;         //Stores temperature value
float ph_offset = 0.00;
float ph;
int chk;            //Read sensor value
String humidity;    // display humidity
String temperature; // display temperature
int hum_filter[10];
int temp_filter[10];

void setup()
{
    Serial.begin(9600);
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

void pollTempSensor(){
    //Temperature and Humidity Sampling
    chk = DHT.read22(DHT22_PIN);
    hum = movingAverageFilter.process(DHT.humidity);
    temp= movingAverageFilter.process(DHT.temperature);

    //Print temp and humidity values to serial monitor
    humidity = String(hum);
    temperature = String(temp);
    Serial.print("Humidity: " + humidity + " %, Temperature : " + temperature + " Celsius \n");
    Serial.print("Without Filter => Humidity: " + String(DHT.humidity) + " %, Temperature : " + String(DHT.temperature) + " Celsius \n");

    
    delay(2000); //Delays 2 sec.
}

void pollPHSensor() {
    float voltage = 5.0 * analogRead(PH_SENSOR) / 1024;
    float rawPH = 3.5*voltage+ph_offset; //to millivolt, to phValue
    ph = movingAverageFilter.process(rawPH);
    Serial.println("PH avg value:" + String(ph));
    delay(2000);
}

void setLEDRows(){
    digitalWrite(LED_ROW_1,HIGH);
    digitalWrite(LED_ROW_2,HIGH);
}

void disableLEDRows(){
  digitalWrite(LED_ROW_1, LOW);
  digitalWrite(LED_ROW_2, LOW);
}

void setAirPump(){
  digitalWrite(AIR_PUMP,HIGH);
}
void disableAirPump(){
  digitalWrite(AIR_PUMP,LOW);
}

void setWaterPump(){
  digitalWrite(WATER_PUMP,HIGH);
}

void disableWaterPump(){
  digitalWrite(WATER_PUMP,LOW);
}

void setSolutionPump(){
  digitalWrite(SOLUTION_PUMP,HIGH);
}

void disableSolutionPump(){
  digitalWrite(SOLUTION_PUMP,LOW);
}

void setPipeMotors(){
  digitalWrite(PIPE_SERVO_1,HIGH);
  digitalWrite(PIPE_SERVO_2,HIGH);
}

void disablePipeMotors(){
  digitalWrite(PIPE_SERVO_1, LOW);
  digitalWrite(PIPE_SERVO_2, LOW);
}

void setRelays(){
  digitalWrite(RELAY1_IN_1, HIGH);
  digitalWrite(RELAY1_IN_2, HIGH);
  digitalWrite(RELAY2_IN_1, HIGH);
  digitalWrite(RELAY2_IN_2, HIGH);
}

void disableRelays(){
  digitalWrite(RELAY1_IN_1, LOW);
  digitalWrite(RELAY1_IN_2, LOW);
  digitalWrite(RELAY2_IN_1, LOW);
  digitalWrite(RELAY2_IN_2, LOW);
}

void loop()
{
//    pollTempSensor();
      pollPHSensor();
//    setLEDRows();
//    setWaterPump();
//    setSolutionPump();
//    setAirPump();
//    setPipeMotors();
//    setRelays();
//    delay(1000);
//    disablePipeMotors();
//    disableLEDRows();
//    disableWaterPump();
//    disableSolutionPump();
//    disableAirPump();
//    disableRelays();
//    delay(1000);
}
