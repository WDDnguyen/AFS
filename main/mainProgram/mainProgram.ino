//Libraries
#include "dht.h"
dht DHT;

//Constants
//DC
#define DHT22_PIN 2
#define LED_ROW_1 3
#define LED_ROW_2 4
#define SERVO_MOTOR_1 5
#define SERVO_MOTOR_2 6
#define RELAY1_IN_1 7
#define RELAY1_IN_2 8
#define RELAY2_IN_1 9
#define RELAY2_IN_2 10
#define AIR_PUMP 11
#define SOLUTION_PUMP 12
#define WATER_PUMP 13 

//Variables
float hum;  //Stores humidity value
float temp; //Stores temperature value
int chk;
String humidity;
String temperature;

void setup()
{
    Serial.begin(9600);
    pinMode(LED_ROW_1, OUTPUT);
    pinMode(LED_ROW_2, OUTPUT);
    pinMode(AIR_PUMP, OUTPUT);
    pinMode(SOLUTION_PUMP, OUTPUT);
    pinMode(WATER_PUMP, OUTPUT);
}

void pollTempSensor(){
    //Temperature and Humidity Sampling
    chk = DHT.read22(DHT22_PIN);
    hum = DHT.humidity;
    temp= DHT.temperature;
    //Print temp and humidity values to serial monitor
    humidity = String(hum);
    temperature = String(temp);
    Serial.print("Humidity: " + humidity + " %, Temperature : " + temperature + " Celsius \n");
    delay(2000); //Delay 2 sec.
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

void loop()
{
    pollTempSensor();
    setLEDRows();
    setWaterPump();
    setSolutionPump();
    setAirPump();
    delay(1000);
    disableLEDRows();
    disableWaterPump();
    disableSolutionPump();
    disableAirPump();
    delay(1000);
}
