//Libraries
#include "dht.h"
//Constants
#define DHT22_PIN 2
dht DHT;

//Variables
float hum;  //Stores humidity value
float temp; //Stores temperature value

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    //Temperature and Humidity Sampling
    int chk = DHT.read22(DHT22_PIN);
    hum = DHT.humidity;
    temp= DHT.temperature;
    //Print temp and humidity values to serial monitor
    String humidity = String(hum);
    String temperature = String(temp);
    Serial.print("Humidity: " + humidity + " %, Temperature : " + temperature + " Celsius \n");
    delay(2000); //Delay 2 sec.

}
