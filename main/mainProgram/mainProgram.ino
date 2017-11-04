//Libraries
#include "dht.h"
#include "TimeLib.h"
#include "Time.h"
#include "MovingAverageFilter.h"
#include "OneWire.h"


dht DHT;
MovingAverageFilter movingAverageFilter(15); //might be necessary to create mult objects for mul

//Constants, on schematic, pin - 1
#define DHT22_PIN       2   // Temperature and Humidity Sensor
#define PH_SENSOR       0   // pH Sensor (POLL)
#define EC_SENSOR       1   // EC Sensor (POLL)

#define EC_TEMPSENSOR   2   // Liquid TEMP Sensor (POLL)
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

#define SPROUT_PHASE 0
#define GROWTH_PHASE 1
#define COLLECTION_PHASE 2

#define SPROUT_PERIOD 7
#define GROWTH_PERIOD 25

//For the EC Sensor
#define StartConvert 0
#define ReadTemperature 1

//Variables
float hum;          //Stores humidity value
float temp;         //Stores temperature value
float ph_offset = 0.00;
float ph;
float ECcurrent;
int chk;            //Read sensor value
String humidity;    // display humidity
String temperature; // display temperature
int hum_filter[10];
int temp_filter[10];

OneWire ds(EC_TEMPSENSOR);

int sproutFinalDayFromStart;
int growthFinalDayFromStart;
int currentPhase;

void setup()
{
  Serial.begin(9600);
  sproutFinalDayFromStart = day() + SPROUT_PERIOD ;
  growthFinalDayFromStart = day() + SPROUT_PERIOD + GROWTH_PERIOD ;
  currentPhase = SPROUT_PHASE;
  
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
  TempProcess(StartConvert);
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

void pollECSensor() {
    float avgAnalogValue = movingAverageFilter.process(analogRead(EC_SENSOR));
    //Serial.println(String(avgAnalogValue));
    float temperature = TempProcess(ReadTemperature);  // read the current temperature from the  DS18B20
    TempProcess(StartConvert);                   //after the reading,start the convert for next reading
Serial.println(String(temperature));
    
    float avgVoltage = avgAnalogValue * (float) 5000/1024;
    float TempCoefficient=1.0+0.0185*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.0185*(fTP-25.0));
    float CoefficientVolatge=(float)avgVoltage/TempCoefficient;   
    Serial.println(String(CoefficientVolatge));
    if(CoefficientVolatge<150)Serial.println("No solution!");   //25^C 1413us/cm<-->about 216mv  if the voltage(compensate)<150,that is <1ms/cm,out of the range
    else if(CoefficientVolatge>3300)Serial.println("Out of the range!");  //>20ms/cm,out of the range
    else
    { 
      if(CoefficientVolatge<=448)ECcurrent=6.84*CoefficientVolatge-64.32;   //1ms/cm<EC<=3ms/cm
      else if(CoefficientVolatge<=1457)ECcurrent=6.98*CoefficientVolatge-127;  //3ms/cm<EC<=10ms/cm
      else ECcurrent=5.3*CoefficientVolatge+2278;                           //10ms/cm<EC<20ms/cm
      ECcurrent/=1000;    //convert us/cm to ms/cm
      Serial.print(ECcurrent,2);  //two decimal
      Serial.println("ms/cm");
    }

    delay(2000);
}

/*
ch=0,let the DS18B20 start the convert;ch=1,MCU read the current temperature from the DS18B20.
*/
float TempProcess(bool ch)
{
  //returns the temperature from one DS18B20 in DEG Celsius
  static byte data[12];
  static byte addr[8];
  static float TemperatureSum;
  if(!ch){
          if ( !ds.search(addr)) {
              Serial.println("no more sensors on chain, reset search!");
              ds.reset_search();
              return 0;
          }      
          if ( OneWire::crc8( addr, 7) != addr[7]) {
              Serial.println("CRC is not valid!");
              return 0;
          }        
          if ( addr[0] != 0x10 && addr[0] != 0x28) {
              Serial.print("Device is not recognized!");
              return 0;
          }      
          ds.reset();
          ds.select(addr);
          ds.write(0x44,1); // start conversion, with parasite power on at the end
  }
  else{  
          byte present = ds.reset();
          ds.select(addr);    
          ds.write(0xBE); // Read Scratchpad            
          for (int i = 0; i < 9; i++) { // we need 9 bytes
            data[i] = ds.read();
          }         
          ds.reset_search();           
          byte MSB = data[1];
          byte LSB = data[0];        
          float tempRead = ((MSB << 8) | LSB); //using two's compliment
          TemperatureSum = tempRead / 16;
    }
          return TemperatureSum;  
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

void sproutPhase(){
  
}

void growthPhase(){
  
}

void collectionPhase(){
  
}

void loop()
{
  digitalClockDisplay();
  determinePhase();
  if (currentPhase == SPROUT_PHASE){
    sproutPhase();
  }else if (currentPhase == GROWTH_PHASE) {
    growthPhase();
  }else if (currentPhase == COLLECTION_PHASE) {
    collectionPhase();
  }
  
  delay(5000);
  /*
  pollTempSensor();
  pollPHSensor();
  pollECSensor();
  pollTempSensor();
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

int determinePhase(){
  if (day() >= sproutFinalDayFromStart){
    currentPhase = GROWTH_PHASE;
  } else if ( day() >= growthFinalDayFromStart){
    currentPhase = COLLECTION_PHASE;
  }
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

