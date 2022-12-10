#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor
//#define dhttype    DHT11     // DHT 11
//#define dhttype    DHT22     // DHT 22 (AM2302)
//#define dhttype    DHT21     // DHT 21 (AM2301)

#define output
#define dhttype DHT22
#define dhtpin 3
#define relay1  4
#define relay2  5
#define relay3  6
#define relay4  7
#define pumpSwitch  13
#define pumpTimeout 2000
#define maxHumidity 40
#define minHumidity 30

bool outOfWater = false;
bool pumpStatus;
bool vaporizerStatus;

DHT_Unified dht(dhtpin, dhttype);
uint32_t delayMS;

void pumpTime(unsigned long beginTime, unsigned long currentTime){
  if (beginTime+pumpTimeout >= currentTime){
    outOfWater = true;
    digitalWrite(relay2,LOW);
    digitalWrite(relay1,LOW);
  }
}

void pumpWater(){
  cli();
  unsigned long beginTime = millis();
  while (pumpStatus &! outOfWater)
  {
    unsigned long currentTime = millis();
    if (pumpStatus){
      digitalWrite(relay2,HIGH);
      pumpTime(beginTime, currentTime);
    }
    pumpStatus = digitalRead(pumpSwitch);
  }
  digitalWrite(relay2,LOW);
  sei();
}

void setup()
{
  #ifdef output
    Serial.begin(115200);
  #endif
  delay(1000);
  dht.begin();
  sensor_t sensor;
  pinMode(dhtpin, INPUT);
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);
  pinMode(pumpSwitch, INPUT);
  pinMode(A1, OUTPUT);
  analogWrite(A1, 255);
  sensor.min_delay = 5000;
  delayMS = sensor.min_delay;
}

void loop()
{
  pumpWater();
  delay(delayMS);
  pumpStatus = digitalRead(pumpSwitch);
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  
  if (isnan(event.temperature))
  {
    #ifdef output
    Serial.println(F("Error reading temperature!"));
    #endif
  }
  else
  {
    #ifdef output
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
    #endif
  }

  dht.humidity().getEvent(&event);

  if (isnan(event.relative_humidity)) {
    #ifdef output
    Serial.println(F("Error reading humidity!"));
    #endif
  }
  else
  {
    float rh = event.relative_humidity;
    #ifdef output
    Serial.print(F("Humidity: "));
    Serial.print(rh);
    Serial.println(F("%"));
    #endif
    if ( ((rh < minHumidity) &! outOfWater) ){
      digitalWrite(relay1,HIGH);
    }
    else if ( (rh >= maxHumidity) )
    {
      digitalWrite(relay1,LOW);
    }
    if(outOfWater){
      Serial.println("Out of water...");
      Serial.println("Manual reset needed...");
    }

    #ifdef output
    if(digitalRead(relay1) == HIGH){
      Serial.println("Vaporizor on...\n");
    } else {
      Serial.println("Vaporizor off...\n");
    }
    #endif
  }
}