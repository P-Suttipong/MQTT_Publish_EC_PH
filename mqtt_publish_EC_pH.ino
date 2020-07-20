#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <EasyScheduler.h>
#include "DFRobot_EC.h"
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

DFRobot_EC ec;

#define EC_PIN A0

float voltage, ecValue, pHValue, temperature = 25.0;
float AVG_VOL, EC, AVG_EC, TOTAL_EC = 0.0, AVG_PH, TOTAL_PH = 0.0;
int cnt = 0, num = 0, state = 0;



#define WIFI_STA_NAME "DataBroker_1"
#define WIFI_STA_PASS  "12345678"
#define MQTT_SERVER   "192.168.1.177"
#define MQTT_PORT     1883
#define MQTT_USERNAME "testmqtt"
#define MQTT_PASSWORD "123456"
#define MQTT_NAME     "esp8266"
char msg[50];


WiFiClient client;
PubSubClient mqtt(client);

Schedular ReadDataTask;
Schedular PublishTask;

// __________________CALLBACK_FUNCTION_FOR_MQTT________________
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
}

// ________________PUBLISH_TO_MQTT_BROKER______________________
void publish_mqttt() {
  if (mqtt.connect(MQTT_NAME, MQTT_USERNAME, MQTT_PASSWORD)) {
    snprintf (msg, 75, "%.2f,%.2f", AVG_PH, AVG_EC);
    Serial.print("Publish message: ");
    Serial.println(msg);
    if (mqtt.publish("mynew/test", msg) == true) {
      Serial.println("Success sending");
    } else {
      Serial.println("Fail sending");
    }
  }
}


//======================================================
void setup()
{
  Serial.begin(115200);

  // Initial EC Sensor
  sensors.begin();
  delay(100);
  ec.begin();

  // Initial Wifi
  WiFi.mode(WIFI_STA);
  while (!Serial) ;
  delay(250);
  Serial.println("");
  Serial.println(WIFI_STA_NAME);
  Serial.println("WIFI Connecting");
  WiFi.begin(WIFI_STA_NAME, WIFI_STA_PASS); //เชื่อมต่อ wifi
  while (num < 40) {
    delay(500);
    Serial.print(".");
    num++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\n WiFi Connected. \n");
  } else {
    Serial.print("\n WIFI Connect fail. ");
  }

  //  Initial MQTT SERVER
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(callback);

  ReadDataTask.start();
  PublishTask.start();
}

void read_data() {
  for (cnt = 0; cnt < 5; cnt++) {
    sensors.requestTemperatures();
    delay(10);
    //    Serial.print(sensors.getTempCByIndex(0));
    //    Serial.println(" *C");
    temperature = sensors.getTempCByIndex(0);
    char i;

    AVG_VOL = 0;
    for (i = 0; i < 20; i++) {
      AVG_VOL = AVG_VOL + analogRead(EC_PIN);
      delay(20);
    }
    AVG_VOL = AVG_VOL / 20;
    //    Serial.print("Average Voltage : ");
    //    Serial.println(AVG_VOL);
    voltage = (AVG_VOL / 1024.0) * 3300;
    Serial.print("voltage : ");
    Serial.println(voltage);
    delay(10);

    ecValue = EC_READ(voltage, temperature);
    //    Serial.print("ecValue : ");
    //    Serial.println(ecValue);

    pHValue = PH_READ(voltage, temperature);
    //    Serial.print("pHValue : ");
    //    Serial.println(pHValue);


    //    Serial.print("EC Value : ");
    //    Serial.println(ecValue);
    //    Serial.println("######################");
    TOTAL_EC = TOTAL_EC + ecValue;
    TOTAL_PH = TOTAL_PH + pHValue;
  }
  AVG_EC = TOTAL_EC / 5;
  AVG_PH = TOTAL_PH / 5;
  cnt = 0;

  Serial.print("Temperature : ");
  Serial.println(temperature);
  Serial.print("Average pH Value : ");
  Serial.println(AVG_PH);
  Serial.print("Average EC Value : ");
  Serial.println(AVG_EC);
  Serial.println("##########################");
  TOTAL_PH = 0;
  TOTAL_EC = 0;
}


float k_value = 0.995;
float k_valueHigh = 1.6;
float k_valueLow = 1.05 ;

float EC_READ(float vol, float temp)
{
  float rawEC = (1000 * vol) / 820.0 / 200.0;
  float valueTemp = rawEC * k_value;
  //  Serial.print("valueTemp : ");
  //  Serial.println(valueTemp);
  if (valueTemp > 2.5) {
    k_value = k_valueHigh;
  } else if (valueTemp < 2.0) {
    k_value = k_valueLow;
  }
  float value = rawEC * k_value;
  value = value / (1.0 + 0.0185 * (temperature - 25.0));
  float EC_value = value;
  return EC_value;
}

float PH_READ(float vol, float temp)
{
  int k_pH = 0;
  if (vol < 1000) {
    k_pH = 220;
  } else if (vol > 1000 && vol < 1400 ) {
    k_pH = 200;
  } else if (vol > 1400) {
    k_pH = 180;
  }
  float pH_value = vol / k_pH;
  return pH_value;
}

float CHANGE_TANK()
{
  if (state == 0) {
    while (isFulltank == false) {
      //Input Valve open
    }
    for (int count = 0; count < 6; count++) {
      static unsigned long timepoint = millis();
      if (millis() - timepoint > 10000U) { //time interval: 10s
        read_data();
      }
    }
    //Output Valve open
    state == 1;
  } else if (state == 1) {
    while (isFulltank == false) {
      //Input Valve open
    }
    for (int count = 0; count < 6; count++) {
      static unsigned long timepoint = millis();
      if (millis() - timepoint > 10000U) { //time interval: 10s
        read_data();
      }
    }
    //Output Valve open
    state == 2;
  } else if (state == 2) {
    while (isFulltank == false) {
      //Input Valve open
    }
    for (int count = 0; count < 6; count++) {
      static unsigned long timepoint = millis();
      if (millis() - timepoint > 10000U) { //time interval: 10s
        read_data();
      }
    }
    //Output Valve open
    state == 0;
  }
}

float isFulltank()
{
  return false;
}

//=======================================================
void loop()
{
  ReadDataTask.check(read_data, 5000);
  PublishTask.check(publish_mqttt, 6000);
}
