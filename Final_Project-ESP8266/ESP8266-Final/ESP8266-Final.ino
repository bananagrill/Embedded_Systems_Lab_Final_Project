#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "PubSubClient.h"
#include "ArduinoJson.h"
#include "Adafruit_SHT31.h"
//Topic การเขียนข้อมูล Shadow
#define UPDATEDATA   "@shadow/data/update"

#define rxPin 13
#define txPin 15

SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);

//กำหนดข้อมูลเชื่อมต่อ WiFi
const char* ssid      = "Xiaomi 11 Lite 5G NE";     //ต้องแก้ไข
const char* password  = "ef0bXHfGH7";    //ต้องแก้ไข
//กำหนดข้อมูลเชื่อมต่อ NETPIE2020 นำ KEY ที่ได้จาก NETPIE มาใส่
const char* mqtt_server = "broker.netpie.io";
const char* client_id = "9142aafd-879f-486f-968b-680293252f98";//ต้องแก้ไข
const char* token     = "bG15dSwp9SiTF1NBKfNYyTDF6QJsHWkw";    //ต้องแก้ไข
const char* secret    = "4RZrT2!mQ8OPSOyLl49bb~V6WZPQHZuB";   //ต้องแก้ไข
Adafruit_SHT31 sht31 = Adafruit_SHT31();
WiFiClient espClient;
PubSubClient client(espClient);




//ฟังก์ชั่น เชื่อมต่อ WiFi
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print(F("Connecting to "));
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println(F(""));
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
}
//ตรวจสอบการเชื่อมต่อ และเชื่อมต่อ NETPIE2020
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print(F("Attempting MQTT connection..."));
    // Attempt to connect
    if (client.connect(client_id, token, secret)) {
      Serial.println(F("connected"));
    // ... and resubscribe from server
    } else {
      Serial.print(F("failed, rc="));
      Serial.print(client.state());
      Serial.println(F(" try again in 5 seconds"));
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void callback(char* topic, byte* payload, unsigned int length) {

  Serial.print(F("Message arrived ["));
  Serial.print(topic);
  Serial.print(F("] "));
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
void setup() {
  
  //ตั่งค่า serial monitor
  Serial.begin(9600);
  mySerial.begin(9600);
    
  // เชื่อมต่อ WiFi
  setup_wifi();
  // ตั่งค่า Broker Netpie และพอร์ต 
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}


String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

String receivedData = "";
int temp,humid,lux;
float dust;

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

// กำหนดให้ส่งข้อมูลทุกๆ 30 วินาที
  static unsigned long lastTime = 0;
  if (millis() - lastTime >= 1500 || lastTime > millis()) {
    lastTime = millis();
// อ่านค่าอุณหภูมิ และความชื้น
  if(mySerial.available() > 1){
      receivedData = mySerial.readString();
      //receivedData += ' ';
      Serial.println(receivedData);
      if(receivedData.length() >= 5){
        dust = (getValue(receivedData,' ',0).toFloat());
        temp = (getValue(receivedData,' ',1).toInt());
        humid = (getValue(receivedData,' ',2).toInt());
        lux = (getValue(receivedData,' ',3).toInt());
        
      }
      Serial.print(" dust => ");
      Serial.println(dust);
      Serial.print(" temp => ");
      Serial.println(temp);
      Serial.print(" humid => ");
      Serial.println(humid);
      Serial.print(" lux => ");
      Serial.println(lux); 

    }
    
    
// กำหนดข้อมูลให้อยู่ในรูปแบบ Json NETPIE
   const size_t capacity = JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(4);
    StaticJsonDocument<capacity> doc;
    JsonObject data = doc.createNestedObject("data");
    data["dust"]  = dust;
    data["temp"] = temp;
    data["humidity"]  = humid;
    data["lux"] = lux;
    Serial.print(F("Message send: "));
    Serial.println((doc.as<String>()).c_str());
// ส่งข้อมูลไปที่ Shadow Device
   client.publish(UPDATEDATA, (doc.as<String>()).c_str());
  }
}
