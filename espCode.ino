#include <ArduinoJson.h>
#include "WiFi.h"
#include "DHT.h"
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP32Servo.h>

//#define DHTPIN 4
//#define DHTTYPE DHT22
//#define mq2 A0
//#define M1 5
//#define M2 19
//#define buzz 8
//#define sonde 1

#define movSensor 4


const char* mqtt_server = "15.235.140.225";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
bool led = false;

//DHT dht(DHTPIN, DHTTYPE);
//OneWire oneWire(sonde);
//DallasTemperature sondeSensor(&oneWire);
//Servo m1;
//Servo m2;

void setup() {
  Serial.begin(9600);

  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin("Ooredoo _M30_47FB", "B93CCA0F");
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println('.');

  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("connected to wifi");
  }
  Serial.println(WiFi.localIP());


  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
//
//  dht.begin();
//  sondeSensor.begin();
//  m1.attach(M1);
//  m2.attach(M2);
//  pinMode(M1 , OUTPUT);
//  pinMode(M2 , OUTPUT);
  pinMode(movSensor,INPUT);
//  attachInterrupt(digitalPinToInterrupt(movSensor),movDetected, RISING);
}

void movDetected(){
  int movValue = digitalRead(movSensor);
  if(movValue ==1){
    Serial.println("mouvement detected ");
  digitalWrite(2 , HIGH);    
  delay(500);
  digitalWrite(2 , LOW);    
  delay(500);
  digitalWrite(2 , HIGH);    
  delay(500);
  digitalWrite(2 , LOW);    
  delay(500);
  }

}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("iotProject")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("iot/test");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again  ");

    }
  }
}
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

}
//
//void readDht() {
//  float h = dht.readHumidity();
//  float t = dht.readTemperature();
//  StaticJsonDocument<256> doc;
//  doc["humidity"] = h;
//  doc["temperature"] = t;
//
//  Serial.println("Sending message to MQTT topic..");
//  char out[128];
//  int b = serializeJson(doc, out);
//  Serial.print("bytes = ");
//  Serial.println(b, DEC);
//  if (isnan(h) || isnan(t) ) {
//    Serial.println(F("Failed to read from DHT sensor!"));
//    doc["humidity"] = 0;
//    doc["temperature"] = 0;
//    client.publish("iot/test", "error in the sensor");
//  } else {
//    client.publish("smartHouse/tempext", out); 
//  Serial.println("dht sent");
//  }
//}
//
//void readMq2() {
//  int gaz = 0;
//  gaz = analogRead(mq2);
//  if (gaz > 1800) {
//    digitalWrite(buzz, HIGH);
//  } else {
//    digitalWrite(buzz, LOW);
//  }
// String msg = "" ;
// msg.concat(gaz);
//  client.publish("smarthouse/cuisine" , msg.c_str());
//  Serial.println("gaz sent");
//}
//
//void tournerM1(bool etat) {
//  if (etat) {
//    m1.write(0);
//  } else {
//    m1.write(180);
//  }
//}
//
//void tournerM2(bool etat) {
//  if (etat) {
//    m2.write(0);
//  } else {
//    m2.write(180);
//  }
//}
//
//void readSonde() {
//  bool resistance = false;
//  sondeSensor.requestTemperatures();
//  float temperatureC = sondeSensor.getTempCByIndex(0);
//  StaticJsonDocument<256> doc;
//  doc["sonde"] = temperatureC;
//  doc["resistance"] = resistance;
//  char out[128];
//  int b = serializeJson(doc, out);
//  client.publish("smarthouse/tempSonde", out);
//  Serial.println("sonde sent");
//}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
//  delay(2000);
//  readDht();
//  delay(2000);
//  readMq2();
//  delay(2000);
//  readSonde();
movDetected();

}
