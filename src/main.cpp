/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp8266-nodemcu-mqtt-publish-dht11-dht22-arduino/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include "DHT.h"
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>

#define WIFI_SSID "OPTUS_48EA0C"
#define WIFI_PASSWORD "tajesgrist44107"

// Raspberri Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(27, 32, 148, 185)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1884

// Temperature MQTT Topics
#define MQTT_PUB_TEMP1 "esp3/dht1/temperature"
#define MQTT_PUB_HUM1 "esp3/dht1/humidity"
#define MQTT_PUB_TEMP2 "esp3/dht2/temperature"
#define MQTT_PUB_HUM2 "esp3/dht2/humidity"

// Digital pin connected to the DHT sensor
#define DHTPIN1 D5
#define DHTPIN2 D6
#define LED D2
// Uncomment whatever DHT sensor type you're using
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE11 DHT11
#define DHTTYPE22 DHT22      // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)   

// Initialize DHT sensor
DHT dht1(DHTPIN1, DHTTYPE11);
DHT dht2(DHTPIN2, DHTTYPE22);

// Variables to hold sensor readings
float temp1;
float hum1;
float temp2;
float hum2;

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  digitalWrite(2, LOW); // turn the LED on.
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  digitalWrite(2, HIGH);// turn the LED off.
  wifiReconnectTimer.once(2, connectToWifi);
}



void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {

    mqttReconnectTimer.once(5, connectToMqtt);

  }
}

/*void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}*/

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}



void setup() {
  Serial.begin(115200);
  Serial.println();

  pinMode(2, OUTPUT);    // LED pin as output.
  dht1.begin();
  dht2.begin();
  
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
  WiFi.setPhyMode(WIFI_PHY_MODE_11B);
  
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  //mqttClient.setCredentials("REPlACE_WITH_YOUR_USER", "REPLACE_WITH_YOUR_PASSWORD");
  
  connectToWifi();
  

}



void loop() {
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    // New DHT sensor readings
    hum1 = dht1.readHumidity();
    hum2 = dht2.readHumidity();
    // Read temperature as Celsius (the default)
    temp1 = dht1.readTemperature();
    temp2 = dht2.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    //temp = dht.readTemperature(true);
    digitalWrite(2, HIGH);// turn the LED off.
    // Publish an MQTT message on topic esp/dht/temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP1, 1, true, String(temp1).c_str());
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_TEMP2, 1, true, String(temp2).c_str());                              
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_TEMP1, packetIdPub1);
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_TEMP2, packetIdPub2);
    Serial.printf("Message: %.2f \n", temp1);

    // Publish an MQTT message on topic esp/dht/humidity
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_HUM1, 1, true, String(hum1).c_str());   
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_HUM2, 1, true, String(hum2).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM1, packetIdPub3);
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM2, packetIdPub4);
    Serial.printf("Message: %.2f \n", hum1);


 if (WiFi.isConnected()) {
  delay(500);
digitalWrite(2, LOW); // turn the LED on.
 }

  }




}