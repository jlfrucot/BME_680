/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-mqtt-publish-bme680-arduino/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <Wire.h>
#include <SPI.h>
#include <Update.h>
#include <SPIFFS.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <ArduinoJson.h>

// #include "wifikeys.h"

// Enable/Disable Serial
#define DEBUG false // true pour avoir les debug sur le port série
#define Serial if(DEBUG)Serial

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 15       /* Time ESP32 will go to sleep (in seconds) */

// #define WIFI_SSID "Livebox-E4C0"
// #define WIFI_PASSWORD "745gUQSoRdbPfAZbxT"

// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 1, 100)
#define MQTT_PORT 1883

// Temperature MQTT Topics
#define MQTT_PUB_TEMP "esp/bme680/temperature"
#define MQTT_PUB_HUM  "esp/bme680/humidity"
#define MQTT_PUB_PRES "esp/bme680/pressure"
#define MQTT_PUB_GAS  "esp/bme680/gas"
#define BME_TEMP_OFFSET 3.5

/*#define BME_SCK 14
#define BME_MISO 12
#define BME_MOSI 13
#define BME_CS 15*/

Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

// Variables to hold sensor readings
float temperature;
float humidity;
float pressure;
float gasResistance;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
// TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings

void getBME680Readings(){
  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }
  temperature = bme.temperature - BME_TEMP_OFFSET;
  pressure = bme.pressure / 100.0;
  humidity = bme.humidity;
  gasResistance = bme.gas_resistance / 1000.0;
}

// void connectToWifi() {
//   Serial.println("Connecting to Wi-Fi...");
//   WiFi.config(deviceLocaleIP, gateway, subnet); // Parametrer IP fixe
//   WiFi.begin(ssid, password);
// }
void restartESP32Cam()
{
    esp_sleep_enable_timer_wakeup(uS_TO_S_FACTOR * TIME_TO_SLEEP);
    esp_deep_sleep_start();
}
void onWiFiEventFired(WiFiEvent_t event)
{
    // On traite les évènements émis par le wifi
    switch (event)
    {
    case SYSTEM_EVENT_STA_DISCONNECTED:
        // Si on est déconnecté, on redémarre l'esp
        Serial.println("On a perdu le wifi");
        restartESP32Cam();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.print("On a obtenu une adresse IP : ");
        Serial.println(WiFi.localIP());
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        Serial.println("WiFi connecté");
        break;
    default:
        break;
    }
}
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

// void WiFiEvent(WiFiEvent_t event) {
//   Serial.printf("[WiFi-event] event: %d\n", event);
//   switch(event) {
//     case SYSTEM_EVENT_STA_GOT_IP:
//       Serial.println("WiFi connected");
//       Serial.println("IP address: ");
//       Serial.println(WiFi.localIP());
//       connectToMqtt();
//       break;
//     case SYSTEM_EVENT_STA_DISCONNECTED:
//       Serial.println("WiFi lost connection");
//       xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
//       // xTimerStart(wifiReconnectTimer, 0);
//       break;
//     default:
//       break;
//   }
// }

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
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

  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1);
  }
  ////////////////////////     WiFi    ////////////////////////////////
  // On tente de récupérer les paramètres enregistrés dans la mémoire permanente

    // Démarre le système de fichier SPIFFS
    if (!SPIFFS.begin())
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
    }
    File file = SPIFFS.open("/settings.json"); // On ouvre le fichier contenant les différents paramètres

    StaticJsonDocument<500> doc;
    DeserializationError error = deserializeJson(doc, file);
    if (error)
    {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        // return;
    }

    JsonObject wifi = doc["wifi"];

    JsonArray wifi_localeIP = wifi["localeIP"];
    IPAddress mylocalIp(wifi_localeIP[0],
                        wifi_localeIP[1],
                        wifi_localeIP[2],
                        wifi_localeIP[3]);

    JsonArray wifi_subnet = wifi["subnet"];
    IPAddress subnet(wifi_subnet[0],
                     wifi_subnet[1],
                     wifi_subnet[2],
                     wifi_subnet[3]);

    JsonArray wifi_gateway = wifi["gateway"];
    IPAddress gateway(wifi_gateway[0],
                      wifi_gateway[1],
                      wifi_gateway[2],
                      wifi_gateway[3]);
    JsonObject mqtt = doc["remoteMQTT"];
    JsonArray mqttBrokerIP = mqtt["mqttIP"];
    IPAddress mqttIP(mqttBrokerIP[0],
                      mqttBrokerIP[1],
                      mqttBrokerIP[2],
                      mqttBrokerIP[3]);
    const char *mqttUser = doc["mqttkey"]["MQTTUser"];
    const char *mqttPassword = doc["mqttkey"]["MQTTPassword"];
    const char *wifikey_ssid = doc["wifikey"]["ssid"];
    const char *wifikey_password = doc["wifikey"]["password"];

    int webserverPort = doc["webserverPort"];

    // Fix WiFi static IP
    
    Serial.println(wifikey_ssid);
    Serial.println(wifikey_password);
    WiFi.mode(WIFI_STA);
    // WiFi.onEvent(onWiFiEventFired);
    WiFi.config(mylocalIp, gateway, subnet);
    WiFi.begin(wifikey_ssid, wifikey_password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(F("."));
    }

    Serial.println(F("WiFi connected"));
    Serial.println("");
    Serial.println(WiFi.localIP());

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  // wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(onWiFiEventFired);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(mqttIP, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  mqttClient.setCredentials(mqttUser, mqttPassword);
  // connectToWifi();
  
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
file.close();
}

void loop() {
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    
    getBME680Readings();
    Serial.println();
    Serial.printf("Temperature = %.2f ºC \n", temperature - 2.0);
    Serial.printf("Humidity = %.2f % \n", humidity);
    Serial.printf("Pressure = %.2f hPa \n", pressure);
    Serial.printf("Gas Resistance = %.2f KOhm \n", gasResistance);
    
    // Publish an MQTT message on topic esp/bme680/temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temperature).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_TEMP, packetIdPub1);
    Serial.printf("Message: %.2f \n", temperature);

    // Publish an MQTT message on topic esp/bme680/humidity
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(humidity).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub2);
    Serial.printf("Message: %.2f \n", humidity);

    // Publish an MQTT message on topic esp/bme680/pressure
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_PRES, 1, true, String(pressure).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_PRES, packetIdPub3);
    Serial.printf("Message: %.2f \n", pressure);

    // Publish an MQTT message on topic esp/bme680/gas
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_GAS, 1, true, String(gasResistance).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_GAS, packetIdPub4);
    Serial.printf("Message: %.2f \n", gasResistance);
  }
}
