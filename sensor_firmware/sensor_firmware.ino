#include <WiFi.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "Password"

// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST "test.mosquitto.org"
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

#define MQTT_PUB_SENSOR1 "lavatomate/sensor1"
#define MQTT_PUB_SENSOR2 "lavatomate/sensor2"
#define MQTT_PUB_SENSOR3 "lavatomate/sensor3"

const int PIN_TO_SENSOR1 = 18;
const int PIN_TO_SENSOR2 = 19;
const int PIN_TO_SENSOR3 = 21;

String pin1State = "false";  // current state of pin
String pin2State = "false";
String pin3State = "false";

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 5000;        // Interval at which to publish sensor readings

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

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

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  // Start the DS18B20 sensor
  pinMode(PIN_TO_SENSOR1, INPUT);
  pinMode(PIN_TO_SENSOR2, INPUT);
  pinMode(PIN_TO_SENSOR3, INPUT);

  Serial.begin(115200);
  Serial.println();
  Serial.println();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

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

    if (digitalRead(PIN_TO_SENSOR1) == LOW) {
      pin1State = "false";
    }
    else if (digitalRead(PIN_TO_SENSOR1) == HIGH) {
      pin1State = "true";
    }

    if (digitalRead(PIN_TO_SENSOR2) == LOW) {
      pin2State = "false";
    }
    else if (digitalRead(PIN_TO_SENSOR2) == HIGH) {
      pin2State = "true";
    }

    if (digitalRead(PIN_TO_SENSOR3) == LOW) {
      pin3State = "false";
    }
    else if (digitalRead(PIN_TO_SENSOR3) == HIGH) {
      pin3State = "true";
    }

    // Publish an MQTT message on topic esp32/ds18b20/temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_SENSOR1, 1, true, pin1State.c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_SENSOR1);
    Serial.println(packetIdPub1);

    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_SENSOR2, 1, true, pin2State.c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_SENSOR2);
    Serial.println(packetIdPub2);

    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_SENSOR3, 1, true, pin3State.c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_SENSOR3);
    Serial.println(packetIdPub3);
  }
}
