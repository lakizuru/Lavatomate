#include <WiFi.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_MOSI   23
#define OLED_CLK    18
#define OLED_DC     16
#define OLED_CS     5
#define OLED_RESET  17

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
                         OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

//replace with your network credentials
#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "Password"


#define MQTT_HOST "test.mosquitto.org"
#define MQTT_PORT 1883

//MQTT Topics
#define MQTT_SUB_SENSOR_1 "lavatomate/sensor1"
#define MQTT_SUB_SENSOR_2 "lavatomate/sensor2"
#define MQTT_SUB_SENSOR_3 "lavatomate/sensor3"

#define MQTT_SUB_STATUS_1 "lavatomate/status1"
#define MQTT_SUB_STATUS_2 "lavatomate/status2"
#define MQTT_SUB_STATUS_3 "lavatomate/status3"

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

char vacancies[9] = {'A', ' ', 'B', ' ', 'C', ' ', '-', ' ', '-'};

bool disabled[5] = {false, false, false, false, false};

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
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub1 = mqttClient.subscribe(MQTT_SUB_SENSOR_1, 2);
  uint16_t packetIdSub2 = mqttClient.subscribe(MQTT_SUB_SENSOR_2, 2);
  uint16_t packetIdSub3 = mqttClient.subscribe(MQTT_SUB_SENSOR_3, 2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
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
}


void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  String messageTemp;
  for (int i = 0; i < len; i++) {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }

  if (String(topic) == "lavatomate/sensor1") {
    Serial.println(messageTemp);
    if (messageTemp == "true"){
      vacancies[0] = ' ';
    }
    else if (messageTemp == "false" and disabled[0] == true){
      vacancies[0] = ' ';
    }
    else if (messageTemp == "false" and disabled[0] == false){
      vacancies[0] = 'A';
    }
  }

  if (String(topic) == "lavatomate/sensor2") {
    Serial.println(messageTemp);
    if (messageTemp == "true"){
      vacancies[2] = ' ';
    }
    else if (messageTemp == "false" and disabled[1] == true){
      vacancies[2] = ' ';
    }
    else if (messageTemp == "false" and disabled[1] == false){
      vacancies[2] = 'B';
    }
  }

  if (String(topic) == "lavatomate/sensor3") {
    Serial.println(messageTemp);
    if (messageTemp == "true"){
      vacancies[4] = ' ';
    }
    else if (messageTemp == "false" and disabled[2] == true){
      vacancies[4] = ' ';
    }
    else if (messageTemp == "false" and disabled[2] == false){
      vacancies[4] = 'C';
    }
  }

  if (String(topic) == "lavatomate/status1") {
    Serial.println(messageTemp);
    if (messageTemp == "true"){
      disabled[0] = true;
    }
    else if (messageTemp == "false"){
      disabled[0] = false;
    }
  }

  if (String(topic) == "lavatomate/status2") {
    Serial.println(messageTemp);
    if (messageTemp == "true"){
      disabled[1] = true;
    }
    else if (messageTemp == "false"){
      disabled[1] = false;
    }
  }

  if (String(topic) == "lavatomate/status3") {
    Serial.println(messageTemp);
    if (messageTemp == "true"){
      disabled[2] = true;
    }
    else if (messageTemp == "false"){
      disabled[2] = false;
    }
  }
}

void setup() {
  Serial.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //initialize OLED
  delay(2000);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  Serial.println();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToWifi();
}

void loop() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 2);
  display.print("Lavatomate");
  display.setCursor(0, 30);
  display.setTextSize(1.5);
  display.print("Vacant Lavatories: ");
  display.setCursor(0, 50);
  display.setTextSize(2);
  display.print(vacancies);
  display.display();
  display.startscrollleft(0x0D, 0x0F);
}
