/*
  Pool Temperature Sensor for ESP8266 Wemos D1 v4
  - DS18B20 temperature sensor
  - Battery voltage measurement via voltage divider on A0
  - MQTT publish of JSON payload
  - Deep sleep with adjustable interval
  - WiFi + MQTT configuration via WiFiManager
  - Enter Config Mode on double reset using ESP_DoubleResetDetector
  - MQTT authentication with user/pass
  - Adjust sleep interval via MQTT
  - LED blink indication in Config Mode
  - Fallback to Config Mode on WiFi connection failure
*/

#include <ESP8266WiFi.h>
#include <WiFiManager.h>              // https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>            // https://github.com/knolleary/pubsubclient
#include <OneWire.h>                 // https://github.com/PaulStoffregen/OneWire
#include <DallasTemperature.h>       // https://github.com/milesburton/DallasTemperature
#include <EEPROM.h>
#include <ESP_DoubleResetDetector.h> // khoih-prog/ESP_DoubleResetDetector
#include <Ticker.h>                  // for LED blink

// Pin definitions
#define ONE_WIRE_BUS   D2    // DS18B20 data
#define LED_PIN        LED_BUILTIN  // onboard LED (D4/GPIO2)

// EEPROM layout
#define EEPROM_SIZE    512
#define INTERVAL_ADDR  0     // 1 byte
#define SERVER_ADDR    1     // length: 40 bytes
#define SERVER_SIZE    40
#define PORT_ADDR      (SERVER_ADDR + SERVER_SIZE) // length: 6 bytes
#define PORT_SIZE      6
#define USER_ADDR      (PORT_ADDR + PORT_SIZE)    // length: 32 bytes
#define USER_SIZE      32
#define PASS_ADDR      (USER_ADDR + USER_SIZE)    // length: 32 bytes
#define PASS_SIZE      32

// MQTT topics
const char* TOPIC_TEMP     = "bazen/teplota";
const char* TOPIC_INTERVAL = "bazen/interval/set";

// DRD
#define DRD_TIMEOUT    10    // seconds
#define DRD_ADDRESS    0     // EEPROM address for DRD flag

// Forward declarations
void startConfigPortal();
void loadSettings();
void saveSettings();
void connectWiFiAndMQTT();
void checkForMQTTCommands(unsigned long timeoutMs);
void sendData();
void goToSleep();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void blinkLED();

// Globals
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
WiFiClient espClient;
PubSubClient mqttClient(espClient);
WiFiManager wm;
DoubleResetDetector* drd;
Ticker blinkTicker;

int intervalMinutes = 15;  // default
char mqtt_server[SERVER_SIZE] = "mqtt.server.local";
char mqtt_port[PORT_SIZE]    = "1883";
char mqtt_user[USER_SIZE]    = "";
char mqtt_pass[PASS_SIZE]    = "";

// Read battery voltage via voltage divider 1:1 on A0
float readBatteryVoltage() {
  uint16_t raw = analogRead(A0);
  float v = raw * (3.3f / 1023.0f);
  return v * 2.0f;
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);
  sensors.begin();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // LED off (active LOW)

  String reason = ESP.getResetReason();
  Serial.printf("Reset reason: %s\n", reason.c_str());

  // Enter Config Mode on double reset (not on wake)
  if (!reason.equals("Deep-Sleep Wake") && drd->detectDoubleReset()) {
    Serial.println("** Entering Config Mode **");
    startConfigPortal();
  } else {
    loadSettings();
    connectWiFiAndMQTT();
    checkForMQTTCommands(2000);  // allow MQTT interval update
    sendData();
    goToSleep();
  }
}

void loop() {
  drd->loop();
}

// Toggle LED state
void blinkLED() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void startConfigPortal() {
  // LED blink at 0.5s interval
  blinkTicker.attach(0.5, blinkLED);

  // Custom parameters
  WiFiManagerParameter p_mqtt("mqtt",   "MQTT Server",   mqtt_server, SERVER_SIZE);
  WiFiManagerParameter p_port("port",   "MQTT Port",     mqtt_port,   PORT_SIZE);
  WiFiManagerParameter p_user("user",   "MQTT User",     mqtt_user,   USER_SIZE);
  WiFiManagerParameter p_pass("pass",   "MQTT Password", mqtt_pass,   PASS_SIZE);
  WiFiManagerParameter p_int ("interval","Interval (min)", String(intervalMinutes).c_str(), 4);

  wm.addParameter(&p_mqtt);
  wm.addParameter(&p_port);
  wm.addParameter(&p_user);
  wm.addParameter(&p_pass);
  wm.addParameter(&p_int);

  wm.setConfigPortalTimeout(180);
  if (!wm.startConfigPortal("PoolSensorAP")) {
    Serial.println("Config portal timeout");
    ESP.restart();
  }

  // Stop LED blinking
  blinkTicker.detach();
  digitalWrite(LED_PIN, HIGH); // LED off

  // Read configured values
  strncpy(mqtt_server, p_mqtt.getValue(), SERVER_SIZE);
  strncpy(mqtt_port,   p_port.getValue(),   PORT_SIZE);
  strncpy(mqtt_user,   p_user.getValue(),   USER_SIZE);
  strncpy(mqtt_pass,   p_pass.getValue(),   PASS_SIZE);
  intervalMinutes = atoi(p_int.getValue());

  saveSettings();
  ESP.restart();
}

// Save settings to EEPROM
void saveSettings() {
  EEPROM.write(INTERVAL_ADDR, intervalMinutes);
  for (int i = 0; i < SERVER_SIZE; i++) EEPROM.write(SERVER_ADDR + i, mqtt_server[i]);
  for (int i = 0; i < PORT_SIZE;   i++) EEPROM.write(PORT_ADDR   + i, mqtt_port[i]);
  for (int i = 0; i < USER_SIZE;   i++) EEPROM.write(USER_ADDR   + i, mqtt_user[i]);
  for (int i = 0; i < PASS_SIZE;   i++) EEPROM.write(PASS_ADDR   + i, mqtt_pass[i]);
  EEPROM.commit();
}

// Load settings from EEPROM
void loadSettings() {
  intervalMinutes = EEPROM.read(INTERVAL_ADDR);
  if (intervalMinutes < 1 || intervalMinutes > 1440) intervalMinutes = 15;
  for (int i = 0; i < SERVER_SIZE;  i++) mqtt_server[i] = EEPROM.read(SERVER_ADDR + i);
  mqtt_server[SERVER_SIZE-1] = '\0';
  for (int i = 0; i < PORT_SIZE;    i++) mqtt_port[i]   = EEPROM.read(PORT_ADDR   + i);
  mqtt_port[PORT_SIZE-1]   = '\0';
  for (int i = 0; i < USER_SIZE;    i++) mqtt_user[i]   = EEPROM.read(USER_ADDR   + i);
  mqtt_user[USER_SIZE-1]   = '\0';
  for (int i = 0; i < PASS_SIZE;    i++) mqtt_pass[i]   = EEPROM.read(PASS_ADDR   + i);
  mqtt_pass[PASS_SIZE-1]   = '\0';
}

// Connect to WiFi or fallback to config portal
void connectWiFiAndMQTT() {
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to WiFi");
  WiFi.begin();  
  unsigned long start = millis();
  while (millis() - start < 20000) {
    if (WiFi.status() == WL_CONNECTED) break;
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connect failed, entering config mode");
    startConfigPortal();
  }
  Serial.printf("\nWiFi IP: %s\n", WiFi.localIP().toString().c_str());

  mqttClient.setServer(mqtt_server, atoi(mqtt_port));
  mqttClient.setCallback(mqttCallback);
  while (!mqttClient.connected()) {
    Serial.print("Connecting MQTT...");
    if (mqttClient.connect("PoolSensorClient", mqtt_user, mqtt_pass)) {
      Serial.println("connected");
    } else {
      Serial.printf("Failed rc=%d, retry in 5s\n", mqttClient.state());
      delay(5000);
    }
  }
  mqttClient.subscribe(TOPIC_INTERVAL);
  Serial.printf("Subscribed to %s\n", TOPIC_INTERVAL);
}

// Allow processing incoming MQTT for given period
void checkForMQTTCommands(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    mqttClient.loop();
    drd->loop();
  }
}

// MQTT callback to update interval
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String msg = String((char*)payload);
  Serial.printf("MQTT message [%s]: %s\n", topic, msg.c_str());
  if (String(topic) == TOPIC_INTERVAL) {
    int newInterval = msg.toInt();
    if (newInterval >= 1 && newInterval <= 1440) {
      intervalMinutes = newInterval;
      saveSettings();
      Serial.printf("Interval updated to %d minutes via MQTT\n", intervalMinutes);
    } else {
      Serial.println("Invalid interval value");
    }
  }
}

// Publish sensor data then disconnect
void sendData() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  float vbat  = readBatteryVoltage();
  char payload[100];
  snprintf(payload, sizeof(payload), "{\"temperature\": %.2f, \"voltage\": %.2f}", tempC, vbat);
  Serial.printf("Publishing: %s\n", payload);
  mqttClient.publish(TOPIC_TEMP, payload);
  mqttClient.disconnect();
}

// Go to deep sleep for the configured interval
void goToSleep() {
  Serial.printf("Sleeping for %d min...\n", intervalMinutes);
  Serial.flush();
  ESP.deepSleep((uint64_t)intervalMinutes * 60ULL * 1000000ULL);
}