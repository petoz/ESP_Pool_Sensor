/*
  Pool Temperature Sensor for ESP8266 Wemos D1 v4
  - DS18B20 temperature sensor
  - Battery voltage measurement via voltage divider on A0
  - MQTT publish of JSON payload
  - Deep sleep with adjustable interval
  - WiFi + MQTT configuration via WiFiManager
  - Enter Config Mode on double reset using ESP_DoubleResetDetector
*/

#include <ESP8266WiFi.h>
#include <WiFiManager.h>              // https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>            // https://github.com/knolleary/pubsubclient
#include <OneWire.h>                 // https://github.com/PaulStoffregen/OneWire
#include <DallasTemperature.h>       // https://github.com/milesburton/DallasTemperature
#include <EEPROM.h>
#include <ESP_DoubleResetDetector.h> // khoih-prog/ESP_DoubleResetDetector

// Pin definitions
#define ONE_WIRE_BUS   D2    // DS18B20 data

// EEPROM layout
#define EEPROM_SIZE    512
#define INTERVAL_ADDR  0     // 1 byte
#define SERVER_ADDR    1     // length: 40 bytes
#define SERVER_SIZE    40
#define PORT_ADDR      (SERVER_ADDR + SERVER_SIZE) // length: 6 bytes
#define PORT_SIZE      6

// Double reset detector
#define DRD_TIMEOUT    10    // seconds
#define DRD_ADDRESS    0     // EEPROM address for DRD

// Forward declarations
void startConfigPortal();
void loadSettings();
void saveSettings();
void connectWiFiAndMQTT();
void sendData();
void goToSleep();

// Globals
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
WiFiClient espClient;
PubSubClient mqttClient(espClient);
WiFiManager wm;
DoubleResetDetector* drd;

int intervalMinutes = 15;  // default
char mqtt_server[SERVER_SIZE] = "mqtt.server.local";
char mqtt_port[PORT_SIZE]    = "1883";

// Read battery voltage via voltage divider 1:1 on A0
float readBatteryVoltage() {
  uint16_t raw = analogRead(A0);
  float v = raw * (3.3f / 1023.0f);
  return v * 2.0f;
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  // Initialize DRD
  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);

  // Start temperature sensor
  sensors.begin();

  // Check double reset for entering config mode
  if (drd->detectDoubleReset()) {
    Serial.println("** Double Reset Detected - Entering Config Mode **");
    startConfigPortal();
  } else {
    // Normal operation
    loadSettings();
    connectWiFiAndMQTT();
    sendData();
    goToSleep();
  }
}

void loop() {
  // Needed for DRD timing
  drd->loop();
}

// Launch WiFiManager portal to configure MQTT and interval
void startConfigPortal() {
  // Custom parameters
  WiFiManagerParameter custom_mqtt("mqtt",    "MQTT Server", mqtt_server, SERVER_SIZE);
  WiFiManagerParameter custom_port("port",    "MQTT Port",   mqtt_port,   PORT_SIZE);
  WiFiManagerParameter custom_int("interval","Send Interval (min)", String(intervalMinutes).c_str(), 4);

  wm.addParameter(&custom_mqtt);
  wm.addParameter(&custom_port);
  wm.addParameter(&custom_int);

  wm.setConfigPortalTimeout(180);          // 3 minutes
  if (!wm.startConfigPortal("PoolSensorAP")) {
    Serial.println("Config portal timeout");
    ESP.restart();
  }

  // Read back values
  strncpy(mqtt_server, custom_mqtt.getValue(), SERVER_SIZE);
  strncpy(mqtt_port,   custom_port.getValue(),   PORT_SIZE);
  intervalMinutes = atoi(custom_int.getValue());

  saveSettings();
  Serial.println("Settings saved, restarting...");
  delay(500);
  ESP.restart();
}

// Save interval and MQTT settings into EEPROM
void saveSettings() {
  EEPROM.write(INTERVAL_ADDR, intervalMinutes);
  for (uint16_t i = 0; i < SERVER_SIZE; i++) {
    EEPROM.write(SERVER_ADDR + i, mqtt_server[i]);
  }
  for (uint16_t i = 0; i < PORT_SIZE; i++) {
    EEPROM.write(PORT_ADDR + i, mqtt_port[i]);
  }
  EEPROM.commit();
}

// Load settings from EEPROM (or keep defaults)
void loadSettings() {
  intervalMinutes = EEPROM.read(INTERVAL_ADDR);
  if (intervalMinutes < 1 || intervalMinutes > 1440) intervalMinutes = 15;

  for (uint16_t i = 0; i < SERVER_SIZE; i++) {
    mqtt_server[i] = EEPROM.read(SERVER_ADDR + i);
  }
  mqtt_server[SERVER_SIZE - 1] = '\0';

  for (uint16_t i = 0; i < PORT_SIZE; i++) {
    mqtt_port[i] = EEPROM.read(PORT_ADDR + i);
  }
  mqtt_port[PORT_SIZE - 1] = '\0';
}

// Connect to WiFi (stored creds) and then MQTT
void connectWiFiAndMQTT() {
  WiFi.mode(WIFI_STA);
  wm.autoConnect("PoolSensorAP");
  Serial.println("WiFi connected: ");
  Serial.println(WiFi.localIP());

  mqttClient.setServer(mqtt_server, atoi(mqtt_port));
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("PoolSensorClient")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// Read sensors and publish JSON payload
void sendData() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  float vbat  = readBatteryVoltage();

  char payload[100];
  snprintf(payload, sizeof(payload),
           "{\"temperature\": %.2f, \"voltage\": %.2f}", tempC, vbat);

  Serial.print("Publishing: "); Serial.println(payload);
  mqttClient.publish("bazen/teplota", payload);
  mqttClient.disconnect();
}

// Enter deep sleep for "intervalMinutes" minutes
void goToSleep() {
  Serial.print("Sleeping for "); Serial.print(intervalMinutes);
  Serial.println(" minutes...");
  Serial.flush();
  ESP.deepSleep(intervalMinutes * 60ULL * 1000000ULL);
}
