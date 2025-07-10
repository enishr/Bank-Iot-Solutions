/**
 * @file smart_ac_controller.ino
 * @brief ESP32 Smart AC Controller - IR Remote Learning, Sensor Monitoring, and MQTT Control
 *
 * This code allows an ESP32-based system to learn IR signals from AC remotes,
 * store them in EEPROM, and control the AC based on temperature, humidity,
 * and power consumption. Supports MQTT for remote control and logging.
 *
 * Modules used:
 * - DHT21 (AM2301) for Temperature and Humidity
 * - SCT-013-020 for Current Sensing
 * - TSOP1738 (IR Receiver) and IR LED for IR Communication
 * - MQTT via Eclipse Mosquitto for cloud messaging
 */

// ======================= Libraries ==========================
#include <WiFi.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <IRsend.h>
#include <EEPROM.h>
#include <DHT.h>
#include <PubSubClient.h>

// ======================= Configuration ======================
// Wi-Fi Credentials
const char* SSID         = "RJ";
const char* PASS         = "Shikareni";

// MQTT Broker Configuration
const char* MQTT_SERVER  = "172.28.1.8";
const int   MQTT_PORT    = 1883;
const char* DEVICE_ID    = "ac1";

// Pin Configuration
#define DHTPIN              32
#define DHTTYPE             DHT21
#define IR_RECV_PIN         35
#define IR_LED_PIN          4
#define CURRENT_SENSOR_PIN  34
#define BUTTON_PIN          18

// EEPROM Address Mapping
#define EEPROM_SIZE         1200
#define COOL_ADDR           0
#define FAN_ADDR            400
#define OFF_ADDR            800

// Control Thresholds
float TEMP_HIGH    = 27.0;
float TEMP_LOW     = 23.0;
float CURRENT_ZERO = 0.2; // in Amps

// ======================= Global Objects =====================
WiFiClient espClient;
PubSubClient mqtt(espClient);
DHT     dht(DHTPIN, DHTTYPE);
IRrecv  irrecv(IR_RECV_PIN);
IRsend  irsend(IR_LED_PIN);
decode_results results;

// Control Variables
bool mode = true; // true: AUTO mode, false: LEARN mode
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;
unsigned long lastMQTT = 0;

// ================= Function Declarations =================
void sendIRData(int addr);
void saveIRData(int addr, volatile uint16_t* rawbuf, uint16_t len);
void learnMode();
void autoControlMode();
//float readCurrent();

// ======================= MQTT Handlers ======================
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  char msg[128];
  for (unsigned int i = 0; i < len && i < sizeof(msg) - 1; i++) msg[i] = (char)payload[i];
  msg[len] = '\0';

  Serial.println(msg); // [DEBUG] Print received MQTT message
  mqtt.publish("ac1/log", msg);

  if (strcmp(msg, "cool") == 0) sendIRData(COOL_ADDR);
  else if (strcmp(msg, "fan") == 0) sendIRData(FAN_ADDR);
  else if (strcmp(msg, "off") == 0) sendIRData(OFF_ADDR);
  else if (strcmp(msg, "auto") == 0) mode = true;
  else if (strcmp(msg, "learn") == 0) mode = false;
}

void mqttReconnect() {
  while (!mqtt.connected()) {
    //Serial.println("[DEBUG] Attempting MQTT connection..."); // [DEBUG]
    if (mqtt.connect(DEVICE_ID)) {
      mqtt.subscribe("ac1/cmd");
      char buf[64];
      snprintf(buf, sizeof(buf), "MQTT connected and subscribed.");
      Serial.println(buf);
      mqtt.publish("ac1/log", buf);
    } else {
      //Serial.println("[DEBUG] MQTT connection failed, retrying..."); // [DEBUG]
      delay(1000);
    }
  }
}

// ======================= Setup ==============================
void setup() {
  Serial.begin(115200);
  //Serial.println("[DEBUG] Serial started"); // [DEBUG]

  WiFi.begin(SSID, PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250); Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi up, IP: ");
  Serial.println(WiFi.localIP());

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  EEPROM.begin(EEPROM_SIZE);
  irrecv.enableIRIn();
  irsend.begin();
  dht.begin();
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  char buf[64];
  snprintf(buf, sizeof(buf), "System Initialized. Press button to enter Learning Mode.");
  Serial.println(buf);
  mqtt.publish("ac1/log", buf);
}

// ======================= Main Loop ==========================
void loop() {
   if (!mqtt.connected()) mqttReconnect();
 mqtt.loop();

static bool lastButtonState = HIGH;
static unsigned long lastDebounceTime = 0;
bool currentState = digitalRead(BUTTON_PIN);

if (currentState != lastButtonState) {
  lastDebounceTime = millis();
}
if ((millis() - lastDebounceTime) > debounceDelay) {
  if (lastButtonState == HIGH && currentState == LOW) {
    mode = !mode;
    char buf[64];
    snprintf(buf, sizeof(buf), mode ? "Switched to AUTO CONTROL Mode" : "Switched to LEARNING Mode");
    Serial.println(buf);
    mqtt.publish("ac1/log", buf);
  }
}
lastButtonState = currentState;


  if (!mode) learnMode();
  else autoControlMode();
}

// ======================= Learn Mode =========================
void learnMode() {
  static int step = 0;

  if (irrecv.decode(&results)) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Received IR signal %d. Saving...", step + 1);
    Serial.println(buf);
    mqtt.publish("ac1/log", buf);

    uint16_t len = getCorrectedRawLength(&results);
    //Serial.printf("[DEBUG] Raw length = %d\n", len); // [DEBUG]

    if      (step == 0) saveIRData(COOL_ADDR, results.rawbuf, len);
    else if (step == 1) saveIRData(FAN_ADDR,  results.rawbuf, len);
    else if (step == 2) saveIRData(OFF_ADDR,  results.rawbuf, len);
    if (++step >= 3) {
      snprintf(buf, sizeof(buf), "All signals saved. Switching to AUTO.");
      Serial.println(buf);
      mqtt.publish("ac1/log", buf);
      mode = true;
      step = 0;
    }
    irrecv.resume();
  }
}

// ======================= Auto Control Mode ==================
void autoControlMode() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 5000) return;
  lastUpdate = millis();

  float temp = dht.readTemperature();
  float hum  = dht.readHumidity();
  //float current = readCurrent();

 // Serial.printf("[DEBUG] Sensor Readings -> Temp: %.1f, Hum: %.1f, Curr: %.2f\n", temp, hum, current); // [DEBUG]

  char logbuf[128];
  snprintf(logbuf, sizeof(logbuf), "Temp: %.1fC, Hum: %.1f%%, temp, hum" );
  Serial.println(logbuf);
  mqtt.publish("ac1/log", logbuf);

  char msg[128];
  snprintf(msg, sizeof(msg), "{\"temp\":%.1f,\"hum\":%.1f}", temp, hum);
  mqtt.publish("ac1/status", msg);

  if      (temp >= TEMP_HIGH) {
    snprintf(logbuf, sizeof(logbuf), "Sending COOL signal.");
    Serial.println(logbuf);
    mqtt.publish("ac1/log", logbuf);
    sendIRData(COOL_ADDR);
  }
  else if (temp <= TEMP_LOW ) {
    snprintf(logbuf, sizeof(logbuf), "Sending FAN signal.");
    Serial.println(logbuf);
    mqtt.publish("ac1/log", logbuf);
    sendIRData(FAN_ADDR);
  }
}

// ======================= EEPROM Handlers ====================
void saveIRData(int addr, volatile uint16_t* rawbuf, uint16_t len) {
  //Serial.printf("[DEBUG] Saving IR Data to EEPROM at %d, Length: %d\n", addr, len); // [DEBUG]
  EEPROM.write(addr,     len >> 8);
  EEPROM.write(addr + 1, len & 0xFF);
  for (int i = 0; i < len; i++) {
    EEPROM.write(addr + 2 + i * 2, rawbuf[i] >> 8);
    EEPROM.write(addr + 3 + i * 2, rawbuf[i] & 0xFF);
  }
  EEPROM.commit();
  char buf[64];
  snprintf(buf, sizeof(buf), "Saved %d values to EEPROM @%d", len, addr);
  Serial.println(buf);
  mqtt.publish("ac1/log", buf);
}

void sendIRData(int addr) {
  uint16_t len = (EEPROM.read(addr) << 8) | EEPROM.read(addr + 1);
  //Serial.printf("[DEBUG] Sending IR Data from EEPROM at %d, Length: %d\n", addr, len); // [DEBUG]
  uint16_t raw[len];
  for (int i = 0; i < len; i++) {
    raw[i] = (EEPROM.read(addr + 2 + i * 2) << 8) | EEPROM.read(addr + 3 + i * 2);
  }
  irsend.sendRaw(raw, len, 38);
  char buf[64];
  snprintf(buf, sizeof(buf), "Sent IR from EEPROM @%d", addr);
  Serial.println(buf);
  mqtt.publish("ac1/log", buf);
}

// ======================= Current Sensor =====================
// float readCurrent() {
//   int adc = analogRead(CURRENT_SENSOR_PIN);
//   float v = adc * (3.3f / 4095.0f);
//   float amps = (v / 1.0f) * 20.0f; For 20A/V sensor
//   Serial.printf("[DEBUG] ADC: %d, Voltage: %.2f, Current: %.2f\n", adc, v, amps); [DEBUG]
//   return amps;
// }
