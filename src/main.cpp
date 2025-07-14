/**
 * @file smart_ac_controller.ino
 * @brief ESP32 Smart AC Controller - IR Remote Learning, Sensor Monitoring, and MQTT Control
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
const char* MQTT_SERVER  = "test.mosquitto.org";
const int   MQTT_PORT    = 1883;
const char* DEVICE_ID    = "ac1";

// Pin Configuration
constexpr uint8_t DHTPIN           = 32;
constexpr uint8_t DHTTYPE          = DHT21;
constexpr uint8_t IR_RECV_PIN      = 33;
constexpr uint8_t IR_LED_PIN       = 14;
constexpr uint8_t BUTTON_PIN       = 26;

// EEPROM Address Mapping
constexpr int EEPROM_SIZE          = 120;
constexpr int ON_ADDR            = 0;
constexpr int OFF_ADDR             = 10;
constexpr int SET_ADDR             = 20;

// Control Thresholds
constexpr float TEMP_HIGH          = 35.0;
constexpr float TEMP_LOW           = 23.0;

// ======================= Global Objects =====================
WiFiClient espClient;
PubSubClient mqtt(espClient);
DHT     dht(DHTPIN, DHTTYPE);
IRrecv  irrecv(IR_RECV_PIN);
IRsend  irsend(IR_LED_PIN);
decode_results results;

// Control Variables
bool mode = true;  // true = Auto, false = Learn

// Debounce Variables
const unsigned long debounceDelay = 50;
bool lastStableState = HIGH;
bool lastReadState   = HIGH;
unsigned long lastDebounceTime = 0;

// Learning Mode Steps
enum IRStep {
  STEP_ON = 0,
  STEP_OFF  = 1,
  STEP_SET  = 2
};

// =================== Function Prototypes ====================
void sendIRData(int addr);
void saveIRData(int addr, uint32_t code, uint16_t bits);
void learnMode();
void autoControlMode();

// ======================= MQTT Handlers ======================
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  char msg[128];
  for (unsigned int i = 0; i < len && i < sizeof(msg) - 1; i++) {
    msg[i] = (char)payload[i];
  }
  msg[len] = '\0';

  // Debug: Print incoming topic and message
  Serial.print("[DEBUG] MQTT Topic: ");
  Serial.println(topic);
  Serial.print("[DEBUG] MQTT Payload: ");
  Serial.println(msg);

  mqtt.publish("ac1/log", msg);

  if (strcmp(msg, "on") == 0) {
    mqtt.publish("ac1/log", "[DEBUG] Received ON command.");
    sendIRData(ON_ADDR);
  } else if (strcmp(msg, "off") == 0) {
    mqtt.publish("ac1/log", "[DEBUG] Received OFF command.");
    sendIRData(OFF_ADDR);
  } else if (strcmp(msg, "set") == 0) {
    mqtt.publish("ac1/log", "[DEBUG] Received SET command.");
    sendIRData(SET_ADDR);
  } else if (strcmp(msg, "auto") == 0) {
    mode = true;
    mqtt.publish("ac1/log", "[DEBUG] Switched to AUTO MODE from MQTT.");
  } else if (strcmp(msg, "learn") == 0) {
    mode = false;
    mqtt.publish("ac1/log", "[DEBUG] Switched to LEARN MODE from MQTT.");
  } else {
    mqtt.publish("ac1/log", "[DEBUG] Unknown MQTT command received.");
  }
}

void mqttReconnect() {
  while (!mqtt.connected()) {
    Serial.println("[DEBUG] Attempting MQTT connection...");
    if (mqtt.connect(DEVICE_ID)) {
      mqtt.subscribe("ac1/cmd");
      mqtt.publish("ac1/log", "[DEBUG] MQTT connected and subscribed to ac1/cmd.");
      Serial.println("[DEBUG] MQTT connected and subscribed to ac1/cmd.");
    } else {
      Serial.print("[DEBUG] Failed MQTT connection. State: ");
      Serial.println(mqtt.state());
      delay(1000);
    }
  }
}

  // ======================= Setup ==============================
void setup() {
  Serial.begin(115200);

  WiFi.begin(SSID, PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connected. IP: ");
  Serial.println(WiFi.localIP());

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  EEPROM.begin(EEPROM_SIZE);
  irrecv.enableIRIn();
  irsend.begin();
  dht.begin();
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  mqtt.publish("ac1/log", "[DEBUG] System Initialized. Press button to switch mode.");
  Serial.println("[DEBUG] System Initialized. Press button to switch mode.");
}

// ======================= Loop ===============================
void loop() {
  if (!mqtt.connected()) mqttReconnect();
  mqtt.loop();

  // Debounce button
  bool reading = digitalRead(BUTTON_PIN);
  if (reading != lastReadState) lastDebounceTime = millis();
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != lastStableState) {
      lastStableState = reading;
      if (lastStableState == HIGH) {
        mode = !mode;
        const char* msg = mode ? "Switched to AUTO CONTROL Mode" : "Switched to LEARNING Mode";
        Serial.println(msg);
        mqtt.publish("ac1/log", msg);
      }
    }
  }
  lastReadState = reading;

  if (mode)
    autoControlMode();
  else
    learnMode();
}

// ======================= Learn Mode =========================
void learnMode() {
  static IRStep step = STEP_ON;

  if (irrecv.decode(&results)) {
    char buf[64];
    snprintf(buf, sizeof(buf), "[DEBUG] Received IR %d. Saving...", step + 1);
    Serial.println(buf);
    mqtt.publish("ac1/log", buf);

    if (results.decode_type != decode_type_t::UNKNOWN) {
      switch (step) {
        case STEP_ON: saveIRData(ON_ADDR, results.value, results.bits); break;
        case STEP_OFF:  saveIRData(OFF_ADDR,  results.value, results.bits); break;
        case STEP_SET:  saveIRData(SET_ADDR,  results.value, results.bits); break;
      }

      step = static_cast<IRStep>(step + 1);
      if (step > STEP_SET) {
        mqtt.publish("ac1/log", "[DEBUG] All signals saved. Switching to AUTO.");
        step = STEP_ON;
        mode = true;
      }
    }

    irrecv.resume();
  }
}

// =================== Auto Control Mode ======================
void autoControlMode() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 5000) return;
  lastUpdate = millis();

  float temp = dht.readTemperature();
  float hum  = dht.readHumidity();

  // char logbuf[128];
  // snprintf(logbuf, sizeof(logbuf), "[DEBUG] Temp: %.1fC, Hum: %.1f%%", temp, hum);
  // Serial.println(logbuf);
  // mqtt.publish("ac1/log", logbuf);

  char jsonbuf[128];
  snprintf(jsonbuf, sizeof(jsonbuf), "{\"temp\":%.1f,\"hum\":%.1f}", temp, hum);
  mqtt.publish("ac1/status", jsonbuf);

  if (temp >= TEMP_HIGH) {
    mqtt.publish("ac1/log", "[DEBUG] Temp high. Sending ON signal.");
    sendIRData(ON_ADDR);
  } else if (temp <= TEMP_LOW) {
    mqtt.publish("ac1/log", "[DEBUG] Temp low. Sending OFF signal.");
    sendIRData(OFF_ADDR);
  }
}

// ======================= EEPROM I/O =========================
void saveIRData(int addr, uint32_t code, uint16_t bits) {
  EEPROM.put(addr, code);         // 4 bytes
  EEPROM.put(addr + 4, bits);     // 2 bytes
  EEPROM.commit();

  char buf[64];
  snprintf(buf, sizeof(buf), "[DEBUG] Saved IR 0x%08X (%d bits) at %d", code, bits, addr);
  Serial.println(buf);
  mqtt.publish("ac1/log", buf);
}

void sendIRData(int addr) {
  uint32_t code;
  uint16_t bits;
  EEPROM.get(addr, code);
  EEPROM.get(addr + 4, bits);
    // Serial.print("Transmitting IR Code: 0x");
    // Serial.println(code, HEX);
    // Serial.print("Bit Length: ");
    // Serial.println(bits);
  irsend.sendNEC(code, bits);  // Use correct protocol here if not NEC

  char buf[64];
  snprintf(buf, sizeof(buf), "[DEBUG] Sent IR 0x%08X (%d bits) from EEPROM @%d", code, bits, addr);
  Serial.println(buf);
  mqtt.publish("ac1/log", buf);
}
