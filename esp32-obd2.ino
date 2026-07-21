/*
 * ESP32 OBD2 -> Home Assistant (MQTT) + Prometheus (/metrics)
 * ---------------------------------------------------------------
 * Reads live OBD2 PIDs from a Bluetooth CLASSIC (SPP) ELM327 dongle
 * (e.g. Veepeak Mini VP11) and exposes them two ways:
 *   - HTTP  GET /metrics   -> Prometheus text format (scrape this)
 *   - MQTT  (optional)     -> Home Assistant auto-discovery entities
 *   - HTTP  /update        -> ElegantOTA web page (network reflash)
 *
 * First-boot config: no WiFi/MQTT credentials are baked in. The board
 * starts a captive-portal WiFi AP named "ESP32-OBD2" (password: obd2setup).
 * Join it, and a page opens where you enter your WiFi + (optional) MQTT
 * broker. Settings are saved to flash.
 *
 * REQUIRED HARDWARE: original ESP32 (WROOM / esp32dev). The S3/C3 will
 * NOT work with a Classic-Bluetooth dongle (they are BLE-only).
 *
 * Libraries (installed automatically by the GitHub Actions build):
 *   - ELMduino        (PowerBroker2)
 *   - WiFiManager     (tzapu)
 *   - PubSubClient    (knolleary)
 *   - ElegantOTA      (ayushsharma82)   -- set to sync WebServer mode
 *
 * Board core: esp32 by Espressif  (FQBN esp32:esp32:esp32)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "BluetoothSerial.h"
#include "ELMduino.h"
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ElegantOTA.h>

// ----------------------------------------------------------------------------
// Compile-time guards
// ----------------------------------------------------------------------------
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth Classic not available. Use an original ESP32 (esp32dev), not S3/C3."
#endif

// ----------------------------------------------------------------------------
// Config (persisted in NVS via Preferences)
// ----------------------------------------------------------------------------
Preferences prefs;

char mqttHost[64] = "";       // leave blank to disable MQTT (Prometheus still works)
char mqttPort[6]  = "1883";
char mqttUser[32] = "";
char mqttPass[32] = "";
char btName[24]   = "OBDII";  // fallback if MAC is blank/invalid
char btMac[20]    = "00:1D:A5:07:5D:3C"; // preferred: connect by MAC (most reliable)
char btPin[8]     = "1234";
char nodeName[24] = "esp32_obd2";

bool shouldSaveConfig = false;

// ----------------------------------------------------------------------------
// Globals
// ----------------------------------------------------------------------------
BluetoothSerial SerialBT;
ELM327 myELM327;
WebServer server(80);
WiFiClient espClient;
PubSubClient mqtt(espClient);

bool btConnected = false;
bool elmReady    = false;
bool haDiscoverySent = false;

// Latest sensor values (NAN = not yet read / unsupported)
struct Metric {
  const char* key;      // prometheus/mqtt key
  const char* name;     // friendly name for HA
  const char* unit;     // unit of measurement
  const char* devClass; // HA device_class ("" if none)
  float value;
};

enum PidIndex {
  P_RPM = 0, P_SPEED, P_COOLANT, P_LOAD, P_THROTTLE,
  P_INTAKE, P_MAF, P_VBAT, P_COUNT
};

Metric metrics[P_COUNT] = {
  { "engine_rpm",       "Engine RPM",          "rpm", "",            NAN },
  { "vehicle_speed",    "Vehicle Speed",       "km/h","speed",       NAN },
  { "coolant_temp",     "Coolant Temperature", "°C",  "temperature", NAN },
  { "engine_load",      "Engine Load",         "%",   "",            NAN },
  { "throttle_pos",     "Throttle Position",   "%",   "",            NAN },
  { "intake_temp",      "Intake Air Temp",     "°C",  "temperature", NAN },
  { "maf_rate",         "MAF Rate",            "g/s", "",            NAN },
  { "battery_voltage",  "Battery Voltage",     "V",   "voltage",     NAN },
};

uint8_t pidState = P_RPM;         // which PID we are currently querying
unsigned long lastMqttPublish = 0;
const unsigned long MQTT_INTERVAL = 5000;

// ----------------------------------------------------------------------------
// Config persistence
// ----------------------------------------------------------------------------
void loadConfig() {
  prefs.begin("obd2", true);
  prefs.getString("mqttHost", mqttHost, sizeof(mqttHost));
  prefs.getString("mqttPort", mqttPort, sizeof(mqttPort));
  prefs.getString("mqttUser", mqttUser, sizeof(mqttUser));
  prefs.getString("mqttPass", mqttPass, sizeof(mqttPass));
  prefs.getString("btName",   btName,   sizeof(btName));
  prefs.getString("btMac",    btMac,    sizeof(btMac));
  prefs.getString("btPin",    btPin,    sizeof(btPin));
  prefs.end();
}

void saveConfig() {
  prefs.begin("obd2", false);
  prefs.putString("mqttHost", mqttHost);
  prefs.putString("mqttPort", mqttPort);
  prefs.putString("mqttUser", mqttUser);
  prefs.putString("mqttPass", mqttPass);
  prefs.putString("btName",   btName);
  prefs.putString("btMac",    btMac);
  prefs.putString("btPin",    btPin);
  prefs.end();
}

void saveConfigCallback() { shouldSaveConfig = true; }

// ----------------------------------------------------------------------------
// WiFi + captive portal
// ----------------------------------------------------------------------------
void setupWiFi() {
  WiFiManager wm;
  wm.setSaveConfigCallback(saveConfigCallback);

  WiFiManagerParameter p_host("host", "MQTT host (blank = Prometheus only)", mqttHost, sizeof(mqttHost));
  WiFiManagerParameter p_port("port", "MQTT port", mqttPort, sizeof(mqttPort));
  WiFiManagerParameter p_user("user", "MQTT username", mqttUser, sizeof(mqttUser));
  WiFiManagerParameter p_pass("pass", "MQTT password", mqttPass, sizeof(mqttPass));
  WiFiManagerParameter p_mac("mac",   "OBD2 Bluetooth MAC (preferred)", btMac, sizeof(btMac));
  WiFiManagerParameter p_bt("bt",     "OBD2 Bluetooth name (if no MAC)", btName, sizeof(btName));
  WiFiManagerParameter p_pin("pin",   "OBD2 Bluetooth PIN",  btPin,  sizeof(btPin));

  wm.addParameter(&p_host);
  wm.addParameter(&p_port);
  wm.addParameter(&p_user);
  wm.addParameter(&p_pass);
  wm.addParameter(&p_mac);
  wm.addParameter(&p_bt);
  wm.addParameter(&p_pin);

  wm.setConfigPortalTimeout(300); // 5 min then reboot & retry saved creds

  // AP name "ESP32-OBD2", password "obd2setup"
  if (!wm.autoConnect("ESP32-OBD2", "obd2setup")) {
    Serial.println("WiFi connect failed; rebooting.");
    delay(2000);
    ESP.restart();
  }

  if (shouldSaveConfig) {
    strncpy(mqttHost, p_host.getValue(), sizeof(mqttHost));
    strncpy(mqttPort, p_port.getValue(), sizeof(mqttPort));
    strncpy(mqttUser, p_user.getValue(), sizeof(mqttUser));
    strncpy(mqttPass, p_pass.getValue(), sizeof(mqttPass));
    strncpy(btMac,    p_mac.getValue(),  sizeof(btMac));
    strncpy(btName,   p_bt.getValue(),   sizeof(btName));
    strncpy(btPin,    p_pin.getValue(),  sizeof(btPin));
    saveConfig();
    Serial.println("Config saved.");
  }
  Serial.print("WiFi OK, IP: ");
  Serial.println(WiFi.localIP());
}

// ----------------------------------------------------------------------------
// Bluetooth / ELM327
// ----------------------------------------------------------------------------
// Parse "AA:BB:CC:DD:EE:FF" into 6 bytes. Returns true on success.
bool parseMac(const char* s, uint8_t* out) {
  int v[6];
  if (sscanf(s, "%x:%x:%x:%x:%x:%x", &v[0],&v[1],&v[2],&v[3],&v[4],&v[5]) == 6) {
    for (int i = 0; i < 6; i++) out[i] = (uint8_t)v[i];
    return true;
  }
  return false;
}

void connectOBD() {
  SerialBT.end();
  SerialBT.begin("ESP32OBD2", true);       // master mode
  SerialBT.setPin(btPin, strlen(btPin));   // legacy SPP pairing PIN (usually 1234)

  uint8_t mac[6];
  if (parseMac(btMac, mac)) {
    Serial.printf("Connecting to OBD2 dongle by MAC %s ...\n", btMac);
    btConnected = SerialBT.connect(mac);
  } else {
    Serial.printf("Connecting to OBD2 dongle by name '%s' ...\n", btName);
    btConnected = SerialBT.connect(btName);
  }
  if (!btConnected) {
    Serial.println("BT connect failed; will retry.");
    return;
  }
  Serial.println("BT connected. Init ELM327...");
  if (!myELM327.begin(SerialBT, false, 2000)) {
    Serial.println("ELM327 init failed; will retry.");
    btConnected = false;
    return;
  }
  elmReady = true;
  Serial.println("ELM327 ready.");
}

// Non-blocking round-robin over PIDs. Returns when the current PID resolves.
void pollOBD() {
  if (!elmReady) return;

  // Battery voltage (ATRV) is a blocking call in ELMduino -- handle separately.
  if (pidState == P_VBAT) {
    float bv = myELM327.batteryVoltage();
    metrics[P_VBAT].value = (bv > 0) ? bv : NAN;
    pidState = (pidState + 1) % P_COUNT;
    return;
  }

  float v = NAN;
  bool done = false;

  switch (pidState) {
    case P_RPM:      v = myELM327.rpm();                break;
    case P_SPEED:    v = (float)myELM327.kph();         break;
    case P_COOLANT:  v = myELM327.engineCoolantTemp();  break;
    case P_LOAD:     v = myELM327.engineLoad();         break;
    case P_THROTTLE: v = myELM327.throttle();           break;
    case P_INTAKE:   v = myELM327.intakeAirTemp();      break;
    case P_MAF:      v = myELM327.mafRate();            break;
  }

  if (myELM327.nb_rx_state == ELM_SUCCESS) {
    metrics[pidState].value = v;
    done = true;
  } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
    // error / unsupported PID -> mark unknown and move on
    metrics[pidState].value = NAN;
    done = true;
  }

  if (done) {
    pidState = (pidState + 1) % P_COUNT;
  }
}

// ----------------------------------------------------------------------------
// Prometheus endpoint
// ----------------------------------------------------------------------------
void handleMetrics() {
  String out;
  out.reserve(1024);
  out += "# ESP32 OBD2 exporter\n";
  out += "obd2_up ";
  out += (elmReady && btConnected) ? "1\n" : "0\n";
  for (int i = 0; i < P_COUNT; i++) {
    if (isnan(metrics[i].value)) continue;
    out += "# TYPE obd2_" + String(metrics[i].key) + " gauge\n";
    out += "obd2_" + String(metrics[i].key) + " " + String(metrics[i].value, 2) + "\n";
  }
  server.send(200, "text/plain; version=0.0.4", out);
}

void handleRoot() {
  String h = "<html><head><title>ESP32 OBD2</title></head><body>";
  h += "<h2>ESP32 OBD2 exporter</h2>";
  h += "<p>OBD2 link: " + String((elmReady && btConnected) ? "connected" : "DISCONNECTED") + "</p>";
  h += "<ul>";
  for (int i = 0; i < P_COUNT; i++) {
    h += "<li>" + String(metrics[i].name) + ": ";
    h += isnan(metrics[i].value) ? "n/a" : String(metrics[i].value, 2);
    h += " " + String(metrics[i].unit) + "</li>";
  }
  h += "</ul>";
  h += "<p><a href='/metrics'>/metrics</a> (Prometheus) &middot; ";
  h += "<a href='/update'>/update</a> (OTA firmware)</p>";
  h += "</body></html>";
  server.send(200, "text/html", h);
}

// ----------------------------------------------------------------------------
// MQTT / Home Assistant discovery
// ----------------------------------------------------------------------------
String stateTopic(const char* key) {
  return String(nodeName) + "/" + key;
}

void sendDiscovery() {
  for (int i = 0; i < P_COUNT; i++) {
    String cfgTopic = "homeassistant/sensor/" + String(nodeName) + "_" + metrics[i].key + "/config";
    String payload = "{";
    payload += "\"name\":\"" + String(metrics[i].name) + "\",";
    payload += "\"uniq_id\":\"" + String(nodeName) + "_" + metrics[i].key + "\",";
    payload += "\"stat_t\":\"" + stateTopic(metrics[i].key) + "\",";
    payload += "\"unit_of_meas\":\"" + String(metrics[i].unit) + "\",";
    if (strlen(metrics[i].devClass) > 0)
      payload += "\"dev_cla\":\"" + String(metrics[i].devClass) + "\",";
    payload += "\"dev\":{\"ids\":[\"" + String(nodeName) + "\"],\"name\":\"ESP32 OBD2\",\"mf\":\"DIY\",\"mdl\":\"ESP32-WROOM\"}";
    payload += "}";
    mqtt.publish(cfgTopic.c_str(), payload.c_str(), true);
  }
  haDiscoverySent = true;
}

void publishState() {
  for (int i = 0; i < P_COUNT; i++) {
    if (isnan(metrics[i].value)) continue;
    mqtt.publish(stateTopic(metrics[i].key).c_str(), String(metrics[i].value, 2).c_str(), true);
  }
}

void mqttReconnect() {
  if (strlen(mqttHost) == 0) return;         // MQTT disabled
  if (mqtt.connected()) return;
  mqtt.setServer(mqttHost, atoi(mqttPort));
  mqtt.setBufferSize(512);                    // discovery JSON > default 256
  String cid = String(nodeName) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  bool ok = strlen(mqttUser)
    ? mqtt.connect(cid.c_str(), mqttUser, mqttPass)
    : mqtt.connect(cid.c_str());
  if (ok) {
    Serial.println("MQTT connected.");
    sendDiscovery();
  }
}

// ----------------------------------------------------------------------------
// Setup / loop
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nESP32 OBD2 exporter starting...");

  loadConfig();
  setupWiFi();

  server.on("/", handleRoot);
  server.on("/metrics", handleMetrics);
  ElegantOTA.begin(&server);   // serves /update
  server.begin();
  Serial.println("HTTP server up on port 80 (/metrics, /update).");

  connectOBD();
}

void loop() {
  server.handleClient();
  ElegantOTA.loop();

  // Keep OBD2 link alive
  if (!btConnected || !elmReady) {
    static unsigned long lastTry = 0;
    if (millis() - lastTry > 5000) {
      lastTry = millis();
      elmReady = false;
      connectOBD();
    }
  } else {
    pollOBD();
  }

  // MQTT
  if (strlen(mqttHost) > 0) {
    if (!mqtt.connected()) mqttReconnect();
    mqtt.loop();
    if (mqtt.connected() && millis() - lastMqttPublish > MQTT_INTERVAL) {
      lastMqttPublish = millis();
      publishState();
    }
  }
}
