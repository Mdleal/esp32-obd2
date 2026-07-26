/*
 * ESP32 OBD2 Logger  ->  InfluxDB + Home Assistant (MQTT)
 * ------------------------------------------------------------------
 * Board:  LoLin D32 Pro (ESP32-WROVER, classic ESP32 -> Bluetooth Classic OK)
 *
 * Features: OBD2 over BT-Classic ELM327, GPS time+location (HA map via
 * device_tracker), SD store-and-forward to InfluxDB, MQTT publishing with HA
 * auto-discovery, misfire/DTC detection, live dashboard, static IP,
 * GitHub OTA + manual firmware upload. Version = bN-<sha> (build number).
 *
 * First-boot config: captive-portal AP "ESP32-LOGGER" (pw: loggersetup).
 * After it is on WiFi, settings can be edited any time at http://<board-ip>/config
 *
 * Wiring (LoLin D32 Pro):
 *   microSD  -> on-board slot (VSPI: SCK=18 MISO=19 MOSI=23  CS=GPIO4)
 *   GPS  VCC -> 3V3,  GND -> GND
 *   GPS  TX  -> GPIO25 (ESP RX)      GPS RX -> GPIO26 (ESP TX, optional)
 *   NOTE: GPIO16/17 are used by PSRAM on WROVER -- do NOT use them for GPS.
 *
 * Build: generic esp32 (WROVER), FQBN esp32:esp32:esp32:PSRAM=enabled,PartitionScheme=min_spiffs
 * Libraries: ELMduino, WiFiManager, TinyGPSPlus, PubSubClient (+ core SD/HTTPClient/HTTPUpdate/Update)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include "BluetoothSerial.h"
#include "ELMduino.h"
#include <WiFiManager.h>
#include <TinyGPSPlus.h>
#include <PubSubClient.h>
#include <HTTPUpdate.h>
#include <Update.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"

#if __has_include("version.h")
#include "version.h"
#endif
#ifndef FW_VERSION
#define FW_VERSION "dev"
#endif

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth Classic required. Use a classic ESP32 (WROVER/WROOM), not S3/C3."
#endif

// ---------------- Pins ----------------
#define SD_CS       4
#define GPS_RX_PIN  25
#define GPS_TX_PIN  26
#define GPS_BAUD    9600

// ---------------- Config (persisted) ----------------
Preferences prefs;
char influxHost[64]  = "";
char influxPort[6]   = "443";
char influxOrg[48]   = "";
char influxBucket[48]= "";
char influxToken[192]= "";
char mqttHost[64]    = "";
char mqttPort[6]     = "1883";
char mqttUser[32]    = "";
char mqttPass[64]    = "";
char btMac[20]       = "00:1D:A5:07:5D:3C";
char btPin[8]        = "1234";
char vehicleId[24]   = "mycar";
char staticIP[16]    = "";
char staticGW[16]    = "";
char staticSN[16]    = "255.255.255.0";
char staticDNS[16]   = "";
bool shouldSaveConfig = false;

// ---------------- Globals ----------------
BluetoothSerial SerialBT;
ELM327 elm;
WebServer server(80);
TinyGPSPlus gps;
HardwareSerial GPSserial(2);
WiFiClient mqttNet;
PubSubClient mqtt(mqttNet);

bool btConnected = false;
bool elmReady    = false;

float v_rpm=NAN, v_speedMph=NAN, v_coolant=NAN, v_load=NAN, v_throttle=NAN,
      v_intake=NAN, v_maf=NAN, v_vbat=NAN, v_fuel=NAN;
float v_stft=NAN, v_ltft=NAN, v_timing=NAN, v_map=NAN, v_modV=NAN,
      v_runtime=NAN, v_ambient=NAN, v_oil=NAN;
float v_mil=NAN, v_dtcCount=NAN, v_misfire=NAN;
uint8_t pidState = 0;
const uint8_t PID_COUNT = 18;
unsigned long lastDtcMs = 0;
const unsigned long DTC_INTERVAL = 60000;

double g_lat=NAN, g_lon=NAN, g_alt=NAN, g_spdMph=NAN; uint32_t g_sats=0;

const char* BUF = "/buffer.lp";
const char* UP  = "/uploading.lp";
bool sdOk = false;

unsigned long lastLogMs = 0, lastUploadMs = 0, lastBeatMs = 0, lastConnTryMs = 0, lastMqttMs = 0;
const unsigned long LOG_INTERVAL    = 2000;
const unsigned long UPLOAD_INTERVAL = 15000;
const unsigned long MQTT_PUB_INTERVAL = 5000;
const float KMH_TO_MPH = 0.621371f;

int lastInfluxCode = 0;
unsigned long lastInfluxMs = 0;

uint32_t loopCount = 0;
uint32_t loopRate = 0;
unsigned long lastRateMs = 0, lastSdRetryMs = 0;

const char* VERSION_URL = "https://raw.githubusercontent.com/Mdleal/esp32-obd2/logger/firmware/version.txt";
const char* BIN_URL     = "https://raw.githubusercontent.com/Mdleal/esp32-obd2/logger/firmware/logger-app-ota.bin";
bool updateChecked  = false;
bool updateAvailable = false;
char latestVersion[48] = "";
char lastCheckMsg[64] = "";

// ---------------- Config load/save ----------------
void loadConfig() {
  prefs.begin("logger", true);
  prefs.getString("iHost",  influxHost,  sizeof(influxHost));
  prefs.getString("iPort",  influxPort,  sizeof(influxPort));
  prefs.getString("iOrg",   influxOrg,   sizeof(influxOrg));
  prefs.getString("iBucket",influxBucket,sizeof(influxBucket));
  prefs.getString("iToken", influxToken, sizeof(influxToken));
  prefs.getString("mHost",  mqttHost,    sizeof(mqttHost));
  prefs.getString("mPort",  mqttPort,    sizeof(mqttPort));
  prefs.getString("mUser",  mqttUser,    sizeof(mqttUser));
  prefs.getString("mPass",  mqttPass,    sizeof(mqttPass));
  prefs.getString("btMac",  btMac,       sizeof(btMac));
  prefs.getString("btPin",  btPin,       sizeof(btPin));
  prefs.getString("veh",    vehicleId,   sizeof(vehicleId));
  prefs.getString("sIP",    staticIP,    sizeof(staticIP));
  prefs.getString("sGW",    staticGW,    sizeof(staticGW));
  prefs.getString("sSN",    staticSN,    sizeof(staticSN));
  prefs.getString("sDNS",   staticDNS,   sizeof(staticDNS));
  prefs.end();
}
void saveConfig() {
  prefs.begin("logger", false);
  prefs.putString("iHost",  influxHost);
  prefs.putString("iPort",  influxPort);
  prefs.putString("iOrg",   influxOrg);
  prefs.putString("iBucket",influxBucket);
  prefs.putString("iToken", influxToken);
  prefs.putString("mHost",  mqttHost);
  prefs.putString("mPort",  mqttPort);
  prefs.putString("mUser",  mqttUser);
  prefs.putString("mPass",  mqttPass);
  prefs.putString("btMac",  btMac);
  prefs.putString("btPin",  btPin);
  prefs.putString("veh",    vehicleId);
  prefs.putString("sIP",    staticIP);
  prefs.putString("sGW",    staticGW);
  prefs.putString("sSN",    staticSN);
  prefs.putString("sDNS",   staticDNS);
  prefs.end();
}
void saveConfigCallback() { shouldSaveConfig = true; }

// ---------------- WiFi + portal ----------------
void setupWiFi() {
  WiFiManager wm;
  wm.setDebugOutput(true);
  wm.setSaveConfigCallback(saveConfigCallback);
  wm.setAPCallback([](WiFiManager* w){
    Serial.print(">>> Setup portal up. Join 'ESP32-LOGGER' (pw loggersetup), open http://");
    Serial.println(WiFi.softAPIP());
  });

  WiFiManagerParameter p_host ("ihost",  "InfluxDB host/IP",  influxHost,  sizeof(influxHost));
  WiFiManagerParameter p_port ("iport",  "InfluxDB port",     influxPort,  sizeof(influxPort));
  WiFiManagerParameter p_org  ("iorg",   "InfluxDB org",      influxOrg,   sizeof(influxOrg));
  WiFiManagerParameter p_bkt  ("ibkt",   "InfluxDB bucket",   influxBucket,sizeof(influxBucket));
  WiFiManagerParameter p_tok  ("itok",   "InfluxDB API token",influxToken, sizeof(influxToken));
  WiFiManagerParameter p_mh   ("mh",     "MQTT host (blank=off)", mqttHost, sizeof(mqttHost));
  WiFiManagerParameter p_mp   ("mp",     "MQTT port",         mqttPort,    sizeof(mqttPort));
  WiFiManagerParameter p_mu   ("mu",     "MQTT user",         mqttUser,    sizeof(mqttUser));
  WiFiManagerParameter p_mpw  ("mpw",    "MQTT password",     mqttPass,    sizeof(mqttPass));
  WiFiManagerParameter p_mac  ("mac",    "OBD2 Bluetooth MAC",btMac,       sizeof(btMac));
  WiFiManagerParameter p_pin  ("pin",    "OBD2 Bluetooth PIN",btPin,       sizeof(btPin));
  WiFiManagerParameter p_veh  ("veh",    "Vehicle name/tag",  vehicleId,   sizeof(vehicleId));
  WiFiManagerParameter p_sip  ("sip",    "Static IP (blank = DHCP)", staticIP,  sizeof(staticIP));
  WiFiManagerParameter p_sgw  ("sgw",    "Gateway",           staticGW,  sizeof(staticGW));
  WiFiManagerParameter p_ssn  ("ssn",    "Subnet mask",       staticSN,  sizeof(staticSN));
  WiFiManagerParameter p_sdns ("sdns",   "DNS server",        staticDNS, sizeof(staticDNS));
  wm.addParameter(&p_host); wm.addParameter(&p_port); wm.addParameter(&p_org);
  wm.addParameter(&p_bkt);  wm.addParameter(&p_tok);
  wm.addParameter(&p_mh);   wm.addParameter(&p_mp);   wm.addParameter(&p_mu); wm.addParameter(&p_mpw);
  wm.addParameter(&p_mac);  wm.addParameter(&p_pin);  wm.addParameter(&p_veh);
  wm.addParameter(&p_sip);  wm.addParameter(&p_sgw);  wm.addParameter(&p_ssn); wm.addParameter(&p_sdns);

  if (strlen(staticIP) > 0) {
    IPAddress ip, gw, sn, dns;
    ip.fromString(staticIP);
    gw.fromString(strlen(staticGW) ? staticGW : "0.0.0.0");
    sn.fromString(strlen(staticSN) ? staticSN : "255.255.255.0");
    dns.fromString(strlen(staticDNS) ? staticDNS : (strlen(staticGW) ? staticGW : "8.8.8.8"));
    wm.setSTAStaticIPConfig(ip, gw, sn, dns);
    Serial.printf("Static IP configured: %s\n", staticIP);
  }

  wm.setConfigPortalTimeout(0);
  Serial.println("autoConnect('ESP32-LOGGER')...");
  if (!wm.autoConnect("ESP32-LOGGER", "loggersetup")) {
    Serial.println("Portal timeout; rebooting."); delay(2000); ESP.restart();
  }
  if (shouldSaveConfig) {
    strncpy(influxHost,  p_host.getValue(), sizeof(influxHost));
    strncpy(influxPort,  p_port.getValue(), sizeof(influxPort));
    strncpy(influxOrg,   p_org.getValue(),  sizeof(influxOrg));
    strncpy(influxBucket,p_bkt.getValue(),  sizeof(influxBucket));
    strncpy(influxToken, p_tok.getValue(),  sizeof(influxToken));
    strncpy(mqttHost,    p_mh.getValue(),   sizeof(mqttHost));
    strncpy(mqttPort,    p_mp.getValue(),   sizeof(mqttPort));
    strncpy(mqttUser,    p_mu.getValue(),   sizeof(mqttUser));
    strncpy(mqttPass,    p_mpw.getValue(),  sizeof(mqttPass));
    strncpy(btMac,       p_mac.getValue(),  sizeof(btMac));
    strncpy(btPin,       p_pin.getValue(),  sizeof(btPin));
    strncpy(vehicleId,   p_veh.getValue(),  sizeof(vehicleId));
    strncpy(staticIP,    p_sip.getValue(),  sizeof(staticIP));
    strncpy(staticGW,    p_sgw.getValue(),  sizeof(staticGW));
    strncpy(staticSN,    p_ssn.getValue(),  sizeof(staticSN));
    strncpy(staticDNS,   p_sdns.getValue(), sizeof(staticDNS));
    saveConfig();
    Serial.println("Config saved.");
  }
  Serial.print("WiFi OK, IP: "); Serial.println(WiFi.localIP());
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
}

// ---------------- Bluetooth / ELM327 ----------------
bool parseMac(const char* s, uint8_t* out) {
  int v[6];
  if (sscanf(s, "%x:%x:%x:%x:%x:%x", &v[0],&v[1],&v[2],&v[3],&v[4],&v[5]) == 6) {
    for (int i=0;i<6;i++) out[i]=(uint8_t)v[i];
    return true;
  }
  return false;
}
void connectOBD() {
  SerialBT.end();
  SerialBT.begin("ESP32LOGGER", true);
  SerialBT.setPin(btPin, strlen(btPin));
  uint8_t mac[6];
  if (parseMac(btMac, mac)) { Serial.printf("OBD connect by MAC %s...\n", btMac); btConnected = SerialBT.connect(mac); }
  else                      { btConnected = SerialBT.connect("OBDII"); }
  if (!btConnected) { Serial.println("BT connect failed; retry later."); return; }
  if (!elm.begin(SerialBT, false, 2000)) { Serial.println("ELM init failed."); btConnected=false; return; }
  elmReady = true; Serial.println("ELM327 ready.");
}
void pollOBD() {
  if (!elmReady) return;
  if (pidState == 7) { float bv=elm.batteryVoltage(); v_vbat=(bv>0)?bv:NAN; pidState=(pidState+1)%PID_COUNT; return; }
  if (pidState == 17) {
    uint32_t ms = elm.monitorStatus();
    if (elm.nb_rx_state == ELM_SUCCESS) {
      uint8_t A = (ms >> 24) & 0xFF;
      v_mil = (A & 0x80) ? 1 : 0;
      v_dtcCount = (float)(A & 0x7F);
    }
    if (elm.nb_rx_state != ELM_GETTING_MSG) pidState=(pidState+1)%PID_COUNT;
    return;
  }
  float val=NAN;
  switch (pidState) {
    case 0:  val=elm.rpm();                   break;
    case 1:  val=(float)elm.kph();            break;
    case 2:  val=elm.engineCoolantTemp();     break;
    case 3:  val=elm.engineLoad();            break;
    case 4:  val=elm.throttle();              break;
    case 5:  val=elm.intakeAirTemp();         break;
    case 6:  val=elm.mafRate();               break;
    case 8:  val=elm.fuelLevel();             break;
    case 9:  val=elm.shortTermFuelTrimBank_1(); break;
    case 10: val=elm.longTermFuelTrimBank_1();  break;
    case 11: val=elm.timingAdvance();         break;
    case 12: val=(float)elm.manifoldPressure(); break;
    case 13: val=elm.ctrlModVoltage();        break;
    case 14: val=(float)elm.runTime();        break;
    case 15: val=elm.ambientAirTemp();        break;
    case 16: val=elm.oilTemp();               break;
  }
  bool resolved=false;
  if (elm.nb_rx_state == ELM_SUCCESS) {
    switch (pidState) {
      case 0:  v_rpm=val;              break;
      case 1:  v_speedMph=val*KMH_TO_MPH; break;
      case 2:  v_coolant=val;          break;
      case 3:  v_load=val;             break;
      case 4:  v_throttle=val;         break;
      case 5:  v_intake=val;           break;
      case 6:  v_maf=val;              break;
      case 8:  v_fuel=val;             break;
      case 9:  v_stft=val;             break;
      case 10: v_ltft=val;             break;
      case 11: v_timing=val;           break;
      case 12: v_map=val;              break;
      case 13: v_modV=val;             break;
      case 14: v_runtime=val;          break;
      case 15: v_ambient=val;          break;
      case 16: v_oil=val;              break;
    }
    resolved=true;
  } else if (elm.nb_rx_state != ELM_GETTING_MSG) {
    resolved=true;
  }
  if (resolved) pidState=(pidState+1)%PID_COUNT;
}
void checkDTCs() {
  elm.currentDTCCodes(true);
  int mis = 0;
  for (uint8_t i = 0; i < elm.DTC_Response.codesFound; i++) {
    Serial.printf("DTC: %s\n", elm.DTC_Response.codes[i]);
    if (strncmp(elm.DTC_Response.codes[i], "P030", 4) == 0) mis = 1;
  }
  v_misfire = mis;
}

// ---------------- GPS ----------------
void feedGPS() {
  while (GPSserial.available()) gps.encode(GPSserial.read());
  if (gps.location.isValid()) { g_lat=gps.location.lat(); g_lon=gps.location.lng(); }
  if (gps.altitude.isValid()) g_alt=gps.altitude.meters();
  if (gps.speed.isValid())    g_spdMph=gps.speed.mph();
  if (gps.satellites.isValid()) g_sats=gps.satellites.value();
  if (time(nullptr) < 1700000000 && gps.date.isValid() && gps.time.isValid() && gps.date.year() > 2020) {
    struct tm t = {};
    t.tm_year = gps.date.year()-1900; t.tm_mon = gps.date.month()-1; t.tm_mday = gps.date.day();
    t.tm_hour = gps.time.hour(); t.tm_min = gps.time.minute(); t.tm_sec = gps.time.second();
    time_t epoch = mktime(&t);
    struct timeval tv = { epoch, 0 };
    settimeofday(&tv, nullptr);
    Serial.println("Clock set from GPS.");
  }
}

// ---------------- Line-protocol build ----------------
String ns2str(uint64_t ns) {
  char b[24]; snprintf(b, sizeof(b), "%llu", (unsigned long long)ns); return String(b);
}
void addF(String& s, const char* k, float v, bool& first) {
  if (isnan(v)) return;
  if (!first) s += ",";
  s += k; s += "="; s += String(v, 2);
  first = false;
}
void logSnapshot() {
  time_t now = time(nullptr);
  if (now < 1700000000) return;
  uint64_t ns = (uint64_t)now * 1000000000ULL;

  String line;
  line = "obd2,vehicle="; line += vehicleId; line += " ";
  bool first = true;
  addF(line,"rpm",v_rpm,first);      addF(line,"speed_mph",v_speedMph,first);
  addF(line,"coolant_c",v_coolant,first); addF(line,"load_pct",v_load,first);
  addF(line,"throttle_pct",v_throttle,first); addF(line,"intake_c",v_intake,first);
  addF(line,"maf",v_maf,first);      addF(line,"battery_v",v_vbat,first);
  addF(line,"fuel_pct",v_fuel,first);
  addF(line,"stft_b1",v_stft,first);      addF(line,"ltft_b1",v_ltft,first);
  addF(line,"timing_adv",v_timing,first); addF(line,"map_kpa",v_map,first);
  addF(line,"module_v",v_modV,first);     addF(line,"runtime_s",v_runtime,first);
  addF(line,"ambient_c",v_ambient,first); addF(line,"oil_c",v_oil,first);
  addF(line,"mil_on",v_mil,first);        addF(line,"dtc_count",v_dtcCount,first);
  addF(line,"misfire",v_misfire,first);
  if (!first) { line += " "; line += ns2str(ns); line += "\n"; }
  else line = "";

  if (!isnan(g_lat) && !isnan(g_lon)) {
    String gl = "gps,vehicle="; gl += vehicleId; gl += " ";
    bool gf = true;
    addF(gl,"lat",(float)g_lat,gf); addF(gl,"lon",(float)g_lon,gf);
    addF(gl,"alt_m",(float)g_alt,gf); addF(gl,"speed_mph",(float)g_spdMph,gf);
    if (!gf) { gl += " "; gl += ns2str(ns); gl += "\n"; line += gl; }
  }

  {
    String sys = "system,vehicle="; sys += vehicleId; sys += " ";
    sys += "heap_free=" + String(ESP.getFreeHeap());
    sys += ",heap_min=" + String(ESP.getMinFreeHeap());
    sys += ",heap_largest=" + String(ESP.getMaxAllocHeap());
    sys += ",psram_free=" + String(ESP.getFreePsram());
    sys += ",loop_rate=" + String(loopRate);
    sys += ",chip_temp=" + String(temperatureRead(), 1);
    sys += ",uptime_s=" + String(millis() / 1000);
    sys += " "; sys += ns2str(ns); sys += "\n";
    line += sys;
  }
  if (line.length() == 0) return;

  if (sdOk) {
    File f = SD.open(BUF, FILE_APPEND);
    if (f) { f.print(line); f.close(); }
  } else {
    Serial.print("[no SD] "); Serial.print(line);
  }
}

// ---------------- InfluxDB uploader ----------------
bool influxHttps() { return strcmp(influxPort, "443") == 0; }
String influxUrl() {
  bool https = influxHttps();
  String u = https ? "https://" : "http://";
  u += influxHost;
  bool defaultPort = (https && strcmp(influxPort,"443")==0) || (!https && strcmp(influxPort,"80")==0);
  if (!defaultPort) { u += ":"; u += influxPort; }
  u += "/api/v2/write?org="; u += influxOrg;
  u += "&bucket="; u += influxBucket; u += "&precision=ns";
  return u;
}
void flushBuffer() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (strlen(influxHost) == 0) return;
  if (!sdOk) return;
  if (!SD.exists(UP)) {
    if (!SD.exists(BUF)) return;
    File b = SD.open(BUF, FILE_READ);
    size_t sz = b ? b.size() : 0; if (b) b.close();
    if (sz == 0) return;
    SD.rename(BUF, UP);
  }
  File f = SD.open(UP, FILE_READ);
  if (!f) return;
  size_t sz = f.size();
  if (sz == 0) { f.close(); SD.remove(UP); return; }
  HTTPClient http;
  WiFiClientSecure tls;
  WiFiClient plain;
  int code;
  String auth = "Token "; auth += influxToken;
  if (influxHttps()) { tls.setInsecure(); http.begin(tls, influxUrl()); }
  else               { http.begin(plain, influxUrl()); }
  http.addHeader("Content-Type", "text/plain; charset=utf-8");
  http.addHeader("Authorization", auth);
  code = http.sendRequest("POST", &f, sz);
  f.close();
  http.end();
  lastInfluxCode = code;
  lastInfluxMs = millis();
  if (code == 204 || code == 200) { SD.remove(UP); Serial.printf("Uploaded %u bytes (HTTP %d).\n", (unsigned)sz, code); }
  else Serial.printf("InfluxDB upload failed (HTTP %d).\n", code);
}

// ---------------- MQTT -> Home Assistant (auto-discovery) ----------------
struct MField { const char* key; const char* name; const char* unit; const char* dev; float* val; };
MField mf[] = {
  {"rpm",         "Engine RPM",         "rpm", "",            &v_rpm},
  {"speed_mph",   "Vehicle Speed",      "mph", "speed",       &v_speedMph},
  {"coolant_c",   "Coolant Temp",       "°C", "temperature", &v_coolant},
  {"load_pct",    "Engine Load",        "%",   "",            &v_load},
  {"throttle_pct","Throttle",           "%",   "",            &v_throttle},
  {"intake_c",    "Intake Air Temp",    "°C", "temperature", &v_intake},
  {"maf",         "MAF Rate",           "g/s", "",            &v_maf},
  {"battery_v",   "Battery",            "V",   "voltage",     &v_vbat},
  {"fuel_pct",    "Fuel Level",         "%",   "",            &v_fuel},
  {"stft_b1",     "Short Fuel Trim B1", "%",   "",            &v_stft},
  {"ltft_b1",     "Long Fuel Trim B1",  "%",   "",            &v_ltft},
  {"timing_adv",  "Timing Advance",     "°",  "",        &v_timing},
  {"map_kpa",     "Manifold Pressure",  "kPa", "pressure",    &v_map},
  {"module_v",    "Module Voltage",     "V",   "voltage",     &v_modV},
  {"runtime_s",   "Runtime",            "s",   "duration",    &v_runtime},
  {"ambient_c",   "Ambient Temp",       "°C", "temperature", &v_ambient},
  {"oil_c",       "Oil Temp",           "°C", "temperature", &v_oil},
  {"mil_on",      "Check Engine (MIL)", "",    "",            &v_mil},
  {"dtc_count",   "Trouble Codes",      "",    "",            &v_dtcCount},
  {"misfire",     "Misfire",            "",    "",            &v_misfire},
};
const int MF_COUNT = sizeof(mf) / sizeof(mf[0]);
const char* DEVJSON = "\"dev\":{\"ids\":[\"esp32logger\"],\"name\":\"ESP32 OBD2 Logger\",\"mf\":\"DIY\",\"mdl\":\"LoLin D32 Pro\"}";

void sendDiscovery() {
  for (int i = 0; i < MF_COUNT; i++) {
    String t = "homeassistant/sensor/esp32logger/" + String(mf[i].key) + "/config";
    String p = "{";
    p += "\"name\":\"" + String(mf[i].name) + "\",";
    p += "\"uniq_id\":\"esp32logger_" + String(mf[i].key) + "\",";
    p += "\"stat_t\":\"esp32logger/" + String(mf[i].key) + "\",";
    p += "\"avty_t\":\"esp32logger/status\",";
    if (strlen(mf[i].unit) > 0) p += "\"unit_of_meas\":\"" + String(mf[i].unit) + "\",";
    if (strlen(mf[i].dev) > 0)  p += "\"dev_cla\":\"" + String(mf[i].dev) + "\",";
    p += "\"stat_cla\":\"measurement\",";
    p += DEVJSON;
    p += "}";
    mqtt.publish(t.c_str(), p.c_str(), true);
  }
  // GPS as a device_tracker -> shows the car on the HA map
  {
    String t = "homeassistant/device_tracker/esp32logger/car/config";
    String p = "{\"name\":\"Car\",\"uniq_id\":\"esp32logger_car\",";
    p += "\"json_attr_t\":\"esp32logger/gps_attr\",\"source_type\":\"gps\",";
    p += "\"avty_t\":\"esp32logger/status\",";
    p += DEVJSON;
    p += "}";
    mqtt.publish(t.c_str(), p.c_str(), true);
  }
}
void mqttReconnect() {
  if (strlen(mqttHost) == 0) return;
  if (mqtt.connected()) return;
  mqtt.setServer(mqttHost, atoi(mqttPort));
  mqtt.setBufferSize(640);
  String cid = "esp32logger-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  bool ok = strlen(mqttUser)
    ? mqtt.connect(cid.c_str(), mqttUser, mqttPass, "esp32logger/status", 0, true, "offline")
    : mqtt.connect(cid.c_str(), "esp32logger/status", 0, true, "offline");
  if (ok) {
    Serial.println("MQTT connected.");
    mqtt.publish("esp32logger/status", "online", true);
    sendDiscovery();
  }
}
void publishMqtt() {
  for (int i = 0; i < MF_COUNT; i++) {
    if (isnan(*mf[i].val)) continue;
    String tp = "esp32logger/" + String(mf[i].key);
    mqtt.publish(tp.c_str(), String(*mf[i].val, 2).c_str(), true);
  }
  if (!isnan(g_lat) && !isnan(g_lon)) {
    String a = "{\"latitude\":" + String(g_lat, 6) + ",\"longitude\":" + String(g_lon, 6);
    if (!isnan(g_alt))    a += ",\"altitude\":" + String(g_alt, 1);
    if (!isnan(g_spdMph)) a += ",\"gps_speed_mph\":" + String(g_spdMph, 1);
    a += ",\"gps_accuracy\":10,\"satellites\":" + String(g_sats) + "}";
    mqtt.publish("esp32logger/gps_attr", a.c_str(), true);
  }
}

// ---------------- Web status ----------------
size_t bufBytes() {
  size_t n = 0;
  if (sdOk && SD.exists(BUF)) { File f=SD.open(BUF,FILE_READ); if(f){n+=f.size();f.close();} }
  if (sdOk && SD.exists(UP))  { File f=SD.open(UP,FILE_READ);  if(f){n+=f.size();f.close();} }
  return n;
}
String sensorRow(const char* n, float v, const char* u) {
  String r = "<tr><td>"; r += n; r += "</td><td align=right>";
  r += isnan(v) ? String("n/a") : String(v, 2);
  r += "</td><td>"; r += u; r += "</td></tr>";
  return r;
}
void handleRoot() {
  String h = "<html><head><meta http-equiv='refresh' content='5'>";
  h += "<style>body{font-family:sans-serif}table{border-collapse:collapse}td{border:1px solid #ccc;padding:2px 8px}</style></head><body>";
  h += "<h2>ESP32 OBD2 Logger</h2>";
  h += "<p><b>OBD link:</b> " + String(elmReady&&btConnected?"connected":"DISCONNECTED");
  h += " &nbsp; <b>GPS:</b> " + String(g_sats) + " sats, fix " + String(!isnan(g_lat)?"yes":"no");
  h += " &nbsp; <b>WiFi:</b> " + String(WiFi.status()==WL_CONNECTED?WiFi.localIP().toString():"down") + "</p>";
  h += "<p><b>SD:</b> " + String(sdOk?"ok":"FAIL (reformat FAT32 / reseat)") + " &nbsp; buffered: " + String(bufBytes()) + " bytes</p>";
  String ist;
  if (lastInfluxCode == 0)                          ist = "no upload yet";
  else if (lastInfluxCode == 200 || lastInfluxCode == 204) ist = "CONNECTED (last OK " + String((millis()-lastInfluxMs)/1000) + "s ago)";
  else                                              ist = "ERROR (HTTP " + String(lastInfluxCode) + ")";
  h += "<p><b>InfluxDB:</b> " + String(influxHost) + ":" + String(influxPort) + " / " + String(influxBucket) + " &mdash; " + ist + "</p>";
  h += "<p><b>MQTT:</b> " + String(strlen(mqttHost)==0 ? "disabled" : (mqtt.connected()?"connected to "+String(mqttHost):"disconnected")) + "</p>";
  String faults;
  if (isnan(v_mil)) faults = "not read yet";
  else faults = String(v_mil>0.5?"MIL ON":"MIL off") + ", " + String((int)v_dtcCount) + " code(s)" + (v_misfire>0.5 ? ", MISFIRE code present" : "");
  h += "<p><b>Faults:</b> " + faults + "</p>";

  h += "<h3>Sensors</h3><table>";
  h += sensorRow("Engine RPM", v_rpm, "rpm");
  h += sensorRow("Vehicle speed", v_speedMph, "mph");
  h += sensorRow("Coolant temp", v_coolant, "C");
  h += sensorRow("Engine load", v_load, "%");
  h += sensorRow("Throttle", v_throttle, "%");
  h += sensorRow("Intake air temp", v_intake, "C");
  h += sensorRow("MAF rate", v_maf, "g/s");
  h += sensorRow("Battery (ATRV)", v_vbat, "V");
  h += sensorRow("Fuel level", v_fuel, "%");
  h += sensorRow("Short-term fuel trim B1", v_stft, "%");
  h += sensorRow("Long-term fuel trim B1", v_ltft, "%");
  h += sensorRow("Timing advance", v_timing, "deg");
  h += sensorRow("Manifold pressure", v_map, "kPa");
  h += sensorRow("Module voltage", v_modV, "V");
  h += sensorRow("Runtime", v_runtime, "s");
  h += sensorRow("Ambient temp", v_ambient, "C");
  h += sensorRow("Oil temp", v_oil, "C");
  h += sensorRow("GPS latitude", (float)g_lat, "deg");
  h += sensorRow("GPS longitude", (float)g_lon, "deg");
  h += "</table>";

  h += "<h3>System</h3><ul>";
  h += "<li>Uptime: " + String(millis()/1000) + " s</li>";
  h += "<li>Free heap: " + String(ESP.getFreeHeap()) + " B (min " + String(ESP.getMinFreeHeap()) + ", largest block " + String(ESP.getMaxAllocHeap()) + ")</li>";
  h += "<li>PSRAM: free " + String(ESP.getFreePsram()) + " / " + String(ESP.getPsramSize()) + " B</li>";
  h += "<li>CPU: " + String(ESP.getCpuFreqMHz()) + " MHz &nbsp; loop rate " + String(loopRate) + "/s</li>";
  h += "<li>Chip temp: " + String(temperatureRead(), 1) + " C (rough)</li>";
  h += "</ul>";

  h += "<h3>Firmware</h3>";
  h += "<p>Running: <b>" + String(FW_VERSION) + "</b></p>";
  h += "<form method='POST' action='/checkupdate' style='display:inline'><button>Check for update</button></form>";
  if (updateChecked) {
    h += " &mdash; last check: " + String(lastCheckMsg);
    if (updateAvailable) {
      h += " &mdash; <b>Update available:</b> " + String(latestVersion);
      h += " <form method='POST' action='/doupdate' style='display:inline'><button>Update now</button></form>";
    } else if (strlen(latestVersion) > 0) {
      h += " (up to date)";
    }
  }
  h += "<p><a href='/config'>/config</a> (edit settings) &middot; <a href='/manualupdate'>manual firmware upload</a></p></body></html>";
  server.send(200, "text/html", h);
}

// ---------------- Config web page ----------------
void handleConfig() {
  String h = "<html><body><h2>Logger Configuration</h2>";
  h += "<form method='POST' action='/save'>";
  h += "InfluxDB host: <input name='host' value='" + String(influxHost) + "'><br><br>";
  h += "Port: <input name='port' value='" + String(influxPort) + "'><br><br>";
  h += "Org: <input name='org' value='" + String(influxOrg) + "'><br><br>";
  h += "Bucket: <input name='bucket' value='" + String(influxBucket) + "'><br><br>";
  h += "InfluxDB token (blank = keep): <input name='token' value='' size='40'><br><br>";
  h += "<hr>MQTT host (blank = off): <input name='mh' value='" + String(mqttHost) + "'><br><br>";
  h += "MQTT port: <input name='mp' value='" + String(mqttPort) + "'><br><br>";
  h += "MQTT user: <input name='mu' value='" + String(mqttUser) + "'><br><br>";
  h += "MQTT password (blank = keep): <input name='mpw' value=''><br><br>";
  h += "<hr>Vehicle tag: <input name='veh' value='" + String(vehicleId) + "'><br><br>";
  h += "OBD2 Bluetooth MAC: <input name='mac' value='" + String(btMac) + "'><br><br>";
  h += "OBD2 Bluetooth PIN: <input name='pin' value='" + String(btPin) + "'><br><br>";
  h += "<hr>Static IP (blank = DHCP): <input name='sip' value='" + String(staticIP) + "'><br><br>";
  h += "Gateway: <input name='sgw' value='" + String(staticGW) + "'><br><br>";
  h += "Subnet mask: <input name='ssn' value='" + String(staticSN) + "'><br><br>";
  h += "DNS server: <input name='sdns' value='" + String(staticDNS) + "'><br><br>";
  h += "<input type='submit' value='Save'></form>";
  h += "<p><i>MQTT/static-IP changes take effect after a reboot.</i></p>";
  h += "<form method='POST' action='/reboot'><button>Reboot now</button></form>";
  h += "<p><a href='/'>&larr; status</a></p></body></html>";
  server.send(200, "text/html", h);
}
void handleSave() {
  if (server.hasArg("host"))   snprintf(influxHost,   sizeof(influxHost),   "%s", server.arg("host").c_str());
  if (server.hasArg("port"))   snprintf(influxPort,   sizeof(influxPort),   "%s", server.arg("port").c_str());
  if (server.hasArg("org"))    snprintf(influxOrg,    sizeof(influxOrg),    "%s", server.arg("org").c_str());
  if (server.hasArg("bucket")) snprintf(influxBucket, sizeof(influxBucket), "%s", server.arg("bucket").c_str());
  if (server.hasArg("token") && server.arg("token").length() > 0)
    snprintf(influxToken, sizeof(influxToken), "%s", server.arg("token").c_str());
  if (server.hasArg("mh"))     snprintf(mqttHost,     sizeof(mqttHost),     "%s", server.arg("mh").c_str());
  if (server.hasArg("mp"))     snprintf(mqttPort,     sizeof(mqttPort),     "%s", server.arg("mp").c_str());
  if (server.hasArg("mu"))     snprintf(mqttUser,     sizeof(mqttUser),     "%s", server.arg("mu").c_str());
  if (server.hasArg("mpw") && server.arg("mpw").length() > 0)
    snprintf(mqttPass, sizeof(mqttPass), "%s", server.arg("mpw").c_str());
  if (server.hasArg("veh"))    snprintf(vehicleId,    sizeof(vehicleId),    "%s", server.arg("veh").c_str());
  if (server.hasArg("mac"))    snprintf(btMac,        sizeof(btMac),        "%s", server.arg("mac").c_str());
  if (server.hasArg("pin"))    snprintf(btPin,        sizeof(btPin),        "%s", server.arg("pin").c_str());
  if (server.hasArg("sip"))    snprintf(staticIP,     sizeof(staticIP),     "%s", server.arg("sip").c_str());
  if (server.hasArg("sgw"))    snprintf(staticGW,     sizeof(staticGW),     "%s", server.arg("sgw").c_str());
  if (server.hasArg("ssn"))    snprintf(staticSN,     sizeof(staticSN),     "%s", server.arg("ssn").c_str());
  if (server.hasArg("sdns"))   snprintf(staticDNS,    sizeof(staticDNS),    "%s", server.arg("sdns").c_str());
  saveConfig();
  Serial.println("Config updated via web page.");
  String h = "<html><body><h2>Saved.</h2>";
  h += "<p>Influx: " + String(influxHost) + " / " + String(influxBucket) + " &nbsp; MQTT: " + String(strlen(mqttHost)?mqttHost:"off") + "</p>";
  h += "<p>Reboot to apply MQTT / static IP.</p>";
  h += "<p><a href='/config'>&larr; edit again</a> &middot; <a href='/'>status</a></p></body></html>";
  server.send(200, "text/html", h);
}

// ---------------- OTA self-update (manual) ----------------
String fetchLatestVersion() {
  SerialBT.end(); btConnected=false; elmReady=false; delay(150);
  WiFiClientSecure c; c.setInsecure();
  HTTPClient http; http.setConnectTimeout(6000); http.setTimeout(6000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  String latest = "";
  int code = -1;
  if (http.begin(c, VERSION_URL)) { code = http.GET(); if (code == 200) latest = http.getString(); }
  http.end();
  latest.trim();
  snprintf(lastCheckMsg, sizeof(lastCheckMsg), "HTTP %d", code);
  return latest;
}
void handleCheckUpdate() {
  String latest = fetchLatestVersion();
  snprintf(latestVersion, sizeof(latestVersion), "%s", latest.c_str());
  updateChecked = true;
  updateAvailable = (latest.length() > 0 && latest != String(FW_VERSION));
  Serial.printf("[OTA] check: running=%s latest=%s (%s) avail=%d\n", FW_VERSION, latestVersion, lastCheckMsg, updateAvailable);
  server.sendHeader("Location", "/");
  server.send(303);
}
void handleDoUpdate() {
  server.send(200, "text/html", "<html><body><h3>Updating...</h3><p>Downloading firmware and rebooting. ~30s, then <a href='/'>reload</a>.</p></body></html>");
  delay(300);
  Serial.println("[OTA] Manual update; pausing Bluetooth for RAM.");
  SerialBT.end(); btConnected=false; elmReady=false; delay(200);
  WiFiClientSecure uc; uc.setInsecure();
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  httpUpdate.rebootOnUpdate(true);
  t_httpUpdate_return r = httpUpdate.update(uc, BIN_URL);
  if (r == HTTP_UPDATE_FAILED) {
    Serial.printf("[OTA] failed (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
    connectOBD();
  }
}

// ---------------- Manual firmware upload ----------------
void handleManualPage() {
  String h = "<html><body><h2>Manual firmware upload</h2>";
  h += "<p>Upload <b>logger-app-ota.bin</b> (the app image -- NOT logger-firmware.bin).</p>";
  h += "<form method='POST' action='/doupload' enctype='multipart/form-data'>";
  h += "<input type='file' name='f' accept='.bin'> <input type='submit' value='Upload &amp; Flash'></form>";
  h += "<p><a href='/'>&larr; status</a></p></body></html>";
  server.send(200, "text/html", h);
}
void handleUploadDone() {
  bool ok = !Update.hasError();
  server.send(200, "text/html", ok
    ? "<html><body><h3>Flashed OK -- rebooting...</h3><p>Reload <a href='/'>status</a> in ~10s.</p></body></html>"
    : "<html><body><h3>Upload FAILED</h3><p><a href='/manualupdate'>try again</a></p></body></html>");
  delay(400);
  if (ok) ESP.restart(); else connectOBD();
}
void handleUpload() {
  HTTPUpload& u = server.upload();
  if (u.status == UPLOAD_FILE_START) {
    Serial.printf("[OTA] Manual upload: %s\n", u.filename.c_str());
    SerialBT.end(); btConnected=false; elmReady=false;
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
  } else if (u.status == UPLOAD_FILE_WRITE) {
    if (Update.write(u.buf, u.currentSize) != u.currentSize) Update.printError(Serial);
  } else if (u.status == UPLOAD_FILE_END) {
    if (Update.end(true)) Serial.printf("[OTA] Manual OK: %u bytes\n", u.totalSize);
    else Update.printError(Serial);
  }
}
void handleReboot() {
  server.send(200, "text/html", "<html><body><h3>Rebooting...</h3><p>Reload <a href='/'>status</a> in ~10s.</p></body></html>");
  delay(400); ESP.restart();
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);
  delay(1200);
  setenv("TZ", "UTC0", 1); tzset();
  Serial.println("\n==== ESP32 OBD2 Logger boot ====");
  Serial.printf("FW %s | Reset reason: %d | heap: %u\n", FW_VERSION, (int)esp_reset_reason(), ESP.getFreeHeap());

  loadConfig();

  SPI.begin(18, 19, 23, SD_CS);
  for (int i = 0; i < 5 && !sdOk; i++) { sdOk = SD.begin(SD_CS); if (!sdOk) delay(400); }
  Serial.printf("SD card: %s\n", sdOk ? "OK" : "FAILED (reformat FAT32 / reseat / check power)");

  GPSserial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS serial started.");

  setupWiFi();

  server.on("/", handleRoot);
  server.on("/config", handleConfig);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/checkupdate", HTTP_POST, handleCheckUpdate);
  server.on("/doupdate", HTTP_POST, handleDoUpdate);
  server.on("/manualupdate", HTTP_GET, handleManualPage);
  server.on("/doupload", HTTP_POST, handleUploadDone, handleUpload);
  server.on("/reboot", HTTP_POST, handleReboot);
  server.begin();
  Serial.println("HTTP server up.");

  connectOBD();
}

void loop() {
  server.handleClient();
  feedGPS();

  loopCount++;
  if (millis() - lastRateMs >= 1000) { loopRate = loopCount; loopCount = 0; lastRateMs = millis(); }
  if (!sdOk && millis() - lastSdRetryMs > 10000) {
    lastSdRetryMs = millis();
    sdOk = SD.begin(SD_CS);
    if (sdOk) Serial.println("SD card mounted (retry).");
  }

  if (!btConnected || !elmReady) {
    if (millis() - lastConnTryMs > 5000) { lastConnTryMs = millis(); elmReady=false; connectOBD(); }
  } else {
    pollOBD();
  }

  if (elmReady && millis() - lastDtcMs > DTC_INTERVAL) { lastDtcMs = millis(); checkDTCs(); }

  if (millis() - lastLogMs > LOG_INTERVAL) { lastLogMs = millis(); logSnapshot(); }
  if (millis() - lastUploadMs > UPLOAD_INTERVAL) { lastUploadMs = millis(); flushBuffer(); }

  if (strlen(mqttHost) > 0 && WiFi.status() == WL_CONNECTED) {
    if (!mqtt.connected()) mqttReconnect();
    mqtt.loop();
    if (mqtt.connected() && millis() - lastMqttMs > MQTT_PUB_INTERVAL) { lastMqttMs = millis(); publishMqtt(); }
  }

  if (millis() - lastBeatMs > 10000) {
    lastBeatMs = millis();
    Serial.printf("[beat] up=%lus heap=%u WiFi=%s SD=%d buf=%u BT=%d MQTT=%d GPSsat=%u timeOK=%d\n",
                  millis()/1000, ESP.getFreeHeap(),
                  WiFi.status()==WL_CONNECTED?"yes":"no", sdOk, (unsigned)bufBytes(),
                  btConnected, mqtt.connected(), g_sats, time(nullptr) > 1700000000);
  }
}
