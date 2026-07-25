/*
 * ESP32 OBD2 Logger  ->  InfluxDB (with SD store-and-forward + GPS)
 * ------------------------------------------------------------------
 * Board:  LoLin D32 Pro (ESP32-WROVER, classic ESP32 -> Bluetooth Classic OK)
 *
 * Features: OBD2 over BT-Classic ELM327, GPS time+location, SD store-and-forward
 * to InfluxDB, misfire/DTC detection, live dashboard, and manual OTA self-update
 * (Check-for-update / Update-now buttons that pull firmware from raw.githubusercontent).
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
 * Libraries: ELMduino, WiFiManager, TinyGPSPlus (+ core SD/HTTPClient/HTTPUpdate)
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
#include <HTTPUpdate.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"

// Firmware version (commit SHA) is injected by the CI build via version.h.
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
#define GPS_RX_PIN  25   // ESP RX  <- GPS TX
#define GPS_TX_PIN  26   // ESP TX  -> GPS RX
#define GPS_BAUD    9600

// ---------------- Config (persisted) ----------------
Preferences prefs;
char influxHost[64]  = "";
char influxPort[6]   = "443";      // 443 = HTTPS (domain behind reverse proxy)
char influxOrg[48]   = "";
char influxBucket[48]= "";
char influxToken[192]= "";
char btMac[20]       = "00:1D:A5:07:5D:3C";
char btPin[8]        = "1234";
char vehicleId[24]   = "mycar";
bool shouldSaveConfig = false;

// ---------------- Globals ----------------
BluetoothSerial SerialBT;
ELM327 elm;
WebServer server(80);
TinyGPSPlus gps;
HardwareSerial GPSserial(2);

bool btConnected = false;
bool elmReady    = false;

// Latest OBD values (NAN = unknown / unsupported by this car -> skipped in output)
float v_rpm=NAN, v_speedMph=NAN, v_coolant=NAN, v_load=NAN, v_throttle=NAN,
      v_intake=NAN, v_maf=NAN, v_vbat=NAN, v_fuel=NAN;
float v_stft=NAN, v_ltft=NAN, v_timing=NAN, v_map=NAN, v_modV=NAN,
      v_runtime=NAN, v_ambient=NAN, v_oil=NAN;
float v_mil=NAN, v_dtcCount=NAN, v_misfire=NAN;
uint8_t pidState = 0;
const uint8_t PID_COUNT = 18;   // round-robin length (indices 0..17)
unsigned long lastDtcMs = 0;
const unsigned long DTC_INTERVAL = 60000;   // check trouble codes every 60s

// GPS latest
double g_lat=NAN, g_lon=NAN, g_alt=NAN, g_spdMph=NAN; uint32_t g_sats=0;

// Buffering
const char* BUF = "/buffer.lp";
const char* UP  = "/uploading.lp";
bool sdOk = false;

unsigned long lastLogMs = 0, lastUploadMs = 0, lastBeatMs = 0, lastConnTryMs = 0;
const unsigned long LOG_INTERVAL    = 2000;   // snapshot every 2s
const unsigned long UPLOAD_INTERVAL = 15000;  // try to flush every 15s
const float KMH_TO_MPH = 0.621371f;

// InfluxDB upload health (shown on status page)
int lastInfluxCode = 0;              // 0 = no upload attempted yet; 200/204 = ok; else error
unsigned long lastInfluxMs = 0;     // millis() of last upload attempt

// System / loop metrics
uint32_t loopCount = 0;             // counts loop() iterations within the current second
uint32_t loopRate = 0;             // loops/sec (proxy for CPU headroom)
unsigned long lastRateMs = 0, lastSdRetryMs = 0;

// OTA self-update (manual). Served from raw.githubusercontent (direct, no redirect).
const char* VERSION_URL = "https://raw.githubusercontent.com/Mdleal/esp32-obd2/logger/firmware/version.txt";
const char* BIN_URL     = "https://raw.githubusercontent.com/Mdleal/esp32-obd2/logger/firmware/logger-app-ota.bin";
bool updateChecked  = false;       // has the user run a check this session?
bool updateAvailable = false;      // latest != running
char latestVersion[48] = "";
char lastCheckMsg[64] = "";        // human-readable result of the last check

// ---------------- Config load/save ----------------
void loadConfig() {
  prefs.begin("logger", true);
  prefs.getString("iHost",  influxHost,  sizeof(influxHost));
  prefs.getString("iPort",  influxPort,  sizeof(influxPort));
  prefs.getString("iOrg",   influxOrg,   sizeof(influxOrg));
  prefs.getString("iBucket",influxBucket,sizeof(influxBucket));
  prefs.getString("iToken", influxToken, sizeof(influxToken));
  prefs.getString("btMac",  btMac,       sizeof(btMac));
  prefs.getString("btPin",  btPin,       sizeof(btPin));
  prefs.getString("veh",    vehicleId,   sizeof(vehicleId));
  prefs.end();
}
void saveConfig() {
  prefs.begin("logger", false);
  prefs.putString("iHost",  influxHost);
  prefs.putString("iPort",  influxPort);
  prefs.putString("iOrg",   influxOrg);
  prefs.putString("iBucket",influxBucket);
  prefs.putString("iToken", influxToken);
  prefs.putString("btMac",  btMac);
  prefs.putString("btPin",  btPin);
  prefs.putString("veh",    vehicleId);
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
  WiFiManagerParameter p_mac  ("mac",    "OBD2 Bluetooth MAC",btMac,       sizeof(btMac));
  WiFiManagerParameter p_pin  ("pin",    "OBD2 Bluetooth PIN",btPin,       sizeof(btPin));
  WiFiManagerParameter p_veh  ("veh",    "Vehicle name/tag",  vehicleId,   sizeof(vehicleId));
  wm.addParameter(&p_host); wm.addParameter(&p_port); wm.addParameter(&p_org);
  wm.addParameter(&p_bkt);  wm.addParameter(&p_tok);  wm.addParameter(&p_mac);
  wm.addParameter(&p_pin);  wm.addParameter(&p_veh);

  wm.setConfigPortalTimeout(0);  // stay until configured
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
    strncpy(btMac,       p_mac.getValue(),  sizeof(btMac));
    strncpy(btPin,       p_pin.getValue(),  sizeof(btPin));
    strncpy(vehicleId,   p_veh.getValue(),  sizeof(vehicleId));
    saveConfig();
    Serial.println("Config saved.");
  }
  Serial.print("WiFi OK, IP: "); Serial.println(WiFi.localIP());
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");  // UTC via NTP when online
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

  // Battery voltage (ATRV) -- blocking call, handle separately
  if (pidState == 7) { float bv=elm.batteryVoltage(); v_vbat=(bv>0)?bv:NAN; pidState=(pidState+1)%PID_COUNT; return; }

  // Monitor status (PID 0x01): MIL on/off + stored DTC count
  if (pidState == 17) {
    uint32_t ms = elm.monitorStatus();
    if (elm.nb_rx_state == ELM_SUCCESS) {
      uint8_t A = (ms >> 24) & 0xFF;   // byte A: bit7 = MIL, bits0-6 = DTC count
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
    resolved=true;   // error/unsupported -> keep old value, move on
  }
  if (resolved) pidState=(pidState+1)%PID_COUNT;
}

// Read stored DTCs (Mode 03, blocking) and flag misfire codes (P0300-P030x)
void checkDTCs() {
  elm.currentDTCCodes(true);
  int mis = 0;
  for (uint8_t i = 0; i < elm.DTC_Response.codesFound; i++) {
    Serial.printf("DTC: %s\n", elm.DTC_Response.codes[i]);
    if (strncmp(elm.DTC_Response.codes[i], "P030", 4) == 0) mis = 1;   // P0300-P0309 misfire
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
  // Set system clock from GPS (UTC) if we don't have valid time yet
  if (time(nullptr) < 1700000000 && gps.date.isValid() && gps.time.isValid() && gps.date.year() > 2020) {
    struct tm t = {};
    t.tm_year = gps.date.year()-1900; t.tm_mon = gps.date.month()-1; t.tm_mday = gps.date.day();
    t.tm_hour = gps.time.hour(); t.tm_min = gps.time.minute(); t.tm_sec = gps.time.second();
    time_t epoch = mktime(&t);   // TZ is UTC (set in setup) so this == UTC epoch
    struct timeval tv = { epoch, 0 };
    settimeofday(&tv, nullptr);
    Serial.println("Clock set from GPS.");
  }
}

// ---------------- Line-protocol build ----------------
String ns2str(uint64_t ns) {            // Arduino String has no uint64_t ctor
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
  if (now < 1700000000) return;                 // no valid time yet -> skip (GPS/NTP not ready)
  uint64_t ns = (uint64_t)now * 1000000000ULL;

  String line;
  // measurement: obd2
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
  else line = "";                                // no OBD fields -> skip obd line

  // measurement: gps
  if (!isnan(g_lat) && !isnan(g_lon)) {
    String gl = "gps,vehicle="; gl += vehicleId; gl += " ";
    bool gf = true;
    addF(gl,"lat",(float)g_lat,gf); addF(gl,"lon",(float)g_lon,gf);
    addF(gl,"alt_m",(float)g_alt,gf); addF(gl,"speed_mph",(float)g_spdMph,gf);
    if (!gf) { gl += " "; gl += ns2str(ns); gl += "\n"; line += gl; }
  }

  // measurement: system (ESP health -> graph memory/CPU in Grafana)
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
    Serial.print("[no SD] "); Serial.print(line);   // fallback: at least print it
  }
}

// ---------------- Uploader ----------------
bool influxHttps() { return strcmp(influxPort, "443") == 0; }  // 443 -> use TLS
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

  // If no pending upload, rotate current buffer into the upload slot.
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
  if (influxHttps()) {
    tls.setInsecure();                 // skip cert validation (fine for a homelab)
    http.begin(tls, influxUrl());
  } else {
    http.begin(plain, influxUrl());
  }
  http.addHeader("Content-Type", "text/plain; charset=utf-8");
  http.addHeader("Authorization", auth);
  code = http.sendRequest("POST", &f, sz);   // streams file as body (no big RAM use)
  f.close();
  http.end();
  lastInfluxCode = code;
  lastInfluxMs = millis();

  if (code == 204 || code == 200) {
    SD.remove(UP);
    Serial.printf("Uploaded %u bytes to InfluxDB (HTTP %d).\n", (unsigned)sz, code);
  } else {
    Serial.printf("InfluxDB upload failed (HTTP %d); will retry.\n", code);
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
  h += "</table>";

  h += "<h3>System</h3><ul>";
  h += "<li>Uptime: " + String(millis()/1000) + " s</li>";
  h += "<li>Free heap: " + String(ESP.getFreeHeap()) + " B (min " + String(ESP.getMinFreeHeap()) + ", largest block " + String(ESP.getMaxAllocHeap()) + ", total " + String(ESP.getHeapSize()) + ")</li>";
  h += "<li>PSRAM: free " + String(ESP.getFreePsram()) + " / " + String(ESP.getPsramSize()) + " B</li>";
  h += "<li>CPU: " + String(ESP.getCpuFreqMHz()) + " MHz &nbsp; loop rate " + String(loopRate) + "/s</li>";
  h += "<li>Chip temp: " + String(temperatureRead(), 1) + " C (rough)</li>";
  h += "</ul>";

  h += "<h3>Firmware</h3>";
  h += "<p>Running: " + String(FW_VERSION).substring(0, 12) + "</p>";
  h += "<form method='POST' action='/checkupdate' style='display:inline'><button>Check for update</button></form>";
  if (updateChecked) {
    h += " &mdash; last check: " + String(lastCheckMsg);
    if (updateAvailable) {
      h += " &mdash; <b>Update available:</b> " + String(latestVersion).substring(0, 12);
      h += " <form method='POST' action='/doupdate' style='display:inline'><button>Update now</button></form>";
    } else if (strlen(latestVersion) > 0) {
      h += " (up to date)";
    }
  }

  h += "<p><a href='/config'>/config</a> (edit settings)</p></body></html>";
  server.send(200, "text/html", h);
}

// ---------------- Config web page (edit settings while on WiFi) ----------------
void handleConfig() {
  String h = "<html><body><h2>Logger Configuration</h2>";
  h += "<form method='POST' action='/save'>";
  h += "InfluxDB host: <input name='host' value='" + String(influxHost) + "'><br><br>";
  h += "Port: <input name='port' value='" + String(influxPort) + "'><br><br>";
  h += "Org: <input name='org' value='" + String(influxOrg) + "'><br><br>";
  h += "Bucket: <input name='bucket' value='" + String(influxBucket) + "'><br><br>";
  h += "Token (leave blank to keep current): <input name='token' value='' size='50'><br><br>";
  h += "Vehicle tag: <input name='veh' value='" + String(vehicleId) + "'><br><br>";
  h += "OBD2 Bluetooth MAC: <input name='mac' value='" + String(btMac) + "'><br><br>";
  h += "OBD2 Bluetooth PIN: <input name='pin' value='" + String(btPin) + "'><br><br>";
  h += "<input type='submit' value='Save'></form>";
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
  if (server.hasArg("veh"))    snprintf(vehicleId,    sizeof(vehicleId),    "%s", server.arg("veh").c_str());
  if (server.hasArg("mac"))    snprintf(btMac,        sizeof(btMac),        "%s", server.arg("mac").c_str());
  if (server.hasArg("pin"))    snprintf(btPin,        sizeof(btPin),        "%s", server.arg("pin").c_str());
  saveConfig();
  Serial.println("Config updated via web page.");
  String h = "<html><body><h2>Saved.</h2>";
  h += "<p>Now sending to: " + String(influxHost) + ":" + String(influxPort) + " / bucket " + String(influxBucket) + "</p>";
  h += "<p><a href='/config'>&larr; edit again</a> &middot; <a href='/'>status</a></p></body></html>";
  server.send(200, "text/html", h);
}

// ---------------- OTA self-update (manual) ----------------
// Pause Bluetooth (frees ~40KB so the TLS fetch fits), grab latest version.
// BT is left down on purpose; loop() re-connects it so the web server stays responsive.
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
  server.send(303);   // back to dashboard
}
void handleDoUpdate() {
  server.send(200, "text/html", "<html><body><h3>Updating...</h3><p>Downloading firmware and rebooting. Give it ~30s, then <a href='/'>reload</a>.</p></body></html>");
  delay(300);
  Serial.println("[OTA] Manual update starting; pausing Bluetooth for RAM.");
  SerialBT.end(); btConnected=false; elmReady=false; delay(200);
  WiFiClientSecure uc; uc.setInsecure();
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  httpUpdate.rebootOnUpdate(true);
  t_httpUpdate_return r = httpUpdate.update(uc, BIN_URL);   // reboots on success
  if (r == HTTP_UPDATE_FAILED) {
    Serial.printf("[OTA] failed (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
    connectOBD();   // resume BT; user can retry
  }
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);
  delay(1200);
  setenv("TZ", "UTC0", 1); tzset();            // work in UTC for clean epoch timestamps
  Serial.println("\n==== ESP32 OBD2 Logger boot ====");
  Serial.printf("FW %s | Reset reason: %d | heap: %u\n", FW_VERSION, (int)esp_reset_reason(), ESP.getFreeHeap());

  loadConfig();

  // SD card -- retry a few times; some cards need a moment after power-up
  SPI.begin(18, 19, 23, SD_CS);
  for (int i = 0; i < 5 && !sdOk; i++) { sdOk = SD.begin(SD_CS); if (!sdOk) delay(400); }
  Serial.printf("SD card: %s\n", sdOk ? "OK" : "FAILED (reformat FAT32 / reseat / check power)");

  // GPS UART
  GPSserial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS serial started.");

  setupWiFi();

  server.on("/", handleRoot);
  server.on("/config", handleConfig);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/checkupdate", HTTP_POST, handleCheckUpdate);
  server.on("/doupdate", HTTP_POST, handleDoUpdate);
  server.begin();
  Serial.println("HTTP server up (/, /config, /checkupdate).");

  connectOBD();
}

void loop() {
  server.handleClient();
  feedGPS();

  // Loop-rate counter (CPU-headroom proxy) + SD auto-retry if it failed to mount
  loopCount++;
  if (millis() - lastRateMs >= 1000) { loopRate = loopCount; loopCount = 0; lastRateMs = millis(); }
  if (!sdOk && millis() - lastSdRetryMs > 10000) {
    lastSdRetryMs = millis();
    sdOk = SD.begin(SD_CS);
    if (sdOk) Serial.println("SD card mounted (retry).");
  }

  // Keep OBD alive
  if (!btConnected || !elmReady) {
    if (millis() - lastConnTryMs > 5000) { lastConnTryMs = millis(); elmReady=false; connectOBD(); }
  } else {
    pollOBD();
  }

  // Periodic trouble-code / misfire check (blocking, runs once a minute)
  if (elmReady && millis() - lastDtcMs > DTC_INTERVAL) {
    lastDtcMs = millis();
    checkDTCs();
  }

  // Snapshot to buffer
  if (millis() - lastLogMs > LOG_INTERVAL) { lastLogMs = millis(); logSnapshot(); }

  // Flush buffer to InfluxDB
  if (millis() - lastUploadMs > UPLOAD_INTERVAL) { lastUploadMs = millis(); flushBuffer(); }

  // Heartbeat
  if (millis() - lastBeatMs > 10000) {
    lastBeatMs = millis();
    Serial.printf("[beat] up=%lus heap=%u WiFi=%s SD=%d buf=%u BT=%d GPSsat=%u timeOK=%d\n",
                  millis()/1000, ESP.getFreeHeap(),
                  WiFi.status()==WL_CONNECTED?"yes":"no", sdOk, (unsigned)bufBytes(),
                  btConnected, g_sats, time(nullptr) > 1700000000);
  }
}
