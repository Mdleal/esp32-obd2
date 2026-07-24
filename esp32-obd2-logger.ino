/*
 * ESP32 OBD2 Logger  ->  InfluxDB (with SD store-and-forward + GPS)
 * ------------------------------------------------------------------
 * Board:  LoLin D32 Pro (ESP32-WROVER, classic ESP32 -> Bluetooth Classic OK)
 *
 * What it does:
 *   - Reads OBD2 PIDs from a Bluetooth-Classic ELM327 dongle (Veepeak).
 *   - Reads GPS (ATGM336H / NEO-6M) for accurate UTC time + location.
 *   - Every LOG_INTERVAL it snapshots the data into InfluxDB line-protocol
 *     lines with a real timestamp.
 *   - If WiFi + InfluxDB are reachable, the buffer is flushed to the server.
 *   - If offline, lines accumulate on the microSD card and are uploaded
 *     automatically once WiFi returns -- so no data is lost in dead zones.
 *
 * Store-and-forward design (crash-safe, keeps time order):
 *   - New points are always appended to  /buffer.lp  on the SD card.
 *   - The uploader rotates  /buffer.lp -> /uploading.lp,  POSTs it to
 *     InfluxDB, and deletes it on HTTP 204. A failed/pending upload is
 *     retried; fresh data keeps appending to a new /buffer.lp meanwhile.
 *
 * First-boot config: captive-portal AP "ESP32-LOGGER" (pw: loggersetup).
 * Enter WiFi + InfluxDB (host/port/org/bucket/token) + OBD MAC. Saved to flash.
 *
 * Wiring (LoLin D32 Pro):
 *   microSD  -> on-board slot (VSPI: SCK=18 MISO=19 MOSI=23  CS=GPIO4)
 *   GPS  VCC -> 3V3,  GND -> GND
 *   GPS  TX  -> GPIO25 (ESP RX)      GPS RX -> GPIO26 (ESP TX, optional)
 *   NOTE: GPIO16/17 are used by PSRAM on WROVER -- do NOT use them for GPS.
 *
 * Build: FQBN esp32:esp32:lolin_d32_pro:PartitionScheme=min_spiffs
 * Libraries: ELMduino, WiFiManager, TinyGPSPlus, ElegantOTA (+ core SD/HTTPClient)
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
#include <ElegantOTA.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"

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

// Latest OBD values (NAN = unknown)
float v_rpm=NAN, v_speedMph=NAN, v_coolant=NAN, v_load=NAN, v_throttle=NAN,
      v_intake=NAN, v_maf=NAN, v_vbat=NAN, v_fuel=NAN;
uint8_t pidState = 0;
const uint8_t PID_COUNT = 9;

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
  WiFiManagerParameter p_veh  ("veh",    "Vehicle name/tag",  vehicleId,   sizeof(vehicleId));
  wm.addParameter(&p_host); wm.addParameter(&p_port); wm.addParameter(&p_org);
  wm.addParameter(&p_bkt);  wm.addParameter(&p_tok);  wm.addParameter(&p_mac);
  wm.addParameter(&p_veh);

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
  if (pidState == 7) { float bv=elm.batteryVoltage(); v_vbat=(bv>0)?bv:NAN; pidState=(pidState+1)%PID_COUNT; return; }
  float val=NAN; bool done=false;
  switch (pidState) {
    case 0: val=elm.rpm(); break;
    case 1: val=elm.kph(); break;
    case 2: val=elm.engineCoolantTemp(); break;
    case 3: val=elm.engineLoad(); break;
    case 4: val=elm.throttle(); break;
    case 5: val=elm.intakeAirTemp(); break;
    case 6: val=elm.mafRate(); break;
    case 8: val=elm.fuelLevel(); break;
  }
  if (elm.nb_rx_state == ELM_SUCCESS) {
    switch (pidState) {
      case 0: v_rpm=val; break;
      case 1: v_speedMph=val*KMH_TO_MPH; break;
      case 2: v_coolant=val; break;
      case 3: v_load=val; break;
      case 4: v_throttle=val; break;
      case 5: v_intake=val; break;
      case 6: v_maf=val; break;
      case 8: v_fuel=val; break;
    }
    done=true;
  } else if (elm.nb_rx_state != ELM_GETTING_MSG) { done=true; }
  if (done) pidState=(pidState+1)%PID_COUNT;
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
void handleRoot() {
  String h = "<html><body><h2>ESP32 OBD2 Logger</h2>";
  h += "<p>OBD link: " + String(elmReady&&btConnected?"connected":"DISCONNECTED") + "</p>";
  h += "<p>GPS sats: " + String(g_sats) + " | fix: " + String(!isnan(g_lat)?"yes":"no") + "</p>";
  h += "<p>WiFi: " + String(WiFi.status()==WL_CONNECTED?WiFi.localIP().toString():"down") + "</p>";
  h += "<p>SD: " + String(sdOk?"ok":"FAIL") + " | buffered: " + String(bufBytes()) + " bytes</p>";
  h += "<p>InfluxDB: " + String(influxHost) + ":" + String(influxPort) + " / " + String(influxBucket) + "</p>";
  h += "<p>Latest: RPM " + String(v_rpm,0) + ", " + String(v_speedMph,1) + " mph, coolant " + String(v_coolant,0) + "C</p>";
  h += "<p><a href='/update'>/update</a> (OTA)</p></body></html>";
  server.send(200, "text/html", h);
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);
  delay(1200);
  setenv("TZ", "UTC0", 1); tzset();            // work in UTC for clean epoch timestamps
  Serial.println("\n==== ESP32 OBD2 Logger boot ====");
  Serial.printf("Reset reason: %d | heap: %u\n", (int)esp_reset_reason(), ESP.getFreeHeap());

  loadConfig();

  // SD card
  SPI.begin(18, 19, 23, SD_CS);
  sdOk = SD.begin(SD_CS);
  Serial.printf("SD card: %s\n", sdOk ? "OK" : "FAILED (check card/insertion)");

  // GPS UART
  GPSserial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS serial started.");

  setupWiFi();

  server.on("/", handleRoot);
  ElegantOTA.begin(&server);
  server.begin();
  Serial.println("HTTP server up (/, /update).");

  connectOBD();
}

void loop() {
  server.handleClient();
  ElegantOTA.loop();
  feedGPS();

  // Keep OBD alive
  if (!btConnected || !elmReady) {
    if (millis() - lastConnTryMs > 5000) { lastConnTryMs = millis(); elmReady=false; connectOBD(); }
  } else {
    pollOBD();
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
