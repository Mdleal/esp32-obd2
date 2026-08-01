#!/usr/bin/env python3
"""
CHIP OBD2/GPS logger -> MQTT (Home Assistant) + InfluxDB, with store-and-forward.

Reads OBD2 PIDs from a Veepeak over Bluetooth rfcomm (/dev/rfcomm0), position from
gpsd, and publishes to:
  * MQTT with Home Assistant auto-discovery (drop-in replacement for the ESP32 logger:
    same device id "esp32logger" + entity keys, so your existing HA dashboard and the
    "Car" device_tracker keep working).
  * InfluxDB 2.x line protocol (same measurements/fields as the ESP32: obd2/gps/system),
    buffered to disk and retried so nothing is lost when the tunnel/cell drops.

Config is read from environment variables (see chip-obd2-logger.env). Talk to MQTT and
InfluxDB at their HOME LAN IPs -- the WireGuard tunnel makes them reachable from the car.
"""
import os, time, json, math, socket, subprocess, threading
from pathlib import Path

import obd
import paho.mqtt.client as mqtt
import requests

try:
    import gpsd  # gpsd-py3
    HAVE_GPSD = True
except Exception:
    HAVE_GPSD = False

# ------------------------- config -------------------------
def env(k, d=""): return os.environ.get(k, d)

VEHICLE      = env("VEHICLE", "mycar")
OBD_PORT     = env("OBD_PORT", "/dev/rfcomm0")
OBD_BAUD     = int(env("OBD_BAUD", "38400"))

INFLUX_URL    = env("INFLUX_URL", "http://192.168.1.10:8086")   # home LAN IP over the tunnel
INFLUX_ORG    = env("INFLUX_ORG", "")
INFLUX_BUCKET = env("INFLUX_BUCKET", "obd2-ram")
INFLUX_TOKEN  = env("INFLUX_TOKEN", "")

MQTT_HOST = env("MQTT_HOST", "192.168.1.10")                    # home LAN IP over the tunnel
MQTT_PORT = int(env("MQTT_PORT", "1883"))
MQTT_USER = env("MQTT_USER", "")
MQTT_PASS = env("MQTT_PASS", "")

DEV_ID   = env("HA_DEVICE_ID", "esp32logger")                   # reuse ESP32 identity = same dashboard
DEV_NAME = env("HA_DEVICE_NAME", "OBD2 Logger")
BASE     = DEV_ID                                               # MQTT state topic base

LOG_INTERVAL    = float(env("LOG_INTERVAL", "2"))    # build a snapshot every N s
UPLOAD_INTERVAL = float(env("UPLOAD_INTERVAL", "15"))# flush InfluxDB buffer every N s
MQTT_INTERVAL   = float(env("MQTT_INTERVAL", "5"))   # publish to HA every N s
DTC_INTERVAL    = float(env("DTC_INTERVAL", "60"))

BUF = Path(env("BUFFER_FILE", "/var/lib/chip-logger/buffer.lp"))
BUF.parent.mkdir(parents=True, exist_ok=True)

# ------------------------- shared state -------------------------
vals = {}          # key -> float, latest OBD values
gps  = {"lat": None, "lon": None, "alt": None, "spd": None, "sats": 0}
lock = threading.Lock()

# HA discovery table: key, friendly name, unit, device_class
FIELDS = [
    ("rpm",         "Engine RPM",         "rpm", None),
    ("speed_mph",   "Vehicle Speed",      "mph", "speed"),
    ("coolant_c",   "Coolant Temp",       "°C", "temperature"),
    ("load_pct",    "Engine Load",        "%",   None),
    ("throttle_pct","Throttle",           "%",   None),
    ("intake_c",    "Intake Air Temp",    "°C", "temperature"),
    ("maf",         "MAF Rate",           "g/s", None),
    ("battery_v",   "Battery",            "V",   "voltage"),
    ("fuel_pct",    "Fuel Level",         "%",   None),
    ("stft_b1",     "Short Fuel Trim B1", "%",   None),
    ("ltft_b1",     "Long Fuel Trim B1",  "%",   None),
    ("timing_adv",  "Timing Advance",     "°", None),
    ("map_kpa",     "Manifold Pressure",  "kPa", "pressure"),
    ("module_v",    "Module Voltage",     "V",   "voltage"),
    ("runtime_s",   "Runtime",            "s",   "duration"),
    ("ambient_c",   "Ambient Temp",       "°C", "temperature"),
    ("oil_c",       "Oil Temp",           "°C", "temperature"),
    ("mil_on",      "Check Engine (MIL)", "",    None),
    ("dtc_count",   "Trouble Codes",      "",    None),
    ("misfire",     "Misfire",            "",    None),
]

# OBD command name -> (our key, transform)
OBD_MAP = [
    ("RPM",                  "rpm",         lambda v: v),
    ("SPEED",                "speed_mph",   lambda v: v * 0.621371),   # kph -> mph
    ("COOLANT_TEMP",         "coolant_c",   lambda v: v),
    ("ENGINE_LOAD",          "load_pct",    lambda v: v),
    ("THROTTLE_POS",         "throttle_pct",lambda v: v),
    ("INTAKE_TEMP",          "intake_c",    lambda v: v),
    ("MAF",                  "maf",         lambda v: v),
    ("FUEL_LEVEL",           "fuel_pct",    lambda v: v),
    ("SHORT_FUEL_TRIM_1",    "stft_b1",     lambda v: v),
    ("LONG_FUEL_TRIM_1",     "ltft_b1",     lambda v: v),
    ("TIMING_ADVANCE",       "timing_adv",  lambda v: v),
    ("INTAKE_PRESSURE",      "map_kpa",     lambda v: v),
    ("CONTROL_MODULE_VOLTAGE","module_v",   lambda v: v),
    ("RUN_TIME",             "runtime_s",   lambda v: v),
    ("AMBIANT_AIR_TEMP",     "ambient_c",   lambda v: v),
    ("OIL_TEMP",             "oil_c",       lambda v: v),
]

def mag(x):
    """Strip pint units from python-OBD Quantity -> float, else None."""
    try:
        return float(x.magnitude)
    except Exception:
        try:
            return float(x)
        except Exception:
            return None

# ------------------------- OBD thread -------------------------
def obd_loop():
    while True:
        try:
            conn = obd.OBD(OBD_PORT, baudrate=OBD_BAUD, fast=False, timeout=2)
            if not conn.is_connected():
                conn.close(); time.sleep(5); continue
            print("OBD connected:", conn.status())
            supported = {c.name for c in conn.supported_commands}
            last_dtc = 0
            while conn.is_connected():
                for name, key, tf in OBD_MAP:
                    cmd = obd.commands[name] if name in obd.commands else None
                    if cmd is None or (supported and name not in supported):
                        continue
                    r = conn.query(cmd, force=True)
                    if r and not r.is_null():
                        m = mag(r.value)
                        if m is not None:
                            with lock: vals[key] = tf(m)
                # battery voltage via ATRV
                r = conn.query(obd.commands.ELM_VOLTAGE, force=True)
                if r and not r.is_null():
                    m = mag(r.value)
                    if m is not None:
                        with lock: vals["battery_v"] = m
                # MIL + DTC count
                r = conn.query(obd.commands.STATUS, force=True)
                if r and not r.is_null():
                    st = r.value
                    with lock:
                        vals["mil_on"]    = 1.0 if getattr(st, "MIL", False) else 0.0
                        vals["dtc_count"] = float(getattr(st, "DTC_count", 0))
                # misfire from stored DTCs (P030x), throttled
                if time.time() - last_dtc > DTC_INTERVAL:
                    last_dtc = time.time()
                    r = conn.query(obd.commands.GET_DTC, force=True)
                    mis = 0.0
                    if r and not r.is_null():
                        for code, _desc in (r.value or []):
                            if str(code).startswith("P030"):
                                mis = 1.0
                    with lock: vals["misfire"] = mis
                time.sleep(0.2)
            conn.close()
        except Exception as e:
            print("OBD error:", e)
        time.sleep(5)

# ------------------------- GPS thread -------------------------
def gps_loop():
    if not HAVE_GPSD:
        print("gpsd-py3 not installed; GPS disabled")
        return
    while True:
        try:
            gpsd.connect()
            while True:
                p = gpsd.get_current()
                if p.mode >= 2:
                    with lock:
                        gps["lat"] = p.lat; gps["lon"] = p.lon
                        gps["sats"] = p.sats
                        if p.mode >= 3: gps["alt"] = p.alt
                        try: gps["spd"] = p.hspeed * 2.23694  # m/s -> mph
                        except Exception: pass
                time.sleep(1)
        except Exception as e:
            print("GPS error:", e); time.sleep(5)

# ------------------------- InfluxDB store-and-forward -------------------------
def influx_write_url():
    return f"{INFLUX_URL}/api/v2/write?org={INFLUX_ORG}&bucket={INFLUX_BUCKET}&precision=ns"

def snapshot_lp():
    now_ns = int(time.time() * 1e9)
    with lock:
        v = dict(vals); g = dict(gps)
    lines = []
    fields = ",".join(f"{k}={v[k]:.2f}" for k in
                      ["rpm","speed_mph","coolant_c","load_pct","throttle_pct","intake_c","maf",
                       "battery_v","fuel_pct","stft_b1","ltft_b1","timing_adv","map_kpa","module_v",
                       "runtime_s","ambient_c","oil_c","mil_on","dtc_count","misfire"]
                      if k in v and v[k] is not None)
    if fields:
        lines.append(f"obd2,vehicle={VEHICLE} {fields} {now_ns}")
    if g["lat"] is not None and g["lon"] is not None:
        gf = [f"lat={g['lat']:.6f}", f"lon={g['lon']:.6f}"]
        if g["alt"] is not None: gf.append(f"alt_m={g['alt']:.1f}")
        if g["spd"] is not None: gf.append(f"speed_mph={g['spd']:.1f}")
        lines.append(f"gps,vehicle={VEHICLE} {','.join(gf)} {now_ns}")
    return "\n".join(lines) + ("\n" if lines else "")

def influx_loop():
    last = 0
    while True:
        lp = snapshot_lp()
        if lp:
            with BUF.open("a") as f: f.write(lp)
        if time.time() - last >= UPLOAD_INTERVAL:
            last = time.time()
            try:
                data = BUF.read_bytes()
                if data and INFLUX_TOKEN:
                    r = requests.post(influx_write_url(),
                                      headers={"Authorization": f"Token {INFLUX_TOKEN}",
                                               "Content-Type": "text/plain; charset=utf-8"},
                                      data=data, timeout=10)
                    if r.status_code in (200, 204):
                        BUF.write_bytes(b"")  # sent OK, clear buffer
                    else:
                        print("InfluxDB HTTP", r.status_code, r.text[:200])
            except Exception as e:
                print("InfluxDB flush error (buffered):", e)
        time.sleep(LOG_INTERVAL)

# ------------------------- MQTT / Home Assistant -------------------------
DEVJSON = {"ids": [DEV_ID], "name": DEV_NAME, "mf": "DIY", "mdl": "NextThing C.H.I.P."}

def mqtt_discovery(cli):
    for key, name, unit, dc in FIELDS:
        cfg = {"name": name, "uniq_id": f"{DEV_ID}_{key}",
               "stat_t": f"{BASE}/{key}", "avty_t": f"{BASE}/status",
               "stat_cla": "measurement", "dev": DEVJSON}
        if unit: cfg["unit_of_meas"] = unit
        if dc:   cfg["dev_cla"] = dc
        cli.publish(f"homeassistant/sensor/{DEV_ID}/{key}/config", json.dumps(cfg), retain=True)
    # GPS device_tracker (shows the car on the HA map)
    tcfg = {"name": "Car", "uniq_id": f"{DEV_ID}_car",
            "json_attr_t": f"{BASE}/gps_attr", "source_type": "gps",
            "avty_t": f"{BASE}/status", "dev": DEVJSON}
    cli.publish(f"homeassistant/device_tracker/{DEV_ID}/car/config", json.dumps(tcfg), retain=True)

def on_connect(cli, u, flags, rc, props=None):
    print("MQTT connected rc", rc)
    cli.publish(f"{BASE}/status", "online", retain=True)
    mqtt_discovery(cli)

def mqtt_loop():
    cli = mqtt.Client(client_id=f"{DEV_ID}-chip")
    if MQTT_USER: cli.username_pw_set(MQTT_USER, MQTT_PASS)
    cli.will_set(f"{BASE}/status", "offline", retain=True)
    cli.on_connect = on_connect
    while True:
        try:
            cli.connect(MQTT_HOST, MQTT_PORT, 60); break
        except Exception as e:
            print("MQTT connect retry:", e); time.sleep(5)
    cli.loop_start()
    while True:
        with lock:
            v = dict(vals); g = dict(gps)
        for key, *_ in FIELDS:
            if key in v and v[key] is not None:
                cli.publish(f"{BASE}/{key}", f"{v[key]:.2f}", retain=True)
        if g["lat"] is not None and g["lon"] is not None:
            attr = {"latitude": round(g["lat"], 6), "longitude": round(g["lon"], 6),
                    "gps_accuracy": 10, "satellites": g["sats"]}
            if g["alt"] is not None: attr["altitude"] = round(g["alt"], 1)
            if g["spd"] is not None: attr["gps_speed_mph"] = round(g["spd"], 1)
            cli.publish(f"{BASE}/gps_attr", json.dumps(attr), retain=True)
        time.sleep(MQTT_INTERVAL)

# ------------------------- main -------------------------
if __name__ == "__main__":
    for fn in (obd_loop, gps_loop, influx_loop, mqtt_loop):
        threading.Thread(target=fn, daemon=True).start()
    while True:
        time.sleep(3600)
