# CHIP OBD2 logger — install & pair

Run these on the CHIP over SSH. Order matters: pair the Veepeak first, then install.

## 1. Dependencies

```sh
sudo apt update
sudo apt install -y python3-pip bluez gpsd gpsd-clients rfkill
pip3 install --break-system-packages obd paho-mqtt requests gpsd-py3
```

## 2. Pair the Veepeak (Bluetooth, once)

Turn the car's ignition on so the dongle is powered, then:

```sh
sudo rfkill unblock bluetooth
bluetoothctl
  power on
  agent on
  default-agent
  scan on            # wait until you see 00:1D:A5:07:5D:3C
  scan off
  pair 00:1D:A5:07:5D:3C     # PIN is 1234 if asked
  trust 00:1D:A5:07:5D:3C
  quit
```

Bind it to a serial device and confirm it connects:

```sh
sudo rfcomm bind 0 00:1D:A5:07:5D:3C 1
ls -l /dev/rfcomm0
```

## 3. gpsd (already set up earlier — just confirm)

```sh
cgps -s        # should show satellites/fix once the antenna has sky view
```

## 4. Install the logger

```sh
sudo mkdir -p /opt/chip-logger /var/lib/chip-logger
sudo cp logger.py /opt/chip-logger/
sudo cp chip-obd2-logger.env /etc/chip-logger.env
sudo cp chip-obd2-logger.service /etc/systemd/system/
$EDITOR /etc/chip-logger.env     # set INFLUX_URL/ORG/TOKEN + MQTT_HOST to your HOME LAN IPs
```

`INFLUX_URL` and `MQTT_HOST` use your home LAN IPs — the WireGuard tunnel makes them reachable
from the car. Keep `HA_DEVICE_ID=esp32logger` to reuse your existing HA dashboard + "Car" tracker
(retire the ESP32 so they don't both publish), or change it for a separate device.

## 5. Test in the foreground first

```sh
sudo env $(grep -v '^#' /etc/chip-logger.env | xargs) python3 /opt/chip-logger/logger.py
```

You should see `OBD connected`, then in Home Assistant the sensors + the "Car" tracker update,
and in InfluxDB the `obd2`/`gps` measurements fill in. Ctrl-C when it looks good.

## 6. Run it as a service

```sh
sudo systemctl daemon-reload
sudo systemctl enable --now chip-obd2-logger
journalctl -u chip-obd2-logger -f
```

## Notes

- **Store-and-forward:** InfluxDB writes are buffered to `/var/lib/chip-logger/buffer.lp` and
  retried, so a dropped tunnel/cell link loses nothing — it backfills when the link returns.
  (HA/MQTT is live-only; that's expected — InfluxDB is your gapless history.)
- **rfcomm on boot:** the service binds `/dev/rfcomm0` via `ExecStartPre`. If the dongle isn't
  powered (car off), OBD just retries until it is.
- **Graceful shutdown:** next step is a small AXP209 watcher that flushes + powers off cleanly when
  the car cuts 12V — important on raw NAND. Ask and I'll add it.
