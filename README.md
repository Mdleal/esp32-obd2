# ESP32 OBD2 → Home Assistant / Prometheus

Reads live car data from your **Veepeak Mini (Bluetooth Classic)** dongle using an
**original ESP32 (esp32dev / WROOM)** and shows it as:

- a **Prometheus** page at `http://<board-ip>/metrics` (works with zero extra setup)
- optional **Home Assistant** entities over MQTT (fill in a broker during setup)

You don't need to install any programming tools. GitHub builds the firmware file
for you in the cloud, and you flash it from a web page by clicking one button.

Read data included: RPM, speed, coolant temp, engine load, throttle, intake air
temp, MAF, and battery voltage.

---

## What you need

- The original **ESP32 (esp32dev / WROOM)** board — NOT an S3 or C3.
- Your **Veepeak Mini** OBD2 dongle.
- A **USB cable** that connects the ESP32 to your computer.
- **Chrome or Edge** browser on a desktop/laptop (needed to flash).
- The car, with the key/ignition on, when you first pair.

---

## Part 1 — Make the firmware file (about 5 minutes, mostly waiting)

1. **Make a free GitHub account** at https://github.com (skip if you have one).
2. Click the **+** at the top right → **New repository**. Give it any name
   (e.g. `esp32-obd2`), choose **Public**, and click **Create repository**.
3. On the new empty repo page, click **uploading an existing file**.
4. Drag in **everything inside the `esp32-obd2-project` folder** — that's the
   `esp32-obd2` folder AND the `.github` folder (and this README). Then click
   **Commit changes**. (Upload the *contents*, so `.github` ends up at the top
   level of the repo — the build won't run otherwise.)
5. Go to the **Settings** tab → **Pages** (left menu). Under
   **Build and deployment → Source**, choose **GitHub Actions**.
6. Go to the **Actions** tab. You'll see a build running. Wait for the green
   check mark (~3–4 minutes). If the very first run shows a red X on the
   "deploy" step, just click **Re-run all jobs** — that only happens if Pages
   wasn't switched on yet in step 5.

That's it — the firmware is built.

---

## Part 2 — Flash it onto the ESP32 (one click)

1. Open your flasher page in **Chrome or Edge**:
   `https://YOUR-USERNAME.github.io/YOUR-REPO-NAME/`
   (replace with your GitHub username and repo name).
2. Plug the ESP32 into your computer with the USB cable.
3. Click **Install**, pick the ESP32's serial port from the list, and confirm.
   - If no port appears, your USB cable may be charge-only — try another cable.
   - If it fails to start, hold the **BOOT** button on the board while you click
     Install, then release once it starts writing.
4. Wait for it to finish and reboot.

---

## Part 3 — Connect it to your WiFi (on your phone)

1. After flashing, the board creates its own WiFi network called **`ESP32-OBD2`**
   (password **`obd2setup`**). Join it with your phone or laptop.
2. A setup page opens automatically (if not, browse to `http://192.168.4.1`).
   Tap **Configure WiFi**.
3. Pick your home WiFi and type its password.
4. On the same page you'll see extra boxes:
   - **MQTT host** — leave **blank** if you only want Prometheus. To use Home
     Assistant, enter your MQTT broker's IP (e.g. `192.168.1.10`) plus username
     and password if it needs them.
   - **OBD2 Bluetooth MAC** — already filled in with your dongle's address
     (`00:1D:A5:07:5D:3C`). Leave it as-is. (This is the most reliable way to
     connect; the name field is only a fallback.)
5. Save. The board reboots and joins your WiFi.

---

## Part 4 — See your data

Find the board's IP address (check your router's device list, or the ESP32's
serial output). Then:

- **Live view:** `http://<board-ip>/` — shows all values in your browser.
- **Prometheus:** `http://<board-ip>/metrics` — add this scrape job to your
  `prometheus.yml`:

  ```yaml
  scrape_configs:
    - job_name: car_obd2
      scrape_interval: 15s
      static_configs:
        - targets: ["<board-ip>:80"]
  ```

- **Home Assistant** (only if you filled in MQTT): make sure the **MQTT
  integration** is set up in HA pointing at the same broker. A device called
  **ESP32 OBD2** with all the sensors appears automatically — no YAML needed.

The car must be running (or at least ignition on) for the dongle to answer.

---

## Updating later (no USB needed)

The board has a built-in update page. To push a new version over WiFi:

1. In GitHub, open the latest **Actions** run → download the **firmware-bin**
   artifact → unzip it. Use the file named **`app-ota.bin`** (NOT `firmware.bin`).
2. Go to `http://<board-ip>/update` in your browser, choose `app-ota.bin`,
   and upload.

(First-time flashing must use USB + `firmware.bin`. Only later updates can go
over WiFi with `app-ota.bin`.)

---

## Troubleshooting

- **Live view shows "DISCONNECTED":** the board can't reach the dongle. Make
  sure the dongle is plugged into the car's OBD2 port, the ignition is on, and
  the board is powered near the car. If it still won't connect, the dongle's
  Bluetooth name might not be `OBDII` — re-run setup (Part 3) and change it.
- **Some values show "n/a":** not every car reports every PID. That's normal.
- **Forgot the board's IP / want to redo WiFi:** hold the board's BOOT button
  briefly won't reset config; instead, if it can't find your saved WiFi it will
  automatically bring back the `ESP32-OBD2` setup network.
