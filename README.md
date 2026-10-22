# Doll Head Thermal Tracker

An ESP32 inside a doll head. An AMG8833 8×8 thermal sensor finds the warmest
person-shaped blob in view, a pan servo turns the head to follow it, and a
second servo blinks the eyelid. Everything is controlled from a web page
served by the device itself.

## Hardware

| Part                | ESP32 pin            |
| ------------------- | -------------------- |
| Pan servo (90 g)    | GPIO 13              |
| Eyelid servo (90 g) | GPIO 32              |
| AMG8833 SDA / SCL   | GPIO 21 / GPIO 22    |

Board: ESP32 Dev Module (`esp32dev`).

## First-time setup

1. Install [VS Code](https://code.visualstudio.com/) with the **PlatformIO IDE**
   extension.
2. Copy `.env.example` to `.env` and fill it in:

   ```ini
   # Primary network (e.g. home)
   WIFI_SSID=YourHomeNetwork
   WIFI_PASSWORD=YourHomePassword

   # Optional second network (e.g. work)
   WIFI2_SSID=YourWorkNetwork
   WIFI2_PASSWORD=YourWorkPassword

   # Fallback hotspot password (min 8 characters)
   AP_PASSWORD=dollhead

   # Password for wireless firmware uploads
   OTA_PASSWORD=choose-something
   ```

   `.env` is git-ignored. Credentials are compiled into the firmware, so
   **re-flash after changing them**.
3. Plug the board in over USB and click **Upload** in PlatformIO (or run
   `pio run -t upload`). The first build downloads the ESP32 toolchain and can
   take several minutes.

## Connecting in a browser

Open **<http://dollhead.local/>**.

That one address works on whichever network the device is on. At boot it
scans for both networks in `.env` and joins whichever is in range, and it
reconnects on its own if the connection drops. Your phone or computer must be
on the **same WiFi network** as the device.

### If `dollhead.local` doesn't load

- **Use the IP address instead.** Open the Serial Monitor (115200 baud) and
  reset the board. It prints a line like
  `WiFi connected to "..." — IP: 192.168.0.203`. Browse to `http://<that IP>/`.
  Once the page is open, the IP is also shown under **Status**.
- **Android 11 or older** can't resolve `.local` names. Use the IP.
- **Work / guest networks** often block devices from seeing each other
  (client isolation). If neither the name nor the IP works there, only the
  network admin can allow it.

### Backup hotspot

If neither network is found within about 20 seconds of power-up, the device
starts its own WiFi network:

| | |
| --- | --- |
| Network  | `DollHead-Setup` |
| Password | `AP_PASSWORD` from `.env` (default `dollhead`) |
| Address  | <http://dollhead.local/> or <http://192.168.4.1/> |

It keeps looking for your networks every 30 seconds, and the hotspot shuts off
as soon as it joins one.

## Using the web page

- **Live view:** thermal image with min / median / max temperatures. A
  crosshair marks the tracked target (green = tracking, yellow = holding on
  the last position).
- **Display mode:** Thermal (color), Gray, or Person (highlights only
  body-temperature pixels).
- **Range:** auto-scaling by default; tick *Manual range* to set min/max.
- **Mode** (top of the panel). The line underneath shows what the head is
  doing right now:
  - **Active:** normal tracking and blinking.
  - **Standby:** pauses all movement. Always comes back up as Active after a
    reboot.
  - **Periodic:** mostly in standby, waking for brief bursts. It rests for a
    random 2 minutes to 2 hours, wakes for a random 2 to 6 seconds (blinking
    once or twice, at random, as it wakes), and repeats until you pick
    another mode. The status line counts down to the
    next switch. Periodic survives a reboot.
- **Standby schedule:** tick *Standby every day* and set a From/To time (it can
  wrap past midnight, e.g. 22:00 to 07:00). While the window is open the head
  is in standby, whatever the mode. When it closes, the selected mode resumes.
  - Tick *Standby all weekend* to keep it in standby all day Saturday and
    Sunday. This works with or without the daily window.
  - The schedule needs the device's clock. It gets the time from the internet,
    or from your browser when there's no internet (e.g. on the backup
    hotspot). **Device clock** shows the time the device thinks it is.
  - The time zone is taken from your browser the first time you open the page.
    Change it with the dropdown if it's wrong.
- **Motor:** turn tracking on or off. Hold ◀ / ▶ to pan by hand; automatic
  tracking resumes 20 s after you let go.
- **Blink servo:** blink interval (30 s to 24 h in normal mode, or
  milliseconds in debug mode), duration, open/closed angles, and *Blink Now*.
- **Rotate servo:** scan speed.

Settings changes are saved to flash about 2 seconds after you stop adjusting,
so they survive a reboot.

## Wireless (OTA) updates

After the first USB flash, you can upload new firmware over WiFi from any
network the device is on:

```sh
pio run -t upload -e dollhead-ota
```

This uploads to `dollhead.local` using `OTA_PASSWORD` from `.env`.

## Build environments

| Environment    | What it does                                               |
| -------------- | ---------------------------------------------------------- |
| `dollhead`     | Main app over USB (the default for Build / Upload)         |
| `dollhead-ota` | Main app over WiFi                                         |
| `servo-test`   | Servo-only diagnostic, no WiFi (`pio run -t upload -e servo-test`) |

Flashing `servo-test` replaces the main app. Flash `dollhead` again afterwards.

The project is pinned to the [pioarduino](https://github.com/pioarduino/platform-espressif32)
platform (Arduino-ESP32 core 3.x). Stock PlatformIO `espressif32` only ships
core 2.x, which lacks `ledcAttach` and won't compile this code.

## Troubleshooting

- **Build sits at "Building ….bin" and `esptool.py` opens in a text editor:**
  `scripts/fix_esptool.py` works around a pioarduino bug on Windows that makes
  the build open `esptool.py` in your default `.py` app instead of running it.
  Make sure each build prints `[fix_esptool] OBJCOPY -> …esptool.exe`. If it
  doesn't, check that the script is still listed in `extra_scripts`.
- **`pio` is not recognized** in a regular terminal: the PlatformIO extension
  doesn't add `pio` to your PATH. Open a PlatformIO terminal (PlatformIO
  sidebar → *Quick Access → Miscellaneous → New Terminal*), use *Project
  Tasks* in the sidebar instead, or call it by full path:
  `& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run ...` (PowerShell).
- **`Error: uv command not found`** during builds is a harmless warning.
- **Device reachable at home but not at work:** see *If `dollhead.local`
  doesn't load* above.

## HTTP API

The web page is built on these endpoints, which you can also script against:

| Method | Path        | Purpose                                                |
| ------ | ----------- | ------------------------------------------------------ |
| GET    | `/`         | Web UI                                                 |
| GET    | `/frame`    | Latest 8×8 frame, tracker state, and current settings (JSON) |
| GET    | `/settings` | Current settings (JSON)                                |
| POST   | `/settings` | Change settings. JSON body with any subset of keys, e.g. `{"runMode": 2}` or `{"triggerBlink": true}` |

Run and schedule keys for `POST /settings`:

| Key            | Value                                                           |
| -------------- | --------------------------------------------------------------- |
| `runMode`      | `0` Active, `1` Standby, `2` Periodic                           |
| `standbyMode`  | `true` / `false`, same as `runMode` 1 / 0 (older key)           |
| `schedEnabled` | `true` / `false`                                                |
| `weekendStandby` | `true` / `false`: standby all day Saturday and Sunday         |
| `schedStart`, `schedEnd` | minutes after midnight, local time (e.g. `1320` = 22:00) |
| `tz`           | POSIX time zone string, e.g. `CST6CDT,M3.2.0,M11.1.0`           |

## Project layout

```text
src/main.cpp            main firmware (tracking, blink, WiFi, web UI)
src/servo_test.cpp      servo diagnostic
scripts/load_env.py     injects .env values as compile-time defines
scripts/fix_esptool.py  Windows esptool workaround (see Troubleshooting)
platformio.ini          build environments
.env.example            template for your .env
```
