# Octopus OS — Waveshare ESP32-S3-Touch-AMOLED-2.06

Watch firmware with the Octopus PWA app set: face, app grid, texts, calls, AI, vitals, alerts, time set, nav, music, compass, tools, SOS, companion link, and BLE mod injector.

Built with ESP-IDF 5.5.1:

```
idf.py build
idf.py -p /dev/ttyACM0 -b 921600 flash
```

## Wrist

Tap the clock for the app grid. Long-press the clock to set time. BOOT from an app returns to the grid; BOOT on the grid returns to the face.

SOS tap starts an 8-second countdown, then the screen flashes Morse SOS. Tap again to cancel.

## Phone

The watch advertises as **InfiniTime** so Gadgetbridge can forward notifications and time.

Companion PWA: `phone/` (`index.html` / `companion.html`). Chrome, HTTPS or localhost. Connect, then inject mods, push messages, ring, set time, send vitals / nav / music, and write WiFi + AI websocket config (`ssid|password|ws://host:8765/ws`).

Mod injector writes a tile onto the grid immediately (info, action, or AI prompt). Up to 6 mods.

## Hardware notes

This board has no PPG, NFC, or magnetometer. Vitals and wallet-style pay are phone-driven or display-only. Compass heading is gyro-integrated and drifts. AI hold-to-talk uses the onboard mic/speaker over WiFi to `watch-ai/server.py`.
