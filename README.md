# obscure-smartwatch--connect

Octopus OS: phone companion PWA plus firmware for the Waveshare ESP32-S3-Touch-AMOLED-2.06.

## Phone companion

`octopus-tactical/phone/` is the installable Chrome PWA. It talks to the watch over Web Bluetooth (the watch advertises as **InfiniTime**). Use it to set time, inject mods, push messages, ring, send vitals/nav/music, and write WiFi + AI config.

Root `index.html` is the original Octopus OS simulator PWA.

## Watch firmware

`octopus-tactical/` is ESP-IDF 5.5.1 firmware. Face, app grid (texts, call, AI, vitals, alerts, time, nav, music, compass, tools, SOS, link), BLE mod injector, Gadgetbridge time/notifications.

```
cd octopus-tactical
idf.py build
idf.py -p /dev/ttyACM0 -b 921600 flash
```
