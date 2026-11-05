# Octopus Tactical — Waveshare ESP32-S3-Touch-AMOLED-2.06

ESP-IDF app for the 410x502 AMOLED watch. Clock, gyro compass, screen light, and an on-screen Morse SOS flash.

Built with ESP-IDF 5.5.1. From this directory:

```
idf.py build
idf.py -p /dev/ttyACM0 -b 921600 flash
```

Tap on the clock or compass moves to the next view. Tap on Tools cycles the screen light. Tap on SOS arms or disarms the beacon. The BOOT button always moves to the next view. The screen dims after 20 seconds.

Sun and moon markers use Houston (29.7633 N, 95.3633 W) unless `lat` and `lon` in `main/main.c` are changed. Heading comes from the QMI8658 gyro, so it drifts.
