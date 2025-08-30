# ClosestPlane ESP32

An Arduino-compatible sketch for ESP32 boards that connects to a dump1090 server, identifies the closest aircraft to your location and shows details on a 128x64 SSD1306 OLED. If the aircraft comes within 5 km an audible alert is played through a MAX98357 I2S amplifier.

## Required Libraries

Install the following libraries in the Arduino IDE before compiling:

- **ESP32 board support** (`esp32` by Espressif) – provides WiFi, HTTPClient, Wire and I2S functionality.
- **Adafruit GFX Library**
- **Adafruit SSD1306**
- **ArduinoJson**

## Setup
1. Rename `config.h` with your WiFi credentials, dump1090 server address and your latitude/longitude. Adjust I2S pin numbers if required.
2. Ensure the libraries above are installed in the Arduino IDE.
3. Open `closestPlane.ino` in the Arduino IDE, select your ESP32 board and the correct port.
4. Compile and upload.

## Operation
The display refreshes every few seconds showing the closest aircraft's callsign, distance and bearing. When an aircraft is closer than 5 km a short beep is generated on the audio output.

## Building in a Codex/Codespace Environment

To compile this sketch in a container or automated environment using the Arduino CLI:

```bash
arduino-cli core install esp32:esp32
arduino-cli lib install "Adafruit GFX Library" "Adafruit SSD1306" "ArduinoJson"
arduino-cli compile --fqbn esp32:esp32:esp32 closestPlane.ino
```

Replace the FQBN (`esp32:esp32:esp32`) with the identifier for your specific ESP32 board.
