# ClosestPlane ESP32

An Arduino-compatible sketch for ESP32 boards that connects to a dump1090 server, identifies the closest aircraft to your location and shows details on a 128x64 SH1106-based OLED. If the aircraft comes within 5 km an audible alert is played through a MAX98357 I2S amplifier.

## Required Libraries

Install the following libraries in the Arduino IDE before compiling:

- **ESP32 board support** (`esp32` by Espressif) – provides WiFi, HTTPClient, Wire and I2S functionality.
- **Adafruit GFX Library**
- **Adafruit SH110X**
- **ArduinoJson**
- **SimpleRotary** – handles the rotary encoders for volume and range control.

## Hardware Connections

Connect the following components to your ESP32:

- **SH1106 128×64 OLED** (I²C): SDA → GPIO21, SCL → GPIO22, plus 3.3 V and GND.
- **MAX98357A I2S amplifier**: BCLK → GPIO17, LRCLK → GPIO16, DIN → GPIO27, SD → GPIO19, 3.3 V and GND.
- **Volume rotary encoder**: A → GPIO33, B → GPIO4, switch → GPIO23.
- **Range/channel rotary encoder**: A → GPIO25, B → GPIO32, switch → GPIO2.

## Setup
1. Rename `config.h` with your WiFi credentials, dump1090 server address and your latitude/longitude. Adjust I2S pin numbers if required.
2. Ensure the libraries above are installed in the Arduino IDE.
3. Open `closestPlane.ino` in the Arduino IDE, select your ESP32 board and the correct port.
4. Compile and upload.

## Operation
The display refreshes every few seconds showing the closest aircraft's callsign, distance and bearing. When an aircraft is closer than 5 km a short beep is generated on the audio output. Startup and diagnostic messages are printed to the serial monitor at 115200 baud to aid troubleshooting.

## Building in a Codex/Codespace Environment

The following setup script prepares a Codex/Codespace container with all required tools and
libraries for this project. Run it in your container before compiling:

