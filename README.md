# ClosestPlane ESP32

An Arduino-compatible sketch for ESP32 boards that connects to a dump1090 server, identifies the closest aircraft to your location and shows details on a 128x64 SSD1306 OLED. If the aircraft comes within 5 km an audible alert is played through a MAX98357 I2S amplifier.

## Setup
1. Rename `config.h` with your WiFi credentials, dump1090 server address and your latitude/longitude. Adjust I2S pin numbers if required.
2. Install the following libraries in the Arduino IDE:
   - **ESP32 board support** via the Boards Manager
   - **Adafruit SSD1306** and **Adafruit GFX**
   - **ArduinoJson**
3. Open `closestPlane.ino` in the Arduino IDE, select your ESP32 board and the correct port.
4. Compile and upload.

## Operation
The display refreshes every few seconds showing the closest aircraft's callsign, distance and bearing. When an aircraft is closer than 5 km a short beep is generated on the audio output.
