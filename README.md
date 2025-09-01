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

#!/bin/bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

echo "==> Base packages"
apt-get update
apt-get install -y --no-install-recommends \
  curl ca-certificates git python3 tar xz-utils

# --- Proxy handling ---
# Mirror the proxy you see in Codex logs (http://proxy:8080) for all tools.
HTTP_PROXY="${HTTP_PROXY:-${http_proxy:-}}"
HTTPS_PROXY="${HTTPS_PROXY:-${https_proxy:-$HTTP_PROXY}}"
NO_PROXY="${NO_PROXY:-${no_proxy:-localhost,127.0.0.1,::1}}"
export HTTP_PROXY HTTPS_PROXY NO_PROXY
# Some tools only read lowercase:
export http_proxy="${HTTP_PROXY:-}"
export https_proxy="${HTTPS_PROXY:-}"
export no_proxy="${NO_PROXY:-}"

echo "HTTP_PROXY=${HTTP_PROXY:-<unset>}"
echo "HTTPS_PROXY=${HTTPS_PROXY:-<unset>}"
echo "NO_PROXY=${NO_PROXY}"

# --- Prefer IPv4 to avoid IPv6 'network is unreachable' ---
if ! grep -q '^precedence ::ffff:0:0/96 100' /etc/gai.conf 2>/dev/null; then
  echo 'precedence ::ffff:0:0/96 100' >> /etc/gai.conf
fi

# --- Helper: retry with backoff ---
retry () {
  local attempts="$1"; shift
  local sleep_s="$1"; shift
  local n=1
  until "$@"; do
    if [[ $n -ge $attempts ]]; then
      echo "Command failed after $n attempts: $*" >&2
      return 1
    fi
    echo "Retry $n/$attempts failed. Sleeping ${sleep_s}s…"
    n=$((n+1))
    sleep "$sleep_s"
  done
}

echo "==> Installing arduino-cli"
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
install -m 0755 bin/arduino-cli /usr/local/bin/arduino-cli
rm -rf bin
arduino-cli version

echo "==> arduino-cli config"
arduino-cli config init --overwrite

# Boards index + proxy (this is the only network key we need)
arduino-cli config set board_manager.additional_urls \
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

if [[ -n "${HTTPS_PROXY}" || -n "${HTTP_PROXY}" ]]; then
  arduino-cli config set network.proxy "${HTTPS_PROXY:-$HTTP_PROXY}"
fi

# Optional: slightly longer socket timeout (older CLI versions may ignore this; safe if it errors)
arduino-cli config set network.socket_timeout 60s || true

echo "==> Updating indexes (via proxy, with retries)"
retry 5 5 arduino-cli core update-index

echo "==> Installing ESP32 core"
retry 5 5 arduino-cli core install esp32:esp32

echo "==> Installing libraries"
retry 5 5 arduino-cli lib install "Adafruit GFX Library"
retry 5 5 arduino-cli lib install "Adafruit SSD1306"
retry 5 5 arduino-cli lib install "ArduinoJson"

echo "✅ Setup complete. Compile with:"
echo "   arduino-cli compile --fqbn esp32:esp32:esp32 closestPlane.ino"
