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

The following setup script prepares a Codex/Codespace container with all required tools and
libraries for this project. Run it in your container before compiling:

```bash
#!/bin/bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

echo "==> Base packages"
apt-get update
apt-get install -y --no-install-recommends \
  curl ca-certificates git python3 tar xz-utils

# -------- Proxy + IPv4 hardening --------
# If only HTTP_PROXY is set by the runner, mirror it to HTTPS so arduino-cli/curl can use it.
if [[ -n "${HTTP_PROXY:-}" && -z "${HTTPS_PROXY:-}" ]]; then
  export HTTPS_PROXY="$HTTP_PROXY"
fi
# Respect lowercase variants too
if [[ -n "${http_proxy:-}" && -z "${https_proxy:-}" ]]; then
  export https_proxy="$http_proxy"
fi

# Prefer IPv4 to avoid IPv6 unreachable errors seen in logs
# Safe no-op if already present.
if ! grep -q '^precedence ::ffff:0:0/96 100' /etc/gai.conf 2>/dev/null; then
  echo "precedence ::ffff:0:0/96 100" >> /etc/gai.conf
fi

# -------- Helpers --------
retry() {
  # retry <attempts> <sleep_seconds> <command...>
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

# -------- arduino-cli install --------
echo "==> Installing arduino-cli (official script)"
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
install -m 0755 bin/arduino-cli /usr/local/bin/arduino-cli
rm -rf bin
arduino-cli version

# -------- arduino-cli config --------
echo "==> Configuring Arduino CLI"
arduino-cli config init --overwrite
arduino-cli config set board_manager.additional_urls \
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

echo "==> Updating indexes (with retries, IPv4 preference)"
retry 5 5 arduino-cli core update-index

echo "==> Installing ESP32 core"
retry 5 5 arduino-cli core install esp32:esp32

echo "==> Installing libraries"
retry 5 5 arduino-cli lib install "Adafruit GFX Library"
retry 5 5 arduino-cli lib install "Adafruit SSD1306"
retry 5 5 arduino-cli lib install "ArduinoJson"

echo "✅ Setup complete. You can compile with:"
echo "   arduino-cli compile --fqbn esp32:esp32:esp32 closestPlane.ino"
```

After running the script, compile the sketch:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 closestPlane.ino
```

Replace the FQBN (`esp32:esp32:esp32`) with the identifier for your specific ESP32 board.
