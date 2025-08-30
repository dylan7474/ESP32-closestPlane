#ifndef CONFIG_H
#define CONFIG_H

// WiFi credentials
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// ADS-B dump1090 server configuration
#define DUMP1090_SERVER "127.0.0.1" // IP or hostname of dump1090 server
#define DUMP1090_PORT 8080          // Port of dump1090 JSON endpoint
#define USER_LAT 51.5074
#define USER_LON -0.1278

// I2S pins for MAX98357A audio output
#define I2S_BCLK_PIN 26
#define I2S_LRCLK_PIN 25
#define I2S_DOUT_PIN 22

#endif // CONFIG_H
