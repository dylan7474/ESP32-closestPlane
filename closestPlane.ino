#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <esp_err.h>
#include <math.h>

#include "config.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define REFRESH_INTERVAL_MS 5000
#define PROXIMITY_ALERT_KM 5.0
#define EARTH_RADIUS_KM 6371.0

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

struct Aircraft {
  String flight;
  double lat;
  double lon;
  double distanceKm;
  double bearing;
};

Aircraft closest;
unsigned long lastFetch = 0;

void playBeep(int freq, int duration_ms);
void fetchAircraft();
double haversine(double lat1, double lon1, double lat2, double lon2);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);

double deg2rad(double deg) { return deg * PI / 180.0; }

double haversine(double lat1, double lon1, double lat2, double lon2) {
  double dLat = deg2rad(lat2 - lat1);
  double dLon = deg2rad(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return EARTH_RADIUS_KM * c;
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  double lonDiff = deg2rad(lon2 - lon1);
  lat1 = deg2rad(lat1);
  lat2 = deg2rad(lat2);
  double y = sin(lonDiff) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lonDiff);
  double bearing = atan2(y, x);
  bearing = fmod((bearing * 180.0 / PI + 360.0), 360.0);
  return bearing;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Booting ClosestPlane");

  bool displayReady = display.begin(0x3C, true);
  if (displayReady) {
    Serial.println("OLED display initialized");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0,0);
    display.println("ClosestPlane");
    display.setCursor(0,8);
    display.println("Starting...");
    display.display();
  } else {
    Serial.println("Failed to initialize OLED display");
  }

  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  if (displayReady) {
    display.setCursor(0,16);
    display.print("WiFi...");
    display.display();
  }
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.print("\nWiFi connected. IP: ");
  Serial.println(WiFi.localIP());
  if (displayReady) {
    display.setCursor(0,24);
    display.println("Connected");
    display.display();
  }

  // I2S setup for MAX98357A
  Serial.println("Initializing I2S amplifier");
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK_PIN,
    .ws_io_num = I2S_LRCLK_PIN,
    .data_out_num = I2S_DOUT_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (err == ESP_OK) {
    Serial.println("I2S driver installed");
  } else {
    Serial.printf("I2S driver install failed: %s\n", esp_err_to_name(err));
  }
  err = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (err == ESP_OK) {
    Serial.println("I2S pins configured");
  } else {
    Serial.printf("I2S pin config failed: %s\n", esp_err_to_name(err));
  }
}

void loop() {
  if (millis() - lastFetch > REFRESH_INTERVAL_MS) {
    fetchAircraft();
    lastFetch = millis();
  }
}

void fetchAircraft() {
  HTTPClient http;
  char url[128];
  snprintf(url, sizeof(url), "http://%s/dump1090/data/aircraft.json", DUMP1090_SERVER);
  http.begin(url);
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    DynamicJsonDocument doc(20 * 1024);
    DeserializationError err = deserializeJson(doc, http.getStream());
    if (!err) {
      JsonArray arr = doc["aircraft"].as<JsonArray>();
      double minDist = 1e9;
      for (JsonObject plane : arr) {
        if (plane.containsKey("lat") && plane.containsKey("lon")) {
          double plat = plane["lat"].as<double>();
          double plon = plane["lon"].as<double>();
          double dist = haversine(USER_LAT, USER_LON, plat, plon);
          if (dist < minDist) {
            minDist = dist;
            closest.flight = plane["flight"].as<String>();
            closest.lat = plat;
            closest.lon = plon;
            closest.distanceKm = dist;
            closest.bearing = calculateBearing(USER_LAT, USER_LON, plat, plon);
          }
        }
      }

      display.clearDisplay();
      display.setCursor(0,0);
      if (minDist < 1e9) {
        display.print("Flight: "); display.println(closest.flight);
        display.print("Dist: "); display.print(closest.distanceKm, 1); display.println(" km");
        display.print("Bear: "); display.print(closest.bearing, 1); display.println(" deg");
        display.display();
        if (closest.distanceKm < PROXIMITY_ALERT_KM) {
          playBeep(1000, 200);
        }
      } else {
        display.println("No aircraft");
        display.display();
      }
    }
  }
  http.end();
}

void playBeep(int freq, int duration_ms) {
  const int sampleRate = 44100;
  int samples = sampleRate * duration_ms / 1000;
  size_t bytes_written;
  for (int i = 0; i < samples; i++) {
    int16_t sample = (int16_t)(sin(2 * PI * freq * i / sampleRate) * 32767);
    i2s_write(I2S_NUM_0, &sample, sizeof(sample), &bytes_written, portMAX_DELAY);
  }
}
