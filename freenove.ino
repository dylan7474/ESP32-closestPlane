#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <math.h>
#include <cstring>

#include "config.h"

// --- Display & Timing Constants ---
static const uint16_t COLOR_BACKGROUND = TFT_BLACK;
static const uint16_t COLOR_HEADER = TFT_DARKGREEN;
static const uint16_t COLOR_TEXT = TFT_WHITE;
static const int INFO_START_Y = 60;
static const int INFO_LINE_HEIGHT = 28;
static const unsigned long REFRESH_INTERVAL_MS = 5000;
static const unsigned long WIFI_RETRY_INTERVAL_MS = 15000;
static const unsigned long WIFI_CONNECT_TIMEOUT_MS = 10000;

TFT_eSPI tft = TFT_eSPI();

struct AircraftInfo {
  String flight;
  double distanceKm;
  int altitude;
  double bearing;
  bool valid;
};

AircraftInfo closestAircraft;
int aircraftCount = 0;
bool dataConnectionOk = false;
unsigned long lastFetchTime = 0;
unsigned long lastWifiAttempt = 0;

// --- Function Prototypes ---
void drawStaticLayout();
void updateDisplay();
void drawInfoLine(int index, const String &text);
void drawStatusLine(int index, const String &text);
void connectWiFi();
void fetchAircraft();
double haversine(double lat1, double lon1, double lat2, double lon2);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
double deg2rad(double deg);

void setup() {
  Serial.begin(115200);
  tft.begin();
  tft.setRotation(1);
  drawStaticLayout();

  closestAircraft.valid = false;
  updateDisplay();

  connectWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    fetchAircraft();
    lastFetchTime = millis();
  }
}

void loop() {
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    if (now - lastWifiAttempt > WIFI_RETRY_INTERVAL_MS) {
      connectWiFi();
    }
  }

  if (now - lastFetchTime >= REFRESH_INTERVAL_MS) {
    lastFetchTime = now;
    fetchAircraft();
  }

  delay(50);
}

void drawStaticLayout() {
  tft.fillScreen(COLOR_BACKGROUND);
  tft.setTextColor(COLOR_TEXT, COLOR_BACKGROUND);
  tft.setTextDatum(TC_DATUM);
  tft.setTextSize(3);
  tft.drawString("Closest Plane Radar", tft.width() / 2, 18);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_HEADER, COLOR_BACKGROUND);
  tft.drawString("Freenove ESP32 CYD", tft.width() / 2, 42);
  tft.setTextColor(COLOR_TEXT, COLOR_BACKGROUND);
  tft.drawFastHLine(0, 54, tft.width(), TFT_DARKGREY);
  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(2);
}

void drawInfoLine(int index, const String &text) {
  int y = INFO_START_Y + index * INFO_LINE_HEIGHT;
  String content = text.length() ? text : " ";
  tft.setTextPadding(tft.width() - 20);
  tft.drawString(content, 10, y);
}

void drawStatusLine(int index, const String &text) {
  int baseY = tft.height() - 3 * INFO_LINE_HEIGHT;
  int y = baseY + index * INFO_LINE_HEIGHT;
  String content = text.length() ? text : " ";
  tft.setTextPadding(tft.width() - 20);
  tft.drawString(content, 10, y);
}

void updateDisplay() {
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COLOR_TEXT, COLOR_BACKGROUND);
  if (closestAircraft.valid) {
    String flight = closestAircraft.flight.length() ? closestAircraft.flight : String("(unknown)");
    flight.trim();
    drawInfoLine(0, "Flight: " + flight);

    char distanceBuffer[32];
    snprintf(distanceBuffer, sizeof(distanceBuffer), "Dist: %.1f km", closestAircraft.distanceKm);
    drawInfoLine(1, distanceBuffer);

    if (closestAircraft.altitude >= 0) {
      drawInfoLine(2, "Alt: " + String(closestAircraft.altitude) + " ft");
    } else {
      drawInfoLine(2, "Alt: ---");
    }

    char bearingBuffer[32];
    snprintf(bearingBuffer, sizeof(bearingBuffer), "Bearing: %.0f°", closestAircraft.bearing);
    drawInfoLine(3, bearingBuffer);
  } else {
    drawInfoLine(0, "No aircraft in range");
    drawInfoLine(1, " ");
    drawInfoLine(2, " ");
    drawInfoLine(3, " ");
  }

  drawStatusLine(0, "Seen aircraft: " + String(aircraftCount));

  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    drawStatusLine(1, "WiFi: connected " + ip.toString());
  } else {
    drawStatusLine(1, "WiFi: offline");
  }

  drawStatusLine(2, dataConnectionOk ? "Data link: OK" : "Data link: waiting...");

  tft.setTextPadding(0);
}

void connectWiFi() {
  lastWifiAttempt = millis();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("Connecting to WiFi %s...\n", WIFI_SSID);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected.");
    WiFi.setSleep(false);
  } else {
    Serial.println("WiFi connection failed.");
  }

  updateDisplay();
}

void fetchAircraft() {
  if (WiFi.status() != WL_CONNECTED) {
    dataConnectionOk = false;
    closestAircraft.valid = false;
    aircraftCount = 0;
    updateDisplay();
    return;
  }

  HTTPClient http;
  char url[160];
  snprintf(url, sizeof(url), "http://%s:%d/dump1090-fa/data/aircraft.json", DUMP1090_SERVER, DUMP1090_PORT);

  if (!http.begin(url)) {
    dataConnectionOk = false;
    closestAircraft.valid = false;
    aircraftCount = 0;
    updateDisplay();
    return;
  }

  http.setTimeout(4000);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    DynamicJsonDocument doc(16384);
    DeserializationError err = deserializeJson(doc, http.getStream());
    if (!err) {
      dataConnectionOk = true;
      JsonArray arr = doc["aircraft"].as<JsonArray>();
      double bestDistance = 1e12;
      AircraftInfo best;
      best.valid = false;
      aircraftCount = 0;

      for (JsonObject plane : arr) {
        if (!plane.containsKey("lat") || !plane.containsKey("lon")) {
          continue;
        }

        double lat = plane["lat"].as<double>();
        double lon = plane["lon"].as<double>();
        double distance = haversine(USER_LAT, USER_LON, lat, lon);
        if (isnan(distance) || isinf(distance)) {
          continue;
        }

        aircraftCount++;

        if (distance < bestDistance) {
          bestDistance = distance;
          best.valid = true;
          best.distanceKm = distance;
          best.bearing = calculateBearing(USER_LAT, USER_LON, lat, lon);

          if (plane.containsKey("flight")) {
            const char *flightStr = plane["flight"].as<const char*>();
            best.flight = flightStr ? String(flightStr) : String();
          } else {
            best.flight = "";
          }

          if (plane.containsKey("alt_baro")) {
            JsonVariant alt = plane["alt_baro"];
            if (alt.is<int>()) {
              best.altitude = alt.as<int>();
            } else if (alt.is<const char*>() && strcmp(alt.as<const char*>(), "ground") == 0) {
              best.altitude = 0;
            } else {
              best.altitude = -1;
            }
          } else {
            best.altitude = -1;
          }
        }
      }

      closestAircraft = best;
    } else {
      dataConnectionOk = false;
      closestAircraft.valid = false;
      aircraftCount = 0;
    }
  } else {
    dataConnectionOk = false;
    closestAircraft.valid = false;
    aircraftCount = 0;
  }

  http.end();
  updateDisplay();
}

double deg2rad(double deg) { return deg * PI / 180.0; }

double haversine(double lat1, double lon1, double lat2, double lon2) {
  double dLat = deg2rad(lat2 - lat1);
  double dLon = deg2rad(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  const double EARTH_RADIUS_KM = 6371.0;
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
