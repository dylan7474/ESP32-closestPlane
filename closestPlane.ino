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
  double distanceKm;
  double bearing;
};

// --- MULTI-CORE & SHARED DATA ---
// This data is shared between the two cores
Aircraft closest;
int16_t blipX = -1, blipY = -1;
// The Mutex (like a lock) to protect the shared data
SemaphoreHandle_t dataMutex;

// --- RADAR ANIMATION VARIABLES ---
float sweepAngle = 0.0;
const int16_t radarCenterX = 96;
const int16_t radarCenterY = 36;
const int16_t radarRadius = 26;

// Function Prototypes
void fetchAircraft();
void drawRadarScreen();
void fetchDataTask(void *pvParameters); // The function for our second core
double haversine(double lat1, double lon1, double lat2, double lon2);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
void playBeep(int freq, int duration_ms);

// --- TASK FOR CORE 0: DATA FETCHING ---
// This function runs independently on the second core.
void fetchDataTask(void *pvParameters) {
  Serial.println("Fetch data task started on core 0");
  for (;;) { // Infinite loop for this task
    fetchAircraft(); // Call the data fetching function
    // Wait for the refresh interval before running again
    vTaskDelay(REFRESH_INTERVAL_MS / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Booting ClosestPlane Radar (Dual Core)");

  // Create the mutex to protect our shared data
  dataMutex = xSemaphoreCreateMutex();

  display.begin(0x3C, true);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("ClosestPlane Radar");
  display.setCursor(0, 16);
  display.println("Connecting WiFi...");
  display.display();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println("\nWiFi connected.");

  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), .sample_rate = 44100, .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, .communication_format = I2S_COMM_FORMAT_I2S_MSB, .intr_alloc_flags = 0, .dma_buf_count = 4, .dma_buf_len = 64, .use_apll = false, .tx_desc_auto_clear = true, .fixed_mclk = 0};
  i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_BCLK_PIN, .ws_io_num = I2S_LRCLK_PIN, .data_out_num = I2S_DOUT_PIN, .data_in_num = I2S_PIN_NO_CHANGE};
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  // --- START THE SECOND CORE ---
  // Create the data fetching task and pin it to Core 0
  xTaskCreatePinnedToCore(
      fetchDataTask, // Function to run
      "FetchData",   // Name of the task
      4096,          // Stack size
      NULL,          // Task input parameter
      1,             // Task priority
      NULL,          // Task handle
      0);            // Core to run on (0)
}

// --- MAIN LOOP ON CORE 1: ANIMATION ONLY ---
// This loop now only handles drawing the screen, so it runs fast and smooth.
void loop() {
  drawRadarScreen();

  sweepAngle += 4.0;
  if (sweepAngle >= 360.0) {
    sweepAngle = 0.0;
  }

  delay(10);
}

// --- SCREEN DRAWING FUNCTION ---
void drawRadarScreen() {
  // Create local variables to hold copies of the shared data
  String flight;
  double distanceKm;
  int16_t currentBlipX, currentBlipY;

  // Lock the mutex, quickly copy the shared data, then unlock
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  flight = closest.flight;
  distanceKm = closest.distanceKm;
  currentBlipX = blipX;
  currentBlipY = blipY;
  xSemaphoreGive(dataMutex);

  // Now perform all the drawing using the local copies
  display.clearDisplay();

  if (currentBlipX != -1) {
    display.setCursor(0, 0);
    display.print("Flt: ");
    display.println(flight.substring(0, 7));
    display.print("Dst: ");
    display.print(distanceKm, 1);
    display.println("km");
  } else {
    display.setCursor(0, 0);
    display.println("Scanning...");
  }

  display.drawCircle(radarCenterX, radarCenterY, radarRadius, SH110X_WHITE);
  display.setCursor(radarCenterX - 3, radarCenterY - radarRadius - 9);
  display.print("N");

  if (currentBlipX != -1) {
    display.fillCircle(currentBlipX, currentBlipY, 2, SH110X_WHITE);
  }

  double sweepRad = sweepAngle * PI / 180.0;
  int16_t sweepX = radarCenterX + (radarRadius - 1) * sin(sweepRad);
  int16_t sweepY = radarCenterY - (radarRadius - 1) * cos(sweepRad);
  display.drawLine(radarCenterX, radarCenterY, sweepX, sweepY, SH110X_WHITE);

  display.display();
}

// --- DATA FETCHING FUNCTION (Called by Core 0) ---
void fetchAircraft() {
  HTTPClient http;
  char url[160];
  snprintf(url, sizeof(url), "http://%s:%d/dump1090-fa/data/aircraft.json", DUMP1090_SERVER, DUMP1090_PORT);

  if (!http.begin(url)) return;

  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    DynamicJsonDocument doc(20 * 1024);
    if (deserializeJson(doc, http.getStream()) == DeserializationError::Ok) {
      JsonArray arr = doc["aircraft"].as<JsonArray>();
      double minDist = 1e9;
      bool planeFound = false;

      // Local temporary variables to hold the new data
      Aircraft newClosest;
      int16_t newBlipX = -1, newBlipY = -1;

      for (JsonObject plane : arr) {
        if (plane.containsKey("lat") && plane.containsKey("lon")) {
          double dist = haversine(USER_LAT, USER_LON, plane["lat"], plane["lon"]);
          if (dist < minDist) {
            minDist = dist;
            newClosest.flight = plane["flight"].as<String>();
            newClosest.distanceKm = dist;
            newClosest.bearing = calculateBearing(USER_LAT, USER_LON, plane["lat"], plane["lon"]);
            planeFound = true;
          }
        }
      }

      if (planeFound) {
        Serial.printf("Closest: %s %.2f km bearing %.1f deg\n", newClosest.flight.c_str(), newClosest.distanceKm, newClosest.bearing);
        double angleRad = newClosest.bearing * PI / 180.0;
        newBlipX = radarCenterX + (radarRadius - 3) * sin(angleRad);
        newBlipY = radarCenterY - (radarRadius - 3) * cos(angleRad);

        if (newClosest.distanceKm < PROXIMITY_ALERT_KM) {
          playBeep(1000, 200);
        }
      }

      // Lock the mutex, update the shared global data, then unlock
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      closest = newClosest;
      blipX = newBlipX;
      blipY = newBlipY;
      xSemaphoreGive(dataMutex);
    }
  } else {
    Serial.printf("HTTP GET failed, error: %d\n", httpCode);
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    blipX = -1; // Hide blip on error
    xSemaphoreGive(dataMutex);
  }
  http.end();
}


double deg2rad(double deg) { return deg * PI / 180.0; }

double haversine(double lat1, double lon1, double lat2, double lon2) {
  double dLat = deg2rad(lat2 - lat1); double dLon = deg2rad(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return EARTH_RADIUS_KM * c;
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  double lonDiff = deg2rad(lon2 - lon1); lat1 = deg2rad(lat1); lat2 = deg2rad(lat2);
  double y = sin(lonDiff) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lonDiff);
  double bearing = atan2(y, x);
  bearing = fmod((bearing * 180.0 / PI + 360.0), 360.0);
  return bearing;
}

void playBeep(int freq, int duration_ms) {
  const int sampleRate = 44100; int samples = sampleRate * duration_ms / 1000; size_t bytes_written;
  for (int i = 0; i < samples; i++) {
    int16_t sample = (int16_t)(sin(2 * PI * freq * i / sampleRate) * 32767);
    i2s_write(I2S_NUM_0, &sample, sizeof(sample), &bytes_written, portMAX_DELAY);
  }
}
