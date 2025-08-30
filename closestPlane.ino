#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <esp_err.h>
#include <math.h>
#include <vector>

#include "config.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define REFRESH_INTERVAL_MS 5000
#define PROXIMITY_ALERT_KM 5.0
#define EARTH_RADIUS_KM 6371.0
#define BLIP_LIFESPAN_FRAMES 90 // Lifespan is ~1 full sweep (360 degrees / 4 degrees per frame)
#define MAX_BLIPS 10           

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

struct Aircraft {
  String flight;
  double distanceKm;
  double bearing;
  bool needsPainting; // Flag to signal that the animation loop should create a new blip
};

struct RadarBlip {
  int16_t x;
  int16_t y;
  int lifespan;
};

// --- MULTI-CORE & SHARED DATA ---
Aircraft closest;
std::vector<RadarBlip> activeBlips;
SemaphoreHandle_t dataMutex;

// --- RADAR ANIMATION VARIABLES ---
float sweepAngle = 0.0;
float lastSweepAngle = 0.0;
bool blipPaintedThisTurn = false; // Flag to ensure blip is painted only once per sweep
const int16_t radarCenterX = 96;
const int16_t radarCenterY = 36;
const int16_t radarRadius = 26;

// Function Prototypes
void fetchAircraft();
void drawRadarScreen();
void fetchDataTask(void *pvParameters);
double haversine(double lat1, double lon1, double lat2, double lon2);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
void playBeep(int freq, int duration_ms);
double deg2rad(double deg);

// --- TASK FOR CORE 0: DATA FETCHING ---
void fetchDataTask(void *pvParameters) {
  Serial.println("Fetch data task started on core 0");
  for (;;) {
    fetchAircraft();
    vTaskDelay(REFRESH_INTERVAL_MS / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Booting ClosestPlane Radar (Final)");

  dataMutex = xSemaphoreCreateMutex();
  closest.needsPainting = false;

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

  xTaskCreatePinnedToCore(fetchDataTask, "FetchData", 4096, NULL, 1, NULL, 0);
}

// --- MAIN LOOP ON CORE 1: ANIMATION, FADING, AND BLIP CREATION ---
void loop() {
  xSemaphoreTake(dataMutex, portMAX_DELAY);

  // Check if there is a valid target to paint
  bool hasTarget = !closest.flight.isEmpty();

  // Determine if the sweep has just crossed the target's bearing
  bool bearingCrossed = (lastSweepAngle < closest.bearing && sweepAngle >= closest.bearing);
  if (lastSweepAngle > sweepAngle) { // Handle 360->0 wrap-around
    if (closest.bearing > lastSweepAngle || closest.bearing <= sweepAngle) {
      bearingCrossed = true;
    }
  }

  // If there's a target and we haven't painted it this turn, and the sweep crosses it...
  if (hasTarget && !blipPaintedThisTurn && bearingCrossed) {
    double angleRad = closest.bearing * PI / 180.0;
    int16_t newBlipX = radarCenterX + (radarRadius - 3) * sin(angleRad);
    int16_t newBlipY = radarCenterY - (radarRadius - 3) * cos(angleRad);
    
    activeBlips.push_back({newBlipX, newBlipY, BLIP_LIFESPAN_FRAMES});

    if (activeBlips.size() > MAX_BLIPS) {
      activeBlips.erase(activeBlips.begin());
    }
    
    blipPaintedThisTurn = true; // Mark as painted for this sweep
  }

  // Age and remove all existing blips
  for (auto it = activeBlips.begin(); it != activeBlips.end(); ) {
    it->lifespan--;
    if (it->lifespan <= 0) {
      it = activeBlips.erase(it);
    } else {
      ++it;
    }
  }
  xSemaphoreGive(dataMutex);

  // Animation continues
  drawRadarScreen();
  
  lastSweepAngle = sweepAngle;
  sweepAngle += 4.0;
  if (sweepAngle >= 360.0) {
    sweepAngle = 0.0;
    blipPaintedThisTurn = false; // Reset the flag for the next sweep
  }
  
  delay(10);
}

// --- SCREEN DRAWING FUNCTION (with enhanced fading) ---
void drawRadarScreen() {
  String currentFlight;
  double currentDistanceKm;
  std::vector<RadarBlip> currentBlips;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  currentFlight = closest.flight;
  currentDistanceKm = closest.distanceKm;
  currentBlips = activeBlips;
  xSemaphoreGive(dataMutex);

  display.clearDisplay();

  if (!currentFlight.isEmpty()) {
    display.setCursor(0, 0);
    display.print("Flt: ");
    display.println(currentFlight.substring(0, 7));
    display.print("Dst: ");
    display.print(currentDistanceKm, 1);
    display.println("km");
  } else {
    display.setCursor(0, 0);
    display.println("Scanning...");
  }

  display.drawCircle(radarCenterX, radarCenterY, radarRadius, SH110X_WHITE);
  display.setCursor(radarCenterX - 3, radarCenterY - radarRadius - 9);
  display.print("N");

  // --- Draw blips with a 4-stage fade ---
  for (const auto& blip : currentBlips) {
    if (blip.lifespan > BLIP_LIFESPAN_FRAMES * 0.70) {
      // Stage 1: Fresh, solid circle
      display.fillCircle(blip.x, blip.y, 2, SH110X_WHITE);
    } else if (blip.lifespan > BLIP_LIFESPAN_FRAMES * 0.40) {
      // Stage 2: Fading, hollow circle (radius 2)
      display.drawCircle(blip.x, blip.y, 2, SH110X_WHITE);
    } else if (blip.lifespan > BLIP_LIFESPAN_FRAMES * 0.10) {
      // Stage 3: Fading further, hollow circle (radius 1)
      display.drawCircle(blip.x, blip.y, 1, SH110X_WHITE);
    } else {
      // Stage 4: Almost gone, single pixel
      display.drawPixel(blip.x, blip.y, SH110X_WHITE);
    }
  }

  double sweepRad = sweepAngle * PI / 180.0;
  int16_t sweepX = radarCenterX + (radarRadius - 1) * sin(sweepRad);
  int16_t sweepY = radarCenterY - (radarRadius - 1) * cos(sweepRad);
  display.drawLine(radarCenterX, radarCenterY, sweepX, sweepY, SH110X_WHITE);

  display.display();
}

// --- DATA FETCHING FUNCTION ---
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
      Aircraft newClosest;
      
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

      xSemaphoreTake(dataMutex, portMAX_DELAY);
      if (planeFound) {
        Serial.printf("Updating target: %s %.1f km, bearing %.0f.\n", newClosest.flight.c_str(), newClosest.distanceKm, newClosest.bearing);
        // Only update if the bearing has changed significantly, to prevent jitter
        if (abs(newClosest.bearing - closest.bearing) > 2 || closest.flight.isEmpty()) {
            closest = newClosest;
            closest.needsPainting = true; // Set the flag for the animation loop
        }
        if (closest.distanceKm < PROXIMITY_ALERT_KM) {
          playBeep(1000, 200);
        }
      } else {
        closest.flight = ""; // Set flight to empty to indicate no target
      }
      xSemaphoreGive(dataMutex);
    }
  } else {
    Serial.printf("HTTP GET failed, error: %d\n", httpCode);
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    closest.flight = "";
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
