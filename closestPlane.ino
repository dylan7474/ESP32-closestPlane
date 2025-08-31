#include <SimpleRotary.h>
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
#include <EEPROM.h> // --- EEPROM: Include EEPROM library ---

#include "config.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define REFRESH_INTERVAL_MS 5000
#define EARTH_RADIUS_KM 6371.0
#define BLIP_LIFESPAN_FRAMES 90
#define MAX_BLIPS 20

// --- EEPROM: Addresses and magic number for saving settings ---
#define EEPROM_SIZE 64
#define EEPROM_ADDR_MAGIC 0
#define EEPROM_ADDR_VOLUME 4
#define EEPROM_ADDR_RANGE_INDEX 8
#define EEPROM_MAGIC_NUMBER 0xAC // A unique number to verify our saved data

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

SimpleRotary VolumeSelector(33, 4, 23);
SimpleRotary ChannelSelector(25, 32, 2);

volatile byte VolumeChange = 0, VolumePush = 0;
volatile byte ChannelChange = 0, ChannelPush = 0;

// Volume and Range Control Variables
int beepVolume; // Will be loaded from EEPROM
bool displayingVolume = false;
unsigned long volumeDisplayTimeout = 0;

float rangeSteps[] = {5, 10, 25, 50, 100, 150, 200, 300};
const int rangeStepsCount = sizeof(rangeSteps) / sizeof(rangeSteps[0]);
int rangeStepIndex; // Will be loaded from EEPROM
float radarRangeKm;
bool displayingRange = false;
unsigned long rangeDisplayTimeout = 0;

struct Aircraft {
  String flight;
  double distanceKm;
  double bearing;
  bool isValid;
};

struct RadarBlip {
  int16_t x;
  int16_t y;
  int lifespan;
};

// --- MULTI-CORE & SHARED DATA ---
Aircraft closestAircraft;
std::vector<Aircraft> trackedAircraft;
std::vector<RadarBlip> activeBlips;
SemaphoreHandle_t dataMutex;

// --- RADAR ANIMATION VARIABLES ---
float sweepAngle = 0.0;
float lastSweepAngle = 0.0;
std::vector<bool> paintedThisTurn;
const int16_t radarCenterX = 96;
const int16_t radarCenterY = 36;
const int16_t radarRadius = 26;

// Function Prototypes
void saveSettings();
void loadSettings();
void fetchAircraft();
void drawRadarScreen();
void fetchDataTask(void *pvParameters);
void encoderTask(void *pvParameters);
void Poweroff(String powermessage);
double haversine(double lat1, double lon1, double lat2, double lon2);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
void playBeep(int freq, int duration_ms);
double deg2rad(double deg);

// --- Task for reading encoders reliably ---
void encoderTask(void *pvParameters) {
  Serial.println("Encoder polling task started on core 1");
  for (;;) {
    byte vol_rotate_event = VolumeSelector.rotate();
    if (vol_rotate_event != 0) { VolumeChange = vol_rotate_event; }
    byte chan_rotate_event = ChannelSelector.rotate();
    if (chan_rotate_event != 0) { ChannelChange = chan_rotate_event; }
    byte vol_push_event = VolumeSelector.pushType(200);
    if (vol_push_event != 0) { VolumePush = vol_push_event; }
    byte chan_push_event = ChannelSelector.pushType(200);
    if (chan_push_event != 0) { ChannelPush = chan_push_event; }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// --- TASK FOR CORE 0: DATA FETCHING ---
void fetchDataTask(void *pvParameters) {
  Serial.println("Fetch data task started on core 0");
  for (;;) {
    fetchAircraft();
    vTaskDelay(REFRESH_INTERVAL_MS / portTICK_PERIOD_MS);
  }
}

void setup() {
  pinMode(19, OUTPUT);
  digitalWrite(19, HIGH);

  Serial.begin(115200);
  Serial.println("Booting Multi-Target Radar");

  // --- EEPROM: Initialize and load settings at the start ---
  loadSettings();

  dataMutex = xSemaphoreCreateMutex();
  closestAircraft.isValid = false;
  
  display.begin(0x3C, true);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("Multi-Target Radar");
  display.setCursor(0, 16);
  display.println("Connecting WiFi...");
  display.display();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println("\nWiFi connected.");
  WiFi.setSleep(false);

  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), .sample_rate = 44100, .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, .communication_format = I2S_COMM_FORMAT_STAND_I2S, .intr_alloc_flags = 0, .dma_buf_count = 8, .dma_buf_len = 64, .use_apll = false, .tx_desc_auto_clear = true, .fixed_mclk = 0};
  i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_BCLK_PIN, .ws_io_num = I2S_LRCLK_PIN, .data_out_num = I2S_DOUT_PIN, .data_in_num = I2S_PIN_NO_CHANGE};
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  xTaskCreatePinnedToCore(
      fetchDataTask, "FetchData", 8192, NULL, 2, NULL, 0);
      
  xTaskCreatePinnedToCore(
      encoderTask, "Encoder", 2048, NULL, 3, NULL, 1);
}

// --- MAIN LOOP ON CORE 1 ---
void loop() {
  byte localVolumeChange = VolumeChange;
  byte localVolumePush = VolumePush;
  byte localChannelChange = ChannelChange;
  byte localChannelPush = ChannelPush;
  
  if (localVolumeChange != 0) VolumeChange = 0;
  if (localVolumePush != 0) VolumePush = 0;
  if (localChannelChange != 0) ChannelChange = 0;
  if (localChannelPush != 0) ChannelPush = 0;
  
  if (localVolumeChange != 0) {
    beepVolume += (localVolumeChange == 1) ? 1 : -1;
    beepVolume = constrain(beepVolume, 0, 20);
    displayingVolume = true;
    volumeDisplayTimeout = millis() + 2000;
  }
  
  if (localVolumePush == 2) { Poweroff("Goodbye"); }

  if (localChannelChange != 0) {
    rangeStepIndex += (localChannelChange == 1) ? 1 : -1;
    rangeStepIndex = constrain(rangeStepIndex, 0, rangeStepsCount - 1);
    radarRangeKm = rangeSteps[rangeStepIndex];
    displayingRange = true;
    rangeDisplayTimeout = millis() + 2000;
  }

  if (displayingVolume && millis() > volumeDisplayTimeout) { displayingVolume = false; }
  if (displayingRange && millis() > rangeDisplayTimeout) { displayingRange = false; }

  xSemaphoreTake(dataMutex, portMAX_DELAY);

  for (int i = 0; i < trackedAircraft.size(); i++) {
    if (i < paintedThisTurn.size() && !paintedThisTurn[i]) {
      double targetBearing = trackedAircraft[i].bearing;
      bool bearingCrossed = (lastSweepAngle < targetBearing && sweepAngle >= targetBearing);
      if (lastSweepAngle > sweepAngle && (targetBearing > lastSweepAngle || targetBearing <= sweepAngle)) {
        bearingCrossed = true;
      }

      if (bearingCrossed) {
        double angleRad = targetBearing * PI / 180.0;
        double realDistance = trackedAircraft[i].distanceKm;
        float screenRadius = map(realDistance, 0, radarRangeKm, 0, radarRadius);

        int16_t newBlipX = radarCenterX + screenRadius * sin(angleRad);
        int16_t newBlipY = radarCenterY - screenRadius * cos(angleRad);
        
        activeBlips.push_back({newBlipX, newBlipY, BLIP_LIFESPAN_FRAMES});
        if (activeBlips.size() > MAX_BLIPS) {
          activeBlips.erase(activeBlips.begin());
        }
        paintedThisTurn[i] = true;
        playBeep(1000, 25);
      }
    }
  }

  for (auto it = activeBlips.begin(); it != activeBlips.end(); ) {
    it->lifespan--;
    if (it->lifespan <= 0) { it = activeBlips.erase(it); } 
    else { ++it; }
  }
  xSemaphoreGive(dataMutex);

  drawRadarScreen();
  
  lastSweepAngle = sweepAngle;
  sweepAngle += 4.0;

  if (sweepAngle >= 360.0) {
    sweepAngle = 0.0;
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    paintedThisTurn.assign(trackedAircraft.size(), false);
    xSemaphoreGive(dataMutex);
  }
  
  delay(10);
}

// --- EEPROM: Poweroff function now saves settings ---
void Poweroff(String powermessage) {
  Serial.println("Powering off. Saving settings...");
  saveSettings(); // Save current settings to EEPROM
  
  i2s_driver_uninstall(I2S_NUM_0);
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(powermessage);
  display.display();
  delay(1000);
  digitalWrite(19, LOW);
}

// --- EEPROM: Functions to save and load settings ---
void saveSettings() {
  EEPROM.put(EEPROM_ADDR_VOLUME, beepVolume);
  EEPROM.put(EEPROM_ADDR_RANGE_INDEX, rangeStepIndex);
  EEPROM.write(EEPROM_ADDR_MAGIC, EEPROM_MAGIC_NUMBER);
  EEPROM.commit();
  Serial.println("Settings saved to EEPROM.");
}

void loadSettings() {
  EEPROM.begin(EEPROM_SIZE);
  
  if (EEPROM.read(EEPROM_ADDR_MAGIC) == EEPROM_MAGIC_NUMBER) {
    Serial.println("Loading settings from EEPROM...");
    EEPROM.get(EEPROM_ADDR_VOLUME, beepVolume);
    EEPROM.get(EEPROM_ADDR_RANGE_INDEX, rangeStepIndex);

    beepVolume = constrain(beepVolume, 0, 20);
    rangeStepIndex = constrain(rangeStepIndex, 0, rangeStepsCount - 1);

  } else {
    Serial.println("First run or invalid EEPROM data. Setting defaults.");
    beepVolume = 10;
    rangeStepIndex = 3; // Corresponds to 50 km
    saveSettings(); 
  }

  radarRangeKm = rangeSteps[rangeStepIndex];
  Serial.printf("Loaded Volume: %d\n", beepVolume);
  Serial.printf("Loaded Range: %.0f km\n", radarRangeKm);
}

// --- SCREEN DRAWING FUNCTION ---
void drawRadarScreen() {
  display.clearDisplay();
  display.setTextSize(1);

  if (displayingRange) {
    display.setCursor(20, 16);
    display.print("Radar Range");
    display.setTextSize(2);
    display.setCursor(20, 32);
    display.print(radarRangeKm, 0);
    display.print(" km");
    display.display();
    return;
  }
  if (displayingVolume) {
    display.setCursor(20, 16);
    display.print("Beep Volume");
    display.drawRect(14, 32, 100, 16, SH110X_WHITE);
    int barWidth = map(beepVolume, 0, 20, 0, 98);
    display.fillRect(15, 33, barWidth, 14, SH110X_WHITE);
    display.display();
    return;
  }

  Aircraft currentClosest;
  int targetCount = 0;
  std::vector<RadarBlip> currentBlips;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  currentClosest = closestAircraft;
  targetCount = trackedAircraft.size();
  currentBlips = activeBlips;
  xSemaphoreGive(dataMutex);

  if (currentClosest.isValid) {
    display.setCursor(0, 0);
    display.print("Flt: ");
    display.println(currentClosest.flight.isEmpty() ? "------" : currentClosest.flight.substring(0, 7));
    display.print("Dst: ");
    display.print(currentClosest.distanceKm, 1);
    display.println("km");
    display.print("Trgts: ");
    display.println(targetCount);
  } else {
    display.setCursor(0, 0);
    display.println("Scanning...");
    display.setCursor(0, 8);
    display.print("Rng: ");
    display.print(radarRangeKm, 0);
    display.print("km");
  }

  display.drawCircle(radarCenterX, radarCenterY, radarRadius, SH110X_WHITE);
  display.setCursor(radarCenterX - 3, radarCenterY - radarRadius - 9);
  display.print("N");

  for (const auto& blip : currentBlips) {
    if (blip.lifespan > (BLIP_LIFESPAN_FRAMES * 2 / 3)) {
      display.fillCircle(blip.x, blip.y, 3, SH110X_WHITE);
    } else if (blip.lifespan > (BLIP_LIFESPAN_FRAMES * 1 / 3)) {
      display.fillCircle(blip.x, blip.y, 2, SH110X_WHITE);
    } else {
      display.fillCircle(blip.x, blip.y, 1, SH110X_WHITE);
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
    DynamicJsonDocument doc(30 * 1024);
    if (deserializeJson(doc, http.getStream()) == DeserializationError::Ok) {
      JsonArray arr = doc["aircraft"].as<JsonArray>();
      
      std::vector<Aircraft> planesInRange;
      Aircraft newClosest;
      newClosest.isValid = false;
      double minDist = radarRangeKm;
      
      for (JsonObject plane : arr) {
        if (plane.containsKey("lat") && plane.containsKey("lon")) {
          double dist = haversine(USER_LAT, USER_LON, plane["lat"], plane["lon"]);
          if (dist < radarRangeKm) {
            Aircraft ac;
            ac.flight = plane["flight"].as<String>();
            ac.distanceKm = dist;
            ac.bearing = calculateBearing(USER_LAT, USER_LON, plane["lat"], plane["lon"]);
            ac.isValid = true;
            planesInRange.push_back(ac);

            if (dist < minDist) {
              minDist = dist;
              newClosest = ac;
            }
          }
        }
      }

      xSemaphoreTake(dataMutex, portMAX_DELAY);
      trackedAircraft = planesInRange;
      closestAircraft = newClosest;
      xSemaphoreGive(dataMutex);
    }
  } else {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    trackedAircraft.clear();
    closestAircraft.isValid = false;
    xSemaphoreGive(dataMutex);
  }
  http.end();
}

void playBeep(int freq, int duration_ms) {
  long amplitude = map(beepVolume, 0, 20, 0, 25000);
  const int sampleRate = 44100; int samples = sampleRate * duration_ms / 1000; size_t bytes_written;
  for (int i = 0; i < samples; i++) {
    int16_t sample = (int16_t)(sin(2 * PI * freq * i / sampleRate) * amplitude);
    i2s_write(I2S_NUM_0, &sample, sizeof(sample), &bytes_written, portMAX_DELAY);
  }
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
