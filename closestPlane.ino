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
#include <EEPROM.h>

#include "config.h"

// --- Display & Timing Constants ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define REFRESH_INTERVAL_MS 5000
#define WIFI_CONNECT_TIMEOUT_MS 10000
#define DISPLAY_TIMEOUT_MS 1000 

// --- Radar Constants ---
#define EARTH_RADIUS_KM 6371.0
#define BLIP_LIFESPAN_FRAMES 90
#define MAX_BLIPS 20
#define RADAR_CENTER_X 96
#define RADAR_CENTER_Y 36
#define RADAR_RADIUS 27

// --- EEPROM Constants ---
#define EEPROM_SIZE 64
#define EEPROM_ADDR_MAGIC 0
#define EEPROM_ADDR_VOLUME 4
#define EEPROM_ADDR_RANGE_INDEX 8
#define EEPROM_ADDR_SPEED_INDEX 12
#define EEPROM_MAGIC_NUMBER 0xAD

// --- Display & Rotary Objects ---
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
SimpleRotary VolumeSelector(33, 4, 23);
SimpleRotary ChannelSelector(25, 32, 2);

// --- Volatile variables for ISR-safe encoder reading ---
volatile byte VolumeChange = 0, VolumePush = 0;
volatile byte ChannelChange = 0, ChannelPush = 0;
volatile bool dataConnectionOk = false;

// --- Control State Machine ---
enum ControlMode { VOLUME, SPEED };
ControlMode currentMode = VOLUME;

// --- Control Variables ---
int beepVolume;
bool displayingVolume = false;
unsigned long volumeDisplayTimeout = 0;

float rangeSteps[] = {5, 10, 25, 50, 100, 150, 200, 300};
const int rangeStepsCount = sizeof(rangeSteps) / sizeof(rangeSteps[0]);
int rangeStepIndex;
float radarRangeKm;
bool displayingRange = false;
unsigned long rangeDisplayTimeout = 0;

float sweepSpeedSteps[] = {90.0, 180.0, 270.0, 360.0};
const int speedStepsCount = sizeof(sweepSpeedSteps) / sizeof(sweepSpeedSteps[0]);
int sweepSpeedIndex;
float sweepSpeed;
bool displayingSpeed = false;
unsigned long speedDisplayTimeout = 0;
bool displayingMode = false;
unsigned long modeDisplayTimeout = 0;

// --- Data Structs ---
struct Aircraft {
  char flight[10];
  double distanceKm;
  double bearing;
  int altitude; 
  float groundSpeed; 
  bool isValid;
};

struct RadarBlip {
  int16_t x;
  int16_t y;
  int lifespan;
};

// --- Multi-core Shared Data ---
Aircraft lastPingedAircraft; // CHANGED: Stores the last aircraft hit by the sweep
std::vector<Aircraft> trackedAircraft;
std::vector<RadarBlip> activeBlips;
SemaphoreHandle_t dataMutex;

// --- Animation Variables ---
float sweepAngle = 0.0;
float lastSweepAngle = 0.0;
std::vector<bool> paintedThisTurn;
unsigned long lastFrameTime = 0;

// --- Function Prototypes ---
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
int getBeepFrequencyForAltitude(int altitude); 
double deg2rad(double deg);
void drawDottedCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);

// --- BEEP FREQUENCY BANDS ---
#define ALT_LOW_FEET 10000
#define ALT_HIGH_FEET 30000
#define FREQ_LOW 800
#define FREQ_MID 1200
#define FREQ_HIGH 1800

// --- Task for reading encoders reliably ---
void encoderTask(void *pvParameters) {
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

// Determines beep pitch based on altitude
int getBeepFrequencyForAltitude(int altitude) {
  if (altitude < 0) { // Default for unknown altitude
    return FREQ_MID;
  }
  if (altitude < ALT_LOW_FEET) {
    return FREQ_LOW;
  } else if (altitude < ALT_HIGH_FEET) {
    return FREQ_MID;
  } else {
    return FREQ_HIGH;
  }
}

// --- TASK FOR CORE 0: DATA FETCHING ---
void fetchDataTask(void *pvParameters) {
  for (;;) {
    fetchAircraft();
    vTaskDelay(REFRESH_INTERVAL_MS / portTICK_PERIOD_MS);
  }
}

void setup() {
  pinMode(19, OUTPUT);
  digitalWrite(19, HIGH);
  Serial.begin(115200);
  Serial.println("Booting Multi-Target Radar (Enhanced)");

  loadSettings();
  dataMutex = xSemaphoreCreateMutex();
  lastPingedAircraft.isValid = false; // CHANGED: Initialize new display variable
  
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
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < WIFI_CONNECT_TIMEOUT_MS) {
    delay(500);
    Serial.print('.');
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected.");
    WiFi.setSleep(false);
  } else {
    Serial.println("\nWiFi connection failed.");
    display.fillRect(0, 16, SCREEN_WIDTH, 16, SH110X_BLACK);
    display.setCursor(0, 16);
    display.println("WiFi Failed");
    display.display();
  }

  i2s_config_t i2s_config = {.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), .sample_rate = 44100, .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, .communication_format = I2S_COMM_FORMAT_STAND_I2S, .intr_alloc_flags = 0, .dma_buf_count = 8, .dma_buf_len = 64, .use_apll = false, .tx_desc_auto_clear = true, .fixed_mclk = 0};
  i2s_pin_config_t pin_config = {.bck_io_num = I2S_BCLK_PIN, .ws_io_num = I2S_LRCLK_PIN, .data_out_num = I2S_DOUT_PIN, .data_in_num = I2S_PIN_NO_CHANGE};
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  xTaskCreatePinnedToCore(fetchDataTask, "FetchData", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(encoderTask, "Encoder", 2048, NULL, 3, NULL, 1);
}

// --- MAIN LOOP ON CORE 1 (Graphics and Logic) ---
void loop() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastFrameTime;

  byte localVolumeChange = VolumeChange;
  byte localVolumePush = VolumePush;
  byte localChannelChange = ChannelChange;
  byte localChannelPush = ChannelPush;
  if (localVolumeChange != 0) VolumeChange = 0;
  if (localVolumePush != 0) VolumePush = 0;
  if (localChannelChange != 0) ChannelChange = 0;
  if (localChannelPush != 0) ChannelPush = 0;

  if (localChannelPush == 1) {
    currentMode = (currentMode == VOLUME) ? SPEED : VOLUME;
    displayingMode = true;
    modeDisplayTimeout = currentTime + DISPLAY_TIMEOUT_MS;
  }

  if (localVolumeChange != 0) {
    if (currentMode == VOLUME) {
      beepVolume += (localVolumeChange == 1) ? 1 : -1;
      beepVolume = constrain(beepVolume, 0, 20);
      displayingVolume = true;
      volumeDisplayTimeout = currentTime + DISPLAY_TIMEOUT_MS;
    } else {
      sweepSpeedIndex += (localVolumeChange == 1) ? 1 : -1;
      sweepSpeedIndex = constrain(sweepSpeedIndex, 0, speedStepsCount - 1);
      sweepSpeed = sweepSpeedSteps[sweepSpeedIndex];
      displayingSpeed = true;
      speedDisplayTimeout = currentTime + DISPLAY_TIMEOUT_MS;
    }
  }
  
  if (localVolumePush == 2) { Poweroff("  Goodbye"); }
  
  if (localChannelChange != 0) {
    rangeStepIndex += (localChannelChange == 1) ? 1 : -1;
    rangeStepIndex = constrain(rangeStepIndex, 0, rangeStepsCount - 1);
    radarRangeKm = rangeSteps[rangeStepIndex];
    displayingRange = true;
    rangeDisplayTimeout = currentTime + DISPLAY_TIMEOUT_MS;
    
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    activeBlips.clear(); 
    std::vector<Aircraft> filteredAircraft;
    // REMOVED: Logic for finding closest aircraft, not needed here anymore
    for (const auto& ac : trackedAircraft) {
      if (ac.distanceKm < radarRangeKm) {
        filteredAircraft.push_back(ac);
      }
    }
    trackedAircraft = filteredAircraft;
    if (trackedAircraft.empty()) { // If no planes left, invalidate display
        lastPingedAircraft.isValid = false;
    }
    paintedThisTurn.assign(trackedAircraft.size(), false);
    xSemaphoreGive(dataMutex);
  }
  
  if (displayingVolume && currentTime > volumeDisplayTimeout) { displayingVolume = false; }
  if (displayingRange && currentTime > rangeDisplayTimeout) { displayingRange = false; }
  if (displayingSpeed && currentTime > speedDisplayTimeout) { displayingSpeed = false; }
  if (displayingMode && currentTime > modeDisplayTimeout) { displayingMode = false; }

  if (deltaTime > 0) {
    lastSweepAngle = sweepAngle;
    sweepAngle += (sweepSpeed / 1000.0) * deltaTime;
    if (sweepAngle >= 360.0) {
      sweepAngle = fmod(sweepAngle, 360.0);
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      paintedThisTurn.assign(trackedAircraft.size(), false);
      xSemaphoreGive(dataMutex);
    }
  }

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
        float screenRadius = map(realDistance, 0, radarRangeKm, 0, RADAR_RADIUS);
        int16_t newBlipX = RADAR_CENTER_X + screenRadius * sin(angleRad);
        int16_t newBlipY = RADAR_CENTER_Y - screenRadius * cos(angleRad);
        activeBlips.push_back({newBlipX, newBlipY, BLIP_LIFESPAN_FRAMES});
        if (activeBlips.size() > MAX_BLIPS) {
          activeBlips.erase(activeBlips.begin());
        }
        
        lastPingedAircraft = trackedAircraft[i]; // NEW: Update display data with current aircraft
        
        paintedThisTurn[i] = true;
        int freq = getBeepFrequencyForAltitude(trackedAircraft[i].altitude);
        playBeep(freq, 20);
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
  lastFrameTime = currentTime;
}

void Poweroff(String powermessage) {
  Serial.println("Powering off. Saving settings...");
  saveSettings();
  i2s_driver_uninstall(I2S_NUM_0);
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(powermessage);
  display.display();
  delay(1000);
  digitalWrite(19, LOW);
}

void saveSettings() {
  EEPROM.put(EEPROM_ADDR_VOLUME, beepVolume);
  EEPROM.put(EEPROM_ADDR_RANGE_INDEX, rangeStepIndex);
  EEPROM.put(EEPROM_ADDR_SPEED_INDEX, sweepSpeedIndex);
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
    EEPROM.get(EEPROM_ADDR_SPEED_INDEX, sweepSpeedIndex);
    beepVolume = constrain(beepVolume, 0, 20);
    rangeStepIndex = constrain(rangeStepIndex, 0, rangeStepsCount - 1);
    sweepSpeedIndex = constrain(sweepSpeedIndex, 0, speedStepsCount - 1);
  } else {
    Serial.println("First run or invalid EEPROM data. Setting defaults.");
    beepVolume = 10;
    rangeStepIndex = 3;
    sweepSpeedIndex = 1;
    saveSettings();
  }
  radarRangeKm = rangeSteps[rangeStepIndex];
  sweepSpeed = sweepSpeedSteps[sweepSpeedIndex];
  Serial.printf("Loaded Volume: %d\n", beepVolume);
  Serial.printf("Loaded Range: %.0f km\n", radarRangeKm);
  Serial.printf("Loaded Speed: %.1f deg/s\n", sweepSpeed);
}

void drawRadarScreen() {
  display.clearDisplay();
  display.setTextSize(1);

  if (displayingMode) {
    display.setCursor(20, 16);
    display.print("Control Mode");
    display.setTextSize(2);
    display.setCursor(20, 32);
    display.print(currentMode == VOLUME ? "Volume" : "Speed");
    display.display();
    return;
  }
  if (displayingSpeed) {
    display.setCursor(20, 16);
    display.print("Sweep Speed");
    display.setTextSize(2);
    display.setCursor(20, 32);
    display.print(sweepSpeed, 0);
    display.print(" d/s");
    display.display();
    return;
  }
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

  Aircraft currentAircraftToDisplay; // CHANGED
  std::vector<RadarBlip> currentBlips;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  currentAircraftToDisplay = lastPingedAircraft; // CHANGED: Get last pinged aircraft
  currentBlips = activeBlips;
  xSemaphoreGive(dataMutex);

  // CHANGED: All display logic now uses currentAircraftToDisplay
  if (currentAircraftToDisplay.isValid) {
    display.setCursor(0, 0);
    display.print("Flt: ");
    display.println(strlen(currentAircraftToDisplay.flight) > 0 ? currentAircraftToDisplay.flight : "------");
    display.print("Dst: ");
    display.print(currentAircraftToDisplay.distanceKm, 1);
    display.println("km");
    display.print("Alt: ");
    if (currentAircraftToDisplay.altitude >= 0) {
      display.print(currentAircraftToDisplay.altitude);
      display.println("ft");
    } else {
      display.println("-----");
    }
    display.print("Spd: ");
    if (currentAircraftToDisplay.groundSpeed >= 0) {
        display.print(currentAircraftToDisplay.groundSpeed, 0);
        display.println("kt");
    } else {
        display.println("---");
    }
    display.print("Rng: ");
    display.print(radarRangeKm, 0);
    display.print("km");
  } else {
    display.setCursor(0, 0);
    display.println("Scanning...");
    display.setCursor(0, 16);
    display.print("Range:");
    display.print(radarRangeKm, 0);
    display.print("km");
  }

  if (dataConnectionOk) {
    int16_t baseX = SCREEN_WIDTH - 6;
    int16_t baseY = 8;
    display.drawLine(baseX, baseY, baseX, baseY - 4, SH110X_WHITE); 
    display.drawLine(baseX, baseY - 4, baseX - 3, baseY - 7, SH110X_WHITE);
    display.drawLine(baseX, baseY - 4, baseX + 3, baseY - 7, SH110X_WHITE);
  }

  display.drawCircle(RADAR_CENTER_X, RADAR_CENTER_Y, RADAR_RADIUS, SH110X_WHITE);
  display.setCursor(RADAR_CENTER_X - 3, RADAR_CENTER_Y - RADAR_RADIUS - 9);
  display.print("N");

  drawDottedCircle(RADAR_CENTER_X, RADAR_CENTER_Y, RADAR_RADIUS * 2 / 3, SH110X_WHITE);
  drawDottedCircle(RADAR_CENTER_X, RADAR_CENTER_Y, RADAR_RADIUS * 1 / 3, SH110X_WHITE);

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
  int16_t sweepX = RADAR_CENTER_X + (RADAR_RADIUS - 1) * sin(sweepRad);
  int16_t sweepY = RADAR_CENTER_Y - (RADAR_RADIUS - 1) * cos(sweepRad);
  display.drawLine(RADAR_CENTER_X, RADAR_CENTER_Y, sweepX, sweepY, SH110X_WHITE);

  display.display();
}

void fetchAircraft() {
  HTTPClient http;
  char url[160];
  snprintf(url, sizeof(url), "http://%s:%d/dump1090-fa/data/aircraft.json", DUMP1090_SERVER, DUMP1090_PORT);

  if (!http.begin(url)) {
    dataConnectionOk = false;
    return;
  }

  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    DynamicJsonDocument filter(512);
    filter["aircraft"][0]["lat"] = true;
    filter["aircraft"][0]["lon"] = true;
    filter["aircraft"][0]["flight"] = true;
    filter["aircraft"][0]["alt_baro"] = true; 
    filter["aircraft"][0]["gs"] = true; 

    DynamicJsonDocument doc(8192); 
    if (deserializeJson(doc, http.getStream(), DeserializationOption::Filter(filter)) == DeserializationError::Ok) {
      dataConnectionOk = true;
      JsonArray arr = doc["aircraft"].as<JsonArray>();
      
      std::vector<Aircraft> planesInRange;
      // REMOVED: Logic for finding closest aircraft, not needed anymore
      
      for (JsonObject plane : arr) {
        if (plane.containsKey("lat") && plane.containsKey("lon")) {
          double dist = haversine(USER_LAT, USER_LON, plane["lat"], plane["lon"]);

          if (isnan(dist) || isinf(dist)) {
            continue;
          }
          
          if (dist < radarRangeKm) {
            Aircraft ac;
            const char* flightStr = plane["flight"].as<const char*>();
            if (flightStr) {
                strncpy(ac.flight, flightStr, sizeof(ac.flight) - 1);
                ac.flight[sizeof(ac.flight) - 1] = '\0';
            } else {
                ac.flight[0] = '\0';
            }

            ac.distanceKm = dist;
            ac.bearing = calculateBearing(USER_LAT, USER_LON, plane["lat"], plane["lon"]);
            
            if (plane.containsKey("alt_baro")) {
              JsonVariant alt = plane["alt_baro"];
              if (alt.is<int>()) {
                ac.altitude = alt.as<int>();
              } else if (alt.is<const char*>() && strcmp(alt.as<const char*>(), "ground") == 0) {
                ac.altitude = 0;
              } else {
                ac.altitude = -1;
              }
            } else {
                ac.altitude = -1;
            }

            if (plane.containsKey("gs")) {
                ac.groundSpeed = plane["gs"].as<float>();
            } else {
                ac.groundSpeed = -1.0;
            }

            if (isnan(ac.bearing) || isinf(ac.bearing)) {
                continue;
            }

            ac.isValid = true;
            planesInRange.push_back(ac);
          }
        }
      }

      xSemaphoreTake(dataMutex, portMAX_DELAY);
      trackedAircraft = planesInRange;
      // REMOVED: No longer need to update closestAircraft
      xSemaphoreGive(dataMutex);
    } else {
      dataConnectionOk = false;
    }
  } else {
    dataConnectionOk = false;
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    trackedAircraft.clear();
    lastPingedAircraft.isValid = false; // Invalidate display on connection loss
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

// Custom function to draw a dotted circle
void drawDottedCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
  for (int i = 0; i < 360; i += 30) {
    double angleRad = i * PI / 180.0;
    int16_t x = x0 + r * sin(angleRad);
    int16_t y = y0 - r * cos(angleRad);
    display.drawPixel(x, y, color);
  }
}

