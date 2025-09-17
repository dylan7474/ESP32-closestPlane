#include <Arduino.h>
#include <SimpleRotary.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <driver/i2s.h>
#include <esp_err.h>
#include <math.h>
#include <vector>
#include <EEPROM.h>
#include <algorithm>
#include <cstring>

// Configure TFT_eSPI with the Freenove ESP32 CYD ST7796 panel setup before compiling.

#include "config.h"

// --- Display & Timing Constants ---
#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 320
#define INFO_PANEL_WIDTH 160
#define RADAR_AREA_WIDTH (SCREEN_WIDTH - INFO_PANEL_WIDTH)
#define RADAR_CENTER_X_LOCAL (RADAR_AREA_WIDTH / 2)
#define REFRESH_INTERVAL_MS 5000
#define FRAME_INTERVAL_MS 33
#define WIFI_CONNECT_TIMEOUT_MS 10000

// --- Radar Constants ---
#define EARTH_RADIUS_KM 6371.0
#define BLIP_LIFESPAN_FRAMES 90
#define MAX_BLIPS 20
#define RADAR_CENTER_X (INFO_PANEL_WIDTH + RADAR_CENTER_X_LOCAL)
#define RADAR_CENTER_Y (SCREEN_HEIGHT / 2)
#define RADAR_RADIUS 150

// --- EEPROM Constants ---
#define EEPROM_SIZE 64
#define EEPROM_ADDR_MAGIC 0
#define EEPROM_ADDR_VOLUME 4
#define EEPROM_ADDR_RANGE_INDEX 8
#define EEPROM_ADDR_SPEED_INDEX 12
#define EEPROM_ADDR_ALERT_DIST 16
#define EEPROM_ADDR_MODE 20
#define EEPROM_ADDR_COMPASS 24
#define EEPROM_MAGIC_NUMBER 0xAD

// --- Display & Rotary Objects ---
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite infoSprite = TFT_eSprite(&tft);
TFT_eSprite radarSprite = TFT_eSprite(&tft);
bool useSpriteRendering = false;

static const uint16_t COLOR_PANEL_BG = 0x1106;
static const uint16_t COLOR_MODE_ACTIVE = 0x0640;
static const uint16_t COLOR_TEXT = 0xF79E;
static const uint16_t COLOR_RADAR_BG = 0x0100;
static const uint16_t COLOR_GRID = 0x0400;
static const uint16_t COLOR_SWEEP = 0x0640;
static const uint16_t COLOR_BLIP_INBOUND = 0xFA08;
static const uint16_t COLOR_BLIP_OUTBOUND = 0x463F;
static const uint16_t COLOR_INFO_TEXT = COLOR_TEXT;
static const uint16_t COLOR_INFO_BG = COLOR_PANEL_BG;
// Rotary wired as "volume" now adjusts radar range (switch still powers off)
SimpleRotary VolumeSelector(33, 4, 23);
// Rotary wired as "channel" now controls volume/speed/alert/compass (switch still cycles modes)
SimpleRotary ChannelSelector(25, 32, 2);

// --- Volatile variables for ISR-safe encoder reading ---
volatile byte VolumeChange = 0, VolumePush = 0;
volatile byte ChannelChange = 0, ChannelPush = 0;
volatile bool dataConnectionOk = false;

// --- Control State Machine ---
enum ControlMode { VOLUME, SPEED, ALERT, RADAR };
ControlMode currentMode = VOLUME;

// --- Control Variables ---
int beepVolume;

float rangeSteps[] = {5, 10, 25, 50, 100, 150, 200, 300};
const int rangeStepsCount = sizeof(rangeSteps) / sizeof(rangeSteps[0]);
int rangeStepIndex;
float radarRangeKm;

float sweepSpeedSteps[] = {90.0, 180.0, 270.0, 360.0};
const int speedStepsCount = sizeof(sweepSpeedSteps) / sizeof(sweepSpeedSteps[0]);
int sweepSpeedIndex;
float sweepSpeed;

#define INBOUND_ALERT_DISTANCE_KM 5.0
float inboundAlertDistanceKm;

float compassPoints[] = {0.0, 90.0, 180.0, 270.0};
const int compassPointsCount = sizeof(compassPoints) / sizeof(compassPoints[0]);
int compassIndex;
float radarOrientation;

// --- Data Structs ---
struct Aircraft {
  char flight[10];
  double distanceKm;
  double bearing;
  int altitude;
  float groundSpeed;
  float track;
  bool isInbound;
  float minutesToClosest;
  bool isValid;
};

struct RadarBlip {
  int16_t x;
  int16_t y;
  int lifespan;
  bool inbound;
};

// --- Multi-core Shared Data ---
Aircraft lastPingedAircraft; // CHANGED: Stores the last aircraft hit by the sweep
std::vector<Aircraft> trackedAircraft;
std::vector<RadarBlip> activeBlips;
SemaphoreHandle_t dataMutex;
Aircraft closestInboundAircraft;
std::vector<String> alertedFlights;

// --- Animation Variables ---
float sweepAngle = 0.0;
float lastSweepAngle = 0.0;
std::vector<bool> paintedThisTurn;
unsigned long lastFrameTime = 0;
unsigned long lastDrawTime = 0;

// --- UI State Cache ---
struct PanelSnapshot {
  bool hasAircraft;
  Aircraft aircraft;
  bool hasInbound;
  Aircraft inbound;
  ControlMode mode;
  int volume;
  int rangeIndex;
  int speedIndex;
  float alertDistanceKm;
  float orientation;
  float rangeKm;
};

PanelSnapshot lastPanelSnapshot = {};
bool lastPanelValid = false;
bool panelDirty = true;

// --- Function Prototypes ---
void saveSettings();
void loadSettings();
void fetchAircraft();
void drawRadarScreen();
void renderInfoPanel(TFT_eSPI &canvas, const PanelSnapshot &snapshot);
bool panelSnapshotsDiffer(const PanelSnapshot &a, const PanelSnapshot &b);
void drawControlItem(TFT_eSPI &canvas, const char *label, const String &value, bool active, int &y);
void drawStatusIndicators(TFT_eSPI &canvas, int16_t originX);
void drawRadarGrid(TFT_eSPI &canvas, int16_t originX);
void fetchDataTask(void *pvParameters);
void encoderTask(void *pvParameters);
void Poweroff(String powermessage);
double haversine(double lat1, double lon1, double lat2, double lon2);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
void playBeep(int freq, int duration_ms);
void playSiren(int startFreq, int endFreq, int duration_ms);
int getBeepFrequencyForAltitude(int altitude);
double deg2rad(double deg);
void drawDottedCircle(TFT_eSPI &canvas, int16_t x0, int16_t y0, int16_t r, uint16_t color);

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
    if (vol_rotate_event != 0) { ChannelChange = vol_rotate_event; }
    byte chan_rotate_event = ChannelSelector.rotate();
    if (chan_rotate_event != 0) { VolumeChange = chan_rotate_event; }
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
  closestInboundAircraft.isInbound = false;
  closestInboundAircraft.isValid = false;
  
  tft.init();
  tft.initDMA();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(COLOR_TEXT, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(16, 24);
  tft.println("Multi-Target Radar");
  tft.setCursor(16, 64);
  tft.println("Connecting WiFi...");

  infoSprite.setColorDepth(16);
  radarSprite.setColorDepth(16);
  uint16_t *infoBuffer = infoSprite.createSprite(INFO_PANEL_WIDTH, SCREEN_HEIGHT);
  uint16_t *radarBuffer = radarSprite.createSprite(RADAR_AREA_WIDTH, SCREEN_HEIGHT);
  if (infoBuffer && radarBuffer) {
    useSpriteRendering = true;
    infoSprite.setSwapBytes(true);
    radarSprite.setSwapBytes(true);
  } else {
    Serial.println("Sprite allocation failed, falling back to direct rendering.");
    if (infoBuffer) {
      infoSprite.deleteSprite();
    }
    if (radarBuffer) {
      radarSprite.deleteSprite();
    }
  }
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < WIFI_CONNECT_TIMEOUT_MS) {
    delay(500);
    Serial.print('.');
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected.");
    WiFi.setSleep(false);
    tft.fillRect(0, 56, SCREEN_WIDTH, 32, TFT_BLACK);
    tft.setCursor(16, 64);
    tft.println("WiFi Connected");
  } else {
    Serial.println("\nWiFi connection failed.");
    tft.fillRect(0, 56, SCREEN_WIDTH, 32, TFT_BLACK);
    tft.setCursor(16, 64);
    tft.println("WiFi Failed");
  }

  i2s_config_t i2s_config = {};
  i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  i2s_config.sample_rate = 44100;
  i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2s_config.intr_alloc_flags = 0;
  i2s_config.dma_buf_count = 8;
  i2s_config.dma_buf_len = 64;
  i2s_config.use_apll = false;
  i2s_config.tx_desc_auto_clear = true;
  i2s_config.fixed_mclk = 0;

  i2s_pin_config_t pin_config = {};
  pin_config.bck_io_num = I2S_BCLK_PIN;
  pin_config.ws_io_num = I2S_LRCLK_PIN;
  pin_config.data_out_num = I2S_DOUT_PIN;
  pin_config.data_in_num = I2S_PIN_NO_CHANGE;
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  xTaskCreatePinnedToCore(fetchDataTask, "FetchData", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(encoderTask, "Encoder", 2048, NULL, 3, NULL, 1);

  lastFrameTime = millis();
  lastDrawTime = lastFrameTime;
}

// --- MAIN LOOP ON CORE 1 (Graphics and Logic) ---
void loop() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastFrameTime;
  lastFrameTime = currentTime;

  byte localVolumeChange = VolumeChange;
  byte localVolumePush = VolumePush;
  byte localChannelChange = ChannelChange;
  byte localChannelPush = ChannelPush;
  if (localVolumeChange != 0) VolumeChange = 0;
  if (localVolumePush != 0) VolumePush = 0;
  if (localChannelChange != 0) ChannelChange = 0;
  if (localChannelPush != 0) ChannelPush = 0;

  if (localChannelPush == 1) {
    if (currentMode == VOLUME) currentMode = SPEED;
    else if (currentMode == SPEED) currentMode = ALERT;
    else if (currentMode == ALERT) currentMode = RADAR;
    else currentMode = VOLUME;
  }

  if (localVolumeChange != 0) {
    if (currentMode == VOLUME) {
      beepVolume += (localVolumeChange == 1) ? 1 : -1;
      beepVolume = constrain(beepVolume, 0, 20);
    } else if (currentMode == SPEED) {
      sweepSpeedIndex += (localVolumeChange == 1) ? 1 : -1;
      sweepSpeedIndex = constrain(sweepSpeedIndex, 0, speedStepsCount - 1);
      sweepSpeed = sweepSpeedSteps[sweepSpeedIndex];
    } else if (currentMode == ALERT) {
      inboundAlertDistanceKm += (localVolumeChange == 1) ? 1 : -1;
      inboundAlertDistanceKm = constrain(inboundAlertDistanceKm, 1.0, 50.0);
    } else if (currentMode == RADAR) {
      compassIndex += (localVolumeChange == 1) ? 1 : -1;
      if (compassIndex < 0) compassIndex = compassPointsCount - 1;
      if (compassIndex >= compassPointsCount) compassIndex = 0;
      radarOrientation = compassPoints[compassIndex];
      activeBlips.clear();
      paintedThisTurn.assign(trackedAircraft.size(), false);
      lastSweepAngle = sweepAngle;
    }
  }
  
  if (localVolumePush == 2) { Poweroff("  Goodbye"); }
  
  if (localChannelChange != 0) {
    rangeStepIndex += (localChannelChange == 1) ? 1 : -1;
    rangeStepIndex = constrain(rangeStepIndex, 0, rangeStepsCount - 1);
    radarRangeKm = rangeSteps[rangeStepIndex];
    
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
      double targetBearing = trackedAircraft[i].bearing - radarOrientation;
      if (targetBearing < 0) targetBearing += 360.0;
      bool bearingCrossed = (lastSweepAngle < targetBearing && sweepAngle >= targetBearing);
      if (lastSweepAngle > sweepAngle && (targetBearing > lastSweepAngle || targetBearing <= sweepAngle)) {
        bearingCrossed = true;
      }
      if (bearingCrossed) {
        double angleRad = targetBearing * PI / 180.0;
        double realDistance = trackedAircraft[i].distanceKm;
        float ratio = radarRangeKm > 0 ? std::min<float>(static_cast<float>(realDistance / radarRangeKm), 1.0f) : 0.0f;
        float screenRadius = ratio * RADAR_RADIUS;
        int16_t newBlipX = RADAR_CENTER_X_LOCAL + screenRadius * sin(angleRad);
        int16_t newBlipY = RADAR_CENTER_Y - screenRadius * cos(angleRad);
        activeBlips.push_back({newBlipX, newBlipY, BLIP_LIFESPAN_FRAMES, trackedAircraft[i].isInbound});
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

  if (currentTime - lastDrawTime >= FRAME_INTERVAL_MS) {
    drawRadarScreen();
    lastDrawTime = currentTime;
  } else {
    delay(1);
  }
}

void Poweroff(String powermessage) {
  Serial.println("Powering off. Saving settings...");
  saveSettings();
  i2s_driver_uninstall(I2S_NUM_0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(COLOR_TEXT, TFT_BLACK);
  tft.setTextSize(3);
  tft.setCursor(40, SCREEN_HEIGHT / 2 - 24);
  tft.println(powermessage);
  delay(1000);
  digitalWrite(19, LOW);
}

void saveSettings() {
  EEPROM.put(EEPROM_ADDR_VOLUME, beepVolume);
  EEPROM.put(EEPROM_ADDR_RANGE_INDEX, rangeStepIndex);
  EEPROM.put(EEPROM_ADDR_SPEED_INDEX, sweepSpeedIndex);
  EEPROM.put(EEPROM_ADDR_ALERT_DIST, inboundAlertDistanceKm);
  EEPROM.put(EEPROM_ADDR_MODE, currentMode);
  EEPROM.put(EEPROM_ADDR_COMPASS, compassIndex);
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
    EEPROM.get(EEPROM_ADDR_ALERT_DIST, inboundAlertDistanceKm);
    EEPROM.get(EEPROM_ADDR_MODE, currentMode);
    EEPROM.get(EEPROM_ADDR_COMPASS, compassIndex);
    beepVolume = constrain(beepVolume, 0, 20);
    rangeStepIndex = constrain(rangeStepIndex, 0, rangeStepsCount - 1);
    sweepSpeedIndex = constrain(sweepSpeedIndex, 0, speedStepsCount - 1);
    inboundAlertDistanceKm = constrain(inboundAlertDistanceKm, 1.0f, 50.0f);
    currentMode = (ControlMode)constrain((int)currentMode, 0, RADAR);
    compassIndex = constrain(compassIndex, 0, compassPointsCount - 1);
  } else {
    Serial.println("First run or invalid EEPROM data. Setting defaults.");
    beepVolume = 10;
    rangeStepIndex = 3;
    sweepSpeedIndex = 1;
    inboundAlertDistanceKm = INBOUND_ALERT_DISTANCE_KM;
    currentMode = VOLUME;
    compassIndex = 0;
    saveSettings();
  }
  radarRangeKm = rangeSteps[rangeStepIndex];
  sweepSpeed = sweepSpeedSteps[sweepSpeedIndex];
  radarOrientation = compassPoints[compassIndex];
  Serial.printf("Loaded Volume: %d\n", beepVolume);
  Serial.printf("Loaded Range: %.0f km\n", radarRangeKm);
  Serial.printf("Loaded Speed: %.1f deg/s\n", sweepSpeed);
  Serial.printf("Loaded Alert Dist: %.1f km\n", inboundAlertDistanceKm);
}

void drawControlItem(TFT_eSPI &canvas, const char *label, const String &value, bool active, int &y) {
  const int boxHeight = 28;
  const int boxX = 8;
  const int boxWidth = INFO_PANEL_WIDTH - 16;
  uint16_t bg = active ? COLOR_MODE_ACTIVE : COLOR_PANEL_BG;
  uint16_t fg = active ? TFT_BLACK : COLOR_TEXT;
  canvas.fillRect(boxX, y, boxWidth, boxHeight, bg);
  canvas.setTextSize(2);
  canvas.setTextColor(fg, bg);
  canvas.setCursor(boxX + 6, y + 6);
  canvas.print(label);
  canvas.print(": ");
  canvas.print(value);
  y += boxHeight + 4;
  canvas.setTextColor(COLOR_TEXT, COLOR_PANEL_BG);
}

void drawStatusIndicators(TFT_eSPI &canvas, int16_t originX) {
  const int baseX = originX + RADAR_AREA_WIDTH - 52;
  const int baseY = 44;
  const int barWidth = 6;
  const int barSpacing = 4;
  int bars = 0;
  if (WiFi.status() == WL_CONNECTED) {
    long rssi = WiFi.RSSI();
    if (rssi > -55)
      bars = 4;
    else if (rssi > -65)
      bars = 3;
    else if (rssi > -75)
      bars = 2;
    else if (rssi > -85)
      bars = 1;
  }
  for (int i = 0; i < 4; i++) {
    int height = (i + 1) * 10;
    int x = baseX + i * (barWidth + barSpacing);
    int y = baseY - height;
    if (i < bars) {
      canvas.fillRect(x, y, barWidth, height, COLOR_TEXT);
    } else {
      canvas.drawRect(x, y, barWidth, height, COLOR_TEXT);
    }
  }

  int dotX = baseX - 16;
  int dotY = baseY - 10;
  if (dataConnectionOk) {
    canvas.fillCircle(dotX, dotY, 6, COLOR_SWEEP);
  } else {
    canvas.drawCircle(dotX, dotY, 6, COLOR_TEXT);
    canvas.drawLine(dotX - 4, dotY - 4, dotX + 4, dotY + 4, COLOR_TEXT);
    canvas.drawLine(dotX - 4, dotY + 4, dotX + 4, dotY - 4, COLOR_TEXT);
  }
}

void drawRadarGrid(TFT_eSPI &canvas, int16_t originX) {
  int16_t centerX = originX + RADAR_CENTER_X_LOCAL;
  int16_t centerY = RADAR_CENTER_Y;
  canvas.drawCircle(centerX, centerY, RADAR_RADIUS, COLOR_GRID);
  drawDottedCircle(canvas, centerX, centerY, RADAR_RADIUS * 2 / 3, COLOR_GRID);
  drawDottedCircle(canvas, centerX, centerY, RADAR_RADIUS / 3, COLOR_GRID);
  canvas.drawLine(centerX - RADAR_RADIUS, centerY, centerX + RADAR_RADIUS, centerY, COLOR_GRID);
  canvas.drawLine(centerX, centerY - RADAR_RADIUS, centerX, centerY + RADAR_RADIUS, COLOR_GRID);

  static const char compassLetters[] = {'N', 'E', 'S', 'W'};
  canvas.setTextColor(COLOR_TEXT, COLOR_RADAR_BG);
  canvas.setTextSize(2);
  canvas.setCursor(centerX - 10, centerY - RADAR_RADIUS - 24);
  canvas.print(compassLetters[compassIndex]);
}

bool panelSnapshotsDiffer(const PanelSnapshot &a, const PanelSnapshot &b) {
  if (a.hasAircraft != b.hasAircraft) return true;
  if (a.hasAircraft) {
    if (strncmp(a.aircraft.flight, b.aircraft.flight, sizeof(a.aircraft.flight)) != 0) return true;
    if (fabs(a.aircraft.distanceKm - b.aircraft.distanceKm) > 0.1f) return true;
    if (fabs(a.aircraft.bearing - b.aircraft.bearing) > 0.5f) return true;
    if (a.aircraft.altitude != b.aircraft.altitude) return true;
    if (fabs(a.aircraft.groundSpeed - b.aircraft.groundSpeed) > 0.5f) return true;
  }
  if (a.hasInbound != b.hasInbound) return true;
  if (a.hasInbound) {
    if (strncmp(a.inbound.flight, b.inbound.flight, sizeof(a.inbound.flight)) != 0) return true;
    if (fabs(a.inbound.minutesToClosest - b.inbound.minutesToClosest) > 0.1f) return true;
  }
  if (a.mode != b.mode) return true;
  if (a.volume != b.volume) return true;
  if (a.rangeIndex != b.rangeIndex) return true;
  if (a.speedIndex != b.speedIndex) return true;
  if (fabs(a.alertDistanceKm - b.alertDistanceKm) > 0.1f) return true;
  if (fabs(a.orientation - b.orientation) > 0.5f) return true;
  if (fabs(a.rangeKm - b.rangeKm) > 0.1f) return true;
  return false;
}

void renderInfoPanel(TFT_eSPI &canvas, const PanelSnapshot &snapshot) {
  canvas.fillRect(0, 0, INFO_PANEL_WIDTH, SCREEN_HEIGHT, COLOR_INFO_BG);

  int panelY = 8;
  drawControlItem(canvas, "VOL", String(snapshot.volume), snapshot.mode == VOLUME, panelY);
  int speedIdx = constrain(snapshot.speedIndex, 0, speedStepsCount - 1);
  drawControlItem(canvas, "SPD", String((int)sweepSpeedSteps[speedIdx]), snapshot.mode == SPEED, panelY);
  drawControlItem(canvas, "ALR", String((int)snapshot.alertDistanceKm) + "km", snapshot.mode == ALERT, panelY);
  drawControlItem(canvas, "HDG", String((int)snapshot.orientation), snapshot.mode == RADAR, panelY);
  drawControlItem(canvas, "RNG", String((int)snapshot.rangeKm) + "km", false, panelY);

  int infoY = panelY + 4;
  canvas.setTextColor(COLOR_INFO_TEXT, COLOR_INFO_BG);
  canvas.setTextSize(2);

  if (snapshot.hasAircraft) {
    const char *flight = strlen(snapshot.aircraft.flight) > 0 ? snapshot.aircraft.flight : "------";
    canvas.setCursor(12, infoY);
    canvas.print("Flight: ");
    canvas.println(flight);
    infoY += 24;
    canvas.setCursor(12, infoY);
    canvas.print("Dist: ");
    canvas.print(snapshot.aircraft.distanceKm, 1);
    canvas.println(" km");
    infoY += 24;
    canvas.setCursor(12, infoY);
    canvas.print("Bear: ");
    canvas.print(snapshot.aircraft.bearing, 0);
    canvas.println(" deg");
    infoY += 24;
    canvas.setCursor(12, infoY);
    canvas.print("Alt: ");
    if (snapshot.aircraft.altitude >= 0) {
      canvas.print(snapshot.aircraft.altitude);
      canvas.println(" ft");
    } else {
      canvas.println("---");
    }
    infoY += 24;
    canvas.setCursor(12, infoY);
    canvas.print("Speed: ");
    if (snapshot.aircraft.groundSpeed >= 0) {
      canvas.print(snapshot.aircraft.groundSpeed, 0);
      canvas.println(" kt");
    } else {
      canvas.println("---");
    }
  } else {
    canvas.setCursor(12, infoY);
    canvas.println("Scanning...");
    infoY += 24;
    canvas.setCursor(12, infoY);
    canvas.print("Range: ");
    canvas.print(snapshot.rangeKm, 0);
    canvas.println(" km");
  }

  int inboundY = SCREEN_HEIGHT - 56;
  if (snapshot.hasInbound && snapshot.inbound.minutesToClosest >= 0) {
    canvas.setTextColor(COLOR_BLIP_INBOUND, COLOR_INFO_BG);
    canvas.setCursor(12, inboundY - 4);
    canvas.print("Inbound: ");
    canvas.println(strlen(snapshot.inbound.flight) > 0 ? snapshot.inbound.flight : "------");
    canvas.setCursor(12, inboundY + 20);
    canvas.print("ETA: ");
    canvas.print(snapshot.inbound.minutesToClosest, 1);
    canvas.println(" min");
    canvas.setTextColor(COLOR_INFO_TEXT, COLOR_INFO_BG);
  } else {
    canvas.setTextColor(COLOR_INFO_TEXT, COLOR_INFO_BG);
    canvas.setCursor(12, inboundY + 8);
    canvas.println("No inbound alert");
  }
}

void drawRadarScreen() {
  Aircraft currentAircraftToDisplay;
  Aircraft currentClosestInbound;
  std::vector<RadarBlip> currentBlips;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  currentAircraftToDisplay = lastPingedAircraft;
  currentBlips = activeBlips;
  currentClosestInbound = closestInboundAircraft;
  xSemaphoreGive(dataMutex);

  PanelSnapshot snapshot = {};
  snapshot.hasAircraft = currentAircraftToDisplay.isValid;
  snapshot.aircraft = currentAircraftToDisplay;
  snapshot.hasInbound = currentClosestInbound.isInbound && currentClosestInbound.minutesToClosest >= 0;
  snapshot.inbound = currentClosestInbound;
  snapshot.mode = currentMode;
  snapshot.volume = beepVolume;
  snapshot.rangeIndex = rangeStepIndex;
  snapshot.speedIndex = sweepSpeedIndex;
  snapshot.alertDistanceKm = inboundAlertDistanceKm;
  snapshot.orientation = radarOrientation;
  snapshot.rangeKm = radarRangeKm;

  if (!lastPanelValid || panelSnapshotsDiffer(snapshot, lastPanelSnapshot)) {
    if (useSpriteRendering) {
      renderInfoPanel(infoSprite, snapshot);
      panelDirty = true;
    } else {
      renderInfoPanel(tft, snapshot);
      panelDirty = false;
    }
    lastPanelSnapshot = snapshot;
    lastPanelValid = true;
  }

  TFT_eSPI *radarCanvas = useSpriteRendering ? static_cast<TFT_eSPI*>(&radarSprite) : &tft;
  int16_t originX = useSpriteRendering ? 0 : INFO_PANEL_WIDTH;

  radarCanvas->fillRect(originX, 0, RADAR_AREA_WIDTH, SCREEN_HEIGHT, COLOR_RADAR_BG);

  radarCanvas->setTextSize(2);
  radarCanvas->setTextColor(COLOR_TEXT, COLOR_RADAR_BG);
  radarCanvas->setCursor(originX + 12, 12);
  radarCanvas->print("Range: ");
  radarCanvas->print(radarRangeKm, 0);
  radarCanvas->println(" km");
  radarCanvas->setCursor(originX + 12, 36);
  radarCanvas->print("Heading: ");
  radarCanvas->print(radarOrientation, 0);
  radarCanvas->println(" deg");

  drawRadarGrid(*radarCanvas, originX);
  drawStatusIndicators(*radarCanvas, originX);

  unsigned long flashPhase = millis() / 200;
  for (const auto &blip : currentBlips) {
    if (blip.inbound && (flashPhase % 2 == 1)) {
      continue;
    }
    uint16_t color = blip.inbound ? COLOR_BLIP_INBOUND : COLOR_BLIP_OUTBOUND;
    int radius;
    if (blip.lifespan > (BLIP_LIFESPAN_FRAMES * 2 / 3)) {
      radius = 6;
    } else if (blip.lifespan > (BLIP_LIFESPAN_FRAMES / 3)) {
      radius = 4;
    } else {
      radius = 2;
    }
    int16_t drawX = originX + blip.x;
    int16_t drawY = blip.y;
    radarCanvas->fillCircle(drawX, drawY, radius, color);
  }

  double sweepRad = sweepAngle * PI / 180.0;
  int16_t sweepX = originX + RADAR_CENTER_X_LOCAL + (RADAR_RADIUS - 2) * sin(sweepRad);
  int16_t sweepY = RADAR_CENTER_Y - (RADAR_RADIUS - 2) * cos(sweepRad);
  radarCanvas->drawLine(originX + RADAR_CENTER_X_LOCAL, RADAR_CENTER_Y, sweepX, sweepY, COLOR_SWEEP);

  if (useSpriteRendering) {
    if (panelDirty) {
      infoSprite.pushSprite(0, 0);
      panelDirty = false;
    }
    radarSprite.pushSprite(INFO_PANEL_WIDTH, 0);
  }
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
    ArduinoJson::DynamicJsonDocument filter(512);
    filter["aircraft"][0]["lat"] = true;
    filter["aircraft"][0]["lon"] = true;
    filter["aircraft"][0]["flight"] = true;
    filter["aircraft"][0]["alt_baro"] = true;
    filter["aircraft"][0]["gs"] = true;
    filter["aircraft"][0]["track"] = true;

    ArduinoJson::DynamicJsonDocument doc(8192);
    if (deserializeJson(doc, http.getStream(), ArduinoJson::DeserializationOption::Filter(filter)) == ArduinoJson::DeserializationError::Ok) {
      dataConnectionOk = true;
      ArduinoJson::JsonArray arr = doc["aircraft"].as<ArduinoJson::JsonArray>();
      
      std::vector<Aircraft> planesInRange;
      Aircraft bestInbound;
      bestInbound.isInbound = false;
      bestInbound.isValid = false;

      for (ArduinoJson::JsonObject plane : arr) {
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
              ArduinoJson::JsonVariant alt = plane["alt_baro"];
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
            if (plane.containsKey("track")) {
                ac.track = plane["track"].as<float>();
            } else {
                ac.track = -1.0;
            }

            if (isnan(ac.bearing) || isinf(ac.bearing)) {
                continue;
            }

            // Inbound prediction
            ac.isInbound = false;
            ac.minutesToClosest = -1.0;
            if (ac.track >= 0 && ac.groundSpeed > 0) {
              double toBase = fmod(ac.bearing + 180.0, 360.0);
              double angleDiff = fabs(ac.track - toBase);
              if (angleDiff > 180.0) angleDiff = 360.0 - angleDiff;
              double crossTrack = ac.distanceKm * sin(deg2rad(angleDiff));
              double alongTrack = ac.distanceKm * cos(deg2rad(angleDiff));
              if (angleDiff < 90.0 && fabs(crossTrack) < inboundAlertDistanceKm) {
                ac.isInbound = true;
                double speedKmMin = ac.groundSpeed * 1.852 / 60.0;
                if (speedKmMin > 0) {
                  ac.minutesToClosest = alongTrack / speedKmMin;
                }
                bool already = std::find(alertedFlights.begin(), alertedFlights.end(), String(ac.flight)) != alertedFlights.end();
                if (!already) {
                  alertedFlights.push_back(String(ac.flight));
                  playSiren(800, 1800, 500);
                }
              } else {
                auto it = std::find(alertedFlights.begin(), alertedFlights.end(), String(ac.flight));
                if (it != alertedFlights.end()) alertedFlights.erase(it);
              }
            }

            ac.isValid = true;
            planesInRange.push_back(ac);
            if (ac.isInbound && ac.minutesToClosest >= 0) {
              if (!bestInbound.isInbound || ac.minutesToClosest < bestInbound.minutesToClosest) {
                bestInbound = ac;
              }
            }
          }
        }
      }

      xSemaphoreTake(dataMutex, portMAX_DELAY);
      trackedAircraft = planesInRange;
      if (bestInbound.isInbound) {
        closestInboundAircraft = bestInbound;
      } else {
        closestInboundAircraft.isInbound = false;
        closestInboundAircraft.isValid = false;
      }
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

void playSiren(int startFreq, int endFreq, int duration_ms) {
  long amplitude = map(beepVolume, 0, 20, 0, 25000);
  const int sampleRate = 44100;
  int samples = sampleRate * duration_ms / 1000;
  size_t bytes_written;
  for (int i = 0; i < samples; i++) {
    float progress = (float)i / samples;
    float freq = startFreq + (endFreq - startFreq) * progress;
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
void drawDottedCircle(TFT_eSPI &canvas, int16_t x0, int16_t y0, int16_t r, uint16_t color) {
  const int step = 12;
  for (int angle = 0; angle < 360; angle += step) {
    double angleRad = angle * PI / 180.0;
    int16_t x = x0 + r * sin(angleRad);
    int16_t y = y0 - r * cos(angleRad);
    canvas.drawPixel(x, y, color);
  }
}
