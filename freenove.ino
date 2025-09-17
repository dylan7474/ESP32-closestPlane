#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>

TFT_eSPI tft = TFT_eSPI();

void setup() {
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(TC_DATUM);
  tft.setTextSize(2);

  int16_t screenWidth = tft.width();
  int16_t screenHeight = tft.height();

  tft.drawString("Closest Plane", screenWidth / 2, screenHeight / 2 - 20);
  tft.drawString("Display Test", screenWidth / 2, screenHeight / 2 + 10);
  tft.drawString("Freenove FNK1013", screenWidth / 2, screenHeight / 2 + 40);
}

void loop() {
  delay(100);
}
