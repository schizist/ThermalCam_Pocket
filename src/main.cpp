#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();  
Adafruit_AMG88xx amg;

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

const int GRID_W = 8;
const int GRID_H = 8;

#define SCREEN_W 240
#define SCREEN_H 135

#define UPSCALE 8
const int INTERP_W = GRID_W * UPSCALE;
const int INTERP_H = GRID_H * UPSCALE;
float interpBuf[INTERP_W * INTERP_H];

// Button pins
#define BUTTON_INTERP 0   // GPIO0: short press = interpolation toggle, long press = power off
#define BUTTON_RANGE  35  // GPIO35: manual/auto range toggle

bool useInterpolation = true;
bool useManualRange = false;
float manualMin = 20.0f;
float manualMax = 40.0f;

// ------------- Power Off -------------
void powerOff() {
  tft.writecommand(ST7789_DISPOFF);
  tft.writecommand(ST7789_SLPIN);
  esp_deep_sleep_start();
}

// ------------- Color Mapping -------------
float gammaAdjust(float ratio, float gamma) {
  if (ratio < 0) ratio = 0;
  if (ratio > 1) ratio = 1;
  return powf(ratio, gamma);
}

uint16_t thermalGradient(float ratio) {
  if (ratio < 0) ratio = 0;
  if (ratio > 1) ratio = 1;

  uint8_t r, g, b;
  if (ratio <= 0.25f) {
    float f = ratio / 0.25f;
    r = 0; g = (uint8_t)(255 * f); b = 255;
  } else if (ratio <= 0.50f) {
    float f = (ratio - 0.25f) / 0.25f;
    r = 0; g = 255; b = (uint8_t)(255 * (1 - f));
  } else if (ratio <= 0.75f) {
    float f = (ratio - 0.50f) / 0.25f;
    r = (uint8_t)(255 * f); g = 255; b = 0;
  } else {
    float f = (ratio - 0.75f) / 0.25f;
    r = 255; g = (uint8_t)(255 * (1 - f)); b = 0;
  }
  return tft.color565(r, g, b);
}

uint16_t tempToColor(float temp, float tMin, float tMax) {
  float ratio = (temp - tMin) / (tMax - tMin);
  ratio = gammaAdjust(ratio, 0.6f);
  return thermalGradient(ratio);
}

// ------------- Button Handling -------------
bool lastInterpState = HIGH;
unsigned long interpPressStart = 0;

bool lastRangeState = HIGH;
unsigned long lastRangeToggle = 0;

void checkButtons() {
  // Interpolation button (GPIO0)
  int interpReading = digitalRead(BUTTON_INTERP);
  if (interpReading == LOW && lastInterpState == HIGH) {
    interpPressStart = millis(); // button pressed
  }
  if (interpReading == HIGH && lastInterpState == LOW) {
    unsigned long pressDuration = millis() - interpPressStart;
    if (pressDuration < 2000) {
      useInterpolation = !useInterpolation; // short press toggle interpolation
    }
  }
  if (interpReading == LOW && millis() - interpPressStart > 2000) {
    powerOff();  // long press power off
  }
  lastInterpState = interpReading;

  // Manual/auto range button (GPIO35)
  int rangeReading = digitalRead(BUTTON_RANGE);
  if (rangeReading == LOW && lastRangeState == HIGH && millis() - lastRangeToggle > 250) {
    useManualRange = !useManualRange;
    lastRangeToggle = millis();
  }
  lastRangeState = rangeReading;
}

// ------------- Drawing -------------
void drawInterpolatedHeatmap(const float *pix, float tMin, float tMax) {
  int W = tft.width();
  int H = tft.height();
  int cellW = (W + INTERP_W - 1) / INTERP_W;
  int cellH = (H + INTERP_H - 1) / INTERP_H;
  int gridWpx = cellW * INTERP_W;
  int gridHpx = cellH * INTERP_H;
  int x0 = (W - gridWpx) / 2;
  int y0 = (H - gridHpx) / 2;

  tft.startWrite();
  for (int y = 0; y < INTERP_H; y++) {
    for (int x = 0; x < INTERP_W; x++) {
      int i = y * INTERP_W + x;
      uint16_t c = tempToColor(pix[i], tMin, tMax);
      tft.fillRect(x0 + x * cellW, y0 + y * cellH, cellW, cellH, c);
    }
  }
  tft.endWrite();
}

void drawHeatmap(float *pixels, float tMin, float tMax) {
  int cellW = ceilf((float)SCREEN_W / GRID_W);
  int cellH = ceilf((float)SCREEN_H / GRID_H);
  for (int y = 0; y < GRID_H; y++) {
    for (int x = 0; x < GRID_W; x++) {
      int i = y * GRID_W + x;
      uint16_t c = tempToColor(pixels[i], tMin, tMax);
      tft.fillRect(x * cellW, y * cellH, cellW, cellH, c);
    }
  }
}

// ------------- Interpolation -------------
void bilinearInterpolate(const float *src, float *dst, int srcW, int srcH, int scale) {
  int dstW = srcW * scale;
  int dstH = srcH * scale;
  for (int y = 0; y < dstH; y++) {
    float gy = (float)(y) / scale;
    int y0 = (int)gy;
    int y1 = min(y0 + 1, srcH - 1);
    float fy = gy - y0;
    for (int x = 0; x < dstW; x++) {
      float gx = (float)(x) / scale;
      int x0 = (int)gx;
      int x1 = min(x0 + 1, srcW - 1);
      float fx = gx - x0;

      float v00 = src[y0 * srcW + x0];
      float v01 = src[y0 * srcW + x1];
      float v10 = src[y1 * srcW + x0];
      float v11 = src[y1 * srcW + x1];

      float v0 = v00 + fx * (v01 - v00);
      float v1 = v10 + fx * (v11 - v10);
      dst[y * dstW + x] = v0 + fy * (v1 - v0);
    }
  }
}

// ------------- Main -------------
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  if (!amg.begin()) {
    tft.setTextColor(TFT_RED);
    tft.drawString("AMG8833 not found!", 10, 10, 2);
    while (1);
  }

  pinMode(BUTTON_INTERP, INPUT_PULLUP);
  pinMode(BUTTON_RANGE, INPUT_PULLUP);

  tft.setTextColor(TFT_WHITE);
  tft.drawString("Thermal Imager Init OK", 10, 10, 2);
  delay(1000);
  tft.fillScreen(TFT_BLACK);
}

void loop() {
  amg.readPixels(pixels);

  float tMin, tMax;
  if (useManualRange) {
    tMin = manualMin;
    tMax = manualMax;
  } else {
    tMin = 1000.0f;
    tMax = -1000.0f;
    for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      if (pixels[i] < tMin) tMin = pixels[i];
      if (pixels[i] > tMax) tMax = pixels[i];
    }
    if (tMax - tMin < 1.0f) tMax = tMin + 1.0f;
    const float pad = 1.5f;
    tMin -= pad; tMax += pad;
  }

  checkButtons();

  if (useInterpolation) {
    bilinearInterpolate(pixels, interpBuf, GRID_W, GRID_H, UPSCALE);
    drawInterpolatedHeatmap(interpBuf, tMin, tMax);
  } else {
    drawHeatmap(pixels, tMin, tMax);
  }

  delay(60);
}
