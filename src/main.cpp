#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <algorithm>
#include <Adafruit_AMG88xx.h>
#include <TFT_eSPI.h>

#ifndef WIFI_SSID
#define WIFI_SSID "Advanced Alien Technology Mk II"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "uireo-89pqk-qsknl"
#endif

WebServer server(80);

TFT_eSPI tft = TFT_eSPI();  
Adafruit_AMG88xx amg;

bool autoRangeInitialized = false;
float smoothRangeMin = 0.0f;
float smoothRangeMax = 0.0f;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float displayPixels[AMG88xx_PIXEL_ARRAY_SIZE];

const int GRID_W = 8;
const int GRID_H = 8;


#define SCREEN_W 240
#define SCREEN_H 135

// Battery voltage reading
#define BATTERY_PIN 34  // GPIO34 (ADC1_CH6)
#define BATTERY_VOLTAGE_DIVIDER 2.0f // Adjust if using a resistor divider
#define BATTERY_MIN_V 3.2f  // Minimum voltage (empty)
#define BATTERY_MAX_V 4.2f  // Maximum voltage (full)
// ------------- Battery Indicator -------------
float readBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  float voltage = (raw / 4095.0f) * 3.3f * BATTERY_VOLTAGE_DIVIDER;
  return voltage;
}

void drawBatteryIndicator(int x, int y, int w, int h, float voltage) {
  // Clear the area to reduce flicker
  int clearW = w + 44; // battery + text
  int clearH = h;
  tft.fillRect(x - 44, y, clearW, clearH, TFT_BLACK);

  float pct = (voltage - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V);
  if (pct < 0) pct = 0;
  if (pct > 1) pct = 1;
  int fillW = (int)((w - 4) * pct);
  // Draw battery outline
  tft.drawRect(x, y, w, h, TFT_WHITE);
  tft.fillRect(x + w, y + h/4, 3, h/2, TFT_WHITE); // battery tip
  // Fill level
  uint16_t fillColor = (pct > 0.2f) ? TFT_GREEN : TFT_RED;
  tft.fillRect(x + 2, y + 2, fillW, h - 4, fillColor);
  // Percentage text (to the left of the battery, right-aligned)
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  char buf[8];
  int percent = (int)(pct * 100.0f + 0.5f);
  if (percent > 100) percent = 100;
  if (percent < 0) percent = 0;
  snprintf(buf, sizeof(buf), "%d%%", percent);
  int textX = x - 2 - tft.textWidth(buf, 2);
  int textY = y + (h - 16) / 2;
  tft.drawString(buf, textX, textY, 2);
}

#define UPSCALE 8
const int INTERP_W = GRID_W * UPSCALE;
const int INTERP_H = GRID_H * UPSCALE;
float interpBuf[INTERP_W * INTERP_H];

// Button pins
#define BUTTON_INTERP 0   // GPIO0: short press = interpolation toggle, long press = power off
#define BUTTON_RANGE  35  // GPIO35: manual/auto range toggle

bool useInterpolation = true;
bool useManualRange = false;
float manualMin = 18.0f;
float manualMax = 30.0f;

const float AMBIENT_DEADBAND = 0.5f;
const float AUTO_RANGE_PAD = 0.1f;
const float MIN_AUTO_RANGE = 1.0f;
const float RANGE_EXPAND_RATE = 0.55f;
const float RANGE_CONTRACT_RATE = 0.18f;

bool frameAvailable = false;
float latestFrame[AMG88xx_PIXEL_ARRAY_SIZE];
float latestFrameMin = 0.0f;
float latestFrameMax = 0.0f;
float latestFrameMedian = 0.0f;
unsigned long latestFrameMillis = 0;
String deviceIp;

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ThermalCam Live Viewer</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: "Segoe UI", Tahoma, Geneva, Verdana, sans-serif;
    }
    body {
      margin: 0 auto;
      max-width: 960px;
      padding: 1.5rem;
      background: #111;
      color: #f0f0f0;
    }
    h1 {
      font-weight: 600;
      margin-bottom: 1rem;
      text-align: center;
    }
    .viewer {
      display: flex;
      flex-wrap: wrap;
      gap: 1.5rem;
      justify-content: center;
      align-items: flex-start;
    }
    canvas {
      border: 1px solid #2f2f2f;
      border-radius: 8px;
      width: min(90vw, 320px);
      height: min(90vw, 320px);
      image-rendering: pixelated;
      background: #000;
    }
    .panel {
      background: rgba(32, 32, 32, 0.8);
      border-radius: 12px;
      padding: 1rem 1.25rem;
      min-width: 280px;
      box-shadow: 0 10px 30px rgba(0, 0, 0, 0.35);
    }
    .stats {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
      gap: 0.75rem;
      margin-bottom: 1rem;
    }
    .stat {
      background: rgba(255, 255, 255, 0.04);
      border-radius: 8px;
      padding: 0.75rem;
      text-align: center;
    }
    .stat span {
      display: block;
      font-size: 1.35rem;
      font-weight: 600;
      margin-top: 0.25rem;
    }
    label {
      display: flex;
      flex-direction: column;
      gap: 0.35rem;
      margin-bottom: 1rem;
      font-size: 0.95rem;
    }
    input[type="range"] {
      width: 100%;
    }
    .toggle-row {
      display: flex;
      gap: 1rem;
      margin-bottom: 1rem;
      align-items: center;
      flex-wrap: wrap;
    }
    .toggle-row label {
      flex-direction: row;
      align-items: center;
      gap: 0.5rem;
      margin: 0;
    }
    .status {
      font-size: 0.9rem;
      opacity: 0.8;
    }
  </style>
</head>
<body>
  <h1>ThermalCam Live Viewer</h1>
  <div class="viewer">
    <canvas id="heatmap" width="320" height="320"></canvas>
    <div class="panel">
      <div class="stats">
        <div class="stat">Min<span id="stat-min">--</span></div>
        <div class="stat">Median<span id="stat-median">--</span></div>
        <div class="stat">Max<span id="stat-max">--</span></div>
      </div>
      <div class="toggle-row">
        <label><input type="checkbox" id="toggleManual"> Manual range</label>
        <label><input type="checkbox" id="toggleInterp"> Interpolation</label>
      </div>
      <label for="sliderMin">Manual minimum: <strong><span id="sliderMinValue">--</span> °C</strong>
        <input type="range" id="sliderMin" min="-20" max="120" step="0.1">
      </label>
      <label for="sliderMax">Manual maximum: <strong><span id="sliderMaxValue">--</span> °C</strong>
        <input type="range" id="sliderMax" min="-20" max="120" step="0.1">
      </label>
      <div class="status">IP address: <span id="ip-address">retrieving…</span></div>
      <div class="status">Last frame: <span id="last-frame">--</span></div>
    </div>
  </div>
  <script>
    const heatmap = document.getElementById('heatmap');
    const ctx = heatmap.getContext('2d');
    const statMin = document.getElementById('stat-min');
    const statMax = document.getElementById('stat-max');
    const statMedian = document.getElementById('stat-median');
    const sliderMin = document.getElementById('sliderMin');
    const sliderMax = document.getElementById('sliderMax');
    const sliderMinValue = document.getElementById('sliderMinValue');
    const sliderMaxValue = document.getElementById('sliderMaxValue');
    const toggleManual = document.getElementById('toggleManual');
    const toggleInterp = document.getElementById('toggleInterp');
    const ipAddress = document.getElementById('ip-address');
    const lastFrame = document.getElementById('last-frame');

    let pendingSettings = {};
    let pendingTimeout = null;
    let lastFrameReceived = 0;

    function queueSettingUpdate(key, value) {
      pendingSettings[key] = value;
      if (pendingTimeout) {
        return;
      }
      pendingTimeout = setTimeout(() => {
        fetch('/settings', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(pendingSettings)
        }).catch(console.error);
        pendingSettings = {};
        pendingTimeout = null;
      }, 150);
    }

    function updateFrameAge() {
      if (!lastFrameReceived) {
        lastFrame.textContent = '--';
        return;
      }
      const seconds = (Date.now() - lastFrameReceived) / 1000;
      if (seconds < 0.5) {
        lastFrame.textContent = 'just now';
      } else {
        lastFrame.textContent = `${seconds.toFixed(1)} s ago`;
      }
    }

    function gammaAdjust(ratio, gamma) {
      ratio = Math.min(Math.max(ratio, 0), 1);
      return Math.pow(ratio, gamma);
    }

    function thermalGradient(ratio) {
      ratio = Math.min(Math.max(ratio, 0), 1);
      let r = 0, g = 0, b = 0;
      if (ratio <= 0.25) {
        const f = ratio / 0.25;
        g = Math.round(255 * f);
        b = 255;
      } else if (ratio <= 0.5) {
        const f = (ratio - 0.25) / 0.25;
        g = 255;
        b = Math.round(255 * (1 - f));
      } else if (ratio <= 0.75) {
        const f = (ratio - 0.5) / 0.25;
        r = Math.round(255 * f);
        g = 255;
      } else {
        const f = (ratio - 0.75) / 0.25;
        r = 255;
        g = Math.round(255 * (1 - f));
      }
      return `rgb(${r}, ${g}, ${b})`;
    }

    function drawHeatmap(pixels, minT, maxT) {
      const cellW = heatmap.width / 8;
      const cellH = heatmap.height / 8;
      ctx.clearRect(0, 0, heatmap.width, heatmap.height);
      for (let y = 0; y < 8; y++) {
        for (let x = 0; x < 8; x++) {
          const temp = pixels[y * 8 + x];
          const ratio = (temp - minT) / (maxT - minT || 1);
          const color = thermalGradient(gammaAdjust(ratio, 0.6));
          ctx.fillStyle = color;
          ctx.fillRect(x * cellW, y * cellH, cellW, cellH);
        }
      }
    }

    function updateSliders(data) {
      if (document.activeElement !== sliderMin) {
        sliderMin.value = data.manualMin.toFixed(1);
      }
      if (document.activeElement !== sliderMax) {
        sliderMax.value = data.manualMax.toFixed(1);
      }
      sliderMinValue.textContent = Number(sliderMin.value).toFixed(1);
      sliderMaxValue.textContent = Number(sliderMax.value).toFixed(1);
      sliderMin.disabled = !data.useManualRange;
      sliderMax.disabled = !data.useManualRange;
      toggleManual.checked = data.useManualRange;
      toggleInterp.checked = data.useInterpolation;
    }

    function fetchFrame() {
      fetch('/frame')
        .then(r => r.json())
        .then(data => {
          if (!data.frameReady) {
            return;
          }
          drawHeatmap(data.pixels, data.tMin, data.tMax);
          statMin.textContent = `${data.tMin.toFixed(1)} °C`;
          statMax.textContent = `${data.tMax.toFixed(1)} °C`;
          statMedian.textContent = `${data.median.toFixed(1)} °C`;
          lastFrameReceived = Date.now();
          updateFrameAge();
          ipAddress.textContent = data.ip || ipAddress.textContent;
          updateSliders(data);
        })
        .catch(err => {
          console.error(err);
          lastFrameReceived = 0;
          lastFrame.textContent = 'disconnected';
        });
    }

    toggleManual.addEventListener('change', () => {
      queueSettingUpdate('useManualRange', toggleManual.checked);
      sliderMin.disabled = sliderMax.disabled = !toggleManual.checked;
    });

    toggleInterp.addEventListener('change', () => {
      queueSettingUpdate('useInterpolation', toggleInterp.checked);
    });

    sliderMin.addEventListener('input', () => {
      const minValue = Number(sliderMin.value);
      sliderMinValue.textContent = minValue.toFixed(1);
      queueSettingUpdate('manualMin', minValue);
    });

    sliderMax.addEventListener('input', () => {
      const maxValue = Number(sliderMax.value);
      sliderMaxValue.textContent = maxValue.toFixed(1);
      queueSettingUpdate('manualMax', maxValue);
    });

    function refreshSettings() {
      fetch('/settings')
        .then(r => r.json())
        .then(data => {
          updateSliders(data);
          ipAddress.textContent = data.ip || ipAddress.textContent;
        })
        .catch(console.error);
    }

    refreshSettings();
    fetchFrame();
    setInterval(fetchFrame, 400);
    setInterval(updateFrameAge, 250);
  </script>
</body>
</html>
)rawliteral";

void sendJson(const JsonDocument &doc) {
  String payload;
  serializeJson(doc, payload);
  server.send(200, "application/json", payload);
}

void sendJsonError(int code, const char *message) {
  StaticJsonDocument<128> doc;
  doc["error"] = message;
  String payload;
  serializeJson(doc, payload);
  server.send(code, "application/json", payload);
}

void handleIndex() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleFrame() {
  StaticJsonDocument<4096> doc;
  doc["frameReady"] = frameAvailable;
  doc["ip"] = deviceIp;
  doc["useInterpolation"] = useInterpolation;
  doc["useManualRange"] = useManualRange;
  doc["manualMin"] = manualMin;
  doc["manualMax"] = manualMax;
  if (frameAvailable) {
    doc["timestamp"] = latestFrameMillis;
    doc["tMin"] = latestFrameMin;
    doc["tMax"] = latestFrameMax;
    doc["median"] = latestFrameMedian;
    JsonArray arr = doc.createNestedArray("pixels");
    for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      arr.add(latestFrame[i]);
    }
  } else {
    doc["timestamp"] = 0;
    doc["tMin"] = manualMin;
    doc["tMax"] = manualMax;
    doc["median"] = 0;
    doc.createNestedArray("pixels");
  }
  sendJson(doc);
}

void handleSettingsGet() {
  StaticJsonDocument<256> doc;
  doc["manualMin"] = manualMin;
  doc["manualMax"] = manualMax;
  doc["useManualRange"] = useManualRange;
  doc["useInterpolation"] = useInterpolation;
  doc["ip"] = deviceIp;
  sendJson(doc);
}

void handleSettingsPost() {
  if (!server.hasArg("plain")) {
    sendJsonError(400, "Missing body");
    return;
  }

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    sendJsonError(400, "Invalid JSON");
    return;
  }

  bool rangeChanged = false;
  if (doc.containsKey("manualMin")) {
    manualMin = doc["manualMin"].as<float>();
    rangeChanged = true;
  }
  if (doc.containsKey("manualMax")) {
    manualMax = doc["manualMax"].as<float>();
    rangeChanged = true;
  }
  if (rangeChanged && manualMax - manualMin < 0.1f) {
    manualMax = manualMin + 0.1f;
  }

  if (doc.containsKey("useManualRange")) {
    bool newMode = doc["useManualRange"].as<bool>();
    if (newMode != useManualRange) {
      useManualRange = newMode;
      autoRangeInitialized = false;
    }
  }

  if (doc.containsKey("useInterpolation")) {
    useInterpolation = doc["useInterpolation"].as<bool>();
  }

  handleSettingsGet();
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

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
    autoRangeInitialized = false;
  }
  lastRangeState = rangeReading;
}

float computeMedian(float *arr, int n) {
  // Copy to temp array so we don't modify original
  float temp[AMG88xx_PIXEL_ARRAY_SIZE];
  memcpy(temp, arr, n * sizeof(float));
  std::sort(temp, temp + n);
  if (n % 2 == 0) {
    return 0.5f * (temp[n/2 - 1] + temp[n/2]);
  } else {
    return temp[n/2];
  }
}

void drawTempStats(float tMin, float tMax, float median) {
  char buf[32];

  // Position in bottom-left corner relative to current rotation
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  int textY = tft.height() - 45;  // roughly above bottom edge
  int textX = 5;

  snprintf(buf, sizeof(buf), "Min: %.1f", tMin);
  tft.drawString(buf, textX, textY, 2);

  snprintf(buf, sizeof(buf), "Max: %.1f", tMax);
  tft.drawString(buf, textX, textY + 15, 2);

  snprintf(buf, sizeof(buf), "Med: %.1f", median);
  tft.drawString(buf, textX, textY + 30, 2);
}

float suppressAmbientNoise(float value, float ambient) {
  float diff = value - ambient;
  if (diff > AMBIENT_DEADBAND) {
    return ambient + (diff - AMBIENT_DEADBAND);
  }
  if (diff < -AMBIENT_DEADBAND) {
    return ambient + (diff + AMBIENT_DEADBAND);
  }
  return ambient;
}

void computeDisplayRange(const float *data, int count, float *outMin, float *outMax) {
  if (count <= 0) {
    *outMin = 0.0f;
    *outMax = 1.0f;
    return;
  }

  float minVal = data[0];
  float maxVal = data[0];
  for (int i = 1; i < count; i++) {
    if (data[i] < minVal) minVal = data[i];
    if (data[i] > maxVal) maxVal = data[i];
  }

  if (maxVal - minVal < MIN_AUTO_RANGE) {
    float mid = 0.5f * (maxVal + minVal);
    minVal = mid - MIN_AUTO_RANGE * 0.5f;
    maxVal = mid + MIN_AUTO_RANGE * 0.5f;
  }

  *outMin = minVal - AUTO_RANGE_PAD;
  *outMax = maxVal + AUTO_RANGE_PAD;
}

void applyRangeSmoothing(float targetMin, float targetMax, float *outMin, float *outMax) {
  if (!autoRangeInitialized) {
    smoothRangeMin = targetMin;
    smoothRangeMax = targetMax;
    autoRangeInitialized = true;
  } else {
    float expandMinRate = (targetMin < smoothRangeMin) ? RANGE_EXPAND_RATE : RANGE_CONTRACT_RATE;
    float expandMaxRate = (targetMax > smoothRangeMax) ? RANGE_EXPAND_RATE : RANGE_CONTRACT_RATE;
    smoothRangeMin += (targetMin - smoothRangeMin) * expandMinRate;
    smoothRangeMax += (targetMax - smoothRangeMax) * expandMaxRate;

    if (smoothRangeMax - smoothRangeMin < MIN_AUTO_RANGE) {
      float mid = 0.5f * (smoothRangeMin + smoothRangeMax);
      smoothRangeMin = mid - MIN_AUTO_RANGE * 0.5f;
      smoothRangeMax = mid + MIN_AUTO_RANGE * 0.5f;
    }
  }

  *outMin = smoothRangeMin;
  *outMax = smoothRangeMax;
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

void drawHeatmap(const float *pixels, float tMin, float tMax) {
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

  analogReadResolution(12); // 12-bit ADC
  pinMode(BATTERY_PIN, INPUT);

  if (!amg.begin()) {
    tft.setTextColor(TFT_RED);
    tft.drawString("AMG8833 not found!", 10, 10, 2);
    while (1);
  }

  pinMode(BUTTON_INTERP, INPUT_PULLUP);
  pinMode(BUTTON_RANGE, INPUT_PULLUP);

  tft.setTextColor(TFT_WHITE);
  tft.drawString("Thermal Imager Init OK", 10, 10, 2);
  tft.drawString("Connecting WiFi...", 10, 30, 2);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    deviceIp = WiFi.localIP().toString();
    Serial.print("WiFi connected, IP: ");
    Serial.println(deviceIp);
    tft.fillRect(0, 30, SCREEN_W, 40, TFT_BLACK);
    tft.drawString("WiFi connected", 10, 30, 2);
    String ipText = "IP: " + deviceIp;
    tft.drawString(ipText.c_str(), 10, 50, 2);
  } else {
    deviceIp = "not connected";
    Serial.println("WiFi connection failed");
    tft.fillRect(0, 30, SCREEN_W, 40, TFT_BLACK);
    tft.drawString("WiFi failed", 10, 30, 2);
  }

  server.on("/", handleIndex);
  server.on("/frame", HTTP_GET, handleFrame);
  server.on("/settings", HTTP_GET, handleSettingsGet);
  server.on("/settings", HTTP_POST, handleSettingsPost);
  server.onNotFound(handleNotFound);
  server.begin();

  delay(1000);
  tft.fillScreen(TFT_BLACK);
}

void loop() {
  server.handleClient();

  if (WiFi.status() == WL_CONNECTED) {
    String currentIp = WiFi.localIP().toString();
    if (currentIp != deviceIp) {
      deviceIp = currentIp;
    }
  } else {
    deviceIp = "not connected";
  }

  amg.readPixels(pixels);

  float medianTemp = computeMedian(pixels, AMG88xx_PIXEL_ARRAY_SIZE);

  const float *framePixels = pixels;
  float tMin, tMax;
  if (useManualRange) {
    tMin = manualMin;
    tMax = manualMax;
  } else {
    for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      displayPixels[i] = suppressAmbientNoise(pixels[i], medianTemp);
    }
    float targetMin, targetMax;
    framePixels = displayPixels;
    computeDisplayRange(displayPixels, AMG88xx_PIXEL_ARRAY_SIZE, &targetMin, &targetMax);
    applyRangeSmoothing(targetMin, targetMax, &tMin, &tMax);
    framePixels = displayPixels;
    computeDisplayRange(displayPixels, AMG88xx_PIXEL_ARRAY_SIZE, &tMin, &tMax);
  }

  checkButtons();

  for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    latestFrame[i] = framePixels[i];
  }
  latestFrameMin = tMin;
  latestFrameMax = tMax;
  latestFrameMedian = medianTemp;
  latestFrameMillis = millis();
  frameAvailable = true;

  if (useInterpolation) {
    bilinearInterpolate(framePixels, interpBuf, GRID_W, GRID_H, UPSCALE);
    drawInterpolatedHeatmap(interpBuf, tMin, tMax);
  } else {
    drawHeatmap(framePixels, tMin, tMax);
  }

  // Save current rotation
  uint8_t prevRot = tft.getRotation();

  // Set rotation to match the image rotation for overlay drawing
  tft.setRotation(1);

  drawTempStats(tMin, tMax, medianTemp);

  // Draw battery indicator (always top-right in current orientation)
  float battV = readBatteryVoltage();
  int battW = 32, battH = 14;

  // Use TFT_eSPI's width/height for current rotation
  int battX = tft.width() - battW - 6;  // 6px margin from right
  int battY = 6;                        // 6px margin from top

  drawBatteryIndicator(battX, battY, battW, battH, battV);

  // Restore original rotation
  tft.setRotation(prevRot);

  for (int i = 0; i < 6; i++) {
    server.handleClient();
    delay(15);
  }
}
