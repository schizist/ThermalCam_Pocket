// Thermal Imager with Web UI and T-Display rendering
// Rebuilt clean to fix previous merge artifacts and compile errors

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <Adafruit_AMG88xx.h>
#include <TFT_eSPI.h>
#include <algorithm>
#include <cmath>
#include <ESP32Servo.h>
Servo servo;
const int SERVO_PIN = 27;

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "thermal123"
#endif

WebServer server(80);
TFT_eSPI tft = TFT_eSPI();
Adafruit_AMG88xx amg;

// Sensor grid
static const int GRID_W = 8;
static const int GRID_H = 8;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float displayPixels[AMG88xx_PIXEL_ARRAY_SIZE];
// PID control for servo
float kp = 70.0f; //Controls oscillation
float ki = 10.0f; //Controls steady state error
float kd = 40.0f; //Controls overshoot
bool useGrayscale = false;
bool motorEnabled = true;

// Screen size (T-Display)
#define SCREEN_W 240
#define SCREEN_H 135

// Battery voltage reading
#define BATTERY_PIN 34
#define BATTERY_VOLTAGE_DIVIDER 2.0f
#define BATTERY_MIN_V 3.2f
#define BATTERY_MAX_V 4.2f

static float readBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  float voltage = (raw / 4095.0f) * 3.3f * BATTERY_VOLTAGE_DIVIDER;
  return voltage;
}

static void drawBatteryIndicator(int x, int y, int w, int h, float voltage) {
  float pct = (voltage - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V);
  if (pct < 0) pct = 0; if (pct > 1) pct = 1;

  // Body
  tft.drawRect(x, y, w, h, TFT_WHITE);
  // Terminal nub
  int nubW = max(2, w / 8);
  int nubH = max(2, h / 3);
  tft.fillRect(x + w, y + (h - nubH) / 2, nubW, nubH, TFT_WHITE);

  // Fill
  int innerW = w - 4;
  int fillW = (int)roundf(innerW * pct);
  tft.fillRect(x + 2, y + 2, fillW, h - 4, (pct < 0.15f) ? TFT_RED : TFT_GREEN);

  // Text percentage
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  char buf[8];
  int percent = (int)(pct * 100.0f + 0.5f);
  if (percent > 100) percent = 100; if (percent < 0) percent = 0;
  snprintf(buf, sizeof(buf), "%d%%", percent);
  int textX = x - 2 - tft.textWidth(buf, 2);
  int textY = y + (h - 16) / 2;
  tft.drawString(buf, textX, textY, 2);
}

// Interpolation
#define UPSCALE 8
static const int INTERP_W = GRID_W * UPSCALE;
static const int INTERP_H = GRID_H * UPSCALE;
float interpBuf[INTERP_W * INTERP_H];

// Buttons
#define BUTTON_INTERP 0   // GPIO0
#define BUTTON_RANGE  35  // GPIO35

bool useInterpolation = true;
bool useManualRange = false;
float manualMin = 8.50f;
float manualMax = 30.50f;

// Runtime-tunable auto-range parameters
static const float DEFAULT_AMBIENT_DEADBAND = 0.1f;
static const float DEFAULT_AUTO_RANGE_PAD   = 0.01f;
static const float DEFAULT_MIN_AUTO_RANGE   = 10.0f;
static const float DEFAULT_RANGE_EXPAND_RATE   = 0.55f;
static const float DEFAULT_RANGE_CONTRACT_RATE = 0.50f;

float ambientDeadband = DEFAULT_AMBIENT_DEADBAND;
float autoRangePad = DEFAULT_AUTO_RANGE_PAD;
float minAutoRange = DEFAULT_MIN_AUTO_RANGE;
float rangeExpandRate = DEFAULT_RANGE_EXPAND_RATE;
float rangeContractRate = DEFAULT_RANGE_CONTRACT_RATE;

// Smoothing state
bool autoRangeInitialized = false;
float smoothRangeMin = 0.0f;
float smoothRangeMax = 0.0f;

// Stream cache for web
bool frameAvailable = false;
float latestFrame[AMG88xx_PIXEL_ARRAY_SIZE];
float latestFrameMin = 0.0f;
float latestFrameMax = 0.0f;
float latestFrameMedian = 0.0f;
unsigned long latestFrameMillis = 0;
String deviceIp;

// Web page
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ThermalCam Live Viewer</title>
  <style>
    :root { color-scheme: dark; font-family: Segoe UI, Arial, sans-serif; }
    body { margin:0 auto; max-width:960px; padding:1.5rem; background:#111; color:#f0f0f0; }
    h1 { font-weight:600; margin-bottom:1rem; text-align:center; }
    .viewer { display:flex; flex-wrap:wrap; gap:1.5rem; justify-content:center; align-items:flex-start; }
    canvas { border:1px solid #2f2f2f; border-radius:8px; width:min(90vw,320px); height:min(90vw,320px); image-rendering:pixelated; background:#000; }
    .panel { background:#222c; border-radius:12px; padding:1rem 1.25rem; min-width:280px; box-shadow:0 10px 30px #0006; }
    .stats { display:grid; grid-template-columns:repeat(auto-fit,minmax(120px,1fr)); gap:.75rem; margin-bottom:1rem; }
    .stat { background:#fff1; border-radius:8px; padding:.75rem; text-align:center; }
    .stat span { display:block; font-size:1.35rem; font-weight:600; margin-top:.25rem; }
    label { display:flex; flex-direction:column; gap:.35rem; margin-bottom:1rem; font-size:.95rem; }
    input[type=range] { width:100%; }
    .toggle-row { display:flex; gap:1rem; margin-bottom:1rem; align-items:center; flex-wrap:wrap; }
    .toggle-row label { flex-direction:row; align-items:center; gap:.5rem; margin:0; }
    .status { font-size:.9rem; opacity:.8; }
    .group-title { margin:1.25rem 0 .75rem; font-size:1rem; font-weight:600; letter-spacing:.02em; }
    .setting-grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(140px,1fr)); gap:.75rem; margin-bottom:.75rem; }
    input[type=number] { background:#1e1e1e; border:1px solid #333; border-radius:6px; color:#f0f0f0; padding:.45rem .5rem; }
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
      <label for="sliderMin">Manual minimum: <strong><span id="sliderMinValue">--</span> C</strong>
        <input type="range" id="sliderMin" min="-20" max="120" step="0.1">
      </label>
      <label for="sliderMax">Manual maximum: <strong><span id="sliderMaxValue">--</span> C</strong>
        <input type="range" id="sliderMax" min="-20" max="120" step="0.1">
      </label>
      <div class="group-title">Servo PID</div>
      <label for="kp">Kp: <strong><span id="kpVal">--</span></strong>
        <input type="range" id="kp" min="0" max="200" step="1">
      </label>
      <label for="ki">Ki: <strong><span id="kiVal">--</span></strong>
        <input type="range" id="ki" min="0" max="50" step="1">
      </label>
      <label for="kd">Kd: <strong><span id="kdVal">--</span></strong>
        <input type="range" id="kd" min="0" max="100" step="1">
      </label>
      <!-- Auto range tuning removed from UI -->
      <div class="status">IP address: <span id="ip-address">--</span></div>
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
    const kpSlider = document.getElementById('kp');
    const kiSlider = document.getElementById('ki');
    const kdSlider = document.getElementById('kd');
    const kpVal = document.getElementById('kpVal');
    const kiVal = document.getElementById('kiVal');
    const kdVal = document.getElementById('kdVal');

    function updatePIDControls(data) {
      kpSlider.value = data.kp;
      kiSlider.value = data.ki;
      kdSlider.value = data.kd;
      kpVal.textContent = data.kp.toFixed(1);
      kiVal.textContent = data.ki.toFixed(1);
      kdVal.textContent = data.kd.toFixed(1);
    }

    kpSlider.addEventListener('input', () => {
      kpVal.textContent = kpSlider.value;
      queueSettingUpdate('kp', parseFloat(kpSlider.value));
    });
    kiSlider.addEventListener('input', () => {
      kiVal.textContent = kiSlider.value;
      queueSettingUpdate('ki', parseFloat(kiSlider.value));
    });
    kdSlider.addEventListener('input', () => {
      kdVal.textContent = kdSlider.value;
      queueSettingUpdate('kd', parseFloat(kdSlider.value));
    });

  // Auto-range tuning controls removed from UI; keep defaults in firmware

    const GRID_SIZE = 8;
    const UPSCALE = 8;
    const INTERP_SIZE = GRID_SIZE * UPSCALE;
    const interpBuffer = new Float32Array(INTERP_SIZE * INTERP_SIZE);
    ctx.imageSmoothingEnabled = false;

    let pendingSettings = {};
    let pendingTimeout = null;
    let lastFrameReceived = 0;

    function queueSettingUpdate(key, value) {
      pendingSettings[key] = value;
      if (pendingTimeout) return;
      pendingTimeout = setTimeout(() => {
        fetch('/settings', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(pendingSettings) }).catch(console.error);
        pendingSettings = {}; pendingTimeout = null;
      }, 150);
    }

    function bindNumberInput(element, key, { min=-Infinity, max=Infinity, decimals=2 }={}) {
      element.addEventListener('change', () => {
        let value = parseFloat(element.value);
        if (Number.isNaN(value)) return;
        if (value < min) value = min; if (value > max) value = max;
        element.value = value.toFixed(decimals);
        queueSettingUpdate(key, value);
      });
    }

    function updateFrameAge() {
      if (!lastFrameReceived) { lastFrame.textContent = '--'; return; }
      const seconds = (Date.now() - lastFrameReceived) / 1000;
      lastFrame.textContent = seconds < 0.5 ? 'just now' : `${seconds.toFixed(1)} s ago`;
    }

    function thermalColor(ratio) {
      ratio = Math.max(0, Math.min(1, ratio));
      let r=0,g=0,b=0;
      if (ratio <= 0.25) { const f = ratio/0.25; g = Math.round(255*f); b=255; }
      else if (ratio <= 0.5) { const f=(ratio-0.25)/0.25; g=255; b=Math.round(255*(1-f)); }
      else if (ratio <= 0.75) { const f=(ratio-0.5)/0.25; r=Math.round(255*f); g=255; }
      else { const f=(ratio-0.75)/0.25; r=255; g=Math.round(255*(1-f)); }
      return `rgb(${r},${g},${b})`;
    }

    function bilinearInterpolate(src, srcW, srcH, scale, dst) {
      const dstW = srcW * scale, dstH = srcH * scale;
      for (let y=0; y<dstH; y++) {
        const gy = y/scale; const y0=Math.floor(gy), y1=Math.min(y0+1, srcH-1); const fy=gy-y0;
        for (let x=0; x<dstW; x++) {
          const gx = x/scale; const x0=Math.floor(gx), x1=Math.min(x0+1, srcW-1); const fx=gx-x0;
          const v00=src[y0*srcW+x0], v01=src[y0*srcW+x1], v10=src[y1*srcW+x0], v11=src[y1*srcW+x1];
          const v0=v00+fx*(v01-v00), v1=v10+fx*(v11-v10); dst[y*dstW+x]=v0+fy*(v1-v0);
        }
      }
      return dst;
    }

    function drawHeatmap(pixels, minT, maxT, useInterp) {
      const safeRange = (maxT - minT) || 1;
      ctx.clearRect(0, 0, heatmap.width, heatmap.height);
      if (useInterp) {
        const interp = bilinearInterpolate(pixels, GRID_SIZE, GRID_SIZE, UPSCALE, interpBuffer);
        const cellW = heatmap.width / INTERP_SIZE, cellH = heatmap.height / INTERP_SIZE;
        for (let y=0; y<INTERP_SIZE; y++) {
          for (let x=0; x<INTERP_SIZE; x++) {
            const temp = interp[y*INTERP_SIZE+x];
            const ratio = Math.pow((temp - minT) / safeRange, 0.6);
            ctx.fillStyle = thermalColor(ratio);
            ctx.fillRect(x*cellW, y*cellH, cellW, cellH);
          }
        }
      } else {
        const cellW = heatmap.width / GRID_SIZE, cellH = heatmap.height / GRID_SIZE;
        for (let y=0; y<GRID_SIZE; y++) {
          for (let x=0; x<GRID_SIZE; x++) {
            const temp = pixels[y*GRID_SIZE+x];
            const ratio = Math.pow((temp - minT) / safeRange, 0.6);
            ctx.fillStyle = thermalColor(ratio);
            ctx.fillRect(x*cellW, y*cellH, cellW, cellH);
          }
        }
      }
    }

    function updateControls(data) {
      if (document.activeElement !== sliderMin) sliderMin.value = data.manualMin.toFixed(1);
      if (document.activeElement !== sliderMax) sliderMax.value = data.manualMax.toFixed(1);
      sliderMinValue.textContent = Number(sliderMin.value).toFixed(1);
      sliderMaxValue.textContent = Number(sliderMax.value).toFixed(1);
      sliderMin.disabled = sliderMax.disabled = !data.useManualRange;
      toggleManual.checked = data.useManualRange;
      toggleInterp.checked = data.useInterpolation;
      updatePIDControls(data);
  // Auto-range tuning values are not exposed in the web UI.
    }

    function fetchFrame() {
      fetch('/frame').then(r => r.json()).then(data => {
        updateControls(data);
        if (data.ip) ipAddress.textContent = data.ip;
        if (!data.frameReady) return;
        drawHeatmap(data.pixels, data.tMin, data.tMax, data.useInterpolation);
        statMin.textContent = `${data.tMin.toFixed(1)} C`;
        statMax.textContent = `${data.tMax.toFixed(1)} C`;
        statMedian.textContent = `${data.median.toFixed(1)} C`;
        lastFrameReceived = Date.now();
        updateFrameAge();
      }).catch(err => { console.error(err); lastFrameReceived = 0; lastFrame.textContent = 'disconnected'; });
    }

    toggleManual.addEventListener('change', () => { queueSettingUpdate('useManualRange', toggleManual.checked); sliderMin.disabled = sliderMax.disabled = !toggleManual.checked; });
    toggleInterp.addEventListener('change', () => { queueSettingUpdate('useInterpolation', toggleInterp.checked); });
  // Auto-range tuning bindings removed.
    sliderMin.addEventListener('input', () => { const v=Number(sliderMin.value); sliderMinValue.textContent = v.toFixed(1); queueSettingUpdate('manualMin', v); });
    sliderMax.addEventListener('input', () => { const v=Number(sliderMax.value); sliderMaxValue.textContent = v.toFixed(1); queueSettingUpdate('manualMax', v); });

    (function init(){ fetch('/settings').then(r=>r.json()).then(updateControls).catch(console.error); fetchFrame(); setInterval(fetchFrame, 400); setInterval(updateFrameAge, 250); })();
  </script>
</body>
</html>
)rawliteral";

// JSON helpers
static void sendJson(const JsonDocument &doc) {
  String payload; serializeJson(doc, payload); server.send(200, "application/json", payload);
}

static void sendJsonError(int code, const char *message) {
  StaticJsonDocument<128> doc; doc["error"] = message; String payload; serializeJson(doc, payload); server.send(code, "application/json", payload);
}

static void handleIndex() { server.send_P(200, "text/html", INDEX_HTML); }

static void handleFrame() {
  StaticJsonDocument<4096> doc;
  doc["frameReady"] = frameAvailable;
  doc["ip"] = deviceIp;
  doc["useInterpolation"] = useInterpolation;
  doc["useManualRange"] = useManualRange;
  doc["manualMin"] = manualMin;
  doc["manualMax"] = manualMax;
  doc["kp"] = kp;
  doc["ki"] = ki;
  doc["kd"] = kd;
  if (frameAvailable) {
    doc["timestamp"] = latestFrameMillis;
    doc["tMin"] = latestFrameMin;
    doc["tMax"] = latestFrameMax;
    doc["median"] = latestFrameMedian;
    JsonArray arr = doc.createNestedArray("pixels");
    for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) arr.add(latestFrame[i]);
  } else {
    doc["timestamp"] = 0;
    doc["tMin"] = manualMin;
    doc["tMax"] = manualMax;
    doc["median"] = 0;
    doc.createNestedArray("pixels");
  }
  sendJson(doc);
}
static void handleSettingsGet() {
  StaticJsonDocument<256> doc;
  doc["manualMin"] = manualMin;
  doc["manualMax"] = manualMax;
  doc["useManualRange"] = useManualRange;
  doc["useInterpolation"] = useInterpolation;
  doc["kp"] = kp;
  doc["ki"] = ki;
  doc["kd"] = kd;
  doc["ip"] = deviceIp;
  sendJson(doc);
}

static void handleSettingsPost() {
  if (!server.hasArg("plain")) { sendJsonError(400, "Missing body"); return; }
  StaticJsonDocument<512> doc; DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) { sendJsonError(400, "Invalid JSON"); return; }

  if (doc.containsKey("manualMin")) manualMin = doc["manualMin"].as<float>();
  if (doc.containsKey("manualMax")) manualMax = doc["manualMax"].as<float>();
  if (doc.containsKey("useManualRange")) useManualRange = doc["useManualRange"].as<bool>();
  if (doc.containsKey("useInterpolation")) useInterpolation = doc["useInterpolation"].as<bool>();

  if (doc.containsKey("kp")) kp = doc["kp"].as<float>();
  if (doc.containsKey("ki")) ki = doc["ki"].as<float>();
  if (doc.containsKey("kd")) kd = doc["kd"].as<float>();

  handleSettingsGet();
}

static void handleNotFound() {
  // Redirect unknown requests to root to trigger captive portal behavior on clients
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "Redirecting to captive portal");
}

// Power control
static void powerOff() {
  tft.writecommand(ST7789_DISPOFF);
  tft.writecommand(ST7789_SLPIN);
  esp_deep_sleep_start();
}

// Color mapping
static float gammaAdjust(float ratio, float gamma) {
  if (ratio < 0) ratio = 0; if (ratio > 1) ratio = 1; return powf(ratio, gamma);
}
static uint16_t thermalGradient(float ratio) {
  if (ratio < 0) ratio = 0; if (ratio > 1) ratio = 1;
  uint8_t r=0,g=0,b=0;
  if (ratio <= 0.25f) { float f = ratio/0.25f; g = (uint8_t)(255*f); b=255; }
  else if (ratio <= 0.50f) { float f=(ratio-0.25f)/0.25f; g=255; b=(uint8_t)(255*(1-f)); }
  else if (ratio <= 0.75f) { float f=(ratio-0.50f)/0.25f; r=(uint8_t)(255*f); g=255; }
  else { float f=(ratio-0.75f)/0.25f; r=255; g=(uint8_t)(255*(1-f)); }
  return tft.color565(r,g,b);
}
static uint16_t tempToColor(float temp, float tMin, float tMax) {
  float ratio = (temp - tMin) / (tMax - tMin);
  ratio = gammaAdjust(ratio, 0.6f);

  if (useGrayscale) {
    uint8_t v = (uint8_t)(255 * ratio);
    return tft.color565(v, v, v);
  } else {
    return thermalGradient(ratio);
  }
}

// Buttons
bool lastInterpState = HIGH; unsigned long interpPressStart = 0;
bool lastRangeState = HIGH;  unsigned long lastRangeToggle = 0;

static void checkButtons() {
  int interpReading = digitalRead(BUTTON_INTERP);
  int rangeReading  = digitalRead(BUTTON_RANGE);

  // --- Interpolation button: short press toggles grayscale, long press toggles interpolation ---
  static unsigned long interpPressStart = 0;
  if (interpReading == LOW && lastInterpState == HIGH) interpPressStart = millis();
  if (interpReading == HIGH && lastInterpState == LOW) {
    unsigned long pressDuration = millis() - interpPressStart;
    if (pressDuration < 800) useGrayscale = !useGrayscale;  // short press = toggle grayscale
    else useInterpolation = !useInterpolation;              // long press = toggle interpolation
  }
  lastInterpState = interpReading;

  // --- Combined press: toggle motor function ---
  static unsigned long bothPressStart = 0;
  if (interpReading == LOW && rangeReading == LOW) {
    if (bothPressStart == 0) bothPressStart = millis();
    else if (millis() - bothPressStart > 800) {
      motorEnabled = !motorEnabled;
      bothPressStart = 0;
      tft.fillRect(0, tft.height() - 20, tft.width(), 20, TFT_BLACK);
      tft.setTextColor(motorEnabled ? TFT_GREEN : TFT_RED, TFT_BLACK);
      tft.drawString(motorEnabled ? "MOTOR: ON" : "MOTOR: OFF", 5, tft.height() - 20, 2);
      delay(500);
    }
  } else {
    bothPressStart = 0;
  }

  // --- Range button: long press to power off, short press toggles range ---
  static unsigned long rangePressStart = 0;
  if (rangeReading == LOW && lastRangeState == HIGH) rangePressStart = millis();
  if (rangeReading == HIGH && lastRangeState == LOW) {
    unsigned long pressDuration = millis() - rangePressStart;
    if (pressDuration < 2000) {
      // short press toggles manual range
      if (millis() - lastRangeToggle > 250) {
        useManualRange = !useManualRange;
        lastRangeToggle = millis();
        autoRangeInitialized = false;
      }
    }
  }
  if (rangeReading == LOW && millis() - rangePressStart > 2000) {
    // long press powers off
    powerOff();
  }
  lastRangeState = rangeReading;
}

// Stats and filtering
static float computeMedian(float *arr, int n) {
  float temp[AMG88xx_PIXEL_ARRAY_SIZE];
  memcpy(temp, arr, n * sizeof(float));
  std::sort(temp, temp + n);
  if (n % 2 == 0) return 0.5f * (temp[n/2 - 1] + temp[n/2]);
  return temp[n/2];
}

static void drawTempStats(float tMin, float tMax, float median) {
  char buf[32];
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  // Bottom row
  int y = tft.height() - 20;
  int x = 5;

  // Min
  snprintf(buf, sizeof(buf), "Min: %.1fC", tMin);
  tft.drawString(buf, x, y, 2);
  x += tft.textWidth(buf, 2) + 10;

  // Median
  snprintf(buf, sizeof(buf), "Med: %.1fC", median);
  tft.drawString(buf, x, y, 2);
  x += tft.textWidth(buf, 2) + 10;

  // Max
  snprintf(buf, sizeof(buf), "Max: %.1fC", tMax);
  tft.drawString(buf, x, y, 2);
}

static float suppressAmbientNoise(float value, float ambient) {
  float diff = value - ambient;
  if (diff > ambientDeadband) return ambient + (diff - ambientDeadband);
  if (diff < -ambientDeadband) return ambient + (diff + ambientDeadband);
  return ambient;
}

static void computeDisplayRange(const float *data, int count, float *outMin, float *outMax) {
  if (count <= 0) { *outMin = 0.0f; *outMax = 1.0f; return; }
  float minVal = data[0], maxVal = data[0];
  for (int i = 1; i < count; i++) { if (data[i] < minVal) minVal = data[i]; if (data[i] > maxVal) maxVal = data[i]; }
  if (maxVal - minVal < minAutoRange) {
    float mid = 0.5f * (maxVal + minVal);
    minVal = mid - minAutoRange * 0.5f; maxVal = mid + minAutoRange * 0.5f;
  }
  *outMin = minVal - autoRangePad; *outMax = maxVal + autoRangePad;
}

static void applyRangeSmoothing(float targetMin, float targetMax, float *outMin, float *outMax) {
  if (!autoRangeInitialized) { smoothRangeMin = targetMin; smoothRangeMax = targetMax; autoRangeInitialized = true; }
  else {
    float expandMinRate = (targetMin < smoothRangeMin) ? rangeExpandRate : rangeContractRate;
    float expandMaxRate = (targetMax > smoothRangeMax) ? rangeExpandRate : rangeContractRate;
    smoothRangeMin += (targetMin - smoothRangeMin) * expandMinRate;
    smoothRangeMax += (targetMax - smoothRangeMax) * expandMaxRate;
    if (smoothRangeMax - smoothRangeMin < minAutoRange) {
      float mid = 0.5f * (smoothRangeMin + smoothRangeMax);
      smoothRangeMin = mid - minAutoRange * 0.5f; smoothRangeMax = mid + minAutoRange * 0.5f;
    }
  }
  *outMin = smoothRangeMin; *outMax = smoothRangeMax;
}

// Drawing
static void drawInterpolatedHeatmap(const float *pix, float tMin, float tMax) {
  int W = tft.width(), H = tft.height();
  int cellW = (W + INTERP_W - 1) / INTERP_W;
  int cellH = (H + INTERP_H - 1) / INTERP_H;
  int gridWpx = cellW * INTERP_W, gridHpx = cellH * INTERP_H;
  int x0 = (W - gridWpx) / 2, y0 = (H - gridHpx) / 2;
  tft.startWrite();
  for (int y = 0; y < INTERP_H; y++) {
    for (int x = 0; x < INTERP_W; x++) {
      int i = y * INTERP_W + x; uint16_t c = tempToColor(pix[i], tMin, tMax);
      tft.fillRect(x0 + x * cellW, y0 + y * cellH, cellW, cellH, c);
    }
  }
  tft.endWrite();
}

static void bilinearInterpolate(const float *src, float *dst, int srcW, int srcH, int scale) {
  int dstW = srcW * scale, dstH = srcH * scale;
  for (int y = 0; y < dstH; y++) {
    float gy = (float)y / (float)scale; int y0 = (int)floorf(gy); int y1 = (y0 + 1 < srcH) ? y0 + 1 : srcH - 1; float fy = gy - (float)y0;
    for (int x = 0; x < dstW; x++) {
      float gx = (float)x / (float)scale; int x0 = (int)floorf(gx); int x1 = (x0 + 1 < srcW) ? x0 + 1 : srcW - 1; float fx = gx - (float)x0;
      float v00 = src[y0 * srcW + x0], v01 = src[y0 * srcW + x1], v10 = src[y1 * srcW + x0], v11 = src[y1 * srcW + x1];
      float v0 = v00 + fx * (v01 - v00), v1 = v10 + fx * (v11 - v10);
      dst[y * dstW + x] = v0 + fy * (v1 - v0);
    }
  }
}

static void drawHeatmap(const float *pix, float tMin, float tMax) {
  int W = tft.width(), H = tft.height();
  int cellW = (W + GRID_W - 1) / GRID_W;
  int cellH = (H + GRID_H - 1) / GRID_H;
  int gridWpx = cellW * GRID_W, gridHpx = cellH * GRID_H;
  int x0 = (W - gridWpx) / 2, y0 = (H - gridHpx) / 2;
  tft.startWrite();
  for (int y = 0; y < GRID_H; y++) {
    for (int x = 0; x < GRID_W; x++) {
      int i = y * GRID_W + x; uint16_t c = tempToColor(pix[i], tMin, tMax);
      tft.fillRect(x0 + x * cellW, y0 + y * cellH, cellW, cellH, c);
    }
  }
  tft.endWrite();
}

static void trackHottestPixel(const float *pixels) {
  static float integral = 0.0f;
  static int lastPulse = 1500;
  static float lastOffset = 0.0f;

  int hottestIndex = 0;
  float maxTemp = pixels[0];
  for (int i = 1; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (pixels[i] > maxTemp) {
      maxTemp = pixels[i];
      hottestIndex = i;
    }
  }

  int x = hottestIndex % GRID_W;
  int centerX = GRID_W / 2;
  float offset = (centerX - x) / (float)centerX;  // -1 to +1

  // Deadband
  if (fabs(offset) < 0.2f) {
    servo.writeMicroseconds(1500);
    integral = 0;
    return;
  }
  
  if (!motorEnabled) {
    servo.writeMicroseconds(1500);
    return;
  }
  
  integral += offset * 0.05f;               // small accumulation
  float derivative = offset - lastOffset;   // change rate
  lastOffset = offset;

  float control = kp * offset + ki * integral + kd * derivative;

  // smooth the step
  int pulse = 1500 + (int)control;
  pulse = constrain(pulse, 1000, 2000);
  pulse = (int)(0.7f * lastPulse + 0.3f * pulse);
  lastPulse = pulse;

  servo.writeMicroseconds(pulse);
}


// Main
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  analogReadResolution(12);
  pinMode(BATTERY_PIN, INPUT);

  if (!amg.begin()) {
    tft.setTextColor(TFT_RED); tft.drawString("AMG8833 not found!", 10, 10, 2); while (1) { delay(1000); }
  }

  pinMode(BUTTON_INTERP, INPUT_PULLUP);
  pinMode(BUTTON_RANGE, INPUT_PULLUP);

  tft.setTextColor(TFT_WHITE);
  tft.drawString("Thermal Imager Init OK", 10, 10, 2);
  tft.drawString("Starting Access Point...", 10, 30, 2);

  // Start as WiFi access point so clients can connect directly to the ESP
  WiFi.mode(WIFI_AP);
  // Build a short, simple SSID based on device MAC for uniqueness
  String baseSsid = "ThermalCam-";
  String initMac = WiFi.macAddress();
  String initSuffix;
  int initLen = initMac.length();
  if (initLen >= 5) initSuffix = initMac.substring(initLen - 5);
  else initSuffix = initMac;
  initSuffix.replace(":", "");
  String apSsid = baseSsid + initSuffix;
  // Explicitly set AP IP and subnet (common default)
  IPAddress apLocal(192,168,4,1);
  IPAddress apGateway(192,168,4,1);
  IPAddress apSubnet(255,255,255,0);
  WiFi.softAPConfig(apLocal, apGateway, apSubnet);
  bool apStarted = false;
  // Start AP on channel 1, not hidden
  #ifdef WIFI_PASSWORD
    if (strlen(WIFI_PASSWORD) > 0) apStarted = WiFi.softAP(apSsid.c_str(), WIFI_PASSWORD, 1, 0, 4);
    else apStarted = WiFi.softAP(apSsid.c_str(), NULL, 1, 0, 4);
  #else
    apStarted = WiFi.softAP(apSsid.c_str(), NULL, 1, 0, 4);
  #endif

  if (apStarted) {
    IPAddress apIP = WiFi.softAPIP(); deviceIp = apIP.toString();
    Serial.print("AP started, SSID: "); Serial.print(apSsid); Serial.print(" IP: "); Serial.println(deviceIp);
    // indicate whether AP is secured (password set) or open
    bool apSecured = false;
    #ifdef WIFI_PASSWORD
      if (strlen(WIFI_PASSWORD) > 0) apSecured = true;
    #endif
    Serial.print("AP mode: "); Serial.println(apSecured ? "WPA2-PSK" : "OPEN");
    tft.fillRect(0, 30, SCREEN_W, 40, TFT_BLACK);
    tft.drawString("AP started", 10, 30, 2);
    String ipText = String("AP: ") + deviceIp; tft.drawString(ipText.c_str(), 10, 50, 2);
    tft.drawString(apSecured ? "Secured" : "Open", 10, 70, 2);
    // Start mDNS responder so clients can access via hostname.local
    String mac = WiFi.softAPmacAddress();
  // create short suffix from MAC (last 5 chars) and remove colons
  String macSuffix;
  int len = mac.length();
  if (len >= 5) macSuffix = mac.substring(len - 5);
  else macSuffix = mac;
  macSuffix.replace(":", "");
  // sanitize SSID for hostname: keep alnum and replace others with '-'
  String host = apSsid;
  for (size_t i = 0; i < host.length(); i++) { char c = host.charAt(i); if (!isalnum(c)) host.setCharAt(i, '-'); }
  String mdnsName = host + "-" + macSuffix;
  // force lowercase
  for (size_t i = 0; i < mdnsName.length(); i++) mdnsName.setCharAt(i, tolower(mdnsName.charAt(i)));
    if (MDNS.begin("thermalcam")) {
    Serial.println("mDNS responder started: http://thermalcam.local/");
    
    } else {
      Serial.println("mDNS failed to start");
    }
  } else {
    deviceIp = "not started";
    Serial.println("Failed to start AP");
    tft.fillRect(0, 30, SCREEN_W, 40, TFT_BLACK);
    tft.drawString("AP failed", 10, 30, 2);
  }

  server.on("/", handleIndex);
  server.on("/frame", HTTP_GET, handleFrame);
  server.on("/settings", HTTP_GET, handleSettingsGet);
  server.on("/settings", HTTP_POST, handleSettingsPost);
  server.onNotFound(handleNotFound);
  server.begin();

  delay(500);
  tft.fillScreen(TFT_BLACK);
  servo.attach(SERVO_PIN);
servo.writeMicroseconds(1500);  // neutral stop

}

void loop() {
  server.handleClient();

  // For AP mode the softAP IP is static while AP is running
  deviceIp = WiFi.softAPIP().toString();

  amg.readPixels(pixels);
  float medianTemp = computeMedian(pixels, AMG88xx_PIXEL_ARRAY_SIZE);
  trackHottestPixel(pixels);

  const float *framePixels = pixels;
  float tMin, tMax;
  if (useManualRange) {
    tMin = manualMin; tMax = manualMax;
  } else {
    for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) displayPixels[i] = suppressAmbientNoise(pixels[i], medianTemp);
    float targetMin, targetMax; framePixels = displayPixels;
    computeDisplayRange(displayPixels, AMG88xx_PIXEL_ARRAY_SIZE, &targetMin, &targetMax);
    applyRangeSmoothing(targetMin, targetMax, &tMin, &tMax);
  }

  checkButtons();
  // Rotate for web feed (flip to match orientation)
  for (int y = 0; y < GRID_H; y++) {
    for (int x = 0; x < GRID_W; x++) {
      int srcIndex = y * GRID_W + x;
      int dstIndex = (GRID_H - 1 - y) * GRID_W + (GRID_W - 1 - x);
      latestFrame[dstIndex] = framePixels[srcIndex];
    }
  }
  latestFrameMin = tMin; latestFrameMax = tMax; latestFrameMedian = medianTemp; latestFrameMillis = millis(); frameAvailable = true;

  // Draw image
  if (useInterpolation) { bilinearInterpolate(framePixels, interpBuf, GRID_W, GRID_H, UPSCALE); drawInterpolatedHeatmap(interpBuf, tMin, tMax); }
  else { drawHeatmap(framePixels, tMin, tMax); }

  // Overlay: temporarily set rotation to draw UI consistently
  uint8_t prevRot = tft.getRotation(); tft.setRotation(3);
  drawTempStats(tMin, tMax, medianTemp);
  float battV = readBatteryVoltage(); int battW = 32, battH = 14; int battX = tft.width() - battW - 6; int battY = 6; drawBatteryIndicator(battX, battY, battW, battH, battV);
  tft.setRotation(prevRot);

  // Small pacing and allow web server to run
  delay(60);
  for (int i = 0; i < 6; i++) { server.handleClient(); delay(15); }
}