// Thermal Imager with Web UI and T-Display rendering
// v4: default mode = PERSON, improved button debounce

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
#include <SPIFFS.h>
#include <ArduinoOTA.h>

#ifndef OTA_PASSWORD
#define OTA_PASSWORD ""
#endif

Servo servo;
const int SERVO_PIN = 27;

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

WebServer server(80);
TFT_eSPI tft = TFT_eSPI();
Adafruit_AMG88xx amg;

// Sensor grid
static const int GRID_W = 8;
static const int GRID_H = 8;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float displayPixels[AMG88xx_PIXEL_ARRAY_SIZE];

// PID — kff removed; derivative is now computed on a low-pass filtered error
// to prevent noise-amplified kicks that cause overshoot.
// Defaults chosen conservatively; tune up Kp first via web UI.
float kp = 65.0f;
float ki =  3.0f;
float kd = 12.0f;

// Scan behaviour
// When no target is found the servo sweeps slowly across the full range.
// SCAN_SPEED_US is added to currentPulse each loop tick (~75 ms effective).
// Direction reverses at the pulse limits.
static const int   SCAN_SPEED_US   = 8;    // µs per tick — slow deliberate sweep
static const int   SCAN_MIN_PULSE  = 500;  // full 360° range lower bound
static const int   SCAN_MAX_PULSE  = 2500; // full 360° range upper bound
static int         scanDirection   = 1;    // +1 or -1

enum DisplayMode { MODE_THERMAL = 0, MODE_GRAYSCALE = 1, MODE_PERSON = 2 };
DisplayMode displayMode = MODE_PERSON;
int currentPulse = 1500;
bool motorEnabled = true;

// Dynamic human temp range — offset from median (ambient).
// A pixel is "human" if it is HUMAN_ABOVE_MIN..HUMAN_ABOVE_MAX °C above ambient.
static const float HUMAN_ABOVE_MIN = 1.5f;
static const float HUMAN_ABOVE_MAX = 12.0f;

// Screen size (T-Display)
#define SCREEN_W 240
#define SCREEN_H 135

// Battery
#define BATTERY_PIN             34
#define BATTERY_VOLTAGE_DIVIDER 2.0f
#define BATTERY_MIN_V           3.2f
#define BATTERY_MAX_V           4.2f

static float readBatteryVoltage() {
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) sum += analogReadMilliVolts(BATTERY_PIN);
  return (sum / 16000.0f) * BATTERY_VOLTAGE_DIVIDER;
}

static void drawBatteryIndicator(int x, int y, int w, int h, float voltage) {
  float pct = (voltage - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V);
  pct = constrain(pct, 0.0f, 1.0f);
  tft.drawRect(x, y, w, h, TFT_WHITE);
  int nubW = max(2, w / 8), nubH = max(2, h / 3);
  tft.fillRect(x + w, y + (h - nubH) / 2, nubW, nubH, TFT_WHITE);
  int fillW = (int)roundf((w - 4) * pct);
  tft.fillRect(x + 2, y + 2, fillW, h - 4, (pct < 0.15f) ? TFT_RED : TFT_GREEN);
  char buf[8];
  int percent = constrain((int)(pct * 100.0f + 0.5f), 0, 100);
  snprintf(buf, sizeof(buf), "%d%%", percent);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(buf, x - 2 - tft.textWidth(buf, 2), y + (h - 16) / 2, 2);
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
bool useManualRange   = false;
float manualMin =  8.50f;
float manualMax = 30.50f;

// Auto-range parameters
static const float DEFAULT_AMBIENT_DEADBAND    = 0.1f;
static const float DEFAULT_AUTO_RANGE_PAD      = 0.01f;
static const float DEFAULT_MIN_AUTO_RANGE      = 10.0f;
static const float DEFAULT_RANGE_EXPAND_RATE   = 0.55f;
static const float DEFAULT_RANGE_CONTRACT_RATE = 0.50f;

float ambientDeadband   = DEFAULT_AMBIENT_DEADBAND;
float autoRangePad      = DEFAULT_AUTO_RANGE_PAD;
float minAutoRange      = DEFAULT_MIN_AUTO_RANGE;
float rangeExpandRate   = DEFAULT_RANGE_EXPAND_RATE;
float rangeContractRate = DEFAULT_RANGE_CONTRACT_RATE;

bool  autoRangeInitialized = false;
float smoothRangeMin = 0.0f;
float smoothRangeMax = 0.0f;

// Web frame cache
bool          frameAvailable    = false;
float         latestFrame[AMG88xx_PIXEL_ARRAY_SIZE];
float         latestFrameMin    = 0.0f;
float         latestFrameMax    = 0.0f;
float         latestFrameMedian = 0.0f;
unsigned long latestFrameMillis = 0;
String        deviceIp;

// ─────────────────────────────────────────────
// Blob detection
// ─────────────────────────────────────────────
// Returns true and writes centroid of the largest connected blob of
// human-temperature pixels.  Uses a two-pass union-find on the 8×8 grid.
static bool getPersonCentroid(const float *pix, float ambientTemp,
                               float *outX, float *outY) {
  const float tMin = ambientTemp + HUMAN_ABOVE_MIN;
  const float tMax = ambientTemp + HUMAN_ABOVE_MAX;

  int label[AMG88xx_PIXEL_ARRAY_SIZE] = {0};
  int parent[AMG88xx_PIXEL_ARRAY_SIZE + 1];
  for (int i = 0; i <= AMG88xx_PIXEL_ARRAY_SIZE; i++) parent[i] = i;
  int nextLabel = 1;

  // Path-compressed find
  auto findRoot = [&](int x) -> int {
    while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
    return x;
  };
  auto unite = [&](int a, int b) {
    a = findRoot(a); b = findRoot(b);
    if (a != b) parent[b] = a;
  };

  // First pass: label + union
  for (int y = 0; y < GRID_H; y++) {
    for (int x = 0; x < GRID_W; x++) {
      int i = y * GRID_W + x;
      if (pix[i] < tMin || pix[i] > tMax) continue;
      int L = (x > 0)      ? label[i - 1]     : 0;
      int A = (y > 0)      ? label[i - GRID_W] : 0;
      if      (L && A) { label[i] = L; unite(L, A); }
      else if (L)      { label[i] = L; }
      else if (A)      { label[i] = A; }
      else             { label[i] = nextLabel++; }
    }
  }

  // Second pass: accumulate blob stats by root
  int   blobCount[AMG88xx_PIXEL_ARRAY_SIZE + 1] = {0};
  float blobSumX [AMG88xx_PIXEL_ARRAY_SIZE + 1] = {0};
  float blobSumY [AMG88xx_PIXEL_ARRAY_SIZE + 1] = {0};
  for (int y = 0; y < GRID_H; y++) {
    for (int x = 0; x < GRID_W; x++) {
      int i = y * GRID_W + x;
      if (!label[i]) continue;
      int root = findRoot(label[i]);
      blobCount[root]++;
      blobSumX[root] += x;
      blobSumY[root] += y;
    }
  }

  // Pick largest blob (≥2 pixels required)
  int bestLabel = 0, bestSize = 1;
  for (int l = 1; l < nextLabel; l++) {
    if (blobCount[l] > bestSize) { bestSize = blobCount[l]; bestLabel = l; }
  }
  if (!bestLabel) return false;

  *outX = blobSumX[bestLabel] / bestSize;
  *outY = blobSumY[bestLabel] / bestSize;
  return true;
}

// ─────────────────────────────────────────────
// Web page
// ─────────────────────────────────────────────
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
    .canvasWrap { display:inline-block; }
    canvas { border:1px solid #2f2f2f; border-radius:8px; display:block;
             width:min(90vw,320px); height:auto; aspect-ratio:16/9;
             image-rendering:pixelated; background:#000;
             transform:rotate(180deg); }
    .panel { background:#222c; border-radius:12px; padding:1rem 1.25rem;
             min-width:280px; box-shadow:0 10px 30px #0006; }
    .stats { display:grid; grid-template-columns:repeat(auto-fit,minmax(100px,1fr));
             gap:.75rem; margin-bottom:1rem; }
    .stat { background:#fff1; border-radius:8px; padding:.75rem; text-align:center; }
    .stat span { display:block; font-size:1.25rem; font-weight:600; margin-top:.25rem; }
    label { display:flex; flex-direction:column; gap:.35rem; margin-bottom:1rem; font-size:.95rem; }
    input[type=range] { width:100%; }
    .toggle-row { display:flex; gap:1rem; margin-bottom:1rem; align-items:center; flex-wrap:wrap; }
    .toggle-row label { flex-direction:row; align-items:center; gap:.5rem; margin:0; }
    .status { font-size:.9rem; opacity:.8; margin-bottom:.3rem; }
    .group-title { margin:1.25rem 0 .75rem; font-size:1rem; font-weight:600; letter-spacing:.02em; }
    .state-badge { display:inline-block; padding:.2rem .6rem; border-radius:6px;
                   font-size:.85rem; font-weight:600; margin-left:.5rem; }
    .state-track { background:#1a4a1a; color:#4f4; }
    .state-hold  { background:#4a3a00; color:#fc0; }
    .state-scan  { background:#2a1a4a; color:#a8f; }
    .mode-btn { background:#2a2a2a; border:1px solid #444; border-radius:6px; color:#ccc;
                padding:.35rem .75rem; cursor:pointer; font-size:.9rem; }
    .mode-btn.active { background:#0a3a5a; border-color:#08f; color:#8df; font-weight:600; }
    button.motor-btn { background:#2a2a2a; border:1px solid #444; border-radius:6px; color:#ccc;
                       padding:.35rem .75rem; cursor:pointer; font-size:.9rem; width:100%; }
    button.motor-btn.on { background:#1a4a1a; border-color:#2c2; color:#4f4; font-weight:600; }
  </style>
</head>
<body>
  <h1>ThermalCam Live Viewer</h1>
  <div class="viewer">
    <div class="canvasWrap">
      <canvas id="heatmap" width="320" height="180"></canvas>
    </div>
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
      <label>Manual minimum: <strong><span id="sliderMinValue">--</span> °C</strong>
        <input type="range" id="sliderMin" min="-20" max="120" step="0.1">
      </label>
      <label>Manual maximum: <strong><span id="sliderMaxValue">--</span> °C</strong>
        <input type="range" id="sliderMax" min="-20" max="120" step="0.1">
      </label>
      <div class="group-title">Servo PID</div>
      <label>Kp: <strong><span id="kpVal">--</span></strong>
        <input type="range" id="kp" min="0" max="200" step="1">
      </label>
      <label>Ki: <strong><span id="kiVal">--</span></strong>
        <input type="range" id="ki" min="0" max="50" step="0.5">
      </label>
      <label>Kd: <strong><span id="kdVal">--</span></strong>
        <input type="range" id="kd" min="0" max="100" step="1">
      </label>
      <div class="group-title">Display &amp; Motor</div>
      <div class="toggle-row">
        <button class="mode-btn" data-mode="0">Thermal</button>
        <button class="mode-btn" data-mode="1">Gray</button>
        <button class="mode-btn" data-mode="2">Person</button>
      </div>
      <button class="motor-btn" id="motorBtn">Motor: ON</button>
      <div class="status" style="margin-top:.75rem">State:
        <span id="tracker-state" class="state-badge state-scan">SCAN</span>
      </div>
      <div class="status">Target: <span id="target-pos">--</span></div>
      <div class="status">IP: <span id="ip-address">--</span></div>
      <div class="status">Last frame: <span id="last-frame">--</span></div>
    </div>
  </div>
  <script>
    const heatmap = document.getElementById('heatmap');
    const ctx = heatmap.getContext('2d');
    ctx.imageSmoothingEnabled = false;

    const GRID = 8, UP = 8, ISIZE = GRID * UP;
    const interpBuf = new Float32Array(ISIZE * ISIZE);

    let pending = {}, pendingTimer = null, lastRx = 0;

    function queue(key, val) {
      pending[key] = val;
      if (pendingTimer) return;
      pendingTimer = setTimeout(() => {
        fetch('/settings', { method:'POST',
          headers:{'Content-Type':'application/json'},
          body: JSON.stringify(pending) }).catch(console.error);
        pending = {}; pendingTimer = null;
      }, 150);
    }

    function bindSlider(id, valId, key, fmt) {
      const sl = document.getElementById(id);
      const vl = document.getElementById(valId);
      sl.addEventListener('input', () => { vl.textContent = fmt(sl.value); queue(key, parseFloat(sl.value)); });
      return sl;
    }

    const slMin = bindSlider('sliderMin','sliderMinValue','manualMin', v=>Number(v).toFixed(1));
    const slMax = bindSlider('sliderMax','sliderMaxValue','manualMax', v=>Number(v).toFixed(1));
    const slKp  = bindSlider('kp','kpVal','kp', v=>Number(v).toFixed(0));
    const slKi  = bindSlider('ki','kiVal','ki', v=>Number(v).toFixed(1));
    const slKd  = bindSlider('kd','kdVal','kd', v=>Number(v).toFixed(0));

    document.getElementById('toggleManual').addEventListener('change', e => {
      queue('useManualRange', e.target.checked);
      slMin.disabled = slMax.disabled = !e.target.checked;
    });
    document.getElementById('toggleInterp').addEventListener('change', e =>
      queue('useInterpolation', e.target.checked));

    const modeBtns = document.querySelectorAll('.mode-btn');
    const motorBtn = document.getElementById('motorBtn');
    let motorOn = true;

    modeBtns.forEach(btn => btn.addEventListener('click', () => {
      const m = parseInt(btn.dataset.mode);
      queue('displayMode', m);
      modeBtns.forEach(b => b.classList.toggle('active', parseInt(b.dataset.mode) === m));
    }));

    motorBtn.addEventListener('click', () => {
      motorOn = !motorOn;
      queue('motorEnabled', motorOn);
      motorBtn.textContent = motorOn ? 'Motor: ON' : 'Motor: OFF';
      motorBtn.classList.toggle('on', motorOn);
    });

    function applySettings(d) {
      const tm = document.getElementById('toggleManual');
      const ti = document.getElementById('toggleInterp');
      if (document.activeElement !== slMin) slMin.value = d.manualMin.toFixed(1);
      if (document.activeElement !== slMax) slMax.value = d.manualMax.toFixed(1);
      document.getElementById('sliderMinValue').textContent = Number(slMin.value).toFixed(1);
      document.getElementById('sliderMaxValue').textContent = Number(slMax.value).toFixed(1);
      slMin.disabled = slMax.disabled = !d.useManualRange;
      tm.checked = d.useManualRange;
      ti.checked = d.useInterpolation;
      if (document.activeElement !== slKp) { slKp.value = d.kp; document.getElementById('kpVal').textContent = d.kp.toFixed(0); }
      if (document.activeElement !== slKi) { slKi.value = d.ki; document.getElementById('kiVal').textContent = d.ki.toFixed(1); }
      if (document.activeElement !== slKd) { slKd.value = d.kd; document.getElementById('kdVal').textContent = d.kd.toFixed(0); }
      if (d.displayMode !== undefined) {
        modeBtns.forEach(b => b.classList.toggle('active', parseInt(b.dataset.mode) === d.displayMode));
      }
      if (d.motorEnabled !== undefined) {
        motorOn = d.motorEnabled;
        motorBtn.textContent = motorOn ? 'Motor: ON' : 'Motor: OFF';
        motorBtn.classList.toggle('on', motorOn);
      }
    }

    function thermalColor(r) {
      r = Math.max(0, Math.min(1, r));
      let R=0,G=0,B=0;
      if      (r<=.25) { const f=r/.25;      G=Math.round(255*f); B=255; }
      else if (r<=.50) { const f=(r-.25)/.25; G=255; B=Math.round(255*(1-f)); }
      else if (r<=.75) { const f=(r-.50)/.25; R=Math.round(255*f); G=255; }
      else             { const f=(r-.75)/.25; R=255; G=Math.round(255*(1-f)); }
      return `rgb(${R},${G},${B})`;
    }

    function bilinear(src, dst) {
      for (let y=0; y<ISIZE; y++) {
        const gy=y/UP, y0=Math.floor(gy), y1=Math.min(y0+1,GRID-1), fy=gy-y0;
        for (let x=0; x<ISIZE; x++) {
          const gx=x/UP, x0=Math.floor(gx), x1=Math.min(x0+1,GRID-1), fx=gx-x0;
          const v00=src[y0*GRID+x0], v01=src[y0*GRID+x1],
                v10=src[y1*GRID+x0], v11=src[y1*GRID+x1];
          dst[y*ISIZE+x] = (v00+fx*(v01-v00)) + fy*((v10+fx*(v11-v10))-(v00+fx*(v01-v00)));
        }
      }
    }

    function pixelColor(temp, minT, maxT, mode, ambient) {
      const rng = (maxT - minT) || 1;
      const ratio = Math.pow(Math.max(0, Math.min(1, (temp - minT) / rng)), 0.6);
      if (mode === 2) {
        // Person: white if within human-above-ambient band, else dark blue
        const lo = ambient + 1.5, hi = ambient + 12.0;
        if (temp >= lo && temp <= hi) return '#ffffff';
        const b = Math.round(30 * ratio);
        return `rgb(0,0,${b})`;
      }
      if (mode === 1) {
        const v = Math.round(255 * ratio);
        return `rgb(${v},${v},${v})`;
      }
      return thermalColor(ratio);
    }

    function drawHeatmap(pixels, minT, maxT, interp, mode, ambient) {
      mode = mode || 0; ambient = ambient || 0;
      ctx.clearRect(0, 0, 320, 180);
      if (interp) {
        bilinear(pixels, interpBuf);
        const cw = 320/ISIZE, ch = 180/ISIZE;
        for (let y=0; y<ISIZE; y++) for (let x=0; x<ISIZE; x++) {
          ctx.fillStyle = pixelColor(interpBuf[y*ISIZE+x], minT, maxT, mode, ambient);
          ctx.fillRect(x*cw, y*ch, cw, ch);
        }
      } else {
        const cw = 320/GRID, ch = 180/GRID;
        for (let y=0; y<GRID; y++) for (let x=0; x<GRID; x++) {
          ctx.fillStyle = pixelColor(pixels[y*GRID+x], minT, maxT, mode, ambient);
          ctx.fillRect(x*cw, y*ch, cw, ch);
        }
      }
    }

    // Tracking indicators drawn on top of the heatmap (same canvas).
    // drawHeatmap draws with a 180° canvas transform then restores, so overlay
    // draws in the unrotated canvas space and must use mirrored coordinates to
    // land on the same visual pixel as the rotated heatmap.
    function drawOverlay(d) {
      const cw = 320/GRID, ch = 180/GRID;
      ctx.lineWidth = 2;

      if (d.trackerState === 'SCAN') {
        const sx = 320 - (d.scanPulse - 500) / (2500 - 500) * 320;
        ctx.strokeStyle = 'rgba(160,128,255,0.8)';
        ctx.setLineDash([6, 4]);
        ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, 180); ctx.stroke();
        ctx.setLineDash([]);
        return;
      }

      ctx.setLineDash([]);
      if (!d.targetFound) return;

      const cx = 320 - (d.targetX + 0.5) * cw;
      const cy = 180 - (d.targetY + 0.5) * ch;
      const r  = cw * 0.85;
      ctx.strokeStyle = d.trackerState === 'HOLD' ? 'rgba(255,200,0,0.9)' : 'rgba(0,255,128,0.95)';
      ctx.beginPath(); ctx.arc(cx, cy, r, 0, 2*Math.PI); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(cx-r*1.6, cy); ctx.lineTo(cx+r*1.6, cy); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(cx, cy-r*1.6); ctx.lineTo(cx, cy+r*1.6); ctx.stroke();
    }

    function updateFrameAge() {
      if (!lastRx) { document.getElementById('last-frame').textContent='--'; return; }
      const s = (Date.now()-lastRx)/1000;
      document.getElementById('last-frame').textContent = s<0.5 ? 'just now' : `${s.toFixed(1)}s ago`;
    }

    const stateEl = document.getElementById('tracker-state');
    const stateClass = { TRACK:'state-track', HOLD:'state-hold', SCAN:'state-scan' };

    function fetchFrame() {
      fetch('/frame').then(r=>r.json()).then(d => {
        applySettings(d);
        if (d.ip) document.getElementById('ip-address').textContent = d.ip;
        if (!d.frameReady) return;
        drawHeatmap(d.pixels, d.tMin, d.tMax, d.useInterpolation, d.displayMode || 0, d.median || 0);
        drawOverlay(d);
        document.getElementById('stat-min').textContent    = `${d.tMin.toFixed(1)}°`;
        document.getElementById('stat-max').textContent    = `${d.tMax.toFixed(1)}°`;
        document.getElementById('stat-median').textContent = `${d.median.toFixed(1)}°`;
        document.getElementById('target-pos').textContent  =
          d.targetFound ? `(${d.targetX.toFixed(1)}, ${d.targetY.toFixed(1)})` : 'none';
        stateEl.textContent = d.trackerState || 'SCAN';
        stateEl.className = 'state-badge ' + (stateClass[d.trackerState] || 'state-scan');
        lastRx = Date.now();
        updateFrameAge();
      }).catch(e => {
        console.error(e);
        lastRx = 0;
        document.getElementById('last-frame').textContent = 'disconnected';
      });
    }

    // Pre-select Person mode (matches firmware default) before first settings fetch
    modeBtns.forEach(b => b.classList.toggle('active', b.dataset.mode === '2'));
    motorBtn.classList.add('on');

    fetch('/settings').then(r=>r.json()).then(applySettings).catch(console.error);
    fetchFrame();
    setInterval(fetchFrame, 400);
    setInterval(updateFrameAge, 250);
  </script>
</body>
</html>
)rawliteral";

// ─────────────────────────────────────────────
// Tracker state machine
// ─────────────────────────────────────────────
enum TrackerState { TS_SCAN = 0, TS_HOLD, TS_TRACK };
static TrackerState trackerState  = TS_SCAN;
static float        trackTargetX  = GRID_W / 2.0f;
static float        trackTargetY  = GRID_H / 2.0f;
static bool         trackFound    = false;
static int          trackLostFrames = 0;

// ─────────────────────────────────────────────
// JSON helpers
// ─────────────────────────────────────────────
static void sendJson(const JsonDocument &doc) {
  String p; serializeJson(doc, p); server.send(200, "application/json", p);
}
static void sendJsonError(int code, const char *msg) {
  JsonDocument doc; doc["error"] = msg;
  String p; serializeJson(doc, p); server.send(code, "application/json", p);
}
static void handleIndex() { server.send_P(200, "text/html", INDEX_HTML); }

static const char *trackerStateName() {
  switch (trackerState) {
    case TS_TRACK: return "TRACK";
    case TS_HOLD:  return "HOLD";
    default:       return "SCAN";
  }
}

static void handleFrame() {
  JsonDocument doc;
  doc["frameReady"]       = frameAvailable;
  doc["ip"]               = deviceIp;
  doc["useInterpolation"] = useInterpolation;
  doc["useManualRange"]   = useManualRange;
  doc["manualMin"]        = manualMin;
  doc["manualMax"]        = manualMax;
  doc["kp"]  = kp;
  doc["ki"]  = ki;
  doc["kd"]  = kd;
  doc["targetFound"]   = trackFound;
  doc["targetX"]       = trackTargetX;
  doc["targetY"]       = trackTargetY;
  doc["trackerState"]  = trackerStateName();
  doc["scanPulse"]     = currentPulse;
  doc["displayMode"]   = (int)displayMode;
  if (frameAvailable) {
    doc["timestamp"] = latestFrameMillis;
    doc["tMin"]      = latestFrameMin;
    doc["tMax"]      = latestFrameMax;
    doc["median"]    = latestFrameMedian;
    JsonArray arr = doc["pixels"].to<JsonArray>();
    for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) arr.add(latestFrame[i]);
  } else {
    doc["timestamp"] = 0;
    doc["tMin"]   = manualMin; doc["tMax"] = manualMax; doc["median"] = 0;
    doc["pixels"].to<JsonArray>();
  }
  sendJson(doc);
}

static void handleSettingsGet() {
  JsonDocument doc;
  doc["manualMin"]        = manualMin;
  doc["manualMax"]        = manualMax;
  doc["useManualRange"]   = useManualRange;
  doc["useInterpolation"] = useInterpolation;
  doc["kp"]  = kp;
  doc["ki"]  = ki;
  doc["kd"]  = kd;
  doc["ip"]  = deviceIp;
  doc["displayMode"]  = (int)displayMode;
  doc["motorEnabled"] = motorEnabled;
  sendJson(doc);
}

static void handleSettingsPost() {
  if (!server.hasArg("plain")) { sendJsonError(400, "Missing body"); return; }
  JsonDocument doc;
  if (deserializeJson(doc, server.arg("plain"))) { sendJsonError(400, "Invalid JSON"); return; }
  if (!doc["manualMin"].isNull())        manualMin        = doc["manualMin"].as<float>();
  if (!doc["manualMax"].isNull())        manualMax        = doc["manualMax"].as<float>();
  if (!doc["useManualRange"].isNull())   useManualRange   = doc["useManualRange"].as<bool>();
  if (!doc["useInterpolation"].isNull()) useInterpolation = doc["useInterpolation"].as<bool>();
  if (!doc["kp"].isNull()) kp = doc["kp"].as<float>();
  if (!doc["ki"].isNull()) ki = doc["ki"].as<float>();
  if (!doc["kd"].isNull()) kd = doc["kd"].as<float>();
  if (!doc["displayMode"].isNull())  displayMode  = (DisplayMode)doc["displayMode"].as<int>();
  if (!doc["motorEnabled"].isNull()) {
    motorEnabled = doc["motorEnabled"].as<bool>();
    if (!motorEnabled) servo.writeMicroseconds(1500);
  }
  handleSettingsGet();
}

static void handleNotFound() {
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "Redirecting to captive portal");
}

// ─────────────────────────────────────────────
// Power
// ─────────────────────────────────────────────
static void powerOff() {
  tft.writecommand(TFT_DISPOFF);
  tft.writecommand(0x10);  // SLPIN
  esp_deep_sleep_start();
}

// ─────────────────────────────────────────────
// Color mapping
// ─────────────────────────────────────────────
static uint16_t thermalGradient(float ratio) {
  ratio = constrain(ratio, 0.0f, 1.0f);
  uint8_t r=0,g=0,b=0;
  if      (ratio<=.25f){float f=ratio/.25f;        g=(uint8_t)(255*f); b=255;}
  else if (ratio<=.50f){float f=(ratio-.25f)/.25f; g=255; b=(uint8_t)(255*(1-f));}
  else if (ratio<=.75f){float f=(ratio-.50f)/.25f; r=(uint8_t)(255*f); g=255;}
  else                 {float f=(ratio-.75f)/.25f; r=255; g=(uint8_t)(255*(1-f));}
  return tft.color565(r,g,b);
}

static uint16_t tempToColor(float temp, float tMin, float tMax, float ambient) {
  float ratio = powf(constrain((temp-tMin)/(tMax-tMin),0.f,1.f), 0.6f);
  if (displayMode == MODE_PERSON) {
    float lo = ambient + HUMAN_ABOVE_MIN, hi = ambient + HUMAN_ABOVE_MAX;
    if (temp >= lo && temp <= hi) return TFT_WHITE;
    return tft.color565(0, 0, (uint8_t)(30*ratio));
  }
  if (displayMode == MODE_GRAYSCALE) {
    uint8_t v = (uint8_t)(255*ratio);
    return tft.color565(v,v,v);
  }
  return thermalGradient(ratio);
}

// ─────────────────────────────────────────────
// Buttons
// ─────────────────────────────────────────────
static const unsigned long LONG_MIN_MS = 1500;  // hold longer than this → long press

static void checkButtons() {
  unsigned long now = millis();
  if (now < 2000) return;  // GPIO0 is briefly LOW during ESP32 boot; ignore until stable

  // GPIO0 — short press: cycle display mode   long press: toggle motor
  {
    static bool prev      = false;
    static unsigned long pressAt  = 0;
    static bool longFired = false;

    bool cur = (digitalRead(BUTTON_INTERP) == LOW);

    if (cur && !prev) {                              // falling edge: pressed
      pressAt   = now;
      longFired = false;
      Serial.println("[BTN0] pressed");
    }
    if (cur && !longFired && (now - pressAt) > LONG_MIN_MS) {
      longFired    = true;
      motorEnabled = !motorEnabled;
      if (!motorEnabled) servo.writeMicroseconds(1500);
      Serial.printf("[BTN0] long → motor %s\n", motorEnabled ? "ON" : "OFF");
    }
    if (!cur && prev) {                              // rising edge: released
      if (!longFired) {
        displayMode = (DisplayMode)((displayMode + 1) % 3);
        Serial.printf("[BTN0] short → mode %d\n", (int)displayMode);
      }
    }
    prev = cur;
  }

  // GPIO35 — short press: toggle manual range   long press: power off
  {
    static bool prev      = false;
    static unsigned long pressAt  = 0;
    static bool longFired = false;

    bool cur = (digitalRead(BUTTON_RANGE) == LOW);

    if (cur && !prev) {
      pressAt   = now;
      longFired = false;
      Serial.println("[BTN35] pressed");
    }
    if (cur && !longFired && (now - pressAt) > LONG_MIN_MS) {
      longFired = true;
      powerOff();
    }
    if (!cur && prev) {
      if (!longFired) {
        useManualRange       = !useManualRange;
        autoRangeInitialized = false;
        Serial.printf("[BTN35] short → manualRange %d\n", (int)useManualRange);
      }
    }
    prev = cur;
  }
}

// ─────────────────────────────────────────────
// Stats / filtering
// ─────────────────────────────────────────────
static float computeMedian(float *arr, int n) {
  float tmp[AMG88xx_PIXEL_ARRAY_SIZE];
  memcpy(tmp, arr, n*sizeof(float));
  std::sort(tmp, tmp+n);
  return (n%2==0) ? 0.5f*(tmp[n/2-1]+tmp[n/2]) : tmp[n/2];
}

static void drawModeIndicator() {
  const char *lbl; uint16_t col;
  switch(displayMode){
    case MODE_GRAYSCALE: lbl="GRAY";    col=TFT_WHITE; break;
    case MODE_PERSON:    lbl="PERSON";  col=TFT_GREEN; break;
    default:             lbl="THERMAL"; col=TFT_CYAN;  break;
  }
  tft.setTextColor(col, TFT_BLACK);
  tft.drawString(lbl, 6, 6, 2);
  tft.setTextColor(motorEnabled ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.drawString(motorEnabled ? "MOT:ON " : "MOT:OFF", 6, 22, 1);
}

static void drawTempStats(float tMin, float tMax, float median) {
  char buf[32];
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  int y=tft.height()-20, x=5;
  snprintf(buf,sizeof(buf),"Min:%.1f",tMin); tft.drawString(buf,x,y,2); x+=tft.textWidth(buf,2)+8;
  snprintf(buf,sizeof(buf),"Med:%.1f",median); tft.drawString(buf,x,y,2); x+=tft.textWidth(buf,2)+8;
  snprintf(buf,sizeof(buf),"Max:%.1f",tMax); tft.drawString(buf,x,y,2);
}

static float suppressAmbientNoise(float value, float ambient) {
  float d = value - ambient;
  if (d >  ambientDeadband) return ambient + (d - ambientDeadband);
  if (d < -ambientDeadband) return ambient + (d + ambientDeadband);
  return ambient;
}

static void computeDisplayRange(const float *data, int n, float *oMin, float *oMax) {
  float mn=data[0], mx=data[0];
  for (int i=1;i<n;i++){if(data[i]<mn)mn=data[i];if(data[i]>mx)mx=data[i];}
  if (mx-mn < minAutoRange){float mid=0.5f*(mx+mn); mn=mid-minAutoRange*0.5f; mx=mid+minAutoRange*0.5f;}
  *oMin = mn - autoRangePad; *oMax = mx + autoRangePad;
}

static void applyRangeSmoothing(float tgMin, float tgMax, float *oMin, float *oMax) {
  if (!autoRangeInitialized){smoothRangeMin=tgMin;smoothRangeMax=tgMax;autoRangeInitialized=true;}
  else {
    smoothRangeMin += (tgMin-smoothRangeMin)*((tgMin<smoothRangeMin)?rangeExpandRate:rangeContractRate);
    smoothRangeMax += (tgMax-smoothRangeMax)*((tgMax>smoothRangeMax)?rangeExpandRate:rangeContractRate);
    if (smoothRangeMax-smoothRangeMin < minAutoRange){
      float mid=0.5f*(smoothRangeMin+smoothRangeMax);
      smoothRangeMin=mid-minAutoRange*0.5f; smoothRangeMax=mid+minAutoRange*0.5f;
    }
  }
  *oMin=smoothRangeMin; *oMax=smoothRangeMax;
}

// ─────────────────────────────────────────────
// Drawing
// ─────────────────────────────────────────────
static void bilinearInterpolate(const float *src, float *dst, int sW, int sH, int scale) {
  int dW=sW*scale, dH=sH*scale;
  for(int y=0;y<dH;y++){
    float gy=(float)y/scale; int y0=(int)floorf(gy),y1=(y0+1<sH)?y0+1:sH-1; float fy=gy-y0;
    for(int x=0;x<dW;x++){
      float gx=(float)x/scale; int x0=(int)floorf(gx),x1=(x0+1<sW)?x0+1:sW-1; float fx=gx-x0;
      float v0=src[y0*sW+x0]+fx*(src[y0*sW+x1]-src[y0*sW+x0]);
      float v1=src[y1*sW+x0]+fx*(src[y1*sW+x1]-src[y1*sW+x0]);
      dst[y*dW+x]=v0+fy*(v1-v0);
    }
  }
}

static void drawHeatmapTFT(const float *pix, float tMin, float tMax, float ambient,
                            int gW, int gH) {
  int W=tft.width(), H=tft.height();
  int cW=(W+gW-1)/gW, cH=(H+gH-1)/gH;
  int x0=(W-cW*gW)/2, y0=(H-cH*gH)/2;
  for(int y=0;y<gH;y++){
    tft.startWrite();
    for(int x=0;x<gW;x++)
      tft.fillRect(x0+x*cW, y0+y*cH, cW, cH,
                   tempToColor(pix[y*gW+x], tMin, tMax, ambient));
    tft.endWrite();
    checkButtons();  // poll inputs between rows so presses aren't missed during slow draws
  }
}

static void drawTrackingIndicator() {
  if (trackerState == TS_SCAN) {
    // Show a vertical dashed sweep line
    int W=tft.width(), H=tft.height();
    float frac = (float)(currentPulse - SCAN_MIN_PULSE) / (SCAN_MAX_PULSE - SCAN_MIN_PULSE);
    int sx = (int)(frac * W);
    for (int y=4; y<H-20; y+=6) tft.drawPixel(sx, y, TFT_MAGENTA);
    return;
  }
  if (!trackFound) return;
  int W=tft.width(), H=tft.height();
  int cW=(W+GRID_W-1)/GRID_W, cH=(H+GRID_H-1)/GRID_H;
  int x0=(W-cW*GRID_W)/2, y0=(H-cH*GRID_H)/2;
  int cx=x0+(int)(trackTargetX*cW)+cW/2;
  int cy=y0+(int)(trackTargetY*cH)+cH/2;
  int r=cW;
  uint16_t col = (trackerState==TS_HOLD) ? TFT_YELLOW : TFT_GREEN;
  tft.drawCircle(cx,cy,r,col);
  tft.drawFastHLine(cx-r*2,cy,r*4,col);
  tft.drawFastVLine(cx,cy-r*2,r*4,col);
}

// ─────────────────────────────────────────────
// Servo control
// ─────────────────────────────────────────────

// Scan: advance pulse in scanDirection, bounce at limits
static void doScan() {
  currentPulse += SCAN_SPEED_US * scanDirection;
  if (currentPulse >= SCAN_MAX_PULSE) { currentPulse = SCAN_MAX_PULSE; scanDirection = -1; }
  if (currentPulse <= SCAN_MIN_PULSE) { currentPulse = SCAN_MIN_PULSE; scanDirection =  1; }
  servo.writeMicroseconds(currentPulse);
}

// PID tracking toward a target grid X position.
// Key changes vs previous version:
//  - Derivative is computed on a low-pass filtered error, not raw frame-to-frame delta.
//    This prevents noisy 8x8 data from injecting large Kd spikes that cause overshoot.
//  - No feedforward term (it amplified overshoot on error-derivative).
//  - Integral bleeds on deadband entry rather than hard-zero.
//  - Pulse blend is lighter (more responsive).
static void doTrack(float targetX) {
  static float integral      = 0.0f;
  static int   lastPulse     = 1500;
  static float filteredError = 0.0f;  // low-pass filtered error for derivative

  const float centerX = (GRID_W - 1) / 2.0f;
  float offset = (centerX - targetX) / centerX;   // −1 .. +1, +ve = target left of centre

  // Deadband: hold still and bleed integral when nearly centred
  if (fabsf(offset) < 0.08f) {
    servo.writeMicroseconds(1500);
    integral      *= 0.85f;
    filteredError  = offset;   // keep filter in sync so derivative is 0 on re-engage
    lastPulse      = 1500;
    return;
  }

  // Low-pass filter the error for the derivative term (alpha = 0.4)
  // Smooths out pixel-noise jitter without slowing the proportional response.
  const float DERIV_ALPHA = 0.4f;
  float prevFiltered = filteredError;
  filteredError = DERIV_ALPHA * offset + (1.0f - DERIV_ALPHA) * filteredError;
  float derivative = filteredError - prevFiltered;

  // Integral with anti-windup clamp
  integral += offset * 0.05f;
  integral  = constrain(integral, -3.0f, 3.0f);

  float control = kp * offset + ki * integral + kd * derivative;

  int pulse = 1500 + (int)control;
  pulse = constrain(pulse, SCAN_MIN_PULSE, SCAN_MAX_PULSE);

  // Light output smoothing — damps electrical noise, not tracking lag
  pulse     = (int)(0.80f * pulse + 0.20f * lastPulse);
  lastPulse  = pulse;
  currentPulse = pulse;
  servo.writeMicroseconds(pulse);
}

// Main tracking entry point — runs the state machine
static void updateServo(const float *pix, float ambientTemp) {
  if (!motorEnabled) { servo.writeMicroseconds(1500); return; }

  float blobX, blobY;
  bool blobFound = false;

  if (displayMode == MODE_PERSON) {
    blobFound = getPersonCentroid(pix, ambientTemp, &blobX, &blobY);
  } else {
    // Hottest pixel for non-person modes
    int best = 0; float maxT = pix[0];
    for (int i=1; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) if(pix[i]>maxT){maxT=pix[i];best=i;}
    blobX = best % GRID_W;
    blobY = best / GRID_W;
    blobFound = true;
  }

  switch (trackerState) {

    case TS_SCAN:
      if (blobFound) {
        trackTargetX    = blobX;
        trackTargetY    = blobY;
        trackFound      = true;
        trackLostFrames = 0;
        trackerState    = TS_TRACK;
        doTrack(blobX);
      } else {
        trackFound = false;
        doScan();
      }
      break;

    case TS_TRACK:
      if (blobFound) {
        trackTargetX    = blobX;
        trackTargetY    = blobY;
        trackFound      = true;
        trackLostFrames = 0;
        doTrack(blobX);
      } else {
        // Lost target — hold position for up to 10 frames before scanning
        trackLostFrames++;
        if (trackLostFrames <= 10) {
          trackerState = TS_HOLD;
          doTrack(trackTargetX);   // keep servo aimed at last known position
        } else {
          trackerState = TS_SCAN;
          trackFound   = false;
        }
      }
      break;

    case TS_HOLD:
      if (blobFound) {
        trackTargetX    = blobX;
        trackTargetY    = blobY;
        trackFound      = true;
        trackLostFrames = 0;
        trackerState    = TS_TRACK;
        doTrack(blobX);
      } else {
        trackLostFrames++;
        if (trackLostFrames > 10) {
          trackerState = TS_SCAN;
          trackFound   = false;
        } else {
          doTrack(trackTargetX);
        }
      }
      break;
  }
}

// ─────────────────────────────────────────────
// Logging
// ─────────────────────────────────────────────
static void logFrame(float tMin, float tMedian, float tMax, int servoPulse) {
  fs::File f = SPIFFS.open("/thermal_log.csv", FILE_APPEND);
  if (!f) return;
  f.printf("%lu,%.2f,%.2f,%.2f,%d,%.1f,%.1f,%.1f,%s\n",
           millis(), tMin, tMedian, tMax, servoPulse, kp, ki, kd, trackerStateName());
  f.close();
}

// ─────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  if (!SPIFFS.begin(true)) Serial.println("SPIFFS mount failed");
  else                     Serial.println("SPIFFS ready");

  Wire.begin(21, 22);
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  displayMode = MODE_PERSON;
  analogReadResolution(12);
  pinMode(BATTERY_PIN, INPUT);

  if (!amg.begin()) {
    tft.setTextColor(TFT_RED);
    tft.drawString("AMG8833 not found!", 10, 10, 2);
    while(1) delay(1000);
  }
  pinMode(BUTTON_INTERP, INPUT_PULLUP);
  pinMode(BUTTON_RANGE,  INPUT_PULLUP);

  tft.setTextColor(TFT_WHITE);
  tft.drawString("Thermal Imager Init OK", 10, 10, 2);
  tft.drawString("Connecting to WiFi...", 10, 30, 2);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    deviceIp = WiFi.localIP().toString();
    Serial.printf("WiFi connected  IP: %s\n", deviceIp.c_str());
    tft.fillRect(0, 30, SCREEN_W, 60, TFT_BLACK);
    tft.drawString(deviceIp, 10, 30, 2);
    tft.drawString("thermalcam.local", 10, 50, 2);
    if (MDNS.begin("thermalcam")) Serial.println("mDNS: http://thermalcam.local/");
  } else {
    deviceIp = "no wifi";
    Serial.println("WiFi connection failed");
    tft.fillRect(0, 30, SCREEN_W, 20, TFT_BLACK);
    tft.setTextColor(TFT_RED);
    tft.drawString("WiFi failed!", 10, 30, 2);
    tft.setTextColor(TFT_WHITE);
  }

  server.on("/",         handleIndex);
  server.on("/frame",    HTTP_GET,  handleFrame);
  server.on("/settings", HTTP_GET,  handleSettingsGet);
  server.on("/settings", HTTP_POST, handleSettingsPost);
  server.onNotFound(handleNotFound);
  server.begin();

  // OTA — password from .env via OTA_PASSWORD define
  ArduinoOTA.setHostname("thermalcam");
  if (strlen(OTA_PASSWORD) > 0) ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.drawString("OTA update starting...", 10, 10, 2);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    char buf[32];
    snprintf(buf, sizeof(buf), "OTA: %u%%", progress * 100 / total);
    tft.fillRect(0, 30, SCREEN_W, 20, TFT_BLACK);
    tft.setTextColor(TFT_GREEN);
    tft.drawString(buf, 10, 30, 2);
  });
  ArduinoOTA.onError([](ota_error_t) {
    tft.fillRect(0, 50, SCREEN_W, 20, TFT_BLACK);
    tft.setTextColor(TFT_RED);
    tft.drawString("OTA error!", 10, 50, 2);
  });
  ArduinoOTA.begin();

  delay(500);
  tft.fillScreen(TFT_BLACK);
  servo.attach(SERVO_PIN);
  servo.writeMicroseconds(1500);
}

// ─────────────────────────────────────────────
// Loop
// ─────────────────────────────────────────────
void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  if (WiFi.status() == WL_CONNECTED) deviceIp = WiFi.localIP().toString();

  amg.readPixels(pixels);
  float medianTemp = computeMedian(pixels, AMG88xx_PIXEL_ARRAY_SIZE);

  updateServo(pixels, medianTemp);

  // Build display buffer
  const float *framePixels = pixels;
  float tMin, tMax;
  if (useManualRange) {
    tMin = manualMin; tMax = manualMax;
  } else {
    for (int i=0;i<AMG88xx_PIXEL_ARRAY_SIZE;i++)
      displayPixels[i] = suppressAmbientNoise(pixels[i], medianTemp);
    framePixels = displayPixels;
    float tgMin, tgMax;
    computeDisplayRange(displayPixels, AMG88xx_PIXEL_ARRAY_SIZE, &tgMin, &tgMax);
    applyRangeSmoothing(tgMin, tgMax, &tMin, &tMax);
  }

  checkButtons();

  // Mirror frame for web feed
  for (int y=0;y<GRID_H;y++)
    for (int x=0;x<GRID_W;x++)
      latestFrame[(GRID_H-1-y)*GRID_W+(GRID_W-1-x)] = framePixels[y*GRID_W+x];
  latestFrameMin=tMin; latestFrameMax=tMax;
  latestFrameMedian=medianTemp; latestFrameMillis=millis(); frameAvailable=true;

  // Draw
  if (useInterpolation) {
    bilinearInterpolate(framePixels, interpBuf, GRID_W, GRID_H, UPSCALE);
    drawHeatmapTFT(interpBuf, tMin, tMax, medianTemp, INTERP_W, INTERP_H);
  } else {
    drawHeatmapTFT(framePixels, tMin, tMax, medianTemp, GRID_W, GRID_H);
  }

  uint8_t prevRot = tft.getRotation();
  tft.setRotation(3);
  drawModeIndicator();
  drawTempStats(tMin, tMax, medianTemp);
  drawTrackingIndicator();
  drawBatteryIndicator(tft.width()-38, 6, 32, 14, readBatteryVoltage());
  tft.setRotation(prevRot);

  // Poll buttons and web server frequently so short taps (<150 ms) aren't missed
  for (int i = 0; i < 10; i++) { checkButtons(); server.handleClient(); delay(15); }
  logFrame(latestFrameMin, latestFrameMedian, latestFrameMax, currentPulse);
}
