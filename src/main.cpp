// Doll Head Thermal Tracker v2
// 90g servo on GPIO13 for pan rotation (thermal tracking)
// 90g servo on GPIO32 for eyelid blink
// AMG8833 on I2C GPIO21/22
// WiFi + OTA + web UI with interpolation and display modes

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <Adafruit_AMG88xx.h>
#include <ArduinoOTA.h>
#include <algorithm>
#include <cmath>

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif
#ifndef OTA_PASSWORD
#define OTA_PASSWORD ""
#endif

// ─────────────────────────────────────────────
// Pin definitions
// ─────────────────────────────────────────────
#define ROTATE_SERVO_PIN 13
#define BLINK_SERVO_PIN  32

// ─────────────────────────────────────────────
// Servo PWM — direct LEDC (Arduino 3.x native API)
// ESP32Servo bypasses the peripheral manager and corrupts LEDC state
// across soft resets; driving LEDC directly avoids that entirely.
// ─────────────────────────────────────────────
static const uint32_t SERVO_FREQ       = 50;
static const uint8_t  SERVO_RESOLUTION = 16;
static const float    SERVO_MIN_US     = 500.0f;
static const float    SERVO_MAX_US     = 2500.0f;
static const float    SERVO_PERIOD_US  = 20000.0f;

static void servoWrite(uint8_t pin, float angleDeg) {
  angleDeg = constrain(angleDeg, 0.0f, 180.0f);
  float pulseUs = SERVO_MIN_US + (angleDeg / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
  uint32_t duty = (uint32_t)(pulseUs / SERVO_PERIOD_US * (float)((1u << SERVO_RESOLUTION) - 1));
  ledcWrite(pin, duty);
}

// ─────────────────────────────────────────────
// 90g servo — pan rotation (thermal tracking)
// ─────────────────────────────────────────────
static float rotateAngle   = 90.0f;
static float rotateSpeed   = 1.0f;
static const float ROTATE_MIN_DEG = 10.0f;
static const float ROTATE_MAX_DEG = 170.0f;

// ─────────────────────────────────────────────
// 90g servo — eyelid blink
// ─────────────────────────────────────────────

static int           EYELID_OPEN_DEG   = 10;   // open/resting position
static int           EYELID_CLOSED_DEG = 45;  // closed position (~120° swing)
static unsigned long blinkIntervalMs   = 4000; // ms between blinks
static unsigned long blinkDurationMs   = 150;  // ms lid stays closed
// Time to allow servo to physically travel before advancing state
static const unsigned long SERVO_SETTLE_MS = 400;

enum BlinkPhase { BP_IDLE, BP_CLOSING, BP_CLOSED, BP_OPENING };
static BlinkPhase    blinkPhase   = BP_IDLE;
static unsigned long blinkPhaseMs = 0;
static bool          triggerBlinkFlag = false;
static int           blinkCount = 0;

static bool          debugMode              = false;
static unsigned long normalBlinkIntervalMs  = 60000UL;  // 1 min default in normal mode
static const unsigned long BLINK_NORMAL_MIN_MS = 30000UL;
static const unsigned long BLINK_NORMAL_MAX_MS = 86400000UL;

static bool          manualPanActive        = false;
static unsigned long lastManualPanMs        = 0;
static const unsigned long MANUAL_PAN_TIMEOUT_MS = 20000;

static void startBlink() {
  if (blinkPhase != BP_IDLE) return;
  blinkCount++;
  Serial.printf("[BLINK #%d]\n", blinkCount);
  servoWrite(BLINK_SERVO_PIN, EYELID_CLOSED_DEG);
  blinkPhase   = BP_CLOSING;
  blinkPhaseMs = millis();
}

static void updateBlink() {
  unsigned long now = millis();

  unsigned long activeBlinkInterval = debugMode ? blinkIntervalMs : normalBlinkIntervalMs;
  if (blinkPhase == BP_IDLE && (now - blinkPhaseMs >= activeBlinkInterval)) {
    startBlink();
    return;
  }

  switch (blinkPhase) {
    case BP_IDLE: break;

    case BP_CLOSING:
      if (now - blinkPhaseMs >= SERVO_SETTLE_MS) {
        blinkPhase   = BP_CLOSED;
        blinkPhaseMs = now;
      }
      break;

    case BP_CLOSED:
      if (now - blinkPhaseMs >= blinkDurationMs) {
        servoWrite(BLINK_SERVO_PIN, EYELID_OPEN_DEG);
        blinkPhase   = BP_OPENING;
        blinkPhaseMs = now;
      }
      break;

    case BP_OPENING:
      // Wait for travel, then go idle — servo holds open position quietly
      if (now - blinkPhaseMs >= SERVO_SETTLE_MS) {
        blinkPhase   = BP_IDLE;
        blinkPhaseMs = now;
      }
      break;
  }
}

// ─────────────────────────────────────────────
// AMG8833
// ─────────────────────────────────────────────
Adafruit_AMG88xx amg;
static const int GRID_W = 8;
static const int GRID_H = 8;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

static const float HUMAN_ABOVE_MIN = 1.5f;
static const float HUMAN_ABOVE_MAX = 12.0f;

// ─────────────────────────────────────────────
// Display / range settings (for web UI)
// ─────────────────────────────────────────────
enum DisplayMode { MODE_THERMAL = 0, MODE_GRAYSCALE = 1, MODE_PERSON = 2 };
DisplayMode displayMode = MODE_PERSON;

bool  useInterpolation = true;
bool  useManualRange   = false;
float manualMin        = 18.0f;
float manualMax        = 35.0f;

// Auto-range smoothing
static bool  autoRangeInitialized = false;
static float smoothRangeMin = 0.0f;
static float smoothRangeMax = 0.0f;
static const float RANGE_EXPAND_RATE   = 0.55f;
static const float RANGE_CONTRACT_RATE = 0.50f;
static const float MIN_AUTO_RANGE      = 10.0f;

static void applyRangeSmoothing(float tgMin, float tgMax, float *oMin, float *oMax) {
  if (!autoRangeInitialized) { smoothRangeMin=tgMin; smoothRangeMax=tgMax; autoRangeInitialized=true; }
  else {
    smoothRangeMin += (tgMin-smoothRangeMin)*((tgMin<smoothRangeMin)?RANGE_EXPAND_RATE:RANGE_CONTRACT_RATE);
    smoothRangeMax += (tgMax-smoothRangeMax)*((tgMax>smoothRangeMax)?RANGE_EXPAND_RATE:RANGE_CONTRACT_RATE);
    if (smoothRangeMax-smoothRangeMin < MIN_AUTO_RANGE) {
      float mid=0.5f*(smoothRangeMin+smoothRangeMax);
      smoothRangeMin=mid-MIN_AUTO_RANGE*0.5f; smoothRangeMax=mid+MIN_AUTO_RANGE*0.5f;
    }
  }
  *oMin=smoothRangeMin; *oMax=smoothRangeMax;
}

// ─────────────────────────────────────────────
// Blob detection
// ─────────────────────────────────────────────
static bool getPersonCentroid(const float *pix, float ambientTemp,
                               float *outX, float *outY) {
  const float tMin = ambientTemp + HUMAN_ABOVE_MIN;
  const float tMax = ambientTemp + HUMAN_ABOVE_MAX;
  int label[AMG88xx_PIXEL_ARRAY_SIZE] = {0};
  int parent[AMG88xx_PIXEL_ARRAY_SIZE + 1];
  for (int i = 0; i <= AMG88xx_PIXEL_ARRAY_SIZE; i++) parent[i] = i;
  int nextLabel = 1;
  auto findRoot = [&](int x) -> int {
    while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x;
  };
  auto unite = [&](int a, int b) { a=findRoot(a); b=findRoot(b); if(a!=b) parent[b]=a; };
  for (int y = 0; y < GRID_H; y++) {
    for (int x = 0; x < GRID_W; x++) {
      int i = y*GRID_W+x;
      if (pix[i] < tMin || pix[i] > tMax) continue;
      int L=(x>0)?label[i-1]:0, A=(y>0)?label[i-GRID_W]:0;
      if      (L&&A){ label[i]=L; unite(L,A); }
      else if (L)   { label[i]=L; }
      else if (A)   { label[i]=A; }
      else          { label[i]=nextLabel++; }
    }
  }
  int   blobCount[AMG88xx_PIXEL_ARRAY_SIZE+1]={0};
  float blobSumX [AMG88xx_PIXEL_ARRAY_SIZE+1]={0};
  float blobSumY [AMG88xx_PIXEL_ARRAY_SIZE+1]={0};
  for (int y=0;y<GRID_H;y++) for (int x=0;x<GRID_W;x++) {
    int i=y*GRID_W+x; if(!label[i]) continue;
    int root=findRoot(label[i]); blobCount[root]++; blobSumX[root]+=x; blobSumY[root]+=y;
  }
  int bestLabel=0, bestSize=1;
  for (int l=1;l<nextLabel;l++) if(blobCount[l]>bestSize){bestSize=blobCount[l];bestLabel=l;}
  if (!bestLabel) return false;
  *outX=blobSumX[bestLabel]/bestSize; *outY=blobSumY[bestLabel]/bestSize;
  return true;
}

// ─────────────────────────────────────────────
// Tracker state machine
// ─────────────────────────────────────────────
enum TrackerState { TS_SCAN=0, TS_HOLD, TS_TRACK };
static TrackerState trackerState    = TS_SCAN;
static float        trackTargetX    = GRID_W/2.0f;
static float        trackTargetY    = GRID_H/2.0f;
static bool         trackFound      = false;
static int          trackLostFrames = 0;
bool                motorEnabled    = true;

static int scanDirection = 1;

static const char *trackerStateName() {
  switch(trackerState){ case TS_TRACK: return "TRACK"; case TS_HOLD: return "HOLD"; default: return "SCAN"; }
}

static void doScan() {
  rotateAngle += rotateSpeed * scanDirection;
  if (rotateAngle >= ROTATE_MAX_DEG) { rotateAngle = ROTATE_MAX_DEG; scanDirection = -1; }
  if (rotateAngle <= ROTATE_MIN_DEG) { rotateAngle = ROTATE_MIN_DEG; scanDirection =  1; }
  servoWrite(ROTATE_SERVO_PIN, rotateAngle);
}

static void doTrack(float targetX) {
  const float centerX = (GRID_W - 1) / 2.0f;
  float offset = (centerX - targetX) / centerX;  // −1 to +1
  if (fabsf(offset) < 0.10f) return;
  rotateAngle -= offset * rotateSpeed * 3.0f;
  rotateAngle = constrain(rotateAngle, ROTATE_MIN_DEG, ROTATE_MAX_DEG);
  servoWrite(ROTATE_SERVO_PIN, rotateAngle);
}

static void updateStepper(const float *pix, float ambientTemp) {
  if (!motorEnabled) return;
  if (manualPanActive) {
    if (millis() - lastManualPanMs >= MANUAL_PAN_TIMEOUT_MS) {
      manualPanActive = false;
      trackerState = TS_SCAN;
      Serial.println("[PAN] Manual timeout — resuming auto");
    } else {
      return;
    }
  }
  static unsigned long lastServoMs = 0;
  unsigned long now = millis();
  if (now - lastServoMs < 100) return;
  lastServoMs = now;
  float blobX,blobY;
  bool blobFound=getPersonCentroid(pix,ambientTemp,&blobX,&blobY);
  TrackerState prevState = trackerState;
  switch(trackerState){
    case TS_SCAN:
      if(blobFound){ trackTargetX=blobX; trackTargetY=blobY; trackFound=true; trackLostFrames=0; trackerState=TS_TRACK; doTrack(blobX); }
      else{ trackFound=false; doScan(); }
      break;
    case TS_TRACK:
      if(blobFound){ trackTargetX=blobX; trackTargetY=blobY; trackFound=true; trackLostFrames=0; doTrack(blobX); }
      else{ trackLostFrames++; if(trackLostFrames<=10){trackerState=TS_HOLD;doTrack(trackTargetX);}else{trackerState=TS_SCAN;trackFound=false;} }
      break;
    case TS_HOLD:
      if(blobFound){ trackTargetX=blobX; trackTargetY=blobY; trackFound=true; trackLostFrames=0; trackerState=TS_TRACK; doTrack(blobX); }
      else{ trackLostFrames++; if(trackLostFrames>10){trackerState=TS_SCAN;trackFound=false;}else{doTrack(trackTargetX);} }
      break;
  }
  if (trackerState != prevState) {
    auto sname = [](TrackerState s){ return s==TS_TRACK?"TRACK":s==TS_HOLD?"HOLD":"SCAN"; };
    Serial.printf("[TRACK] %s->%s blob=(%.1f,%.1f) angle=%.1f blinks=%d\n",
      sname(prevState), sname(trackerState),
      blobFound?blobX:-1.0f, blobFound?blobY:-1.0f,
      rotateAngle, blinkCount);
    Serial.flush();
  }
}

// ─────────────────────────────────────────────
// Stats + interpolation
// ─────────────────────────────────────────────
static float computeMedian(float *arr, int n) {
  float tmp[AMG88xx_PIXEL_ARRAY_SIZE];
  memcpy(tmp,arr,n*sizeof(float));
  std::sort(tmp,tmp+n);
  return (n%2==0)?0.5f*(tmp[n/2-1]+tmp[n/2]):tmp[n/2];
}

#define UPSCALE 8
static const int INTERP_W = GRID_W * UPSCALE;
static const int INTERP_H = GRID_H * UPSCALE;
float interpBuf[INTERP_W * INTERP_H];

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

// ─────────────────────────────────────────────
// Web server
// ─────────────────────────────────────────────
WebServer server(80);
bool          frameAvailable    = false;
float         latestFrame[AMG88xx_PIXEL_ARRAY_SIZE];
float         latestInterpFrame[INTERP_W * INTERP_H];
float         latestFrameMin    = 0.0f;
float         latestFrameMax    = 0.0f;
float         latestFrameMedian = 0.0f;
unsigned long latestFrameMillis = 0;
String        deviceIp;

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Doll Head Tracker</title>
  <style>
    :root { color-scheme: dark; font-family: Segoe UI, Arial, sans-serif; }
    body { margin:0 auto; max-width:960px; padding:1.5rem; background:#111; color:#f0f0f0; }
    h1 { font-weight:600; margin-bottom:1rem; text-align:center; }
    .viewer { display:flex; flex-wrap:wrap; gap:1.5rem; justify-content:center; align-items:flex-start; }
    .canvasWrap { display:inline-block; }
    canvas { border:1px solid #2f2f2f; border-radius:8px; display:block;
             width:min(90vw,320px); height:auto; aspect-ratio:1/1;
             image-rendering:pixelated; background:#000; }
    .panel { background:#222c; border-radius:12px; padding:1rem 1.25rem;
             min-width:280px; box-shadow:0 10px 30px #0006; }
    .stats { display:grid; grid-template-columns:repeat(3,1fr); gap:.75rem; margin-bottom:1rem; }
    .stat { background:#fff1; border-radius:8px; padding:.75rem; text-align:center; }
    .stat span { display:block; font-size:1.25rem; font-weight:600; margin-top:.25rem; }
    label { display:flex; flex-direction:column; gap:.35rem; margin-bottom:1rem; font-size:.95rem; }
    input[type=range] { width:100%; }
    .toggle-row { display:flex; gap:1rem; margin-bottom:1rem; align-items:center; flex-wrap:wrap; }
    .toggle-row label { flex-direction:row; align-items:center; gap:.5rem; margin:0; }
    .group-title { margin:1.25rem 0 .75rem; font-size:1rem; font-weight:600; letter-spacing:.02em; }
    .state-badge { display:inline-block; padding:.2rem .6rem; border-radius:6px;
                   font-size:.85rem; font-weight:600; margin-left:.5rem; }
    .state-track { background:#1a4a1a; color:#4f4; }
    .state-hold  { background:#4a3a00; color:#fc0; }
    .state-scan  { background:#2a1a4a; color:#a8f; }
    .mode-btn { background:#2a2a2a; border:1px solid #444; border-radius:6px; color:#ccc;
                padding:.35rem .75rem; cursor:pointer; font-size:.9rem; }
    .mode-btn.active { background:#0a3a5a; border-color:#08f; color:#8df; font-weight:600; }
    button.tog { background:#2a2a2a; border:1px solid #444; border-radius:6px; color:#ccc;
                 padding:.35rem .75rem; cursor:pointer; font-size:.9rem; width:100%; margin-bottom:.5rem; }
    button.tog.on { background:#1a4a1a; border-color:#2c2; color:#4f4; font-weight:600; }
    .status { font-size:.9rem; opacity:.8; margin-bottom:.3rem; }
    .pan-row { display:flex; align-items:center; gap:.75rem; margin:.5rem 0; }
    .pan-btn { background:#2a2a2a; border:1px solid #444; border-radius:8px; color:#ccc;
               font-size:1.4rem; padding:.4rem .9rem; cursor:pointer; flex:0 0 auto;
               user-select:none; -webkit-user-select:none; touch-action:none; }
    .pan-btn:active { background:#0a3a5a; border-color:#08f; }
    .pan-info { flex:1; text-align:center; font-size:.9rem; color:#aaa; }
  </style>
</head>
<body>
  <h1>👁 Doll Head Tracker</h1>
  <div class="viewer">
    <div class="canvasWrap">
      <canvas id="heatmap" width="320" height="320"></canvas>
    </div>
    <div class="panel">
      <div class="stats">
        <div class="stat">Min<span id="stat-min">--</span></div>
        <div class="stat">Median<span id="stat-median">--</span></div>
        <div class="stat">Max<span id="stat-max">--</span></div>
      </div>

      <div class="toggle-row">
        <label><input type="checkbox" id="toggleManual"> Manual range</label>
        <label><input type="checkbox" id="toggleInterp" checked> Interpolation</label>
      </div>
      <label>Manual min: <strong><span id="sliderMinValue">--</span> °C</strong>
        <input type="range" id="sliderMin" min="-20" max="120" step="0.1">
      </label>
      <label>Manual max: <strong><span id="sliderMaxValue">--</span> °C</strong>
        <input type="range" id="sliderMax" min="-20" max="120" step="0.1">
      </label>

      <div class="group-title">Display Mode</div>
      <div class="toggle-row">
        <button class="mode-btn" data-mode="0">Thermal</button>
        <button class="mode-btn" data-mode="1">Gray</button>
        <button class="mode-btn active" data-mode="2">Person</button>
      </div>

      <div class="group-title">Motor</div>
      <button class="tog on" id="motorBtn">Motor: ON</button>
      <div class="pan-row">
        <button class="pan-btn" id="panLeft">&#9664;</button>
        <div class="pan-info"><span id="panAngle">--&deg;</span></div>
        <button class="pan-btn" id="panRight">&#9654;</button>
      </div>
      <div id="panResumeRow" class="status" style="display:none">Auto resumes in <span id="panCountdown">20</span>s</div>

      <div class="group-title">Blink Servo</div>
      <div class="toggle-row">
        <label><input type="checkbox" id="toggleDebug"> Debug blink mode</label>
      </div>
      <div id="normalBlinkSection">
        <div class="status">Cooldown: <strong id="blinkCooldownDisplay">--</strong></div>
        <label>Interval: <strong><span id="normalIntervalVal">--</span></strong>
          <input type="range" id="normalBlinkInterval" min="0" max="100" step="1">
        </label>
      </div>
      <div id="debugBlinkSection" style="display:none">
        <label>Interval (ms): <strong><span id="intervalVal">--</span></strong>
          <input type="range" id="blinkInterval" min="500" max="10000" step="100">
        </label>
      </div>
      <label>Blink duration (ms): <strong><span id="durationVal">--</span></strong>
        <input type="range" id="blinkDuration" min="50" max="500" step="10">
      </label>
      <label>Open angle (&deg;): <strong><span id="openVal">--</span></strong>
        <input type="range" id="openAngle" min="0" max="180" step="1">
      </label>
      <label>Closed angle (&deg;): <strong><span id="closedVal">--</span></strong>
        <input type="range" id="closedAngle" min="0" max="180" step="1">
      </label>
      <button class="tog" id="blinkNowBtn">Blink Now</button>

      <div class="group-title">Rotate Servo</div>
      <label>Scan speed (°/step): <strong><span id="rotateSpeedVal">--</span></strong>
        <input type="range" id="rotateSpeed" min="0.5" max="10" step="0.5">
      </label>

      <div class="group-title">Status</div>
      <div class="status">State: <span id="tracker-state" class="state-badge state-scan">SCAN</span></div>
      <div class="status">Target: <span id="target-pos">--</span></div>
      <div class="status">IP: <span id="ip-address">--</span></div>
      <div class="status">Last frame: <span id="last-frame">--</span></div>
    </div>
  </div>
  <script>
    const canvas = document.getElementById('heatmap');
    const ctx = canvas.getContext('2d');
    ctx.imageSmoothingEnabled = false;
    const GRID = 8, UP = 8, ISIZE = GRID * UP;
    const interpBuf = new Float32Array(ISIZE * ISIZE);
    let lastRx = 0, pending = {}, pendingTimer = null;

    function queue(key, val) {
      pending[key] = val;
      if (pendingTimer) return;
      pendingTimer = setTimeout(() => {
        fetch('/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(pending)}).catch(console.error);
        pending={}; pendingTimer=null;
      }, 150);
    }

    function bindSlider(id, valId, key, fmt) {
      const sl=document.getElementById(id), vl=document.getElementById(valId);
      sl.addEventListener('input',()=>{ vl.textContent=fmt(sl.value); queue(key,parseFloat(sl.value)); });
      return sl;
    }

    const slMin      = bindSlider('sliderMin',    'sliderMinValue', 'manualMin',     v=>Number(v).toFixed(1));
    const slMax      = bindSlider('sliderMax',    'sliderMaxValue', 'manualMax',     v=>Number(v).toFixed(1));
    const slOpen     = bindSlider('openAngle',    'openVal',        'eyeOpen',       v=>v);
    const slClosed   = bindSlider('closedAngle',  'closedVal',      'eyeClosed',     v=>v);
    const slInterval = bindSlider('blinkInterval','intervalVal',    'blinkInterval', v=>v);
    const slDuration = bindSlider('blinkDuration','durationVal',    'blinkDuration', v=>v);
    const slRotSpd   = bindSlider('rotateSpeed',  'rotateSpeedVal', 'rotateSpeed',   v=>parseFloat(v).toFixed(1));

    document.getElementById('toggleManual').addEventListener('change', e=>{
      queue('useManualRange', e.target.checked);
      slMin.disabled = slMax.disabled = !e.target.checked;
    });
    document.getElementById('toggleInterp').addEventListener('change', e=>queue('useInterpolation', e.target.checked));

    const modeBtns = document.querySelectorAll('.mode-btn');
    modeBtns.forEach(btn=>btn.addEventListener('click',()=>{
      const m=parseInt(btn.dataset.mode);
      queue('displayMode',m);
      modeBtns.forEach(b=>b.classList.toggle('active',parseInt(b.dataset.mode)===m));
    }));

    const motorBtn=document.getElementById('motorBtn');
    let motorOn=true;
    motorBtn.addEventListener('click',()=>{
      motorOn=!motorOn; queue('motorEnabled',motorOn);
      motorBtn.textContent=motorOn?'Motor: ON':'Motor: OFF';
      motorBtn.classList.toggle('on',motorOn);
    });

    document.getElementById('blinkNowBtn').addEventListener('click',()=>queue('triggerBlink',true));

    // Log-scale normal blink interval (30s – 86400s)
    const LOG_MIN = Math.log(30), LOG_MAX = Math.log(86400);
    function sliderToSec(v) { return Math.round(Math.exp(LOG_MIN + (v/100)*(LOG_MAX-LOG_MIN))); }
    function secToSlider(s) { return Math.round(((Math.log(Math.max(30,Math.min(86400,s)))-LOG_MIN)/(LOG_MAX-LOG_MIN))*100); }
    function fmtSec(s) {
      if (s<60) return `${s}s`;
      const m=Math.floor(s/60),r=s%60;
      if (s<3600) return r?`${m}m ${r}s`:`${m}m`;
      const h=Math.floor(s/3600),hm=Math.floor((s%3600)/60);
      if (s<86400) return hm?`${h}hr ${hm}m`:`${h}hr`;
      return '24hr';
    }
    const slNormal = document.getElementById('normalBlinkInterval');
    slNormal.addEventListener('input', ()=>{
      const secs=sliderToSec(slNormal.value);
      document.getElementById('normalIntervalVal').textContent=fmtSec(secs);
      document.getElementById('blinkCooldownDisplay').textContent=fmtSec(secs);
      queue('normalBlinkInterval', secs*1000);
    });

    // Debug mode toggle
    let isDebugMode = false;
    document.getElementById('toggleDebug').addEventListener('change', e=>{
      isDebugMode=e.target.checked;
      document.getElementById('debugBlinkSection').style.display=isDebugMode?'':'none';
      document.getElementById('normalBlinkSection').style.display=isDebugMode?'none':'';
      queue('debugMode', isDebugMode);
    });

    // Pan buttons — hold to pan continuously
    let panTimer = null;
    function doPan(dir) {
      fetch('/settings',{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify(dir<0?{panLeft:true}:{panRight:true})}).catch(console.error);
    }
    function startPan(dir) { doPan(dir); panTimer=setInterval(()=>doPan(dir),150); }
    function stopPan() { if(panTimer){clearInterval(panTimer);panTimer=null;} }
    const panLeftBtn  = document.getElementById('panLeft');
    const panRightBtn = document.getElementById('panRight');
    panLeftBtn.addEventListener('mousedown',  e=>{e.preventDefault();startPan(-1);});
    panRightBtn.addEventListener('mousedown', e=>{e.preventDefault();startPan(1);});
    panLeftBtn.addEventListener('touchstart',  e=>{e.preventDefault();startPan(-1);},{passive:false});
    panRightBtn.addEventListener('touchstart', e=>{e.preventDefault();startPan(1); },{passive:false});
    document.addEventListener('mouseup',  stopPan);
    document.addEventListener('touchend', stopPan);

    function thermalColor(r) {
      r=Math.max(0,Math.min(1,r));
      let R=0,G=0,B=0;
      if      (r<=.25){const f=r/.25;       G=Math.round(255*f);B=255;}
      else if (r<=.50){const f=(r-.25)/.25; G=255;B=Math.round(255*(1-f));}
      else if (r<=.75){const f=(r-.50)/.25; R=Math.round(255*f);G=255;}
      else            {const f=(r-.75)/.25; R=255;G=Math.round(255*(1-f));}
      return `rgb(${R},${G},${B})`;
    }

    function pixelColor(temp, minT, maxT, mode, ambient) {
      const rng=(maxT-minT)||1;
      const ratio=Math.pow(Math.max(0,Math.min(1,(temp-minT)/rng)),0.6);
      if (mode===2) {
        const lo=ambient+1.5, hi=ambient+12.0;
        if (temp>=lo && temp<=hi) return '#ffffff';
        const b=Math.round(30*ratio); return `rgb(0,0,${b})`;
      }
      if (mode===1) { const v=Math.round(255*ratio); return `rgb(${v},${v},${v})`; }
      return thermalColor(ratio);
    }

    function bilinear(src, dst) {
      for (let y=0;y<ISIZE;y++){
        const gy=y/UP, y0=Math.floor(gy), y1=Math.min(y0+1,GRID-1), fy=gy-y0;
        for (let x=0;x<ISIZE;x++){
          const gx=x/UP, x0=Math.floor(gx), x1=Math.min(x0+1,GRID-1), fx=gx-x0;
          const v00=src[y0*GRID+x0],v01=src[y0*GRID+x1],v10=src[y1*GRID+x0],v11=src[y1*GRID+x1];
          dst[y*ISIZE+x]=(v00+fx*(v01-v00))+fy*((v10+fx*(v11-v10))-(v00+fx*(v01-v00)));
        }
      }
    }

    function drawHeatmap(pixels, minT, maxT, interp, mode, ambient) {
      ctx.clearRect(0,0,320,320);
      if (interp) {
        bilinear(pixels, interpBuf);
        const cw=320/ISIZE, ch=320/ISIZE;
        for (let y=0;y<ISIZE;y++) for (let x=0;x<ISIZE;x++) {
          ctx.fillStyle=pixelColor(interpBuf[y*ISIZE+x],minT,maxT,mode,ambient);
          ctx.fillRect(x*cw,y*ch,cw,ch);
        }
      } else {
        const cw=320/GRID, ch=320/GRID;
        for (let y=0;y<GRID;y++) for (let x=0;x<GRID;x++) {
          ctx.fillStyle=pixelColor(pixels[y*GRID+x],minT,maxT,mode,ambient);
          ctx.fillRect(x*cw,y*ch,cw,ch);
        }
      }
    }

    function drawOverlay(d) {
      if (d.trackerState==='SCAN'||!d.targetFound) return;
      const cw=320/GRID, ch=320/GRID;
      const cx=(d.targetX+0.5)*cw, cy=(d.targetY+0.5)*ch, r=cw*0.85;
      ctx.lineWidth=2;
      ctx.strokeStyle=d.trackerState==='HOLD'?'rgba(255,200,0,0.9)':'rgba(0,255,128,0.95)';
      ctx.beginPath(); ctx.arc(cx,cy,r,0,2*Math.PI); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(cx-r*1.6,cy); ctx.lineTo(cx+r*1.6,cy); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(cx,cy-r*1.6); ctx.lineTo(cx,cy+r*1.6); ctx.stroke();
    }

    const stateEl=document.getElementById('tracker-state');
    const stateClass={TRACK:'state-track',HOLD:'state-hold',SCAN:'state-scan'};

    function applySettings(d) {
      const tm=document.getElementById('toggleManual');
      const ti=document.getElementById('toggleInterp');
      if (document.activeElement!==slMin)     { slMin.value=d.manualMin;      document.getElementById('sliderMinValue').textContent=Number(d.manualMin).toFixed(1); }
      if (document.activeElement!==slMax)     { slMax.value=d.manualMax;      document.getElementById('sliderMaxValue').textContent=Number(d.manualMax).toFixed(1); }
      if (document.activeElement!==slOpen)    { slOpen.value=d.eyeOpen;       document.getElementById('openVal').textContent=d.eyeOpen; }
      if (document.activeElement!==slClosed)  { slClosed.value=d.eyeClosed;   document.getElementById('closedVal').textContent=d.eyeClosed; }
      if (document.activeElement!==slInterval){ slInterval.value=d.blinkInterval; document.getElementById('intervalVal').textContent=d.blinkInterval; }
      if (document.activeElement!==slDuration){ slDuration.value=d.blinkDuration; document.getElementById('durationVal').textContent=d.blinkDuration; }
      if (document.activeElement!==slRotSpd)  { slRotSpd.value=d.rotateSpeed; document.getElementById('rotateSpeedVal').textContent=parseFloat(d.rotateSpeed).toFixed(1); }
      if (d.useManualRange!==undefined){ tm.checked=d.useManualRange; slMin.disabled=slMax.disabled=!d.useManualRange; }
      if (d.useInterpolation!==undefined) ti.checked=d.useInterpolation;
      if (d.displayMode!==undefined) modeBtns.forEach(b=>b.classList.toggle('active',parseInt(b.dataset.mode)===d.displayMode));
      if (d.motorEnabled!==undefined){ motorOn=d.motorEnabled; motorBtn.textContent=motorOn?'Motor: ON':'Motor: OFF'; motorBtn.classList.toggle('on',motorOn); }
      if (d.debugMode!==undefined && d.debugMode!==isDebugMode) {
        isDebugMode=d.debugMode;
        document.getElementById('toggleDebug').checked=isDebugMode;
        document.getElementById('debugBlinkSection').style.display=isDebugMode?'':'none';
        document.getElementById('normalBlinkSection').style.display=isDebugMode?'none':'';
      }
      if (d.normalBlinkInterval!==undefined && document.activeElement!==slNormal) {
        const secs=Math.round(d.normalBlinkInterval/1000);
        slNormal.value=secToSlider(secs);
        document.getElementById('normalIntervalVal').textContent=fmtSec(secs);
        document.getElementById('blinkCooldownDisplay').textContent=fmtSec(secs);
      }
      if (d.rotateAngle!==undefined) document.getElementById('panAngle').textContent=Math.round(d.rotateAngle)+'°';
      const panRow=document.getElementById('panResumeRow');
      if (d.manualPanActive) { panRow.style.display=''; document.getElementById('panCountdown').textContent=Math.max(0,d.manualPanRemainSec||0); }
      else panRow.style.display='none';
    }

    function updateAge() {
      if (!lastRx){document.getElementById('last-frame').textContent='--';return;}
      const s=(Date.now()-lastRx)/1000;
      document.getElementById('last-frame').textContent=s<0.5?'just now':`${s.toFixed(1)}s ago`;
    }

    function fetchFrame() {
      const ctrl = new AbortController();
      const timer = setTimeout(() => ctrl.abort(), 2500);
      fetch('/frame', {signal: ctrl.signal}).then(r=>r.json()).then(d=>{
        clearTimeout(timer);
        applySettings(d);
        if (d.ip) document.getElementById('ip-address').textContent=d.ip;
        if (d.frameReady) {
          d.pixels = d.pixels.slice().reverse();
          d.targetX = (GRID - 1) - d.targetX;
          d.targetY = (GRID - 1) - d.targetY;
          drawHeatmap(d.pixels,d.tMin,d.tMax,d.useInterpolation,d.displayMode||0,d.median||0);
          drawOverlay(d);
          document.getElementById('stat-min').textContent=`${d.tMin.toFixed(1)}°`;
          document.getElementById('stat-max').textContent=`${d.tMax.toFixed(1)}°`;
          document.getElementById('stat-median').textContent=`${d.median.toFixed(1)}°`;
          document.getElementById('target-pos').textContent=d.targetFound?`(${d.targetX.toFixed(1)}, ${d.targetY.toFixed(1)})`:'none';
          stateEl.textContent=d.trackerState||'SCAN';
          stateEl.className='state-badge '+(stateClass[d.trackerState]||'state-scan');
          lastRx=Date.now();
        }
      }).catch(()=>{
        clearTimeout(timer);
        lastRx=0;
        document.getElementById('last-frame').textContent='disconnected';
      }).finally(()=>{ setTimeout(fetchFrame, 350); });
    }

    fetch('/settings').then(r=>r.json()).then(applySettings).catch(console.error);
    fetchFrame();
    setInterval(updateAge,250);
  </script>
</body>
</html>
)rawliteral";

// ─────────────────────────────────────────────
// JSON helpers
// ─────────────────────────────────────────────
static void sendJson(const JsonDocument &doc) {
  String p; serializeJson(doc,p); server.send(200,"application/json",p);
}
static void sendJsonError(int code, const char *msg) {
  JsonDocument doc; doc["error"]=msg;
  String p; serializeJson(doc,p); server.send(code,"application/json",p);
}
static void handleIndex() { server.send_P(200,"text/html",INDEX_HTML); }

static void handleFrame() {
  JsonDocument doc;
  doc["frameReady"]      = frameAvailable;
  doc["ip"]              = deviceIp;
  doc["targetFound"]     = trackFound;
  doc["targetX"]         = trackTargetX;
  doc["targetY"]         = trackTargetY;
  doc["trackerState"]    = trackerStateName();
  doc["motorEnabled"]         = motorEnabled;
  doc["debugMode"]            = debugMode;
  doc["normalBlinkInterval"]  = (long)normalBlinkIntervalMs;
  doc["manualPanActive"]      = manualPanActive;
  doc["manualPanRemainSec"]   = manualPanActive ? (int)((MANUAL_PAN_TIMEOUT_MS - (millis() - lastManualPanMs)) / 1000) : 0;
  doc["rotateAngle"]          = rotateAngle;
  doc["eyeOpen"]              = EYELID_OPEN_DEG;
  doc["eyeClosed"]            = EYELID_CLOSED_DEG;
  doc["blinkInterval"]        = (int)blinkIntervalMs;
  doc["blinkDuration"]   = (int)blinkDurationMs;
  doc["rotateSpeed"]     = rotateSpeed;
  doc["useInterpolation"]= useInterpolation;
  doc["useManualRange"]  = useManualRange;
  doc["manualMin"]       = manualMin;
  doc["manualMax"]       = manualMax;
  doc["displayMode"]     = (int)displayMode;
  if (frameAvailable) {
    doc["timestamp"]=latestFrameMillis;
    doc["tMin"]=latestFrameMin; doc["tMax"]=latestFrameMax; doc["median"]=latestFrameMedian;
    JsonArray arr=doc["pixels"].to<JsonArray>();
    for (int i=0;i<AMG88xx_PIXEL_ARRAY_SIZE;i++) arr.add(latestFrame[i]);
  } else {
    doc["timestamp"]=0; doc["tMin"]=18; doc["tMax"]=35; doc["median"]=22;
    doc["pixels"].to<JsonArray>();
  }
  sendJson(doc);
}

static void handleSettingsGet() {
  JsonDocument doc;
  doc["ip"]             = deviceIp;
  doc["motorEnabled"]        = motorEnabled;
  doc["debugMode"]           = debugMode;
  doc["normalBlinkInterval"] = (long)normalBlinkIntervalMs;
  doc["manualPanActive"]     = manualPanActive;
  doc["manualPanRemainSec"]  = manualPanActive ? (int)((MANUAL_PAN_TIMEOUT_MS - (millis() - lastManualPanMs)) / 1000) : 0;
  doc["rotateAngle"]         = rotateAngle;
  doc["eyeOpen"]             = EYELID_OPEN_DEG;
  doc["eyeClosed"]           = EYELID_CLOSED_DEG;
  doc["blinkInterval"]       = (int)blinkIntervalMs;
  doc["blinkDuration"]  = (int)blinkDurationMs;
  doc["rotateSpeed"]    = rotateSpeed;
  doc["useInterpolation"]= useInterpolation;
  doc["useManualRange"] = useManualRange;
  doc["manualMin"]      = manualMin;
  doc["manualMax"]      = manualMax;
  doc["displayMode"]    = (int)displayMode;
  sendJson(doc);
}

static void handleSettingsPost() {
  if (!server.hasArg("plain")){ sendJsonError(400,"Missing body"); return; }
  JsonDocument doc;
  if (deserializeJson(doc,server.arg("plain"))){ sendJsonError(400,"Invalid JSON"); return; }
  if (!doc["motorEnabled"].isNull())   motorEnabled    = doc["motorEnabled"].as<bool>();
  if (!doc["eyeOpen"].isNull())        EYELID_OPEN_DEG = doc["eyeOpen"].as<int>();
  if (!doc["eyeClosed"].isNull())      EYELID_CLOSED_DEG=doc["eyeClosed"].as<int>();
  if (!doc["blinkInterval"].isNull())  blinkIntervalMs = doc["blinkInterval"].as<unsigned long>();
  if (!doc["blinkDuration"].isNull())  blinkDurationMs = doc["blinkDuration"].as<unsigned long>();
  if (!doc["rotateSpeed"].isNull())    rotateSpeed     = doc["rotateSpeed"].as<float>();
  if (!doc["useInterpolation"].isNull()) useInterpolation=doc["useInterpolation"].as<bool>();
  if (!doc["useManualRange"].isNull()) { useManualRange=doc["useManualRange"].as<bool>(); autoRangeInitialized=false; }
  if (!doc["manualMin"].isNull())      manualMin       = doc["manualMin"].as<float>();
  if (!doc["manualMax"].isNull())      manualMax       = doc["manualMax"].as<float>();
  if (!doc["displayMode"].isNull())    displayMode     = (DisplayMode)doc["displayMode"].as<int>();
  if (!doc["triggerBlink"].isNull() && doc["triggerBlink"].as<bool>()) triggerBlinkFlag=true;
  if (!doc["debugMode"].isNull()) debugMode = doc["debugMode"].as<bool>();
  if (!doc["normalBlinkInterval"].isNull()) {
    unsigned long v = doc["normalBlinkInterval"].as<unsigned long>();
    normalBlinkIntervalMs = constrain(v, BLINK_NORMAL_MIN_MS, BLINK_NORMAL_MAX_MS);
  }
  if (!doc["panLeft"].isNull() && doc["panLeft"].as<bool>()) {
    rotateAngle = constrain(rotateAngle - rotateSpeed * 3.0f, ROTATE_MIN_DEG, ROTATE_MAX_DEG);
    servoWrite(ROTATE_SERVO_PIN, rotateAngle);
    manualPanActive = true;
    lastManualPanMs = millis();
  }
  if (!doc["panRight"].isNull() && doc["panRight"].as<bool>()) {
    rotateAngle = constrain(rotateAngle + rotateSpeed * 3.0f, ROTATE_MIN_DEG, ROTATE_MAX_DEG);
    servoWrite(ROTATE_SERVO_PIN, rotateAngle);
    manualPanActive = true;
    lastManualPanMs = millis();
  }
  handleSettingsGet();
}

static void handleNotFound() {
  server.sendHeader("Location","/"); server.send(302,"text/plain","Redirecting");
}

// ─────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(50);
  const char* rr[] = {"","POWERON","","SW","OWDT","DEEPSLEEP","SDIO","TG0WDT","TG1WDT","RTCWDT","INTRUSION","TGWDT_CPU","SW_CPU","RTCWDT_CPU","EXT_CPU","BROWNOUT","RTCWDT_RTC"};
  int rrc = (int)esp_reset_reason();
  Serial.printf("\nReset: %s (%d)\n", (rrc<17?rr[rrc]:"?"), rrc);
  Serial.println("Doll Head Tracker v2 booting..."); Serial.flush();

  Serial.println("Servo init..."); Serial.flush();
  ledcAttach(ROTATE_SERVO_PIN, SERVO_FREQ, SERVO_RESOLUTION);
  servoWrite(ROTATE_SERVO_PIN, rotateAngle);
  ledcAttach(BLINK_SERVO_PIN, SERVO_FREQ, SERVO_RESOLUTION);
  servoWrite(BLINK_SERVO_PIN, EYELID_OPEN_DEG);
  Serial.println("Servos OK"); Serial.flush();
  delay(400);
  blinkPhaseMs = millis();

  Serial.println("Wire init..."); Serial.flush();
  Wire.begin(21,22);
  if (!amg.begin()) { Serial.println("ERROR: AMG8833 not found!"); Serial.flush(); }
  else              { Serial.println("AMG8833 OK"); Serial.flush(); }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  unsigned long wifiStart=millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-wifiStart<15000){ delay(500); Serial.print("."); }
  if (WiFi.status()==WL_CONNECTED) {
    deviceIp=WiFi.localIP().toString();
    Serial.printf("\nWiFi connected — IP: %s\n", deviceIp.c_str());
    if (MDNS.begin("dollhead")) Serial.println("mDNS: http://dollhead.local/");
  } else {
    deviceIp="no wifi"; Serial.println("\nWiFi failed.");
  }

  server.on("/",          handleIndex);
  server.on("/frame",     HTTP_GET,  handleFrame);
  server.on("/settings",  HTTP_GET,  handleSettingsGet);
  server.on("/settings",  HTTP_POST, handleSettingsPost);
  server.onNotFound(handleNotFound);
  server.begin();

  ArduinoOTA.setHostname("dollhead");
  if (strlen(OTA_PASSWORD)>0) ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]()  { Serial.println("OTA start"); });
  ArduinoOTA.onProgress([](unsigned int p,unsigned int t){ Serial.printf("OTA: %u%%\r",p*100/t); });
  ArduinoOTA.onEnd([]()    { Serial.println("\nOTA done"); });
  ArduinoOTA.onError([](ota_error_t e){ Serial.printf("OTA error[%u]\n",e); });
  ArduinoOTA.begin();

  Serial.println("Ready."); Serial.flush();
}

// ─────────────────────────────────────────────
// Loop
// ─────────────────────────────────────────────
void loop() {
  static bool firstLoop = true;
  if (firstLoop) { firstLoop = false; Serial.println("Loop start"); Serial.flush(); }

  ArduinoOTA.handle();
  server.handleClient();
  delay(5); // yield to FreeRTOS/WiFi background tasks — prevents watchdog reset

  if (triggerBlinkFlag){ triggerBlinkFlag=false; startBlink(); }
  updateBlink();

  static unsigned long lastAmgMs = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastAmgMs >= 100) {
    lastAmgMs = nowMs;
    amg.readPixels(pixels);
    float medianTemp=computeMedian(pixels,AMG88xx_PIXEL_ARRAY_SIZE);
    updateStepper(pixels,medianTemp);

    float tMin,tMax;
    if (useManualRange) {
      tMin=manualMin; tMax=manualMax;
    } else {
      float mn=pixels[0],mx=pixels[0];
      for (int i=1;i<AMG88xx_PIXEL_ARRAY_SIZE;i++){ if(pixels[i]<mn)mn=pixels[i]; if(pixels[i]>mx)mx=pixels[i]; }
      applyRangeSmoothing(mn,mx,&tMin,&tMax);
    }

    for (int i=0;i<AMG88xx_PIXEL_ARRAY_SIZE;i++) latestFrame[i]=pixels[i];
    latestFrameMin=tMin; latestFrameMax=tMax;
    latestFrameMedian=medianTemp; latestFrameMillis=millis();
    frameAvailable=true;
  }
}
