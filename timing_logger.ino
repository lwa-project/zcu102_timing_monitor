/* XIAO SAMD21 multi-rail monitor + event logger + USB CLI + Serial1 bypass
 *
 * Rails: 12V, 6V, 8.5V (with fixed dividers: 62k/10k, 27k/10k, 43k/10k)
 * Inputs:  LOCK (A0/PA02), PPS (A5/PA09), 3x analog sense (A8/A9/A10)
 * Outputs: 3 LED drive pins (A2/A3/A4) -> "in-range" indicators for 12V/6V/8.5V
 * Events logged with timestamps (RTCZero). CLI over USB: ranges, time, logs, bypass, reset.
 *
 * Notes:
 * - We keep the rails well inside ADC range (mid-scale-ish) per your dividers.
 * - We store thresholds + last-known epoch in flash (FlashStorage_SAMD).
 * - We DO NOT update flash every 0.5 s (flash wear ~10k cycles/row typical); default is 60 s.
 *   You can change SAVEINT via command if you want. :contentReference[oaicite:2]{index=2}
 */

#include <Arduino.h>
#include <RTCZero.h>
#include <FlashStorage_SAMD.h>

// ---------- Pin map (XIAO SAMD21) ----------
const uint8_t PIN_LOCK        = A0;   // PA02 (input)
const uint8_t PIN_LED_12V     = A2;   // PA10 (output)
const uint8_t PIN_LED_6V      = A3;   // PA11 (output)
const uint8_t PIN_LED_85V     = A4;   // PA08 (output)
const uint8_t PIN_PPS         = A5;   // PA09 (input)

const uint8_t PIN_SENSE_6V    = A8;   // PA07 (input)
const uint8_t PIN_SENSE_12V   = A9;   // PA05 (input)
const uint8_t PIN_SENSE_85V   = A10;  // PA06 (input)

const uint8_t PIN_RXD_VALON   = A6;   // PB08 (Serial1 RX)
const uint8_t PIN_TXD_VALON   = A7;   // PB09 (Serial1 TX)

// ---------- ADC & scaling ----------
static const float VREF = 3.3f;       // analog reference is VDD on XIAO (default)
static const uint16_t ADC_MAX = 4095;
static const uint8_t ADC_SAMPLES = 8;

// Divider scale factors: Vrail = Vadc * (Rtop+Rbot)/Rbot
static const float SCALE_12 = (62.0f + 10.0f) / 10.0f;  // 7.2
static const float SCALE_85 = (43.0f + 10.0f) / 10.0f;  // 5.3
static const float SCALE_6  = (27.0f + 10.0f) / 10.0f;  // 3.7

// ---------- Defaults (CLI-changeable) ----------
struct Ranges {
  float v12_min = 11.5f, v12_max = 12.5f;
  float v6_min  =  5.5f, v6_max  =  6.5f;
  float v85_min =  8.0f, v85_max = 10.0f;
  float pps_min = 0.5f,  pps_max = 1.5f;    // seconds
  float hysteresis = 0.05f;                 // 50 mV
};

struct NVConfig {
  uint32_t magic = 0x58494F21;     // "XIO!"
  uint16_t version = 1;
  Ranges ranges;
  uint32_t save_interval_s = 60;   // flash epoch save interval
};

struct NVTime {
  uint32_t magic = 0x54494D45;     // "TIME"
  uint32_t epoch = 0;              // UNIX seconds
};

FlashStorage(config_store, NVConfig);
FlashStorage(time_store,   NVTime);

// ---------- RTC / time ----------
RTCZero rtc;
volatile bool secondTick = false;

// ---------- Event log ----------
enum EventType : uint8_t {
  EVT_V12_OUT=1, EVT_V12_IN,
  EVT_V6_OUT,  EVT_V6_IN,
  EVT_V85_OUT, EVT_V85_IN,
  EVT_LOCK_RISE, EVT_LOCK_FALL,
  EVT_PPS_BAD, EVT_PPS_OK
};

struct Event {
  uint32_t epoch;
  EventType type;
  float value;        // volts for rails; seconds for PPS; 1/0 for lock
};

static const size_t MAX_EVENTS = 512;
Event events[MAX_EVENTS];
volatile size_t ev_head = 0, ev_count = 0;

void logEvent(EventType t, float v) {
  Event &e = events[ev_head];
  e.epoch = rtc.getEpoch();
  e.type  = t;
  e.value = v;
  ev_head = (ev_head + 1) % MAX_EVENTS;
  if (ev_count < MAX_EVENTS) ev_count++;
}

// ---------- State ----------
volatile bool lockState = false;
volatile uint32_t lastPpsMicros = 0;
volatile bool ppsSeen = false;

NVConfig cfg;
NVTime   nvtime;

bool inBypass = false;
unsigned long lastFlashSave = 0;

// ---------- Helpers ----------
float readVoltage(uint8_t pin, float scale) {
  uint32_t acc = 0;
  for (uint8_t i=0; i<ADC_SAMPLES; ++i) acc += analogRead(pin);
  float adc = (float)acc / ADC_SAMPLES;
  float vadc = VREF * (adc / ADC_MAX);
  return vadc * scale;
}

void setLeds(bool ok12, bool ok6, bool ok85) {
  digitalWrite(PIN_LED_12V, ok12 ? HIGH : LOW);
  digitalWrite(PIN_LED_6V,  ok6  ? HIGH : LOW);
  digitalWrite(PIN_LED_85V, ok85 ? HIGH : LOW);
}

// ---------- ISRs (no IRAM_ATTR on SAMD) ----------
void onSecond() { secondTick = true; }

void lockISR() {
  bool s = digitalRead(PIN_LOCK);
  if (s != lockState) {
    lockState = s;
    logEvent(s ? EVT_LOCK_RISE : EVT_LOCK_FALL, s ? 1.0f : 0.0f);
  }
}

void ppsISR() {
  uint32_t now = micros();
  if (lastPpsMicros != 0) {
    float dt = (now - lastPpsMicros) / 1e6f;
    if (dt < cfg.ranges.pps_min || dt > cfg.ranges.pps_max) {
      logEvent(EVT_PPS_BAD, dt);
    } else {
      logEvent(EVT_PPS_OK, dt);
    }
  }
  lastPpsMicros = now;
  ppsSeen = true;
}

// ---------- CLI ----------
String rxLine;

void printHelp() {
  Serial.println(F(
    "\nCommands:"
    "\n  HELP"
    "\n  STATUS"
    "\n  GET LOG"
    "\n  CLEAR LOG"
    "\n  RANGE 12 min max"
    "\n  RANGE 6 min max"
    "\n  RANGE 8.5 min max"
    "\n  HYST x.xx"
    "\n  PPS min max"
    "\n  TIME SET YYYY-MM-DD HH:MM:SS"
    "\n  TIME SET EPOCH <sec>"
    "\n  TIME GET"
    "\n  SAVEINT <seconds>"
    "\n  BYPASS ON [baud]   (escape '+++exit')"
    "\n  BYPASS OFF"
    "\n  RESET"
  ));
}

void dumpStatus() {
  float v12 = readVoltage(PIN_SENSE_12V, SCALE_12);
  float v6  = readVoltage(PIN_SENSE_6V,  SCALE_6);
  float v85 = readVoltage(PIN_SENSE_85V, SCALE_85);
  Serial.print(F("Epoch: ")); Serial.println(rtc.getEpoch());
  Serial.print(F("12V: ")); Serial.print(v12,3);
  Serial.print(F("  (")); Serial.print(cfg.ranges.v12_min,2); Serial.print(".."); Serial.print(cfg.ranges.v12_max,2); Serial.println(")");
  Serial.print(F(" 6V: ")); Serial.print(v6,3);
  Serial.print(F("  (")); Serial.print(cfg.ranges.v6_min,2);  Serial.print(".."); Serial.print(cfg.ranges.v6_max,2);  Serial.println(")");
  Serial.print(F("8.5V: ")); Serial.print(v85,3);
  Serial.print(F(" (")); Serial.print(cfg.ranges.v85_min,2);  Serial.print(".."); Serial.print(cfg.ranges.v85_max,2); Serial.println(")");
  Serial.print(F("LOCK: ")); Serial.println(lockState ? "HIGH" : "LOW");
  Serial.print(F("PPS window (s): ")); Serial.print(cfg.ranges.pps_min,3); Serial.print(".."); Serial.println(cfg.ranges.pps_max,3);
  Serial.print(F("Hysteresis (V): ")); Serial.println(cfg.ranges.hysteresis,3);
  Serial.print(F("SAVEINT (s): ")); Serial.println(cfg.save_interval_s);
  Serial.print(F("Events in buffer: ")); Serial.println(ev_count);
}

void dumpLogs() {
  Serial.println(F("epoch,type,value"));
  size_t idx = (ev_head + MAX_EVENTS - ev_count) % MAX_EVENTS;
  for (size_t i=0; i<ev_count; ++i) {
    const Event &e = events[(idx + i) % MAX_EVENTS];
    Serial.print(e.epoch);
    Serial.print(',');
    Serial.print((int)e.type);
    Serial.print(',');
    Serial.println(e.value, 6);
  }
}

void clearLogs() { ev_head = ev_count = 0; Serial.println(F("OK: log cleared")); }

void parseLine(String s) {
  s.trim();
  if (s.length()==0) return;

  if (s.equalsIgnoreCase("HELP")) { printHelp(); return; }
  if (s.equalsIgnoreCase("STATUS")) { dumpStatus(); return; }
  if (s.equalsIgnoreCase("GET LOG")) { dumpLogs(); return; }
  if (s.equalsIgnoreCase("CLEAR LOG")) { clearLogs(); return; }
  if (s.equalsIgnoreCase("TIME GET")) { Serial.print(F("EPOCH ")); Serial.println(rtc.getEpoch()); return; }
  if (s.equalsIgnoreCase("BYPASS OFF")) { inBypass=false; Serial.println(F("OK: bypass off")); return; }
  if (s.equalsIgnoreCase("RESET")) { Serial.println(F("Resetting...")); delay(50); NVIC_SystemReset(); return; }

  if (s.startsWith("BYPASS ON")) {
    long baud = 115200;
    int sp = s.indexOf(' ', 9);
    if (sp>0) { baud = s.substring(sp+1).toInt(); if (baud<=0) baud = 115200; }
    Serial1.begin(baud); // PB08/PB09 on XIAO
    inBypass = true;
    Serial.print(F("OK: bypass on @")); Serial.println(baud);
    Serial.println(F("(escape with '+++exit')"));
    return;
  }

  if (s.startsWith("SAVEINT")) {
    long x = s.substring(String("SAVEINT").length()).toInt();
    if (x >= 5) { cfg.save_interval_s = (uint32_t)x; config_store.write(cfg); Serial.println(F("OK")); }
    else Serial.println(F("ERR: >=5"));
    return;
  }

  if (s.startsWith("HYST")) {
    float h = s.substring(String("HYST").length()).toFloat();
    if (h >= 0.0f && h <= 0.5f) { cfg.ranges.hysteresis = h; config_store.write(cfg); Serial.println(F("OK")); }
    else Serial.println(F("ERR"));
    return;
  }

  if (s.startsWith("PPS")) {
    float mn, mx;
    if (sscanf(s.c_str(), "PPS %f %f", &mn, &mx) == 2 && mn>0 && mx>mn) {
      cfg.ranges.pps_min = mn; cfg.ranges.pps_max = mx; config_store.write(cfg); Serial.println(F("OK"));
    } else Serial.println(F("ERR"));
    return;
  }

  if (s.startsWith("RANGE ")) {
    // Accepted: "RANGE 12 a b", "RANGE 6 a b", "RANGE 8.5 a b"
    char rail[6]; float mn, mx;
    if (sscanf(s.c_str(), "RANGE %5s %f %f", rail, &mn, &mx) == 3 && mx>mn) {
      String r = String(rail);
      if (r == "12") { cfg.ranges.v12_min=mn; cfg.ranges.v12_max=mx; }
      else if (r == "6") { cfg.ranges.v6_min=mn; cfg.ranges.v6_max=mx; }
      else if (r == "8.5" || r == "8V5" || r == "8") { cfg.ranges.v85_min=mn; cfg.ranges.v85_max=mx; }
      else { Serial.println(F("ERR: rail")); return; }
      config_store.write(cfg);
      Serial.println(F("OK"));
    } else Serial.println(F("ERR"));
    return;
  }

  if (s.startsWith("TIME SET EPOCH")) {
    unsigned long ep=0;
    if (sscanf(s.c_str(), "TIME SET EPOCH %lu", &ep)==1) {
      rtc.setEpoch(ep); nvtime.epoch=ep; time_store.write(nvtime); Serial.println(F("OK"));
    } else Serial.println(F("ERR"));
    return;
  }

  if (s.startsWith("TIME SET ")) {
    // TIME SET YYYY-MM-DD HH:MM:SS
    int Y,M,D,h,m; int s2;
    if (sscanf(s.c_str(), "TIME SET %d-%d-%d %d:%d:%d", &Y,&M,&D,&h,&m,&s2)==6) {
      rtc.setYear((uint8_t)(Y-2000)); rtc.setMonth(M); rtc.setDay(D);
      rtc.setHours(h); rtc.setMinutes(m); rtc.setSeconds(s2);
      nvtime.epoch = rtc.getEpoch(); time_store.write(nvtime); Serial.println(F("OK"));
    } else Serial.println(F("ERR"));
    return;
  }

  Serial.println(F("ERR: unknown (HELP)"));
}

// ---------- Setup & loop ----------
void setup() {
  Serial.begin(1000000);
  while (!Serial && millis() < 4000) { }

  // Load config/time from flash (use reference-taking API)
  config_store.read(cfg);
  if (cfg.magic != 0x58494F21) { cfg = NVConfig(); config_store.write(cfg); }
  time_store.read(nvtime);
  if (nvtime.magic != 0x54494D45) { nvtime = NVTime(); time_store.write(nvtime); }

  analogReadResolution(12);

  pinMode(PIN_LOCK, INPUT_PULLDOWN);
  pinMode(PIN_PPS,  INPUT_PULLDOWN);
  pinMode(PIN_LED_12V, OUTPUT);
  pinMode(PIN_LED_6V,  OUTPUT);
  pinMode(PIN_LED_85V, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_LOCK), lockISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_PPS),  ppsISR,  RISING); // edge mode per Arduino ref :contentReference[oaicite:2]{index=2}

  rtc.begin();
  if (nvtime.epoch > 0) rtc.setEpoch(nvtime.epoch);
  rtc.attachInterrupt(onSecond);
  rtc.setAlarmSeconds(0);
  rtc.enableAlarm(rtc.MATCH_SS);

  printHelp();
}

bool lastOK12=false, lastOK6=false, lastOK85=false;

void loop() {
  // Bypass mode
  if (inBypass) {
    while (Serial.available()) {
      // escape sequence support
      static String esc;
      char c = (char)Serial.read();
      esc += c;
      if (esc.endsWith("+++exit")) { inBypass=false; Serial.println(F("\nOK: exit bypass")); esc=""; break; }
      Serial1.write(c);
    }
    while (Serial1.available()) Serial.write(Serial1.read());
    // Keep logging via ISRs even in bypass
  } else {
    // CLI line reader
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c=='\r') continue;
      if (c=='\n') { parseLine(rxLine); rxLine=""; }
      else rxLine += c;
    }
  }

  // Periodic sensing (100 ms)
  static unsigned long tSense = 0;
  if (millis() - tSense >= 100) {
    tSense = millis();

    const float v12 = readVoltage(PIN_SENSE_12V, SCALE_12);
    const float v6  = readVoltage(PIN_SENSE_6V,  SCALE_6);
    const float v85 = readVoltage(PIN_SENSE_85V, SCALE_85);

    const float h = cfg.ranges.hysteresis;

    bool ok12 = (v12 >= (cfg.ranges.v12_min + (lastOK12? -h:+h))) &&
                (v12 <= (cfg.ranges.v12_max + (lastOK12? +h:-h)));
    bool ok6  = (v6  >= (cfg.ranges.v6_min  + (lastOK6 ? -h:+h))) &&
                (v6  <= (cfg.ranges.v6_max  + (lastOK6 ? +h:-h)));
    bool ok85 = (v85 >= (cfg.ranges.v85_min + (lastOK85? -h:+h))) &&
                (v85 <= (cfg.ranges.v85_max + (lastOK85? +h:-h)));

    setLeds(ok12, ok6, ok85);

    if (ok12 != lastOK12) { logEvent(ok12 ? EVT_V12_IN : EVT_V12_OUT, v12); lastOK12 = ok12; }
    if (ok6  != lastOK6 ) { logEvent(ok6  ? EVT_V6_IN  : EVT_V6_OUT,  v6 ); lastOK6  = ok6;  }
    if (ok85 != lastOK85) { logEvent(ok85 ? EVT_V85_IN : EVT_V85_OUT, v85); lastOK85 = ok85; }
  }

  // Wear-aware time save
  if (secondTick) {
    secondTick = false;
    if (rtc.getEpoch() - lastFlashSave >= cfg.save_interval_s) {
      nvtime.epoch = rtc.getEpoch();
      time_store.write(nvtime);
      lastFlashSave = nvtime.epoch;
    }
  }
}
