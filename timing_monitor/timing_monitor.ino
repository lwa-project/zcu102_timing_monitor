#include <Arduino.h>
#include <RTCZero.h>
#include <TemperatureZero.h>

#define PIN_SENSE_12V A0
#define PIN_SENSE_9V A1
#define PIN_SENSE_6V A2

#define PIN_LOCK A3
#define PIN_SYNC A4

#define PIN_NOT_FAULT D5

// Divider scale factors: Vrail = Vadc * (Rtop+Rbot)/Rbot
static const float SCALE_12 = (62.0f + 10.0f) / 10.0f;  // 7.2
static const float SCALE_9 = (43.0f + 10.0f) / 10.0f;  // 5.3
static const float SCALE_6  = (27.0f + 10.0f) / 10.0f;  // 3.7

// Internal temperature sensor readout
TemperatureZero MCU_TEMP = TemperatureZero();

float readVoltage(uint8_t pin, float scale) {
  uint32_t acc = 0;
  for(uint8_t i=0; i<16; ++i) {
    acc += analogRead(pin);
  }
  float adc = (float) acc / 16;
  float vadc = 3.3 * (adc / 4095);
  return vadc * scale;
}

volatile bool lockState = false;

void lockISR() {
  bool s = digitalRead(PIN_LOCK);
  if( s != lockState ) {
    lockState = s;
  }
}

volatile uint32_t lastSyncMicros = 0;
volatile bool syncSeen = false;

void syncISR() {
  lastSyncMicros = micros();
  syncSeen = true;
}

RTCZero rtc;
bool inBypass = false;
unsigned long tSense = 0;
String rxLine;

void printHelp() {
  Serial.println(F(
    "\nCommands:"
    "\n  HELP or ?"
    "\n  VERSION"
    "\n  STATUS"
    "\n  TIME SET YYYY-MM-DD HH:MM:SS"
    "\n  TIME SET EPOCH <sec>"
    "\n  TIME GET"
    "\n  BYPASS ON [baud]   (escape '+++exit')"
    "\n  BYPASS OFF"
    "\n  RESET"
  ));
}

void printVersion() {
  Serial.println(F(
    "Timestamp:" __TIMESTAMP__ "\n"
    "Compiled: " __DATE__ " " __TIME__ "\n"
  ));
}

void dumpStatus() {
  float v12 = readVoltage(PIN_SENSE_12V, SCALE_12);
  float v6 = readVoltage(PIN_SENSE_6V,  SCALE_6);
  float v9 = readVoltage(PIN_SENSE_9V, SCALE_9);
  
  Serial.print(F("Epoch: ")); Serial.println(rtc.getEpoch());
  Serial.print(F("12V: ")); Serial.print(v12,3);
  Serial.println("");
  Serial.print(F(" 9V: ")); Serial.print(v9,3);
  Serial.println("");
  Serial.print(F(" 6V: ")); Serial.print(v6,3);
  Serial.println("");
  Serial.print(F("Valon Lock: ")); Serial.println(lockState ? "HIGH" : "LOW");
  Serial.print(F("Last Sync Pulse: ")); Serial.print((micros()-lastSyncMicros)/1e6, 3); Serial.println(" s ago");
  Serial.print(F("MCU Temperature: ")); Serial.print(MCU_TEMP.readInternalTemperature()); Serial.println(" C");
}

void parseLine(String s) {
  s.trim();
  if( s.length() == 0) {
    return;
  }
  s.toUpperCase();

  if( s.equals("HELP") || s.equals("?")) {
    printHelp();
    return;
  }
  if( s.equals("VERSION")) {
    printVersion();
    return;
  }
  if( s.equals("STATUS") ) {
    dumpStatus();
    return;
  }
  if( s.equals("TIME GET") ) {
    Serial.print(F("EPOCH "));
    Serial.println(rtc.getEpoch());
    return;
  }
  if( s.equals("BYPASS OFF") ) {
    inBypass=false;
    Serial.println(F("OK: bypass off"));
    return;
  }
  if( s.equals("RESET") ) {
    Serial.println(F("Resetting..."));
    delay(50);
    NVIC_SystemReset();
    return;
  }

  if( s.startsWith("BYPASS ON") ) {
    long baud = 115200;
    int sp = s.indexOf(' ', 9);
    if( sp > 0 ) {
      baud = s.substring(sp+1).toInt();
      if( baud <= 0 ) {
        baud = 115200;
      }
    }
    Serial1.begin(baud); // PB08/PB09 on XIAO
    inBypass = true;
    Serial.print(F("OK: bypass on @")); Serial.println(baud);
    Serial.println(F("(escape with '+++exit')"));
    return;
  }

  if (s.startsWith("TIME SET EPOCH")) {
    unsigned long ep=0;
    if( sscanf(s.c_str(), "TIME SET EPOCH %lu", &ep) == 1 ) {
      rtc.setEpoch(ep); Serial.println(F("OK"));
    } else {
      Serial.println(F("ERR"));
    }
    return;
  }

  if (s.startsWith("TIME SET ")) {
    // TIME SET YYYY-MM-DD HH:MM:SS
    int Y,M,D,h,m; int s2;
    if( sscanf(s.c_str(), "TIME SET %d-%d-%d %d:%d:%d", &Y,&M,&D,&h,&m,&s2) == 6 ) {
      rtc.setYear((uint8_t)(Y-2000)); rtc.setMonth(M); rtc.setDay(D);
      rtc.setHours(h); rtc.setMinutes(m); rtc.setSeconds(s2);
      Serial.println(F("OK"));
    } else {
      Serial.println(F("ERR"));
    }
    return;
  }

  Serial.println(F("ERR: unknown (HELP)"));
}

// ---------- Setup & loop ----------
void setup() {
  // Setup BOD33 - the brown out detector
  // See https://blog.thea.codes/sam-d21-brown-out-detector/ for details
  SYSCTRL->BOD33.bit.ENABLE = 0;
  while( !SYSCTRL->PCLKSR.bit.B33SRDY ) {};
  SYSCTRL->BOD33.reg = SYSCTRL_BOD33_LEVEL(48) \
                       | SYSCTRL_BOD33_ACTION_NONE \
                       | SYSCTRL_BOD33_HYST;
  SYSCTRL->BOD33.bit.ENABLE = 1;
  while( !SYSCTRL->PCLKSR.bit.BOD33RDY ) {};
  while( SYSCTRL->PCLKSR.bit.BOD33DET ) {};

  SYSCTRL->BOD33.bit.ENABLE = 0;
  while( !SYSCTRL->PCLKSR.bit.B33SRDY ) {};
  SYSCTRL->BOD33.reg |= SYSCTRL_BOD33_ACTION_RESET;
  SYSCTRL->BOD33.bit.ENABLE = 1;

  Serial.begin(9600);
  while (!Serial && millis() < 4000) { }

  analogReadResolution(12);

  pinMode(PIN_LOCK, INPUT_PULLDOWN);
  pinMode(PIN_SYNC, INPUT_PULLDOWN);

  pinMode(PIN_NOT_FAULT, OUTPUT);
  digitalWrite(PIN_NOT_FAULT, 1);

  attachInterrupt(digitalPinToInterrupt(PIN_LOCK), lockISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_SYNC), syncISR, RISING);

  rtc.begin();

  MCU_TEMP.init();

  printHelp();
}

void loop() {
  // Bypass mode
  if( inBypass ) {
    while( Serial.available() ) {
      // escape sequence support
      static String esc;
      char c = (char)Serial.read();
      esc += c;
      if( esc.endsWith("+++exit") || esc.endsWith("+++EXIT") ) {
        inBypass=false;
        Serial.println(F("\nOK: exit bypass"));
        esc="";
        break;
      }
      Serial1.write(c);
    }
    while( Serial1.available() ) {
      Serial.write(Serial1.read());
    }
    // Keep logging via ISRs even in bypass
  } else {
    // CLI line reader
    while( Serial.available() ) {
      char c = (char)Serial.read();
      if( c == '\r' ) {
        continue;
      }
      if( c == '\n' ) {
        parseLine(rxLine); rxLine="";
      } else {
        rxLine += c;
      }
    }
  }
  
  // Periodic refreshing of sync pulse state
  if (millis() - tSense >= 1000) {
    tSense = millis();
    if( syncSeen ) {
      if( ((micros() - lastSyncMicros)/1e6) > 6 ) {
        syncSeen = false;
      }
    }
  }
}
