#include <Arduino.h>
#include "hardware/rtc.h"
#include <time.h>

#define PIN_SENSE_12V A0
#define PIN_SENSE_9V A1
#define PIN_SENSE_6V A2

#define PIN_LOCK D3
#define PIN_SYNC D4

#define PIN_NOT_FAULT D5

// Activity LED pins (on-board user RGB LED, active low)
#define USB_LED_PIN PIN_LED_G
#define BUS_LED_PIN PIN_LED_B
#define ACT_LED_ON LOW
#define ACT_LED_OFF HIGH

// Minimum time in ms an activity LED stays lit
#define ACT_LED_HOLD_MS 50

// Timeout in ms before flushing the command input buffer (when not in bypass mode)
#define CMD_TIMEOUT_MS 2000

// Maximum command length in characters
#define MAX_CMD_LEN 128

// Control variables
uint8_t usb_led = 0, bus_led = 0;
unsigned long usb_led_time = 0, bus_led_time = 0;

void usb_activity() {
  // Flash the USB activity LED, unless bus activity is already being shown
  usb_led_time = millis();
  if( !bus_led && !usb_led ) {
    digitalWrite(USB_LED_PIN, ACT_LED_ON);
    usb_led = 1;
  }
}

void bus_activity() {
  // Flash the bus activity LED, which takes precedence over USB activity
  bus_led_time = millis();
  if( usb_led ) {
    digitalWrite(USB_LED_PIN, ACT_LED_OFF);
    usb_led = 0;
  }
  if( !bus_led ) {
    digitalWrite(BUS_LED_PIN, ACT_LED_ON);
    bus_led = 1;
  }
}

// Divider scale factors: Vrail = Vadc * (Rtop+Rbot)/Rbot
static const float SCALE_12 = (62.0f + 10.0f) / 10.0f;  // 7.2
static const float SCALE_9 = (43.0f + 10.0f) / 10.0f;  // 5.3
static const float SCALE_6  = (27.0f + 10.0f) / 10.0f;  // 3.7

float read_voltage(uint8_t pin, float scale) {
  uint32_t acc = 0;
  for(uint8_t i=0; i<16; ++i) {
    acc += analogRead(pin);
  }
  float adc = (float) acc / 16;
  float vadc = 3.3 * (adc / 4095);
  return vadc * scale;
}

volatile bool lock_state = false;

void lock_isr() {
  bool s = digitalRead(PIN_LOCK);
  if( s != lock_state ) {
    lock_state = s;
  }
}

volatile uint32_t last_sync_micros = 0;
volatile bool sync_seen = false;

void sync_isr() {
  last_sync_micros = micros();
  sync_seen = true;
}

bool in_bypass = false;
unsigned long t_sense = 0;
long int last_char = -1;
String cmd;
bool cmd_overflow = false;

uint32_t get_epoch() {
  datetime_t t;
  if(!rtc_get_datetime(&t)) {
    return 0;
  }

  struct tm timeinfo = {0};
  timeinfo.tm_year = t.year - 1900; // Years since 1900
  timeinfo.tm_mon  = t.month - 1;   // Month is 0 - 11
  timeinfo.tm_mday = t.day;
  timeinfo.tm_hour = t.hour;
  timeinfo.tm_min  = t.min;
  timeinfo.tm_sec  = t.sec;
  timeinfo.tm_isdst = -1;           // Disregard DST adjustments

  return (uint32_t) mktime(&timeinfo);
}

void set_epoch(uint32_t e) {
  time_t epoch_t = (time_t) e;
  struct tm *ti = gmtime(&epoch_t);

  datetime_t t = {.year  = (int16_t) (ti->tm_year + 1900),
                  .month = (int8_t) (ti->tm_mon + 1),
                  .day   = (int8_t) ti->tm_mday,
                  .dotw  = (int8_t) ti->tm_wday,
                  .hour  = (int8_t) ti->tm_hour,
                  .min   = (int8_t) ti->tm_min,
                  .sec   = (int8_t) ti->tm_sec
                 };
  rtc_set_datetime(&t);
}

bool datetime_to_epoch(int Y, int M, int D, int h, int m, int s, uint32_t *epoch) {
  struct tm timeinfo = {0};
  timeinfo.tm_year = Y - 1900;
  timeinfo.tm_mon  = M - 1;
  timeinfo.tm_mday = D;
  timeinfo.tm_hour = h;
  timeinfo.tm_min  = m;
  timeinfo.tm_sec  = s;
  timeinfo.tm_isdst = -1;

  // mktime() normalizes out-of-range fields (e.g., Feb 30 -> Mar 2), so if
  // anything changed the input wasn't a real date/time
  time_t e = mktime(&timeinfo);
  if( e < 0 || timeinfo.tm_year != Y - 1900 || timeinfo.tm_mon != M - 1
      || timeinfo.tm_mday != D || timeinfo.tm_hour != h
      || timeinfo.tm_min != m || timeinfo.tm_sec != s ) {
    return false;
  }

  *epoch = (uint32_t) e;
  return true;
}

String get_reset_reason() {
  RP2040::resetReason_t reason = rp2040.getResetReason();
  
  String explain = "unknown";
  if (reason == RP2040::PWRON_RESET) {
    // Power on and brown out are merged on the RP2040
    explain = "power on or brown out";
  } else if (reason == RP2040::RUN_PIN_RESET) {
    explain = "RUN pin triggered";
  } else if (reason == RP2040::SOFT_RESET) {
    explain = "software";
  } else if (reason == RP2040::WDT_RESET) {
    explain = "watchdog";
  }
  return explain;
}

void print_help() {
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

void print_version() {
  Serial.println(F(
    "Timestamp:" __TIMESTAMP__ "\n"
    "Compiled: " __DATE__ " " __TIME__ "\n"
  ));
}

void dump_status() {
  float v12 = read_voltage(PIN_SENSE_12V, SCALE_12);
  float v6 = read_voltage(PIN_SENSE_6V,  SCALE_6);
  float v9 = read_voltage(PIN_SENSE_9V, SCALE_9);
  
  Serial.print(F("Epoch: ")); Serial.println(get_epoch());
  Serial.print(F("12V: ")); Serial.print(v12,3);
  Serial.println("");
  Serial.print(F(" 9V: ")); Serial.print(v9,3);
  Serial.println("");
  Serial.print(F(" 6V: ")); Serial.print(v6,3);
  Serial.println("");
  Serial.print(F("Valon Lock: ")); Serial.println(lock_state ? "HIGH" : "LOW");
  Serial.print(F("Last Sync Pulse: ")); Serial.print((micros()-last_sync_micros)/1e6, 3); Serial.println(" s ago");
  Serial.print(F("MCU Temperature: ")); Serial.print(analogReadTemp()); Serial.println(" C");
  Serial.print(F("MCU Last Reset: ")); Serial.println(get_reset_reason());
}

// ---------- Setup & loop ----------
void setup() {
  // Setup the watchdog
  rp2040.wdt_begin(8000);

  // Serial setup
  Serial.begin(9600);
  while (!Serial && millis() < 4000) { }

  // Setup the ADC resolution
  analogReadResolution(12);

  // Set pins
  pinMode(PIN_LOCK, INPUT_PULLDOWN);
  pinMode(PIN_SYNC, INPUT_PULLDOWN);

  pinMode(PIN_NOT_FAULT, OUTPUT);
  digitalWrite(PIN_NOT_FAULT, 1);

  // Attach the interrupts
  attachInterrupt(digitalPinToInterrupt(PIN_LOCK), lock_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_SYNC), sync_isr, RISING);

  // Setup the RTC
  rtc_init();
  set_epoch(0);

  // LEDs
  pinMode(USB_LED_PIN, OUTPUT);  // USB activity
  pinMode(BUS_LED_PIN, OUTPUT);  // Bus activity
  digitalWrite(USB_LED_PIN, ACT_LED_OFF);
  digitalWrite(BUS_LED_PIN, ACT_LED_OFF);

  print_help();
}

void loop() {
  // Bypass mode
  if( in_bypass ) {
    while( Serial.available() ) {
      // escape sequence support
      static String esc;
      char c = (char)Serial.read();
      usb_activity();
      esc += c;
      if( esc.length() > 7 ) {
        esc.remove(0, esc.length() - 7);
      }
      if( esc.endsWith("+++exit") || esc.endsWith("+++EXIT") ) {
        in_bypass=false;
        Serial.println(F("\nOK: exit bypass"));
        esc="";
        break;
      }
      Serial1.write(c);
      bus_activity();
    }
    while( Serial1.available() ) {
      Serial.write(Serial1.read());
      bus_activity();
    }
    // Keep logging via ISRs even in bypass
  } else {
    // CLI line reader
    while( Serial.available() ) {
      char c = (char)Serial.read();
      last_char = millis();
      usb_activity();
      if( c == '\r' ) {
        continue;
      }
      if( c == '\n' ) {
        // Drop the whole line if it was too long to fit in the buffer
        if( cmd_overflow ) {
          Serial.println(F("ERR: too long"));
          cmd_overflow = false;
          cmd = "";
          continue;
        }

        cmd.trim();
        if( cmd.length() == 0) {
          continue;
        }
        cmd.toUpperCase();

        if( cmd.equals("HELP") || cmd.equals("?") ) {
          print_help();

        } else if( cmd.equals("VERSION") ) {
          print_version();

        } else if( cmd.equals("STATUS") ) {
          dump_status();

        } else if( cmd.equals("TIME GET") ) {
          Serial.print(F("EPOCH "));
          Serial.println(get_epoch());

        } else if( cmd.startsWith("TIME SET EPOCH") ) {
          unsigned long ep=0;
          if( sscanf(cmd.c_str(), "TIME SET EPOCH %lu", &ep) == 1 ) {
            set_epoch(ep);
            Serial.println(F("OK"));
          } else {
            Serial.println(F("ERR"));
          }

        } else if( cmd.startsWith("TIME SET ") ) {
          // TIME SET YYYY-MM-DD HH:MM:SS
          int Y,M,D,h,m,s;
          uint32_t ep;
          if( sscanf(cmd.c_str(), "TIME SET %d-%d-%d %d:%d:%d", &Y,&M,&D,&h,&m,&s) == 6
              && datetime_to_epoch(Y, M, D, h, m, s, &ep) ) {
            set_epoch(ep);
            Serial.println(F("OK"));
          } else {
            Serial.println(F("ERR"));
          }

        } else if( cmd.startsWith("BYPASS ON") ) {
          long baud = 115200;
          int sp = cmd.indexOf(' ', 9);
          if( sp > 0 ) {
            baud = cmd.substring(sp+1).toInt();
            if( baud <= 0 ) {
              baud = 115200;
            }
          }
          Serial1.begin(baud); // D6/D7 (GP0/GP1) on XIAO RP2040
          in_bypass = true;
          Serial.print(F("OK: bypass on @")); Serial.println(baud);
          Serial.println(F("(escape with '+++exit')"));

        } else if( cmd.equals("BYPASS OFF") ) {
          in_bypass=false;
          Serial.println(F("OK: bypass off"));

        } else if( cmd.equals("RESET") ) {
          Serial.println(F("Resetting..."));
          delay(50);
          rp2040.reboot();

        } else {
          Serial.println(F("ERR: unknown (HELP)"));
        }
        cmd = "";
      } else if( cmd.length() < MAX_CMD_LEN ) {
        cmd += c;
      } else {
        cmd_overflow = true;
      }
    }

    // Clear out any dangling text in the command buffer
    if (cmd.length() != 0 && millis() - last_char > CMD_TIMEOUT_MS) {
      cmd = "";
      cmd_overflow = false;
    }
  }

  // Periodic refreshing of sync pulse state
  if (millis() - t_sense >= 1000) {
    t_sense = millis();
    if( sync_seen ) {
      if( ((micros() - last_sync_micros)/1e6) > 6 ) {
        sync_seen = false;
      }
    }
  }

  // Turn off the activity LEDs once their hold time has passed
  if( usb_led && (millis() - usb_led_time) > ACT_LED_HOLD_MS ) {
    digitalWrite(USB_LED_PIN, ACT_LED_OFF);
    usb_led = 0;
  }
  if( bus_led && (millis() - bus_led_time) > ACT_LED_HOLD_MS ) {
    digitalWrite(BUS_LED_PIN, ACT_LED_OFF);
    bus_led = 0;
  }

  // Reset the watchdog
  rp2040.wdt_reset();
}
