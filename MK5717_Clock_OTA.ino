/*
ESP32 sketch to replace MK5717 used in several HeathKit clocks
Heathkit GC-1092D circuit modification: Dimmer non-functional - solder bridge IC101 555 pins 3-4 to disable
All original switches still wired, but not used. Would require 2 additional inputs and level shifters and demux decoding
Display updates are state driven and writing to display is handled by hardware timer and background task
Touch switches from Time to Date, Date to off, off to on
Should work as built on the GC-1005 but no alarm output
OTA - only active after touching bar with Date showing (2 touches)
OTA mode triggers AP mode with password - falls back to STA after 5 min
Select ESP32-WROOM-DA Module
*/

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include "WiFiManager.h"  //https://github.com/tzapu/WiFiManager
#include <ezTime.h>
#include "Arduino.h"
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP32TimerInterrupt.h>


// === Global variables and constants ===

//User configurable stuff - Wifi parameters on/off times, weekend 
//Wifi will revert to setup AP mode if no connection available
//ezTime 
#define LOCALTZ_POSIX "MST+7MDT,M3.2.0/2,M11.1.0/2"  //MST timezone definition including DST On/Off
Timezone MST; //Timezone construct used in updateDisplayValue() and updateDisplayStateIfNeeded()
String ntpserver = pool.ntp.org;  //NTP server 
int ntpinterval = 500;  //NTP update interval
ezDebugLevel_t ntpdebug = INFO; //ezTime debug level

//time and day based blanking, on and off times
bool timeBlankingEnabled = true; //enable/disable time blanking
int ontime = 73000;   //24 hour time, no leading zero
int offtime = 180000;  //24 hour time, no leading zero
String weekendDay1 = "Saturday"; //days to blank display
String weekendDay2 = "Sunday";   //days to blank display
bool weekendBlankingEnabled = true; //enable/disable weekend blanking

//Wifi stuff
String wifihostname = "Heathkit-Clock"; //Hostname of ESP device
String wifisetupssid = "Heathkit-Clock Setup"; //Setup AP mode SSID - no password
String wifiotassid = "Heathkit-OTA"; //OTA AP mode SSID
String wifiotapass = "update123"; //OTA AP mode password

//Display stuff
const uint8_t pinData = 13;  //shift register control lines
const uint8_t pinClock = 14;
const uint8_t pinLatch = 15;
int datecounter = 0;
int showdatesec = 5;
int touchoverride = 0;
String timestring = "";
String datestring = "";

//Should not need to ever adjust these for this type of display and digit count
ESP32Timer ITimer1(1);
TaskHandle_t displayTaskHandle = nullptr;
const uint32_t T_INTERVAL = 525; //timer interval
const uint8_t  NUM_DIGITS = 6;
#define SEGS_OFF 0x00    //      all segments off for blanking
#define SEG_INVERT 0x00  //      use 0xff to invert SEG outputs, 0x00 for std SEG output - 00 for MK5717
#define DIG_INVERT 0x00  //      use 0xff to invert DIG outputs, 0x00 for std DIG output - 00 for MK5717
uint8_t displayBuffer[NUM_DIGITS] = { SEGS_OFF, SEGS_OFF, SEGS_OFF, SEGS_OFF, SEGS_OFF, SEGS_OFF };
//Segment pattern key
//   - a -
//  f    b
//  - g -
//  e    c
//  - d -
const uint8_t
  num[] =  //      Segment patterns for numbers
  {
    //Dgfedcba
    0b00111111,  //0
    0b00000110,  //1
    0b01011011,  //2
    0b01001111,  //3
    0b01100110,  //4
    0b01101101,  //5
    0b01111101,  //6
    0b00000111,  //7
    0b01111111,  //8
    0b01101111   //9
  };
const uint8_t
  chars[] =  //      Segment patterns for characters
  {
    //Dgfedcba
    0b01110111,  //A
    0b00111001,  //C
    0b01011000,  //c
    0b01011110,  //d
    0b01111001,  //E
    0b01010100,  //n
    0b01011100,  //o
    0b01110011,  //P
    0b01010000,  //r
    0b01101101,  //S
    0b01111000,  //t
    0b00111110,  //U
    0b00000000,  //blank
    0b01000000   //Dash
  };

#define CHAR_BLANK 12
#define CHAR_DASH 13

const uint8_t
  digits[] =  //Digit pin positions
  {
    0b00000001,
    0b00000010,
    0b00000100,
    0b00001000,
    0b00010000,
    0b00100000,
  };

 enum Phase {
   DEADBAND,  //blanking period between segs to reduce/prevent "smear" or "ghosting"
   DIGIT_DISP     //digit is actively being displayed
 };
volatile Phase phase = DEADBAND;

enum DisplayMode {
  MODE_TIME,
  MODE_DATE,
  MODE_STARTUP,
  MODE_CONNECT,
  MODE_SETUP,
  MODE_BLANK,
  MODE_DASHES
};
DisplayMode currentDisplayMode = MODE_BLANK;
//end of display stuff

//touch
int threshold = 20;
bool touchActive = false;
bool lastTouchActive = false;
bool testingLower = true;

//Wifi OTA fallback
bool otaModeActive = false;         // Tracks whether OTA mode is currently active
unsigned long otaStartTime = 0;     // Timestamp when OTA mode was triggered
const unsigned long otaTimeout = 300000; // Timeout duration (e.g., 5 minutes)
// === End of global variables and constants ===

// === Forward function declarations ===
bool IRAM_ATTR onDisplayTimer(void*);
void displayTask(void* pvParameters);
void gotTouchEvent();
void handleTouchEvent();
void configModeCallback(WiFiManager *myWiFiManager);
void monitorAndRecoverNtp();
void updateDisplayValue();
void shiftoutdigits();
void updateDisplayStateIfNeeded();
void checkOtaTimeout();
// =====================================


void setup() {
  Serial.begin(115200);
  pinMode(pinData, OUTPUT);
  pinMode(pinClock, OUTPUT);
  pinMode(pinLatch, OUTPUT);
  xTaskCreatePinnedToCore(
  displayTask,         // Task function
  "DisplayTask",       // Name
  2048,                // Stack size
  NULL,                // Parameters
  2,                   // Priority (higher than idle)
  &displayTaskHandle,  // Handle
  1                    // Core (1 = App core, 0 = Pro core)
  );

  ITimer1.attachInterruptInterval(T_INTERVAL, onDisplayTimer);
  touchAttachInterrupt(T0, &gotTouchEvent, threshold);
  // Touch ISR will be activated when touchRead is lower than the Threshold
  touchInterruptSetThresholdDirection(testingLower);
  //Startup display
  currentDisplayMode = MODE_STARTUP;
  updateDisplayValue();
  MST.setPosix(LOCALTZ_POSIX);
  setServer(ntpserver);
  setInterval(ntpinterval);
  setDebug(ntpdebug);
  
  WiFiManager wifiManager;
  wifiManager.setHostname(wifihostname);
  WiFi.mode(WIFI_STA);
  // Optional: reset settings for testing
  // wifiManager.resetSettings();

  // Set callback when entering AP mode
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setConfigPortalTimeout(180);

  // Attempt to connect, fallback to AP if needed
  if (!wifiManager.autoConnect(wifisetupssid.c_str())) {
    Serial.println("failed to connect and hit timeout");
    ESP.restart();
    delay(1000);
  }

  Serial.println("connected...");
  currentDisplayMode = MODE_CONNECT;
  updateDisplayValue();
  delay(2500); // give connection time to stabilize
  waitForSync(30);

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else  // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  currentDisplayMode = MODE_TIME;
  updateDisplayValue();
}  //End of setup

void loop() {
  ArduinoOTA.handle();
  checkOtaTimeout();
  updateDisplayStateIfNeeded();
  monitorAndRecoverNtp();
  handleTouchEvent();
}//End of loop

bool IRAM_ATTR onDisplayTimer(void*) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(displayTaskHandle, &xHigherPriorityTaskWoken);
  return true;
}

void displayTask(void* pvParameters) {
  for (;;) {
    // Wait for notification from ISR
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Perform display update here
    static uint8_t index = 0;
    digitalWrite(pinLatch, LOW);
    shiftOut(pinData, pinClock, MSBFIRST, 0x00);
    shiftOut(pinData, pinClock, MSBFIRST, 0x00);
    digitalWrite(pinLatch, HIGH);
    delayMicroseconds(25); // still not ideal, but now outside ISR
    digitalWrite(pinLatch, LOW);
    shiftOut(pinData, pinClock, MSBFIRST, (digits[index] ^ DIG_INVERT));
    shiftOut(pinData, pinClock, MSBFIRST, (displayBuffer[index] ^ SEG_INVERT));
    digitalWrite(pinLatch, HIGH);
    if (++index >= NUM_DIGITS) index = 0;
  }
}

void gotTouchEvent() {
  if (lastTouchActive != testingLower) {
    touchActive = !touchActive;
    testingLower = !testingLower;
    // Touch ISR will be inverted: Lower <--> Higher than the Threshold after ISR event is noticed
    touchInterruptSetThresholdDirection(testingLower);
  }
}

void handleTouchEvent() {
  if (lastTouchActive != touchActive) {
    lastTouchActive = touchActive;

    if (touchActive && currentDisplayMode == MODE_BLANK) {
      currentDisplayMode = MODE_TIME;
      touchoverride = 1;
      datecounter = 0;
      Serial.println("touched - was off");
    }
    else if (touchActive && currentDisplayMode == MODE_DATE) {
      currentDisplayMode = MODE_BLANK;
      datecounter = 0;
      WiFi.disconnect();
      WiFi.mode(WIFI_AP);
      WiFi.softAP(wifiotassid.c_str(), wifiotapass.c_str());
      ArduinoOTA.begin();
      otaModeActive = true;
      otaStartTime = millis();
      Serial.println("touched - was showing date, now off OTA mode active");
    }
    else if (touchActive && currentDisplayMode == MODE_TIME) {
      currentDisplayMode = MODE_DATE;
      datecounter = showdatesec;
      touchoverride = 0; //Clear override on press - Needs testing
      Serial.println("touched - now showing date");
    }
  }
}

void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("Entered setup mode");
  //Setup display
  currentDisplayMode = MODE_SETUP;
  updateDisplayValue();
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void updateDisplayValue() {
  switch (currentDisplayMode) {
    case MODE_TIME: {
      for (int i = 0; i < NUM_DIGITS; i++) {
        int digit = timestring.charAt(i) - '0';
        if (i == 0 && digit == 0) {
          displayBuffer[i] = SEGS_OFF; // blank leading zero
        } else {
          displayBuffer[i] = num[digit];
        }
      }
      break;
    }
    case MODE_DATE: {
      for (int i = 0; i < NUM_DIGITS; i++) {
        int digit = datestring.charAt(i) - '0';
        if (i == 0 && digit == 0) {
          displayBuffer[i] = SEGS_OFF; // blank leading zero
        } else {
          displayBuffer[i] = num[digit];
        }
      }
      break;
    }
    case MODE_STARTUP: {
      const uint8_t msg[] = { 9, 10, 0, 8, 10, CHAR_BLANK}; // S t A r t
      for (int i = 0; i < sizeof(msg); i++) {
        displayBuffer[i] = chars[msg[i]];
      }
      break;
    }
    case MODE_CONNECT: {
      const uint8_t msg[] = { 1, 6, 5, 2, 10, 3 }; // C o n c t d
      for (int i = 0; i < sizeof(msg); i++) {
        displayBuffer[i] = chars[msg[i]];
      }
      break;
    }
    case MODE_SETUP: {
      const uint8_t msg[] = { 9, 4, 10, 11, 7, CHAR_BLANK }; // S E t U P
      for (int i = 0; i < sizeof(msg); i++) {
        displayBuffer[i] = chars[msg[i]];
      }
      break;
    }
    case MODE_BLANK: {
      for (int i = 0; i < NUM_DIGITS; i++) {
        displayBuffer[i] = chars[CHAR_BLANK];
      }
      break;
    }
    case MODE_DASHES: {
     for (int i = 0; i < NUM_DIGITS; i++) {
        displayBuffer[i] = chars[CHAR_DASH];
      }
      break;
    }
  }
}

void updateDisplayStateIfNeeded() {
  if (secondChanged()) {
    events(); // ezTime tick
    // Get time and date for determining weekend and time-based blanking
    int currentTime = MST.dateTime("His").toInt();  // HHMMSS as int
    String currentDay = MST.dateTime("l");
    bool isWeekend = weekendBlankingEnabled && (currentDay == weekendDay1 || currentDay == weekendDay2);
    bool withinDisplayWindow = (!timeBlankingEnabled) || (currentTime >= ontime && currentTime < offtime);

    // Weekend blanking
    if (isWeekend && touchoverride != 1) {
      currentDisplayMode = MODE_BLANK;
    }
    // Time-based blanking and restore (only if timeBlankingEnabled)
    if (!isWeekend && !withinDisplayWindow && touchoverride != 1) {
      currentDisplayMode = MODE_BLANK;
      touchoverride = 0;
    }

    if (!isWeekend && withinDisplayWindow && touchoverride == 0) {
      currentDisplayMode = MODE_TIME;
      timestring = MST.dateTime("his"); // HHMMSS
    }

    // Date override
    if (datecounter > 0 && currentDisplayMode != MODE_BLANK) {
      currentDisplayMode = MODE_DATE;
      datestring = MST.dateTime("mdy"); // MMDDYY
      datecounter--;
    } else if (currentDisplayMode != MODE_BLANK) {
      currentDisplayMode = MODE_TIME;
      timestring = MST.dateTime("his"); // HHMMSS
    }
    
    updateDisplayValue();
  }
}

void monitorAndRecoverNtp() {
  static unsigned long lastCheck = 0;

  //if (currentDisplayMode != MODE_TIME) return;
  if (millis() - lastCheck < ntpinterval * 1000) return;
  lastCheck = millis();

  time_t nowTime = now();
  time_t lastNtp = lastNtpUpdateTime();

  if ((nowTime - lastNtp) > (ntpinterval * 3)) {
    Serial.println("NTP stale. Attempting resync...");
    currentDisplayMode = MODE_DASHES;
    updateDisplayValue();
    if (!waitForSync(5)) {
      Serial.println("Resync failed. Restarting WiFi...");

      WiFi.disconnect();
      WiFi.mode(WIFI_OFF);
      delay(1000);

      WiFi.mode(WIFI_STA);
      WiFi.begin();

      if (!waitForSync(10)) {
        Serial.println("Still no NTP. Rebooting...");
        ESP.restart();
      } else {
        currentDisplayMode = MODE_TIME;

      }
    }
  }
}

void checkOtaTimeout() {
  if (otaModeActive && millis() - otaStartTime > otaTimeout) {
    Serial.println("OTA timeout exceeded. Reverting to STA mode...");
    WiFi.softAPdisconnect(true); // Shut down OTA AP
    WiFi.mode(WIFI_STA);        // Return to station mode
    WiFi.begin();               // Reconnect using stored credentials
    otaModeActive = false;      // Reset OTA mode flag
  }
}
