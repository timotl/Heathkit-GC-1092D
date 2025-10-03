/*
    OTA - only active after touching bar with Date showing
          Only circuit modification:
          Dimmer - solder bridge IC101 555 pins 3-4
OTA password is wifi password
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
//#include <BluetoothSerial.h>
//BluetoothSerial SerialBT;

#define LOCALTZ_POSIX "MST+7MDT,M3.2.0/2,M11.1.0/2"  //MST timezone definition including DST On/Off
Timezone MST;


//Shift stuff
const uint8_t pinData = 13;  //shift register control lines
const uint8_t pinClock = 14;
const uint8_t pinLatch = 15;
time_t prevDisplay = 0;
int datecounter = 0;
int touchoverride = 0;
long int timeint = 0;
long int dateint = 0;
long int display = 0;
String timestring = "";
String datestring = "";
//int showdate = 0;           //show date or time
int blanking = 0;           //am I blank or not
String ontime = "073000";   //24 hour time
String offtime = "180000";  //24 hour time
String weekend = "0";
#define T_DISPLAY 750ul  //uS    display time for each digit
#define T_DEADBAND 25ul  //uS    blanking time between digits
#define NUM_DIGITS 6     //#     no. of display digits
#define SEGS_OFF 0x00    //      all segments off for blanking
#define SEG_INVERT 0x00  //      use 0xff to invert SEG outputs, 0x00 for std SEG output - 00 for MK5717
#define DIG_INVERT 0x00  //      use 0xff to invert DIG outputs, 0x00 for std DIG output - 00 for MK5717
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
  digits[] =  //Digit pin positions
  {
    0b00000001,
    0b00000010,
    0b00000100,
    0b00001000,
    0b00010000,
    0b00100000,
  };

//digits segment patterns stored here
int digit6 = 0;
int digit5 = 0;
int digit4 = 0;
int digit3 = 0;
int digit2 = 0;
int digit1 = 0;

 enum eDispStates {
   DEADBAND = 0,  //blanking period between segs to reduce/prevent "smear" or "ghosting"
   DIGIT_DISP     //digit is actively being displayed
 };

//end of shift stuff


//touch
int threshold = 20;
bool touchActive = false;
bool lastTouchActive = false;
bool testingLower = true;



void gotTouchEvent() {
  if (lastTouchActive != testingLower) {
    touchActive = !touchActive;
    testingLower = !testingLower;
    // Touch ISR will be inverted: Lower <--> Higher than the Threshold after ISR event is noticed
    touchInterruptSetThresholdDirection(testingLower);
  }
}

void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void setup() {
  setDebug(INFO);
  Serial.begin(115200);
  pinMode(pinData, OUTPUT);
  pinMode(pinClock, OUTPUT);
  pinMode(pinLatch, OUTPUT);

  touchAttachInterrupt(T0, gotTouchEvent, threshold);
  // Touch ISR will be activated when touchRead is lower than the Threshold
  touchInterruptSetThresholdDirection(testingLower);

  MST.setPosix(LOCALTZ_POSIX);
  setServer("time.angrywoodchuck.com");  //added 7.28.25
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  wifiManager.setHostname("Heathkit-Clock");
  WiFi.mode(WIFI_STA);
  //reset settings - for testing
  //wifiManager.resetSettings();

  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setConfigPortalTimeout(180);
  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect()) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(1000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  delay(2500);
  waitForSync(30);
  //End of Wifi manager

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
  shiftoutdigits(888888);

	setInterval(900);
  setDebug(INFO);
}  //End of setup


void loop() {
  ArduinoOTA.handle();
  if (now() != prevDisplay) {  //update the time strings every second
    prevDisplay = now();
    if ((MST.dateTime("l") == "Saturday" || MST.dateTime("l") == "Sunday") && touchoverride != 1) {
      weekend = "1";
      blanking = 1;
    } else {
      weekend = "0";
    }
    if (MST.dateTime("His") == offtime) {
      blanking = 1;
      touchoverride = 0;
    }
    if (MST.dateTime("His") == ontime && weekend == "0") { blanking = 0; }
    events();  //eztime required

    timestring = MST.dateTime("gis");
    timeint = timestring.toInt();

    datestring = MST.dateTime("ndy");
    dateint = datestring.toInt();


    if (datecounter > 0)  //are we showing date or time?
    {
      display = dateint;
      datecounter--;  //decrement counter
    } else {
      display = timeint;
    }
  }

  shiftoutdigits(display);  // always shiftout since not interrupt driven

  //Touch handler
  if (lastTouchActive != touchActive) {  //touch happened
    lastTouchActive = touchActive;
    if (touchActive && blanking == 1) {  //off and touched, turn on
      blanking = 0;
      touchoverride = 1;
      Serial.println("touched- was off");
    } else if (touchActive && datecounter != 0) {  //on and touched and cal active, shut off
      blanking = 1;
      datecounter = 0;
      ArduinoOTA.begin();  //we're going to only enable OTA here
      Serial.println("touched- was showing date - turning off");
    } else if (touchActive && blanking == 0) {  //on and touched, show cal
      blanking = 0;
      datecounter = 5;  //show date for 5 sec
      Serial.println("touched- now showing date");
    }
  }
}  //end of loop

void shiftoutdigits(int) {
  //get separate digits since it's an int
  digit6 = (display / 100000U) % 10;  //tens hours
  digit5 = (display / 10000U) % 10;   //ones hours
  digit4 = (display / 1000U) % 10;    //tens minutes
  digit3 = (display / 100U) % 10;     //ones minutes
  digit2 = (display / 10U) % 10;      //tens seconds
  digit1 = display % 10;              //ones seconds
  int timeValues[] = { digit6, digit5, digit4, digit3, digit2, digit1 };
  static uint8_t
    state = DIGIT_DISP,
    index = 0;
  static uint32_t
    tDisplay = 0ul;
  uint32_t tNow = micros();
  tNow = micros();
  switch (state) {
    case DEADBAND:
      if ((tNow - tDisplay) >= T_DEADBAND) {  //set up for timing the digit display
        //turn on the next digit
        //it has been preloaded so all we have to
        //do is latch it over
        digitalWrite(pinLatch, HIGH);
        tDisplay = micros();
        state = DIGIT_DISP;
        index++;
      }  //if

      break;

    case DIGIT_DISP:
      if ((tNow - tDisplay) >= T_DISPLAY) {
        //turn off current digit for blanking
        digitalWrite(pinLatch, LOW);
        shiftOut(pinData, pinClock, MSBFIRST, (digits[index] ^ DIG_INVERT));
        shiftOut(pinData, pinClock, MSBFIRST, (SEGS_OFF ^ SEG_INVERT));
        digitalWrite(pinLatch, HIGH);

        //index to the next digit
        //index++;
        if (index == NUM_DIGITS)
          index = 0;
        int numval = timeValues[index];
        //and preload the value for this next digit into the shift reg
        //  but don't raise latch yet; that is done after the blanking period
        digitalWrite(pinLatch, LOW);
        shiftOut(pinData, pinClock, MSBFIRST, (digits[index] ^ DIG_INVERT));
        //leading zero blanking
        if ((digit6 == 0 && digits[index] == 1) || blanking == 1) {
          shiftOut(pinData, pinClock, MSBFIRST, (SEGS_OFF ^ SEG_INVERT));
        } else {
          shiftOut(pinData, pinClock, MSBFIRST, (num[numval] ^ SEG_INVERT));
        }

        //we've blanked the current digit and have pre-loaded the
        //next digit's pattern; go time the blanking period
        tDisplay = micros();
        state = DEADBAND;
      }  //if
      break;
  }  //switch
}
