/**
 * Clock controller based on a BKA30D-R5 / VID28-05 dual shaft stepper motor
 * Datasheet at https://cdck-file-uploads-europe1.s3.dualstack.eu-west-1.amazonaws.com/arduino/original/3X/b/0/b0aa1434329fd55ba59f10d853612d71be1a5b07.pdf
 */

// INCLUDES
// AccelStepper library for stepper motor control
// https://www.airspayce.com/mikem/arduino/AccelStepper/
#include <AccelStepper.h>
// For connecting to NTP time server
#include <ESP8266WiFi.h>
// Class for BKA30D pulse sequence control
#include "BKA30D.h"

// CONSTANTS
// The total number of steps required for one complete revolution of the stepper shaft
// From datasheet, in partial step mode each step = 1/3 of a degree, so NUM_STEPS = 360 * 3
constexpr uint16_t NUM_STEPS = 1080;
// There are 12 hours on the clock, so to move one hour requires NUM_STEPS / 12
constexpr uint16_t STEPS_PER_HOUR = 90;
// There are 60 minutes, so each minute requires NUM_STEPS / 60 = 12 steps
constexpr uint16_t STEPS_PER_MINUTE = 18;
// Sensor pin connected to this pin will be used to calirate position of hands
constexpr byte sensorPin = A0;
// Threshold value used to determine whether sensor is obscured by hand
constexpr uint16_t threshold = 500;
// Wi-Fi credentials to retrieve time from NTP server
constexpr char *wifiSSID     = "vodafone1236D8";
constexpr char *wifiPassword = "q3TGpAbgHL7KaYsp";

// GLOBALS
// Motor Control
// When viewed from front and shaft on the right hand side, BKA30D-R5 pins are as follows:
//    ______________
//   / B1 B2  A3 A4 \
//  |          (O)   |
//   \_B4 B3__A2 A1_/
//
// As A2/3 and B2/3 are always driven together, only 6 GPIO pins are needed.
BKA30DStepper minuteMotor(D1, D2, D5);
BKA30DStepper hourMotor(D4, D6, D7);
// Stepper wrappers to work around limitation that you can't pass 
// instance methods to the AccelStepper constructor  
void hourStepForward()   { hourMotor.stepForward(); }
void hourStepBackward()  { hourMotor.stepBackward(); }
void minuteStepForward() { minuteMotor.stepForward(); }
void minuteStepBackward(){ minuteMotor.stepBackward(); }
// Create AccelStepper based on wrapper functions above
AccelStepper hourStepper(hourStepForward, hourStepBackward);
AccelStepper minuteStepper(minuteStepForward, minuteStepBackward);
// Network
// Keep track of network connection state
enum : byte { WLAN_DOWN, WLAN_STARTING, WLAN_UP } connectionState;
byte networkState = WLAN_DOWN;
// WiFi network event handlers, see https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/generic-examples.html
WiFiEventHandler gotIpEventHandler, disconnectedEventHandler;
// Time
time_t now;                         // seconds since Epoch (1970) - UTC
tm tm;                              // Holds time information in a more convenient way

void setup() { 
  // Serial interface for debugging only
  Serial.begin(115200);
  Serial.println(__FILE__ __DATE__);
  // Callback when assigned an IP address
  gotIpEventHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& event) {
    Serial.print(F("Connected to "));
    Serial.print(wifiSSID);
    Serial.print(F(", IP:"));
    Serial.println(WiFi.localIP());
    networkState = WLAN_UP;
  });
  // Called when disconnected due to WiFi.disconnect() (because Wi-Fi signal is weak, or because the access point is switched off)
  disconnectedEventHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& event) {
    // Fix to ensure WiFi.status() reflects correct status
    // see https://github.com/esp8266/Arduino/issues/7432#issuecomment-895352866
    (void)event;
    WiFi.disconnect();  
    Serial.println("Disconnected from WiFi");
    networkState = WLAN_DOWN;
  });

  // Configure each stepper
  hourStepper.setMaxSpeed(200);
  minuteStepper.setMaxSpeed(400);
  hourStepper.setAcceleration(1200);
  minuteStepper.setAcceleration(2000);

  Serial.println("Calibrating...");
  unsigned long startCalibrationTime = millis();
  // Move hands out of the way
  while(millis() - startCalibrationTime < 10000 && isSensorBlocked()){
    hourStepper.move(1080);
    minuteStepper.move(-1080);
    while (hourStepper.distanceToGo() != 0 || minuteStepper.distanceToGo() != 0) {
      hourStepper.run();
      minuteStepper.run();
    }
    yield();
  }
  // Calibrate the hour hand
  calibrateHand(hourStepper, hourMotor);
  // Move the hour hand out of the way
  hourStepper.moveTo(NUM_STEPS/2);
  while(hourStepper.distanceToGo() != 0) {
    hourStepper.run();
    yield();
  }
  // Calibrate the minute hand
  calibrateHand(minuteStepper, minuteMotor);
  // Now move both hands to the origin
  hourStepper.moveTo(0);
  minuteStepper.moveTo(0);
  // And give them time to do it
  nonBlockingDelay(500);

  // Set timezone and sync to NTP
  // See timezone definitions in https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
  configTime("GMT0BST,M3.5.0/1,M10.5.0", "pool.ntp.org");
}

bool isSensorBlocked(){
  return analogRead(sensorPin) < threshold;
}

// Sets zero position of a hand
int calibrateHand(AccelStepper &stepper, BKA30DStepper &driver) {
  Serial.println(F("Starting calibration..."));

  stepper.setMaxSpeed(20.0F / 60.0F * NUM_STEPS); // 20 RPM
  stepper.move(NUM_STEPS * 3); // Enough steps to see the tab fully

  bool isBlocked = isSensorBlocked();
  bool wasBlocked = isBlocked;

  int entryStep = -1;
  int exitStep = -1;

  // Find position at which hand enters and exits sensor detection
  Serial.println(F("Looking for valid shadow pass..."));
  wasBlocked = false;

  while (stepper.distanceToGo() != 0) {
    stepper.run();
    isBlocked = isSensorBlocked();
    if (!wasBlocked && isBlocked) {
      entryStep = stepper.currentPosition();
      Serial.print(F("Detected entry at step: "));
      Serial.println(entryStep);
    }
    if (wasBlocked && !isBlocked && entryStep != -1) {
      exitStep = stepper.currentPosition();
      Serial.print(F("Detected exit at step: "));
      Serial.println(exitStep);
      break;
    }
    wasBlocked = isBlocked;
    yield();
  }
  // Set midpoint as current position
  if (entryStep != -1 && exitStep != -1) {
    int midpoint = (entryStep + exitStep) / 2;
    stepper.setCurrentPosition(stepper.currentPosition() - midpoint);
    Serial.print(F("Calibration successful. Midpoint = "));
    Serial.println(midpoint);
    return midpoint;
  }
  Serial.println(F("Calibration failed — shadow pass not detected."));
  return -1;
}

// The accelstepper run() function *must* be called continuously (even when you don't think the steppers have anything to do)
// so that means using the regular delay() function is a no-no.
// This alternative method performs no program logic, but continues to execute the stepper updates for the specified
// amount of time and is used in place of delay()
void nonBlockingDelay(unsigned long wait) {
  unsigned long waitUntil = millis() + wait;
  while (millis() < waitUntil) {
      hourStepper.run();
      minuteStepper.run();
      yield();
  }
}

/**
 * Main Program Loop
 */
void loop() {
  // Service the network connection
  networkLoop();
  // Keep track of current time
  time(&now);
  localtime_r(&now, &tm);
  // Decide whether the hands need to move
  static int prevHour = -1;
  static int prevMinute = -1;
  if(tm.tm_hour != prevHour || tm.tm_min != prevMinute){
    Serial.printf("Current time: %02d:%02d\n", tm.tm_hour, tm.tm_min);
    hourStepper.moveTo(tm.tm_hour * STEPS_PER_HOUR);
    minuteStepper.moveTo(tm.tm_min * STEPS_PER_MINUTE);
    prevHour = tm.tm_hour;
    prevMinute = tm.tm_min;
   }
  // Call the run() function on both steppers every frame to process any movement
  // required to reach their target position
  hourStepper.run();
  minuteStepper.run();
}

/**
 * A robust method that re-establishes connection to Wi-Fi WLAN network and MQTT server,
 * in the event of dropped connection/power loss etc.
 * See https://esp32.com/viewtopic.php?t=16109#p61846
 **/
void networkLoop() {
  // Keep track of time of last state change
  static unsigned long timeStamp;  

  switch (networkState) {
    // If there is no Wi-Fi connection
    case WLAN_DOWN:
      // Set ESP8266 Wi-Fi Configuration. See https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/generic-class.html
      WiFi.setSleepMode(WIFI_NONE_SLEEP); // Ensure broadcast data is not missed by going to sleep
      WiFi.setOutputPower(17.5); // Setting lower Tx power can reduce signal noise
      WiFi.mode(WIFI_STA); // Operate only in STA station mode (i.e. client), not also in AP access point mode (i.e. server)
      WiFi.persistent(false); // Don't write Wi-Fi credentials to flash
      // Start the connection
      Serial.print("Starting WiFi connection to ");
      Serial.println(wifiSSID);
      WiFi.begin(wifiSSID, wifiPassword);
      // Set the timer
      timeStamp = millis();
      // And advance the state machine to the next state
      networkState = WLAN_STARTING;
      break;
    // If the WLAN router connection was started
    case WLAN_STARTING:
      // Allow 30 seconds since attempting to join the WiFi
      if (millis() - timeStamp >= 30000) {
        // Otherwise, if the WLAN router connection was not established
        Serial.println("Failed to start WiFi. Restarting...");
        // Clear the connection for the next attempt
        // WiFi stack corruption is the biggest reason for WiFi connection failures
        WiFi.disconnect();
        // And reset the state machine to its initial state (restart)
        networkState = WLAN_DOWN;
      }
      break;
    // If the WLAN router connection was established
    case WLAN_UP:
      break;
  }
}
