/* PABUcode_v2025.2.ino
  Code for logging Painted Buntings at feeders
  Settings based on best guesses at what might work
*/

#include "esp_system.h"
#include <SPI.h>    // SD card
#include <SdFat.h>  // Adafruit fork of SdFat for SD card
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Adafruit_GFX.h>
#include <Adafruit_AHTX0.h>    // temp/humidity
#include <Adafruit_NAU7802.h>  // load cell ADC
#include "RTClib.h"            // Real-time clock (PCF8523)
#include <PCA9536D.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "Adafruit_BMP3XX.h"
#include "Adafruit_MLX90393.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "esp_sleep.h"
#include <Dusk2Dawn.h>  // Download using zip file, not IDE installer (https://github.com/dmkishi/Dusk2Dawn?tab=readme-ov-file)
#include <stdlib.h>

// Feeder lat/long
float LATITUDE = 33.89;    // degrees
float LONGITUDE = -83.36;  // degrees
                           // Time difference, in minutes, between sunrise/sunset and sleep time
int SLEEP_AFTER_SUNSET_MIN = 10;
int WAKE_BEFORE_SUNRISE_MIN = 10;
int timeZone = -5;

// If using multiple feeders, change feeder name to be unique for each feeder
String feederName = "Whitehall_test";  //Name of the feeder, change freely


// Tresholds for starting/ending visits based on loadcell readings
float loadCellThresholdStart = 70000;  //absolute increase in loadcell value needed to start visit (14,000 ~= 1g)
float loadCellThresholdEnd = 0.25;     //% decrease in loadcell value needed to end visit
float alphaFast = 0.7;                 // speed at which new values update loadcell mean during visit; values close to 0 update slowy (i.e., new readings don't change mean very much), values close to 1 update very quickly (mean highly influenced by new values)
float alphaSlow = 0.05;
float alphaBaseline = 0.05;
float baselineStableThreshold = 5000;

// If loadcell not working, loops needed to end visit based on consecutive loops w/o seeing RFID tag
int loopsBetweenReads = 20;  //Wait this many loops before ending visit after last tag read; tweak to adjust sensitivity of "depature"

// Loadcell reading parameters
const int loadCellBufferSize = 10;  // How many values loadcell values are used to calculate running baseline mean between visits
const int maxReadingCount = 50;     // Maximum number of loadcell readings to record during one visit
int endVisitSensitivity = 5;        // how many loops must loadcell drop below visitMean before ending visit? Adjusts sensitivity of ending visit based on loadcell deviations

//The following variables can be modified to manipulate the web server.
char* default_ssid = "DefaultSSID";
char* default_password = "password1";
char* default_apssid = "birdfeederAccessPoint";
char* default_appassword = "password1";
String ssid;
String password;
String apssid;
String appassword;

// Intervals between checks (mainly used for power saving)
uint32_t rtcLogIntervalSec = 900;      // log temp/battery interval, in seconds (default is 15 min)
uint32_t deepSleepCheckInterval = 60;  // interval to check deep sleep in seconds (default is 1 min)
uint32_t checkVisitInterval = 500;     // ms between checkVisit() when no active visit (should be > ~200-300 to avoid light sleep issues)
uint32_t maxWifiOffTime = 300000;

struct Setting {
  const char* key;
  String value;
};

Setting tweakableVars[] = {
  { "LATITUDE", String(LATITUDE) },
  { "LONGITUDE", String(LONGITUDE) },
  { "SLEEP_AFTER_SUNSET_MIN", String(SLEEP_AFTER_SUNSET_MIN) },
  { "WAKE_BEFORE_SUNRISE_MIN", String(WAKE_BEFORE_SUNRISE_MIN) },
  { "timeZone", String(timeZone) },
  { "feederName", String(feederName) },
  { "loadCellThresholdStart", String(loadCellThresholdStart) },
  { "loadCellThresholdEnd", String(loadCellThresholdEnd) },
  { "alphaFast", String(alphaFast) },
  { "alphaSlow", String(alphaSlow) },
  { "alphaBaseline", String(alphaBaseline) },
  { "baselineStableThreshold", String(baselineStableThreshold) },
  { "loopsBetweenReads", String(loopsBetweenReads) },
  { "endVisitSensitivity", String(endVisitSensitivity) },
  { "default_ssid", String(default_ssid) },
  { "default_password", String(default_password) },
  { "default_apssid", String(default_apssid) },
  { "default_appassword", String(default_appassword) },
  { "rtcLogIntervalSec", String(rtcLogIntervalSec) },
  { "deepSleepCheckInterval", String(deepSleepCheckInterval) },
  { "checkVisitInterval", String(checkVisitInterval) },
  { "maxWifiOffTime", String(maxWifiOffTime) }
};
//Variables beyong this point should NOT be modified unless specified elsewhere
static float loadCellBuffer[maxReadingCount];
static unsigned long timeBuffer[maxReadingCount];
bool tagCaptured = false;
unsigned long visitStartTime = 0;
unsigned long visitEndTime = 0;  // timestamp of last reading (update always)
String tagNum = "";
String lastTagRead = "";
String visitID = "";
int loopsSinceLastRead = 0;
int readingCount = 0;
int visit = 0;
int loadcell = 0;
int loadcellDiff = 0;
double runningSumBaseline = 0;
double loadCellBaselineMean = 0;
double loadCellVisitMeanFast = 0;
double loadCellVisitMeanSlow = 0;
bool visitActive = false;  // true while animal is on scale
bool loggingEnabled = false;

// Track time remaining until next sleep checks or data logging
DateTime nextRTCLog;          // next time to log temp/battery
DateTime nextDeepSleepCheck;  // next time to evaluate deep-sleep window
unsigned long nextVisitCheckMs;
unsigned long nextWifiShutoffMs;
bool serverAPOn = false;  //If serverAP is on
bool wifiActive = false;

#define SD_CS 10       // SD card chip select
#define I2C_PWR_PIN 7  // pin to turn on/off I2C power, set high for on
SdFat SD;              // required with SdFat.h, not SD.h
SdFile myFile;         // not sure why SdFile and not just File
File32 myFileLoadCell;
File32 myFileHousekeeping;
File32 myFileError;
File32 myFileTempature;
File32 myFileSleep;
String fname = "/settings";  // forward slash critical for ESP32 function
String fname2;
String fname3;
String fname4;
String fname5;
String baseName = "/data";
String ext = ".csv";
String settingFname = "/settings0.csv";  // forward slash critical for ESP32 function
int lastFileNum = 0;
Adafruit_NAU7802 nau;  // load cell
Adafruit_AHTX0 aht;    // Temperature / humidity
RTC_PCF8523 rtc;

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define MLX90393_CS 10

Adafruit_INA219 solar1(0x40);
Adafruit_INA219 solar2(0x44);
Adafruit_INA219 onlyPwr(0x42);
Adafruit_INA219 dataPwr(0x45);
#define BATT_LOG_PERIOD 5000  // ms between battery logs

PCA9536 buzzLED;
#define RED 0
#define GRN 1
#define BLU 2
#define BUZZ 3

Adafruit_BMP3XX bmp;

Adafruit_MLX90393 sensor = Adafruit_MLX90393();

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

struct loadPoint {
  DateTime t;
  int32_t load;
  unsigned long millis;
};

WebServer* serverAP;

#ifdef LED_BUILTIN
const int led = LED_BUILTIN;
#else
const int led = 13;
#endif


/* This method adds padding zeros to ensure data is the same size 
*/
String zeroPad(int ii) {
  String outStr = "";
  if (ii < 10) {
    outStr += "0";
  }
  outStr += String(ii);
  return (outStr);
}


/* This method handles the web servers and gives visual feedbakc for this
*/
void handleRoot(WebServer* server,
                const String& content) {
  digitalWrite(led, 1);
  server->send(200, "text/html", content);
  digitalWrite(led, 0);
}

/*Method for handling downloads
*/
void handleDownload0() {

  if (serverAP->args() == 0 || !serverAP->hasArg("filename")) {
    serverAP->send(400, "text/plain", "Need argument: ?filename=yourfile.txt");
    return;
  }

  String fnameNew = "/" + serverAP->arg("filename");

  Serial.println("Downloading file: " + fnameNew);

  File32 fptr;
  if (!fptr.open(fnameNew.c_str(), O_RDONLY)) {
    serverAP->send(404, "text/plain", "File not found.");
    return;
  }

  // Get filename for Content-Disposition
  char fnameChar[64];
  fptr.getName(fnameChar, sizeof(fnameChar));

  // Prepare HTTP headers
  serverAP->sendHeader("Content-Type", "text/plain");
  serverAP->sendHeader("Content-Disposition", "attachment; filename=" + String(fnameChar));
  serverAP->setContentLength(fptr.fileSize());
  serverAP->send(200, "text/plain", "");  // headers only

  // Stream the file manually
  uint8_t buf[1024];
  while (true) {
    int n = fptr.read(buf, sizeof(buf));
    if (n <= 0) break;
    serverAP->client().write(buf, n);
  }

  fptr.close();
}



/* handleFileExample0

   This shows how to process arguments and create
   a file that the client will be asked to download.
   It expects a URL like: 
   http://birds.local:8080/file?asdf=5&something=else
*/
void handleFileExample0() {
  String message = "";
  message += "# args: ";
  message += serverAP->args();
  message += "\n";
  for (int ii = 0; ii < serverAP->args(); ii++) {
    message += "Arg " + String(ii) + " -> ";
    message += serverAP->argName(ii) + ": ";
    message += serverAP->arg(ii) + "\n";
  }
  serverAP->sendContent("HTTP/1.1 200 OK\n");
  serverAP->sendContent("Content-Type: text/plain\n");
  serverAP->sendContent("Content-Disposition: attachment; filename=junk.txt\n");
  serverAP->sendContent("Content-Length: " + String(message.length()) + "\n");
  serverAP->sendContent("Connection: close\n");
  serverAP->sendContent("\n");  // to indicate headers have ended
  // serverAP->sendHeader("Content-Disposition", "attachment; filename=junk.txt");
  serverAP->sendContent(message);
  // serverAP->send(200,"text/plain", message.c_str());
}

String createForm(String variableName) {
  return "<form action=\"/" + variableName + "Form\" method=\"get\">" + "<label for=\"" + variableName + "Value\">" + variableName + "</label>" + "<input type=\"text\" id=\"" + variableName + "Value\" name=\"" + variableName + "Value\">" + "<input type=\"submit\" value=\"Submit\">" + "</form>";
}

void handleRoot0() {
  serverAP->setContentLength(CONTENT_LENGTH_UNKNOWN);
  serverAP->send(200, "text/html", "");

  //Header
  serverAP->sendContent("<html><head><title>Birdfeeder</title></head><body><h1>Birdfeeder Web Server</h1>");

  //APPSID
  String info = "<strong>AP SSID: </strong>" + String(apssid) + "<br>";
  serverAP->sendContent(info);

  int length = sizeof(tweakableVars) / sizeof(tweakableVars[0]);
  for (int i = 0; i < length; i++) {  //Tweak Var Forms
    serverAP->sendContent(createForm(tweakableVars[i].key));
    serverAP->sendContent(" Current: " + tweakableVars[i].value + "<br><br>");
  }

  // Restart Button
  serverAP->sendContent("<form action='/restart' method='post'><input type='submit' value='Restart ESP32'></form>");
  serverAP->sendContent("</body></html>");
}


void handleNotFound(WebServer* server) {
  digitalWrite(led, 1);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server->uri();
  message += "\nMethod: ";
  message += (server->method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server->args();
  message += "\n";
  for (uint8_t i = 0; i < server->args(); i++) {
    message += " " + server->argName(i) + ": " + server->arg(i) + "\n";
  }
  server->send(404, "text/plain", message);
  digitalWrite(led, 0);
}

void handleNotFound0() {
  handleNotFound(serverAP);
}

int parseSetting(File32* fptr, char* str, int size, char delim) {
  char ch;
  int rtn;
  size_t n = 0;
  bool comment = false;
  bool leadingWhitespace = false;
  while (true) {
    // check for EOF
    if (!fptr->available()) {
      rtn = 0;
      break;
    }
    // read next character
    if (fptr->read(&ch, 1) != 1) {
      // read error
      rtn = -1;
      break;
    }

    // Delete CR.

    if (ch == '\r') {
      continue;
    }

    // ignore comment lines

    if (n == 0 && ch == '#') {
      comment = true;
      continue;
    }

    // end of comment line

    if (comment == true && ch == '\n') {
      rtn = '#';
      break;
    }

    // keep ignoring to the end of the line for comments

    if (comment == true) {
      continue;
    }

    // ignore leading spaces

    if (n == 0 && (ch == ' ' || ch == '\t')) {
      leadingWhitespace = true;
      continue;
    }

    // clip continued white space

    if (leadingWhitespace && (ch == ' ' || ch == '\t')) {
      continue;
    }

    // turn off clipping white space

    if (ch != ' ' && ch != '\t') {
      leadingWhitespace = false;
    }

    // end of parsed word

    if (ch == delim || ch == '\n') {
      rtn = ch;
      break;
    }

    if ((n + 1) >= size) {
      // string too long
      rtn = -2;
      n--;
      break;
    }

    str[n++] = ch;
  }

  str[n] = '\0';

  return (rtn);
}

void loadSettings(String settingFname) {
  File32 fptr;
  if (!fptr.open(settingFname.c_str(), O_RDONLY)) {
    Serial.println(F("No config file found."));
    updateSettings(settingFname);
    return;
  }

  char keyBuf[40];
  char valBuf[80];

  // Read Setting
  while (parseSetting(&fptr, keyBuf, sizeof(keyBuf), ':') > 0) {

    // Read the value (up to the newline)
    if (parseSetting(&fptr, valBuf, sizeof(valBuf), '\n') > 0) {

      // Look for the key in array
      for (int i = 0; i < sizeof(tweakableVars) / sizeof(tweakableVars[0]); i++) {
        if (String(tweakableVars[i].key).equalsIgnoreCase(String(keyBuf))) {
          tweakableVars[i].value = String(valBuf);
          break;
        }
      }
    }
  }
  fptr.close();
  syncGlobalsFromStruct();
}

void syncGlobalsFromStruct() {
  for (int i = 0; i < sizeof(tweakableVars) / sizeof(tweakableVars[0]); i++) {
    String k = tweakableVars[i].key;
    String v = tweakableVars[i].value;

    // Floating point vars
    if (k == "LATITUDE") LATITUDE = v.toFloat();
    else if (k == "LONGITUDE") LONGITUDE = v.toFloat();
    else if (k == "alphaFast") alphaFast = v.toFloat();
    else if (k == "alphaSlow") alphaSlow = v.toFloat();
    else if (k == "alphaBaseline") alphaBaseline = v.toFloat();
    else if (k == "loadCellThresholdStart") loadCellThresholdStart = v.toFloat();
    else if (k == "loadCellThresholdEnd") loadCellThresholdEnd = v.toFloat();
    else if (k == "baselineStableThreshold") baselineStableThreshold = v.toFloat();
    else if (k == "endVisitSensitivity") endVisitSensitivity = v.toFloat();

    //Int Vars
    else if (k == "SLEEP_AFTER_SUNSET_MIN") SLEEP_AFTER_SUNSET_MIN = v.toInt();
    else if (k == "WAKE_BEFORE_SUNRISE_MIN") WAKE_BEFORE_SUNRISE_MIN = v.toInt();
    else if (k == "timeZone") timeZone = v.toInt();
    else if (k == "loopsBetweenReads") loopsBetweenReads = v.toInt();
    else if (k == "rtcLogIntervalSec") rtcLogIntervalSec = v.toInt();
    else if (k == "deepSleepCheckInterval") deepSleepCheckInterval = v.toInt();
    else if (k == "checkVisitInterval") checkVisitInterval = v.toInt();
    else if (k == "maxWifiOffTime") maxWifiOffTime = v.toInt();

    else if (k == "feederName") {
      String feederName = String(v);
    } else if (k == "default_ssid") {
      ssid = String(v);
    } else if (k == "default_password") {
      password = String(v);
    } else if (k == "default_apssid") {
      apssid = String(v);
    } else if (k == "default_appassword") {
      appassword = String(v);
    }
  }
  Serial.println("Globals synchronized.");
}

bool loadCellInit() {
  if (!nau.begin()) {
    Serial.println("Failed to find NAU7802");
    return (false);
  }
  Serial.println("Found NAU7802");

  nau.setLDO(NAU7802_3V0);
  Serial.print("LDO voltage set to ");
  switch (nau.getLDO()) {
    case NAU7802_4V5:
      Serial.println("4.5V");
      break;
    case NAU7802_4V2:
      Serial.println("4.2V");
      break;
    case NAU7802_3V9:
      Serial.println("3.9V");
      break;
    case NAU7802_3V6:
      Serial.println("3.6V");
      break;
    case NAU7802_3V3:
      Serial.println("3.3V");
      break;
    case NAU7802_3V0:
      Serial.println("3.0V");
      break;
    case NAU7802_2V7:
      Serial.println("2.7V");
      break;
    case NAU7802_2V4:
      Serial.println("2.4V");
      break;
    case NAU7802_EXTERNAL:
      Serial.println("External");
      break;
  }

  nau.setGain(NAU7802_GAIN_128);
  Serial.print("Gain set to ");
  switch (nau.getGain()) {
    case NAU7802_GAIN_1:
      Serial.println("1x");
      break;
    case NAU7802_GAIN_2:
      Serial.println("2x");
      break;
    case NAU7802_GAIN_4:
      Serial.println("4x");
      break;
    case NAU7802_GAIN_8:
      Serial.println("8x");
      break;
    case NAU7802_GAIN_16:
      Serial.println("16x");
      break;
    case NAU7802_GAIN_32:
      Serial.println("32x");
      break;
    case NAU7802_GAIN_64:
      Serial.println("64x");
      break;
    case NAU7802_GAIN_128:
      Serial.println("128x");
      break;
  }

  nau.setRate(NAU7802_RATE_10SPS);
  Serial.print("Conversion rate set to ");
  switch (nau.getRate()) {
    case NAU7802_RATE_10SPS:
      Serial.println("10 SPS");
      break;
    case NAU7802_RATE_20SPS:
      Serial.println("20 SPS");
      break;
    case NAU7802_RATE_40SPS:
      Serial.println("40 SPS");
      break;
    case NAU7802_RATE_80SPS:
      Serial.println("80 SPS");
      break;
    case NAU7802_RATE_320SPS:
      Serial.println("320 SPS");
      break;
  }

  // Take 10 readings to flush out readings
  for (uint8_t i = 0; i < 10; i++) {
    while (!nau.available()) delay(1);
    nau.read();
  }

  while (!nau.calibrate(NAU7802_CALMOD_INTERNAL)) {
    Serial.println("Failed to calibrate internal offset, retrying!");
    delay(1000);
  }
  Serial.println("Calibrated internal offset");

  while (!nau.calibrate(NAU7802_CALMOD_OFFSET)) {
    Serial.println("Failed to calibrate system offset, retrying!");
    delay(1000);
  }
  Serial.println("Calibrated system offset");
  return (true);
}

bool tempHumidityInit() {
  if (aht.begin()) {
    Serial.println("Found AHT20");
    return (true);
  } else {
    Serial.println("Didn't find AHT20");
    return (false);
  }
}

String findFname(String name) {
  String fname = name + String(lastFileNum) + ext;
  while (SD.exists(fname)) {
    fname = name + String(++lastFileNum) + ext;
  }
  return (fname);
}

String curTimeStr(DateTime now) {
  //DateTime now = rtc.now();

  String outStr = "";
  outStr += zeroPad(now.month());
  outStr += '/';
  outStr += zeroPad(now.day());
  outStr += '/';
  outStr += now.year();
  outStr += " ";
  outStr += zeroPad(now.hour());
  outStr += ':';
  outStr += zeroPad(now.minute());
  outStr += ':';
  outStr += zeroPad(now.second());

  return (outStr);
}

void printCurTime() {
  DateTime now = rtc.now();

  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print('/');
  Serial.print(now.year(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.print(" unix = ");
  Serial.println(now.unixtime());
}

// callback for SD card write times
void dateTime(uint16_t* date, uint16_t* time) {
  DateTime now = rtc.now();

  *date = FAT_DATE(now.year(), now.month(), now.day());
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

void handleForm(WebServer* server, String inputName) {
  Serial.println("Received input: " + inputName);
  if (server->hasArg(inputName)) {
    String input = server->arg(inputName);
    Serial.println("Received input: " + input);
  }
  Serial.println(inputName);
  updateSettings(settingFname);
  handleRoot0();  //  Setup to make the page refresh and update with new content when a form is submitted
}

void checkWifi(int retryConnect = 1, int retryInterval = 0) {
  // Checks for WiFi availability and connects to it.
  String outStr = "";
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (((currentMillis - previousMillis) >= retryInterval) && (WiFi.status() != WL_CONNECTED)) {
    previousMillis = currentMillis;

    WiFi.begin(ssid, password);
    Serial.print("Attempting to Connect to WiFi access point\n");

    int nWifiCheck = 0;

    while ((WiFi.status() != WL_CONNECTED) && (nWifiCheck++ < retryConnect)) {
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      // outStr += "\n";
      IPAddress curIP = WiFi.localIP();
      outStr += "Connected to \"";
      outStr += ssid;
      outStr += "\", IP address: ";
      outStr += curIP.toString();
      // outStr += "\n";
      Serial.println(outStr);
      //("$Network", outStr);
      syncRTCTime();
    } else {
      WiFi.disconnect();
      outStr += "\nFailed to connect to WiFi access point ";
      outStr += ssid;
      Serial.print(outStr);
      errorWrite("$Error", outStr);
    }
  }
}

DateTime strToDateTime(String str) {
  if (!(str.length() >= 19)) {
    Serial.println("I received" + str);
    return rtc.now();
  }

  // Extract individual components from the string
  int month = str.substring(0, 2).toInt();
  int day = str.substring(3, 5).toInt();
  int year = str.substring(6, 10).toInt();
  int hour = str.substring(11, 13).toInt();
  int minute = str.substring(14, 16).toInt();
  int second = str.substring(17, 19).toInt();

  // Create and return a DateTime object using the extracted components
  return DateTime(year, month, day, hour, minute, second);
}

void updateSettings(String settingFname) {
  File32 settingsFile = SD.open(settingFname, O_WRITE | O_CREAT);
  for (int i = 0; i < sizeof(tweakableVars) / sizeof(tweakableVars[0]); i++) {
    settingsFile.print(tweakableVars[i].key);
    settingsFile.print(":");
    settingsFile.println(tweakableVars[i].value);
  }

  settingsFile.close();
  Serial.println("Settings saved to " + settingFname);
  // loadSettings(settingFname);
}

void syncRTCTime() {
  String outStr = "Old RTC Time: ";
  if (WiFi.status() == WL_CONNECTED) {
    // Configure NTP
    timeClient.begin();
    Serial.println("NTP Time set attempt");
    delay(3000);
    int nTries = 10;   // number of times to try updating RTC
    int tryCount = 0;  // counter for tries to update RTC

    // Wait for time to be set
    while (!timeClient.isTimeSet() && tryCount < nTries) {
      Serial.print("NTP Try ");
      Serial.println(tryCount);
      tryCount++;
      timeClient.update();
      unsigned long epochTime = timeClient.getEpochTime();
      DateTime time1(epochTime + timeZone * 3600);

      outStr += curTimeStr(rtc.now());
      outStr += ", New RTC Time: ";
      outStr += zeroPad(time1.month()) + "/" + zeroPad(time1.day()) + "/" + String(time1.year()) + " " + zeroPad(time1.hour()) + ":" + zeroPad(time1.minute()) + ":" + zeroPad(time1.second()) + "(UTC)";
      //errorWrite("$Time", outStr);

      rtc.adjust(time1);
      Serial.print("Sync RTC time success: ");
      Serial.println(zeroPad(time1.month()) + "/" + zeroPad(time1.day()) + "/" + String(time1.year()) + " " + zeroPad(time1.hour()) + ":" + zeroPad(time1.minute()) + ":" + zeroPad(time1.second()) + " (UTC)");
      delay(500);
    }
    if (tryCount >= 9) {
      Serial.println("Error: Sync RTC time failed. Unable to obtain time from NTP server.");
      errorWrite("$Error", "Error: Sync RTC time failed. Unable to obtain time from NTP server.");
    }
  } else {
    Serial.println("Error: Sync RTC time failed. Not Connected to WiFi.");
    errorWrite("$Error", "Error: Sync RTC time failed. Not Connected to WiFi.");
  }
}

void handleRTCTime() {
  String serverTime = curTimeStr(rtc.now());
  serverTime += ".";
  serverTime += String(millis() % 1000);
  serverAP->send(200, "text/plain", serverTime);
}

void INA219Setup() {
  Serial.println("Starting INA219...");

  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  if (!solar1.begin()) {
    Serial.println("Failed to find solar1 INA219 chip");
  }
  if (!solar2.begin()) {
    Serial.println("Failed to find solar2 INA219 chip");
  }
  if (!onlyPwr.begin()) {
    Serial.println("Failed to find onlyPwr INA219 chip");
  }
  if (!dataPwr.begin()) {
    Serial.println("Failed to find dataPwr INA219 chip");
  }
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();

  Serial.println("Measuring voltage and current with INA219 ...");
}

void PCA9536BuzzSetup() {
  Serial.println("PCA9536 starting...");

  Wire.begin();

  // Initialize the PCA9536 with a begin functbuzzLEDn
  if (buzzLED.begin() == false) {
    Serial.println("PCA9536 not detected. Please check wiring.");
    ;
  }

  // set all I/O as outputs
  for (int i = 0; i < 4; i++) {
    // pinMode can be used to set an I/O as OUTPUT or INPUT
    buzzLED.pinMode(i, OUTPUT);
  }
  sequenceBuzzLED(1, 100, GRN);
}

void sequenceBuzzLED(int iterations, int blinkDuration, int color) {
  for (int i = 0; i < iterations; i++) {
    buzzLED.write(color, LOW);
    buzzLED.write(BUZZ, HIGH);
    delay(blinkDuration);
    buzzLED.write(color, HIGH);
    buzzLED.write(BUZZ, LOW);
    delay(blinkDuration);
  }
}

void readBatteryInfo() {
  static unsigned long lastTime = millis();
  unsigned long curTime = millis();
  if (curTime - lastTime > BATT_LOG_PERIOD) {
    lastTime = curTime;
    float shuntvoltage1 = 0;
    float busvoltage1 = 0;
    float current_mA1 = 0;
    float loadvoltage1 = 0;
    float power_mW1 = 0;

    float shuntvoltage2 = 0;
    float busvoltage2 = 0;
    float current_mA2 = 0;
    float loadvoltage2 = 0;
    float power_mW2 = 0;

    float shuntvoltage3 = 0;
    float busvoltage3 = 0;
    float current_mA3 = 0;
    float loadvoltage3 = 0;
    float power_mW3 = 0;

    float shuntvoltage4 = 0;
    float busvoltage4 = 0;
    float current_mA4 = 0;
    float loadvoltage4 = 0;
    float power_mW4 = 0;
    String outStr = "";

    shuntvoltage1 = solar1.getShuntVoltage_mV();
    busvoltage1 = solar1.getBusVoltage_V();
    current_mA1 = solar1.getCurrent_mA();
    power_mW1 = solar1.getPower_mW();
    loadvoltage1 = busvoltage1 + (shuntvoltage1 / 1000);

    shuntvoltage2 = solar2.getShuntVoltage_mV();
    busvoltage2 = solar2.getBusVoltage_V();
    current_mA2 = solar2.getCurrent_mA();
    power_mW2 = solar2.getPower_mW();
    loadvoltage2 = busvoltage2 + (shuntvoltage2 / 1000);

    shuntvoltage3 = onlyPwr.getShuntVoltage_mV();
    busvoltage3 = onlyPwr.getBusVoltage_V();
    current_mA3 = onlyPwr.getCurrent_mA();
    power_mW3 = onlyPwr.getPower_mW();
    loadvoltage3 = busvoltage3 + (shuntvoltage3 / 1000);

    shuntvoltage4 = dataPwr.getShuntVoltage_mV();
    busvoltage4 = dataPwr.getBusVoltage_V();
    current_mA4 = dataPwr.getCurrent_mA();
    power_mW4 = dataPwr.getPower_mW();
    loadvoltage4 = busvoltage4 + (shuntvoltage4 / 1000);

    // Serial.print("Bus Voltages:   "); Serial.print(busvoltage1); Serial.print(" V, "); Serial.print(busvoltage2); Serial.println(" V");
    // Serial.print("Shunt Voltages: "); Serial.print(shuntvoltage1); Serial.print(" mV, "); Serial.print(shuntvoltage2); Serial.println(" mV");
    // Serial.print("Load Voltages:  "); Serial.print(loadvoltage1); Serial.print(" V, "); Serial.print(loadvoltage2); Serial.println(" V");
    // Serial.print("Currents:       "); Serial.print(current_mA1); Serial.print(" mA, "); Serial.print(current_mA2); Serial.println(" mA");
    // Serial.print("Powers:         "); Serial.print(power_mW1); Serial.print(" mW, "); Serial.print(power_mW2); Serial.println(" mW");
    // Serial.println("");
    // out = "Bus Voltages:   "; out += busvoltage1;out += " V, ";out += busvoltage2; out +=" V\n";
    // out += "Shunt Voltages: "; out += shuntvoltage1;out += " mV, ";out += shuntvoltage2; out +=" mV\n";
    // out += "Load Voltages:  "; out += loadvoltage1;out += " V, ";out += loadvoltage2; out +=" V\n";
    // out += "Currents:       "; out += current_mA1;out += " mA, ";out += current_mA2; out +=" mA\n";
    //outStr += "Solar1,Solar2,onlyPwr,dataPwr:         ";
    //outStr += ",";
    outStr += power_mW1;
    outStr += ",";
    outStr += power_mW2;
    outStr += ",";
    outStr += power_mW3;
    outStr += ",";
    outStr += power_mW4;
    housekeepWrite("$Power (mw)", outStr);
    outStr = "";
    //outStr += "Solar1,Solar2,onlyPwr,dataPwr:         ";
    //outStr += ",";
    outStr += busvoltage1;
    outStr += ",";
    outStr += busvoltage2;
    outStr += ",";
    outStr += busvoltage3;
    outStr += ",";
    outStr += busvoltage4;
    housekeepWrite("$Voltage (V)", outStr);
  }
}

void housekeepWrite(String sentenceID, String data) {
  if (!data.isEmpty()) {
    myFileHousekeeping = SD.open(fname2, O_WRITE | O_APPEND);
    if (myFileHousekeeping) {
      myFileHousekeeping.print(sentenceID);
      myFileHousekeeping.print(",");
      myFileHousekeeping.print(data);
      myFileHousekeeping.print(",");
      myFileHousekeeping.print(curTimeStr(rtc.now()));
      myFileHousekeeping.println("");
      myFileHousekeeping.close();
    } else {
      Serial.println("Failed to open housekeeping file");
    }
  }
}

void errorWrite(String sentenceID, String data) {
  if (!data.isEmpty()) {
    myFileError = SD.open(fname4, O_WRITE | O_APPEND);
    if (myFileError) {
      myFileError.print(sentenceID);
      myFileError.print(",");
      myFileError.print(data);
      myFileError.print(",");
      myFileError.print(curTimeStr(rtc.now()));
      myFileError.println("");
      myFileError.close();
    } else {
      Serial.println("Failed to open error file");
    }
  }
}

void sleepLogWrite(String sleepTime, String wakeTime) {
  if (!sleepTime.isEmpty()) {
    myFileSleep = SD.open(fname5, O_WRITE | O_APPEND);
    if (myFileSleep) {
      myFileSleep.print(curTimeStr(rtc.now()));
      myFileSleep.print(",");
      myFileSleep.print(sleepTime);
      myFileSleep.print(",");
      myFileSleep.print(wakeTime);
      myFileSleep.close();
    } else {
      Serial.println("Failed to open sleeplog file");
    }
  }
}

void tempatureWrite(String loadCellBaselineMean) {
  myFileTempature = SD.open(fname3, O_WRITE | O_APPEND);
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  if (myFileTempature) {
    myFileTempature.print(String(feederName));
    myFileTempature.print(",");
    myFileTempature.print(String(temp.temperature));  //degrees celcius
    myFileTempature.print(",");
    myFileTempature.print(String(humidity.relative_humidity));  //% rH
    myFileTempature.print(",");
    myFileTempature.print(String(loadCellBaselineMean));
    myFileTempature.print(",");
    myFileTempature.print(curTimeStr(rtc.now()));
    myFileTempature.println("");
    myFileTempature.close();
  } else {
    Serial.println("Failed to open templog file");
  }
}

void BMP390Setup() {
  Serial.println("Starting BMP390...");
  if (!bmp.begin_I2C()) {  // hardware I2C mode, can pass in address & alt Wire
    //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode
    //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    errorWrite("%ERROR", "Could not find a valid BMP3 sensor");
  }
  //housekeepWrite("$BMP390", "BMP390 Setup Successful");
}

void MLX90393Setup() {
  Serial.println("Starting MLX90393...");
  if (!sensor.begin_I2C()) {  // hardware I2C mode, can pass in address & alt Wire
    //if (! sensor.begin_SPI(MLX90393_CS)) {  // hardware SPI mode
    Serial.println("No sensor found ... check your wiring?");
    errorWrite("%ERROR", "Could not find a valid MLX90393 sensor");
  }

  sensor.setGain(MLX90393_GAIN_1X);
  // You can check the gain too
  Serial.print("Gain set to: ");
  switch (sensor.getGain()) {
    case MLX90393_GAIN_1X:
      Serial.println("1 x");
      break;
    case MLX90393_GAIN_1_33X:
      Serial.println("1.33 x");
      break;
    case MLX90393_GAIN_1_67X:
      Serial.println("1.67 x");
      break;
    case MLX90393_GAIN_2X:
      Serial.println("2 x");
      break;
    case MLX90393_GAIN_2_5X:
      Serial.println("2.5 x");
      break;
    case MLX90393_GAIN_3X:
      Serial.println("3 x");
      break;
    case MLX90393_GAIN_4X:
      Serial.println("4 x");
      break;
    case MLX90393_GAIN_5X:
      Serial.println("5 x");
      break;
  }

  // Set resolution, per axis. Aim for sensitivity of ~0.3 for all axes.
  sensor.setResolution(MLX90393_X, MLX90393_RES_17);
  sensor.setResolution(MLX90393_Y, MLX90393_RES_17);
  sensor.setResolution(MLX90393_Z, MLX90393_RES_16);

  // Set oversampling
  sensor.setOversampling(MLX90393_OSR_3);

  // Set digital filtering
  sensor.setFilter(MLX90393_FILTER_5);
  //housekeepWrite("$MLX90393", "MLX90393 Setup Successful");
}

/*This method starts the RTC clock and compares it to a given fallback time  (the compliation time of the program)
*/
void rtcInit() {
  if (rtc.begin()) {
    rtc.start();
    Serial.println("RTC is here and started");
    Serial.println("RTC belives it is " + curTimeStr(rtc.now()));
    Serial.println("The fallback time is " + curTimeStr(DateTime(F(__DATE__), F(__TIME__))));
    if (rtc.now() < DateTime(F(__DATE__), F(__TIME__))) {
      Serial.println("The RTC is earlier than the fallback time. Adjusting to the fallback time");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    } else {
      Serial.println("The RTC is ahead of the fallback time. This is good!");
    }
    Serial.flush();
  } else {
    Serial.println("Couldn't find RTC :(, using fallback time.");
    rtc.start();
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println("The RTC Now thinks it is " + curTimeStr(rtc.now()));
    Serial.flush();
  }
}


/* Read loadcell 
*/
double readLoadCell() {
  static int32_t lastValid = 0;

  // Wait for a valid reading
  while (!nau.available()) {
    delayMicroseconds(50);
  }

  int32_t val = nau.read();

  if (val == INT32_MAX) {
    // Return last good value to avoid false jumps
    return lastValid;
  }

  lastValid = val;  // update
  return val;
}

//* Moving average of loadcell values between visits and during visits
void updateBaselineLoadCell() {
  float reading = readLoadCell();
  loadCellBaselineMean = (1.0 - alphaBaseline) * loadCellBaselineMean + alphaBaseline * reading;
}

// ---- Binary UART EM4102 reader ----
uint8_t tagBytes[4];  // 4-byte ID
uint8_t byteCount = 0;
bool tagReady = false;
uint32_t lastTag = 0;  // store last valid tag

uint32_t readTagUART() {
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    tagBytes[byteCount++] = b;

    if (byteCount == 4) {  // full tag received
      // Combine bytes to single 32-bit integer
      uint32_t tag = 0;
      tag |= ((uint32_t)tagBytes[0]) << 24;
      tag |= ((uint32_t)tagBytes[1]) << 16;
      tag |= ((uint32_t)tagBytes[2]) << 8;
      tag |= ((uint32_t)tagBytes[3]);

      lastTag = tag;    // store last valid tag
      tagReady = true;  // indicate tag is available
      byteCount = 0;    // reset for next tag
      return tag;
    }
  }

  // If no new tag, return last valid tag if available
  if (tagReady) {
    tagReady = false;  // consume the last tag
    return lastTag;
  }

  return 0;  // no tag available yet
}

/* If there is a tag on the feeder, 
 * this returns the tag in hex code and prints it to serial
 * Otherwise, it returns nothing and prints nothing.
 */

String readTag() {
  int counter = 0;                //Read first four bytes
  tagNum = "";                    //Current Tag on the Feeder
  if (Serial1.available() > 0) {  //Check if RFID is Reading Anything
    while (Serial1.available() > 0 && counter < 4) {
      digitalWrite(led, HIGH);        //Visual indicator that Serial1 has data.
      byte newTag = Serial1.read();   //Read first byte
      tagNum += String(newTag, HEX);  //Make new tag
      counter++;                      //Iterate counter
      delay(2);                       //Slight delay
    }
    // Serial.println("A tag has been read. It is: " + String(tagNum));
  }
  return tagNum;
}

/* Attempts to write read data to given file,
  * if any data is missing defaults to default params
  * writes error if write fails
  */
void printToFile(String fname, String feederName, String visitID, String readingNum, String loadcell, String millis, String tagNum, String time, String startTime, String endTime,
                 String temp, String humidity, String loadCellBaselineMean) {
  File32 fileToWrite = SD.open(fname, O_WRITE | O_APPEND);
  if (fileToWrite) {
    delay(5);
    fileToWrite.println(String(feederName) + "," + String(visitID) + "," + String(readingNum) + "," + String(loadcell) + "," + String(millis) + "," + String(tagNum) + "," + String(time) + "," + String(startTime) + "," + String(endTime) + "," + String(temp) + "," + String(humidity) + "," + String(loadCellBaselineMean));
    // Serial.println(String(feederName) + "," + String(visitID) + "," + String(readingNum) + "," + String(loadcell) + "," + String(millis) + "," + String(tagNum) + "," + String(time) + "," + String(startTime) + "," + String(endTime) + "," + String(temp) + "," + String(humidity) + "," + String(loadCellBaselineMean));
    fileToWrite.sync();
  } else {
    Serial.println("Attempted to write to file but failed");
    errorWrite("$ERROR", "Failed to write to data file");
  }
  fileToWrite.close();
}

/* Announces a new reading is starting and logs the buffer. (Green Rows)
  * Then, logs the tempature and humidity reading (Red Row)
  */

void logVisitReadings(String fname, String visitID, int numReadings, float loadCellBuffer[], unsigned long timeBuffer[],
                      String tagNum, unsigned long visitStartTime, unsigned long visitEndTime, float loadCellBaselineMean) {
  Serial.println("Logging data for visit " + String(visitID) + ", consisting of " + String(numReadings) + " readings of tag " + String(tagNum));

  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  for (int i = 0; i < numReadings; i++) {
    if (i == 0) {
      printToFile(fname, String(feederName), String(visitID), String(i), String(loadCellBuffer[i]), String(timeBuffer[i]), String(tagNum), curTimeStr(rtc.now()), String(visitStartTime), String(visitEndTime), String(temp.temperature), String(humidity.relative_humidity), String(loadCellBaselineMean));
    } else {
      printToFile(fname, String(feederName), String(visitID), String(i), String(loadCellBuffer[i]), String(timeBuffer[i]), "", "", "", "", "", "", "");
    }
  }

  delay(5);
}

// Check for loadcell increases (start visit) or decreases (end visit); if loadcell not available, scan for tags

void checkVisit() {
  if (nau.available()) {  // if loadcell working
    loadcell = readLoadCell();
    loadcellDiff = loadcell - loadCellBaselineMean;

    if (!visitActive && loadcellDiff >= loadCellThresholdStart) {
      int consecutiveEnd = 0;

      // Capture starting environmental conditions
      visitStartTime = millis();  // timestamp of first reading

      // Visit ID
      visit++;

      char visitID[32];
      snprintf(visitID, sizeof(visitID), "%d_%lu", visit, visitStartTime);

      // initialize empty tag
      tagNum = "";

      // Initialize visit loadcell mean to first reading
      loadCellVisitMeanFast = loadcell;
      loadCellVisitMeanSlow = loadcell;
      visitActive = true;
      loggingEnabled = true;
      readingCount = 0;

      // Flush stale bytes from before visit
      while (Serial1.available()) Serial1.read();

      // --- CONTINUE LOGGING UNTIL ANIMAL LEAVES ---
      while (visitActive) {
        // Running mean of loadcell values during visit
        loadcell = readLoadCell();
        // --- Log loadcell values until maxReadingCount met ---
        if (readingCount < maxReadingCount) {
          loadCellBuffer[readingCount] = loadcell;
          timeBuffer[readingCount] = millis();
          readingCount++;
        }

        loadCellVisitMeanFast = alphaFast * loadcell + (1 - alphaFast) * loadCellVisitMeanFast;
        loadCellVisitMeanSlow = alphaSlow * loadcell + (1 - alphaSlow) * loadCellVisitMeanSlow;


        //if (loadcell > loadCellVisitMean * (1 + loadCellThresholdEnd) || loadcell < loadCellVisitMean * (1 - loadCellThresholdEnd)) {
        if (loadCellVisitMeanFast < loadCellVisitMeanSlow * (1 - loadCellThresholdEnd)) {
          consecutiveEnd++;
        }

        // --- END OF VISIT CONDITION ---
        if (consecutiveEnd >= endVisitSensitivity) {
          visitEndTime = millis();
          visitActive = false;  // Reset
        }
        // Only attempt tag read if not yet captured AND bytes are already waiting
        // Adds no delay if nothing is in the buffer
        if (tagNum.length() < 10 && Serial1.available() >= 10) {
          String newTag = readTag();
          if (newTag.length() == 10) {
            tagNum = newTag;
            //tagCaptured = true;
          }
        }
      }
      // Log all visit information in single write
      logVisitReadings(fname, visitID, readingCount, loadCellBuffer, timeBuffer, tagNum, visitStartTime, visitEndTime, loadCellBaselineMean);
    }
  }
}

// Checks how long to stay in light sleep so that logging functions are not disrupted
uint64_t computeSafeSleepDuration(DateTime nowRTC) {

  // --- Next visit check ---
  long nextVisitMs = (long)nextVisitCheckMs - (long)millis();
  if (nextVisitMs < 0) nextVisitMs = 0;

  uint64_t minSleepUs = (uint64_t)nextVisitMs * 1000ULL;


  // --- Next RTC log ---
  TimeSpan untilLog = nextRTCLog - nowRTC;
  long logMs = untilLog.totalseconds() * 1000;
  if (logMs < 0) logMs = 0;

  uint64_t logUs = (uint64_t)logMs * 1000ULL;
  if (logUs < minSleepUs) minSleepUs = logUs;


  // --- Next deep sleep check ---
  TimeSpan untilDeepCheck = nextDeepSleepCheck - nowRTC;
  long dcMs = untilDeepCheck.totalseconds() * 1000;
  if (dcMs < 0) dcMs = 0;

  uint64_t dcUs = (uint64_t)dcMs * 1000ULL;
  if (dcUs < minSleepUs) minSleepUs = dcUs;


  return minSleepUs;
}


// -------------------
// Daily sunrise/sunset storage
// -------------------
float sunsetToday = 0.0f;
float sunriseTomorrow = 0.0f;
int lastDay = -1;

// -------------------
// Deep sleep function using precomputed sunrise/sunset
// -------------------
void enterNightSleep() {
  DateTime now = rtc.now();
  float currentHr = now.hour() + now.minute() / 60.0f + now.second() / 3600.0f;

  float sleepStartHr = sunsetToday + SLEEP_AFTER_SUNSET_MIN / 60.0f;
  float wakeHr = sunriseTomorrow - WAKE_BEFORE_SUNRISE_MIN / 60.0f;

  // Normalize
  if (sleepStartHr >= 24) sleepStartHr -= 24.0f;
  if (wakeHr < 0) wakeHr += 24.0f;

  bool inSleepWindow = (currentHr >= sleepStartHr) || (currentHr < wakeHr);
  if (!inSleepWindow) {
    Serial.println("Not in sleep window, staying awake.");
    return;
  }

  // Compute sleep duration in seconds
  float sleepSec;
  if (currentHr >= sleepStartHr) {
    sleepSec = (24.0f - currentHr + wakeHr) * 3600.0f;
  } else {
    sleepSec = (wakeHr - currentHr) * 3600.0f;
  }

  Serial.printf("Entering deep sleep for %.0f seconds until wake\n", sleepSec);
  sleepLogWrite(String(sleepStartHr), String(wakeHr));
  esp_sleep_enable_timer_wakeup((uint64_t)(sleepSec * 1000000ULL));
  // delay(20);
  esp_deep_sleep_start();
}

// Calculate sunrise/sunset in decimal hours (local solar time)
// Inputs: DateTime dt, latitude, longitude, timeZone
void calcSunriseSunsetLocal(DateTime dt, float latitude, float longitude, float utcOffset) {
  // Latitude, longitude, timezone offset
  Dusk2Dawn location(latitude, longitude, utcOffset);

  // Sunset, today
  uint8_t day = dt.day();
  uint8_t month = dt.month();
  uint16_t year = dt.year();

  int sunset = location.sunset(year, month, day, false);
  sunsetToday = sunset / 60.0;

  // Sunrise, tomorrow
  DateTime tomorrow = dt + TimeSpan(86400);
  uint8_t tomorrowday = tomorrow.day();
  uint8_t tomorrowmonth = tomorrow.month();
  uint16_t tomorrowyear = tomorrow.year();

  int sunrise = location.sunrise(tomorrowyear, tomorrowmonth, tomorrowday, false);  // minutes from midnight
  sunriseTomorrow = sunrise / 60.0;
}

void setup(void) {
  delay(10000);
  pinMode(led, OUTPUT);
  digitalWrite(led, 0);
  pinMode(0, INPUT_PULLUP);
  pinMode(I2C_PWR_PIN, OUTPUT);
  digitalWrite(I2C_PWR_PIN, HIGH);  // make sure I2C power is on
  // Open serial communications and wait for port to open:
  Serial.begin(115200);  //Serial port for general info
  Serial1.begin(9600);   // serial port for RFID data
  PCA9536BuzzSetup();
  delay(4000);  // wait for serial ports to start if they exist


  // RTC initialization

  rtcInit();
  loadCellInit();
  tempHumidityInit();
  INA219Setup();
  BMP390Setup();
  MLX90393Setup();

  ssid = default_ssid;
  password = default_password;
  apssid = default_apssid;
  appassword = default_appassword;


  SdFile::dateTimeCallback(dateTime);
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("initialization failed!");
  } else {
    Serial.println("initialization done.");
  }

  delay(1000);


  loadSettings(settingFname);

  //Print File headings
  fname = findFname(baseName);
  Serial.println("Filename: " + fname);
  myFileLoadCell = SD.open(fname, O_WRITE | O_CREAT);
  if (myFileLoadCell) {
    myFileLoadCell.println("feeder,visitID,readingNum,loadCell,readtime,RFIDTag,datetime,startTime,endTime,temp,humidity,baselineLoadCell");
    // myFileLoadCell.println("feeder,datetime,time(ms),loadCell,RFIDTag,visit,temp,humidity");
    myFileLoadCell.close();
  } else {
    Serial.println("Failed to open file");
  }

  fname2 = findFname("/log");
  Serial.println("Filename: " + fname2);
  myFileHousekeeping = SD.open(fname2, O_WRITE | O_CREAT);
  if (myFileHousekeeping) {
    //myFileHousekeeping.println("$sentenceID,Data,Date and Time");
    myFileHousekeeping.println("metric,solar1,solar2,onlypwr,datapwr,datetime");
    myFileHousekeeping.close();
  } else {
    Serial.println("Failed to open file");
  }

  fname3 = findFname("/templog");
  Serial.println("Filename: " + fname3);
  myFileTempature = SD.open(fname3, O_WRITE | O_CREAT);
  if (myFileTempature) {
    myFileTempature.println("feeder,tempature,humidity,loadcell,datetime");
    myFileTempature.close();
  } else {
    Serial.println("Failed to open file");
  }

  fname4 = findFname("/errorlog");
  Serial.println("Filename: " + fname4);
  myFileError = SD.open(fname4, O_WRITE | O_CREAT);
  if (myFileError) {
    myFileError.println("type,error,datetime");
    myFileError.close();
  } else {
    Serial.println("Failed to open error file");
  }

  fname5 = findFname("/sleeplog");
  Serial.println("Filename: " + fname5);
  myFileError = SD.open(fname5, O_WRITE | O_CREAT);
  if (myFileError) {
    myFileError.println("date,sleeptime,waketime");
    myFileError.close();
  } else {
    Serial.println("Failed to open sleeplog file");
  }

  WiFi.mode(WIFI_OFF);
  //checkWifi(wifiMaxCheckTimes, 500);

  Serial.print("Soft AP SSID: \"");
  Serial.print(apssid);
  Serial.print("\", IP address: ");
  Serial.println(WiFi.softAPIP());

  if (MDNS.begin(apssid)) {
    Serial.println("MDNS responder started\n");
  }

  serverAP = new WebServer(80);

  serverAP->on("/", handleRoot0);
  serverAP->on("/fileex", handleFileExample0);
  serverAP->on("/download", handleDownload0);
  serverAP->on("/rtcTime", HTTP_GET, handleRTCTime);
  // Handle inputs


  for (int i = 0; i < sizeof(tweakableVars) / sizeof(tweakableVars[0]); i++) {
    String route = "/" + String(tweakableVars[i].key) + "Form";

    serverAP->on(route.c_str(), [i]() {
      String key = tweakableVars[i].key;
      String argName = key + "Value";

      if (serverAP->hasArg(argName)) {
        tweakableVars[i].value = serverAP->arg(argName);
        Serial.println("Updated " + key + " to: " + tweakableVars[i].value);
        updateSettings(settingFname);
      }

      serverAP->sendHeader("Location", "/");
      serverAP->send(303);
    });
  }

  // Add the restart route specifically
  serverAP->on("/restart", HTTP_POST, []() {
    serverAP->send(200, "text/plain", "Restarting...");
    delay(1000);
    ESP.restart();
  });

  serverAP->onNotFound(handleNotFound0);

  Serial.printf("SSID: %s\n\thttp://", apssid);
  Serial.print(WiFi.softAPIP());
  Serial.print(":80\n\thttp://");
  Serial.print(WiFi.softAPIP());
  Serial.println(":8081");
  Serial.printf("Any of the above SSIDs\n\thttp://");
  Serial.print(apssid);
  Serial.print(".local:80\n\thttp://");
  Serial.print(apssid);
  Serial.print(".local:8081\n");


  // First-time sunrise/sunset calculation
  DateTime nowRTC = rtc.now();
  //lastDay = nowRTC.day(nowRTC);
  calcSunriseSunsetLocal(nowRTC, LATITUDE, LONGITUDE, timeZone);

  Serial.printf("Today's sunset: %.2f | Tomorrow's sunrise: %.2f\n",
                sunsetToday, sunriseTomorrow);
  Serial.printf("Sleep start: %.2f | Wake: %.2f\n",
                sunsetToday + SLEEP_AFTER_SUNSET_MIN / 60.0f,
                sunriseTomorrow - WAKE_BEFORE_SUNRISE_MIN / 60.0f);
  sequenceBuzzLED(1, 100, GRN);
}

void loop(void) {
  DateTime nowRTC = rtc.now();

  if (serverAPOn) {
    serverAP->handleClient();
    if (millis() >= nextWifiShutoffMs) {
      Serial.println("Shutting down Access Point\n");
      serverAPOn = false;
      serverAP->close();
      WiFi.softAPdisconnect(true);
      WiFi.disconnect();
    }
  }

  // Wifi Enable/Disable
  if ((digitalRead(0) == LOW) && !serverAPOn) {
    Serial.printf("Starting Up Access Point...\n");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apssid, appassword);
    if (MDNS.begin(apssid)) {
      Serial.println("MDNS responder started");
    }
    serverAP->begin();
    serverAPOn = true;
    Serial.printf("Acccess Point Started\n");
    nextWifiShutoffMs = millis() + maxWifiOffTime;
  }

  // Battery Logging
  if (nowRTC >= nextRTCLog) {
    Serial.println("RTC time: " + curTimeStr(rtc.now()));
    tempatureWrite(String(loadCellBaselineMean));
    readBatteryInfo();
    nextRTCLog = nowRTC + TimeSpan(rtcLogIntervalSec);
  }

  // Deep sleep check
  if (nowRTC >= nextDeepSleepCheck) {
    nextDeepSleepCheck = nowRTC + TimeSpan(deepSleepCheckInterval);
    // Call sleep function with precomputed sunrise/sunset
    enterNightSleep();
  }

  //Visit Detection
  if (millis() >= nextVisitCheckMs) {
    nextVisitCheckMs = millis() + checkVisitInterval;  // if active visit, next checkVisit occurs at fast interval
    checkVisit();

    updateBaselineLoadCell();  // if no visit, update baseline loadcell mean
    //Serial.println("Mean loadcell reading: " + String(loadCellBaselineMean));
    //Serial.println("Current loadcell reading: " + String(readLoadCell()));
  }

  delay(100);
  // -------------------------------------------------------
  // 6. Light sleep until next visit check
  // -------------------------------------------------------
  //uint64_t sleepMicro = computeSafeSleepDuration(nowRTC);
  //if (sleepMicro > 0 && !wifiActive && !visitActive) {
  //  esp_sleep_enable_timer_wakeup(sleepMicro);
  //esp_light_sleep_start();
}