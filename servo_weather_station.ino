/*
  ESP8266 + PCA9685 + SG90
  Weather station with calibration via web interface
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include "FaBoPWM_PCA9685.h"
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <time.h>

// ================= WIFI =================

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

const char* MDNS_NAME = "weatherstation";
const char* TZ_INFO = "CET-1CEST,M3.5.0/2,M10.5.0/3";

// ================= OPEN-METEO =================

const bool WEATHER_LOCATION_USE_COORDINATES = false;

const char* WEATHER_CITY = "Warsaw"; // set your city or switch to coordinates below
const char* WEATHER_COUNTRY = "PL"; //kod kraju ISO 3166-1 alpha-2
const float WEATHER_LATITUDE = 52.229771; //uzywane tylko gdy WEATHER_LOCATION_USE_COORDINATES = true
const float WEATHER_LONGITUDE = 21.011780; //uzywane tylko gdy WEATHER_LOCATION_USE_COORDINATES = true

const unsigned long WEATHER_UPDATE_INTERVAL = 10UL * 60UL * 1000UL;
unsigned long lastWeatherUpdate = 0;

bool weatherLocationResolved = false;
float weatherLatitude = 0.0;
float weatherLongitude = 0.0;

const int FORECAST_MODE_SWITCH_PIN = 13;
const int FORECAST_ITEM_COUNT = 5;

float actualTemperature = 0.0;
int actualWeatherId = 0;
int actualWeatherCategory = 4;
String actualWeatherMain = "";
String actualWeatherDescription = "";
String actualWeatherTime = "";
String lastStationUpdateTime = "";
float actualCloudCover = 0.0;
float actualCloudCoverLow = 0.0;
float actualCloudCoverMid = 0.0;
float actualCloudCoverHigh = 0.0;
float actualPrecipitation = 0.0;
float actualRain = 0.0;
float actualShowers = 0.0;
float actualSnowfall = 0.0;
float actualVisibility = 0.0;
float actualDirectRadiation = 0.0;
int actualIsDay = 0;
float actualVisualCloudScore = 0.0;
float todayMaxTemperature = 0.0;
int lastDisplayedActualWeatherServoValue = -1;

bool forecastModeHourly = true;
int forecastHourlyIntervalHours = 1;
int forecastModeSwitchRaw = 1;
String forecastModeSource = "web";

String forecastLabels[FORECAST_ITEM_COUNT];
float forecastTemperatures[FORECAST_ITEM_COUNT];
int forecastWeatherIds[FORECAST_ITEM_COUNT];
int forecastWeatherCategories[FORECAST_ITEM_COUNT];
String forecastWeatherNames[FORECAST_ITEM_COUNT];
String forecastWeatherDescriptions[FORECAST_ITEM_COUNT];
float forecastVisualCloudScores[FORECAST_ITEM_COUNT];

String hourlyForecastLabels[FORECAST_ITEM_COUNT];
float hourlyForecastTemperatures[FORECAST_ITEM_COUNT];
int hourlyForecastWeatherIds[FORECAST_ITEM_COUNT];
int hourlyForecastWeatherCategories[FORECAST_ITEM_COUNT];
String hourlyForecastWeatherNames[FORECAST_ITEM_COUNT];
String hourlyForecastWeatherDescriptions[FORECAST_ITEM_COUNT];
float hourlyForecastVisualCloudScores[FORECAST_ITEM_COUNT];
float hourlyForecastCloudLow[FORECAST_ITEM_COUNT];
float hourlyForecastCloudMid[FORECAST_ITEM_COUNT];
float hourlyForecastCloudHigh[FORECAST_ITEM_COUNT];
int hourlyForecastIsDay[FORECAST_ITEM_COUNT];

String dailyForecastLabels[FORECAST_ITEM_COUNT];
float dailyForecastTemperatures[FORECAST_ITEM_COUNT];
int dailyForecastWeatherIds[FORECAST_ITEM_COUNT];
int dailyForecastWeatherCategories[FORECAST_ITEM_COUNT];
String dailyForecastWeatherNames[FORECAST_ITEM_COUNT];
String dailyForecastWeatherDescriptions[FORECAST_ITEM_COUNT];
float dailyForecastVisualCloudScores[FORECAST_ITEM_COUNT];

float cachedTodayMaxTemperature = 0.0;

// ================= PCA9685 / SERVOS =================

FaBoPWM faboPWM;
ESP8266WebServer server(80);

const int MIN_VALUE = 115;
const int MAX_VALUE = 555;
const int MID_VALUE = (MAX_VALUE - MIN_VALUE) / 2 + MIN_VALUE;
const float FORECAST_TEMP_SERVO_MARGIN_RATIO = 0.05;

const int BASE_SERVOS = 2;
const int MAX_FORECAST_MODULES = 5;
const int SERVOS_PER_MODULE = 2;
const int MAX_SERVOS = BASE_SERVOS + MAX_FORECAST_MODULES * SERVOS_PER_MODULE;

const int SERVO_ACTUAL_TEMP = 0;
const int SERVO_ACTUAL_WEATHER = 1;

int forecastModulesNumber = 3;
int servoNumber = BASE_SERVOS + forecastModulesNumber * SERVOS_PER_MODULE;

int servoPos[MAX_SERVOS];
int weatherSunnyPos[MAX_SERVOS];
int weatherPartlyCloudyPos[MAX_SERVOS];
int weatherFogPos[MAX_SERVOS];

bool calibrationMode = true;
bool calibrationCompleted = false;

// ================= ACTUAL TEMP CALIBRATION =================

const float ACTUAL_TEMP_MIN_C = -30.0;
const float ACTUAL_TEMP_MAX_C = 30.0;

int actualTempPlus30Pos = MIN_VALUE;
int actualTempZeroPos = MID_VALUE;
int actualTempMinus30Pos = MAX_VALUE;

// ================= EEPROM =================

const int EEPROM_SIZE = 160;
const int EEPROM_MAGIC_ADDR = 0;
const int EEPROM_MODULES_ADDR = 1;
const int EEPROM_CALIBRATION_DONE_ADDR = 2;
const int EEPROM_FORECAST_MODE_ADDR = 3;
const int EEPROM_FORECAST_INTERVAL_ADDR = 4;
const int EEPROM_SERVO_START_ADDR = 10;

const int EEPROM_TEMP_PLUS30_ADDR = 50;
const int EEPROM_TEMP_ZERO_ADDR = 52;
const int EEPROM_TEMP_MINUS30_ADDR = 54;
const int EEPROM_WEATHER_SUNNY_START_ADDR = 60;
const int EEPROM_WEATHER_PARTLY_CLOUDY_START_ADDR = 84;
const int EEPROM_WEATHER_FOG_START_ADDR = 108;

const byte EEPROM_MAGIC = 77;

// ================= HELPERS =================

void updateServoNumber() {
  servoNumber = BASE_SERVOS + forecastModulesNumber * SERVOS_PER_MODULE;
}

void writeIntToEEPROM(int addr, int value) {
  EEPROM.write(addr, value & 0xFF);
  EEPROM.write(addr + 1, (value >> 8) & 0xFF);
}

int readIntFromEEPROM(int addr) {
  return EEPROM.read(addr) | (EEPROM.read(addr + 1) << 8);
}

void setServo(int channel, int value) {
  if (channel < 0 || channel >= MAX_SERVOS) return;

  value = constrain(value, MIN_VALUE, MAX_VALUE);
  servoPos[channel] = value;

  faboPWM.set_channel_value(channel, value);

  Serial.print("Servo ");
  Serial.print(channel);
  Serial.print(" = ");
  Serial.println(value);
}

bool isWeatherServo(int index) {
  return (index == SERVO_ACTUAL_WEATHER) || (index >= 2 && ((index - 2) % 2 == 1));
}

bool isForecastTempServo(int index) {
  return index >= 2 && ((index - 2) % 2 == 0);
}

void displayServoOnly(int channel, int value) {
  if (channel < 0 || channel >= MAX_SERVOS) return;

  value = constrain(value, MIN_VALUE, MAX_VALUE);
  faboPWM.set_channel_value(channel, value);

  Serial.print("Display servo ");
  Serial.print(channel);
  Serial.print(" = ");
  Serial.println(value);
}

void applyActiveServos() {
  for (int i = 0; i < servoNumber; i++) {
    faboPWM.set_channel_value(i, servoPos[i]);
    delay(40);
  }
}

String servoName(int index) {
  if (index == 0) return "Base: current temperature";
  if (index == 1) return "Base: current weather";

  int module = ((index - 2) / 2) + 1;
  bool tempServo = ((index - 2) % 2 == 0);

  if (tempServo) {
    return "Module " + String(module) + ": forecast temperature";
  } else {
    return "Module " + String(module) + ": forecast weather";
  }
}

void initializeWeatherCalibrationDefaults() {
  for (int i = 0; i < MAX_SERVOS; i++) {
    weatherSunnyPos[i] = MIN_VALUE;
    weatherPartlyCloudyPos[i] = MID_VALUE;
    weatherFogPos[i] = MAX_VALUE;
  }
}

String weatherCategoryName(int category) {
  switch (category) {
    case 0: return "Full sun";
    case 1: return "Snow";
    case 2: return "Rain";
    case 3: return "Partly cloudy";
    case 4: return "Cloudy";
    case 5: return "Thunderstorm";
    case 6: return "Fog";
    default: return "Unknown";
  }
}

String forecastModeName(bool hourlyMode) {
  return hourlyMode ? "hourly" : "daily";
}

String formatIsoTimeToDisplay(String isoTime) {
  if (isoTime.length() >= 16) {
    return isoTime.substring(0, 10) + " " + isoTime.substring(11, 16);
  }
  return isoTime;
}

String currentLocalTimeString() {
  time_t now = time(nullptr);
  if (now < 100000) {
    return "Clock not synced";
  }

  struct tm timeInfo;
  localtime_r(&now, &timeInfo);

  char buffer[20];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M", &timeInfo);
  return String(buffer);
}

int classifyWeatherCategoryFromCodeOnly(int weatherId) {
  if (weatherId == 95 || weatherId == 96 || weatherId == 99) return 5;
  if (weatherId == 71 || weatherId == 73 || weatherId == 75 ||
      weatherId == 77 || weatherId == 85 || weatherId == 86) return 1;
  if (weatherId == 51 || weatherId == 53 || weatherId == 55 ||
      weatherId == 56 || weatherId == 57 || weatherId == 61 ||
      weatherId == 63 || weatherId == 65 || weatherId == 66 ||
      weatherId == 67 || weatherId == 80 || weatherId == 81 ||
      weatherId == 82) return 2;
  if (weatherId == 45 || weatherId == 48) return 6;
  if (weatherId == 0) return 0;
  if (weatherId == 1 || weatherId == 2) return 3;
  if (weatherId == 3) return 4;
  return 4;
}

int classifyWeatherCategoryFromValues(
  int weatherId,
  float precipitation,
  float rain,
  float showers,
  float snowfall,
  float visibility,
  int isDay,
  float directRadiation,
  float cloudLow,
  float cloudMid,
  float cloudHigh,
  float* visualCloudScore
) {
  if (weatherId == 95 || weatherId == 96 || weatherId == 99) {
    *visualCloudScore = 100.0;
    return 5;
  }

  if (snowfall > 0.0 ||
      weatherId == 71 || weatherId == 73 || weatherId == 75 ||
      weatherId == 77 || weatherId == 85 || weatherId == 86) {
    *visualCloudScore = 100.0;
    return 1;
  }

  if (rain > 0.0 || showers > 0.0 || precipitation > 0.2 ||
      weatherId == 51 || weatherId == 53 || weatherId == 55 ||
      weatherId == 56 || weatherId == 57 || weatherId == 61 ||
      weatherId == 63 || weatherId == 65 || weatherId == 66 ||
      weatherId == 67 || weatherId == 80 || weatherId == 81 ||
      weatherId == 82) {
    *visualCloudScore = 100.0;
    return 2;
  }

  if (weatherId == 45 || weatherId == 48 || visibility < 1000.0) {
    *visualCloudScore = 100.0;
    return 6;
  }

  float visualCloud =
    0.55 * cloudLow +
    0.30 * cloudMid +
    0.15 * cloudHigh;

  if (isDay == 1) {
    if (directRadiation > 600.0) {
      visualCloud -= 20.0;
    } else if (directRadiation > 400.0) {
      visualCloud -= 10.0;
    }
  }

  visualCloud = constrain(visualCloud, 0.0, 100.0);
  *visualCloudScore = visualCloud;

  if (visualCloud <= 20.0) return 0;
  if (visualCloud <= 55.0) return 3;
  return 4;
}

void clearForecastData() {
  for (int i = 0; i < FORECAST_ITEM_COUNT; i++) {
    forecastLabels[i] = "";
    forecastTemperatures[i] = 0.0;
    forecastWeatherIds[i] = 0;
    forecastWeatherCategories[i] = 4;
    forecastWeatherNames[i] = "";
    forecastWeatherDescriptions[i] = "";
    forecastVisualCloudScores[i] = 0.0;
  }
}

void clearForecastCache(
  String labels[],
  float temperatures[],
  int weatherIds[],
  int weatherCategories[],
  String weatherNames[],
  String weatherDescriptions[],
  float visualCloudScores[]
) {
  for (int i = 0; i < FORECAST_ITEM_COUNT; i++) {
    labels[i] = "";
    temperatures[i] = 0.0;
    weatherIds[i] = 0;
    weatherCategories[i] = 4;
    weatherNames[i] = "";
    weatherDescriptions[i] = "";
    visualCloudScores[i] = 0.0;
  }
}

void clearAllForecastCaches() {
  clearForecastData();
  clearForecastCache(
    hourlyForecastLabels,
    hourlyForecastTemperatures,
    hourlyForecastWeatherIds,
    hourlyForecastWeatherCategories,
    hourlyForecastWeatherNames,
    hourlyForecastWeatherDescriptions,
    hourlyForecastVisualCloudScores
  );
  clearForecastCache(
    dailyForecastLabels,
    dailyForecastTemperatures,
    dailyForecastWeatherIds,
    dailyForecastWeatherCategories,
    dailyForecastWeatherNames,
    dailyForecastWeatherDescriptions,
    dailyForecastVisualCloudScores
  );

  for (int i = 0; i < FORECAST_ITEM_COUNT; i++) {
    hourlyForecastCloudLow[i] = 0.0;
    hourlyForecastCloudMid[i] = 0.0;
    hourlyForecastCloudHigh[i] = 0.0;
    hourlyForecastIsDay[i] = 0;
  }
}

void copyForecastCacheToActive(
  String labels[],
  float temperatures[],
  int weatherIds[],
  int weatherCategories[],
  String weatherNames[],
  String weatherDescriptions[],
  float visualCloudScores[]
) {
  for (int i = 0; i < FORECAST_ITEM_COUNT; i++) {
    forecastLabels[i] = labels[i];
    forecastTemperatures[i] = temperatures[i];
    forecastWeatherIds[i] = weatherIds[i];
    forecastWeatherCategories[i] = weatherCategories[i];
    forecastWeatherNames[i] = weatherNames[i];
    forecastWeatherDescriptions[i] = weatherDescriptions[i];
    forecastVisualCloudScores[i] = visualCloudScores[i];
  }
}

void applySelectedForecastCache() {
  if (forecastModeHourly) {
    copyForecastCacheToActive(
      hourlyForecastLabels,
      hourlyForecastTemperatures,
      hourlyForecastWeatherIds,
      hourlyForecastWeatherCategories,
      hourlyForecastWeatherNames,
      hourlyForecastWeatherDescriptions,
      hourlyForecastVisualCloudScores
    );
  } else {
    copyForecastCacheToActive(
      dailyForecastLabels,
      dailyForecastTemperatures,
      dailyForecastWeatherIds,
      dailyForecastWeatherCategories,
      dailyForecastWeatherNames,
      dailyForecastWeatherDescriptions,
      dailyForecastVisualCloudScores
    );
    todayMaxTemperature = cachedTodayMaxTemperature;
  }
}

// ================= EEPROM =================

void saveSettings(bool markCalibrationDone = true) {
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  EEPROM.write(EEPROM_MODULES_ADDR, forecastModulesNumber);
  EEPROM.write(EEPROM_CALIBRATION_DONE_ADDR, markCalibrationDone ? 1 : 0);
  EEPROM.write(EEPROM_FORECAST_MODE_ADDR, forecastModeHourly ? 1 : 0);
  EEPROM.write(EEPROM_FORECAST_INTERVAL_ADDR, forecastHourlyIntervalHours);

  for (int i = 0; i < MAX_SERVOS; i++) {
    int addr = EEPROM_SERVO_START_ADDR + i * 2;
    writeIntToEEPROM(addr, servoPos[i]);
  }

  writeIntToEEPROM(EEPROM_TEMP_PLUS30_ADDR, actualTempPlus30Pos);
  writeIntToEEPROM(EEPROM_TEMP_ZERO_ADDR, actualTempZeroPos);
  writeIntToEEPROM(EEPROM_TEMP_MINUS30_ADDR, actualTempMinus30Pos);

  for (int i = 0; i < MAX_SERVOS; i++) {
    writeIntToEEPROM(EEPROM_WEATHER_SUNNY_START_ADDR + i * 2, weatherSunnyPos[i]);
    writeIntToEEPROM(EEPROM_WEATHER_PARTLY_CLOUDY_START_ADDR + i * 2, weatherPartlyCloudyPos[i]);
    writeIntToEEPROM(EEPROM_WEATHER_FOG_START_ADDR + i * 2, weatherFogPos[i]);
  }

  EEPROM.commit();
  calibrationCompleted = markCalibrationDone;
  Serial.println("Settings saved to EEPROM");
}

void loadSettings() {
  EEPROM.begin(EEPROM_SIZE);

  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC) {
    Serial.println("No saved settings. Using defaults.");

    forecastModulesNumber = 3;
    updateServoNumber();

    for (int i = 0; i < MAX_SERVOS; i++) {
      servoPos[i] = MID_VALUE;
    }

    initializeWeatherCalibrationDefaults();

    servoPos[0] = MID_VALUE;
    servoPos[1] = MID_VALUE;

    actualTempPlus30Pos = MIN_VALUE;
    actualTempZeroPos = MID_VALUE;
    actualTempMinus30Pos = MAX_VALUE;
    forecastModeHourly = true;
    forecastHourlyIntervalHours = 1;

    calibrationCompleted = false;
    calibrationMode = true;
    saveSettings(false);
    return;
  }

  forecastModulesNumber = EEPROM.read(EEPROM_MODULES_ADDR);
  forecastModulesNumber = constrain(forecastModulesNumber, 1, MAX_FORECAST_MODULES);
  calibrationCompleted = EEPROM.read(EEPROM_CALIBRATION_DONE_ADDR) == 1;
  calibrationMode = !calibrationCompleted;
  forecastModeHourly = EEPROM.read(EEPROM_FORECAST_MODE_ADDR) != 0;
  forecastHourlyIntervalHours = constrain(EEPROM.read(EEPROM_FORECAST_INTERVAL_ADDR), 1, 3);
  updateServoNumber();

  for (int i = 0; i < MAX_SERVOS; i++) {
    int addr = EEPROM_SERVO_START_ADDR + i * 2;
    servoPos[i] = constrain(readIntFromEEPROM(addr), MIN_VALUE, MAX_VALUE);
  }

  initializeWeatherCalibrationDefaults();
  for (int i = 0; i < MAX_SERVOS; i++) {
    weatherSunnyPos[i] = constrain(readIntFromEEPROM(EEPROM_WEATHER_SUNNY_START_ADDR + i * 2), MIN_VALUE, MAX_VALUE);
    weatherPartlyCloudyPos[i] = constrain(readIntFromEEPROM(EEPROM_WEATHER_PARTLY_CLOUDY_START_ADDR + i * 2), MIN_VALUE, MAX_VALUE);
    weatherFogPos[i] = constrain(readIntFromEEPROM(EEPROM_WEATHER_FOG_START_ADDR + i * 2), MIN_VALUE, MAX_VALUE);

    if (isWeatherServo(i) &&
        weatherSunnyPos[i] == weatherFogPos[i] &&
        weatherSunnyPos[i] == weatherPartlyCloudyPos[i]) {
      weatherSunnyPos[i] = MIN_VALUE;
      weatherPartlyCloudyPos[i] = MID_VALUE;
      weatherFogPos[i] = MAX_VALUE;
    }

    int lowBound = min(weatherSunnyPos[i], weatherFogPos[i]);
    int highBound = max(weatherSunnyPos[i], weatherFogPos[i]);
    if (isWeatherServo(i) &&
        (weatherPartlyCloudyPos[i] <= lowBound || weatherPartlyCloudyPos[i] >= highBound)) {
      weatherPartlyCloudyPos[i] = weatherSunnyPos[i] +
        (weatherFogPos[i] - weatherSunnyPos[i]) / 2;
    }
  }

  actualTempPlus30Pos = constrain(readIntFromEEPROM(EEPROM_TEMP_PLUS30_ADDR), MIN_VALUE, MAX_VALUE);
  actualTempZeroPos = constrain(readIntFromEEPROM(EEPROM_TEMP_ZERO_ADDR), MIN_VALUE, MAX_VALUE);
  actualTempMinus30Pos = constrain(readIntFromEEPROM(EEPROM_TEMP_MINUS30_ADDR), MIN_VALUE, MAX_VALUE);

  Serial.println("Settings loaded from EEPROM");
}

// ================= TEMPERATURE MAPPING =================

int mapActualTemperatureToServo(float temperature) {
  temperature = constrain(temperature, ACTUAL_TEMP_MIN_C, ACTUAL_TEMP_MAX_C);

  if (temperature >= 0.0) {
    float ratio = temperature / ACTUAL_TEMP_MAX_C;

    return actualTempZeroPos +
           ratio * (actualTempPlus30Pos - actualTempZeroPos);
  } else {
    float ratio = temperature / ACTUAL_TEMP_MIN_C;

    return actualTempZeroPos +
           ratio * (actualTempMinus30Pos - actualTempZeroPos);
  }
}

int mapWeatherCategoryToServo(int channel, int category) {
  category = constrain(category, 0, 6);
  if (!isWeatherServo(channel)) {
    return MID_VALUE;
  }

  int sunnyPos = weatherSunnyPos[channel];
  int partlyCloudyPos = weatherPartlyCloudyPos[channel];
  int fogPos = weatherFogPos[channel];

  if (category <= 3) {
    float ratio = category / 3.0;
    return constrain(sunnyPos + ratio * (partlyCloudyPos - sunnyPos), MIN_VALUE, MAX_VALUE);
  }

  float ratio = (category - 3) / 3.0;
  return constrain(partlyCloudyPos + ratio * (fogPos - partlyCloudyPos), MIN_VALUE, MAX_VALUE);
}

// ================= WEB UI =================

String htmlPage() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Weather Station Calibration</title>

<style>
body { font-family: Arial; background:#f2f2f2; padding:20px; }
.card { background:white; max-width:820px; margin:auto; padding:20px; border-radius:14px; }
.servo-box { margin:10px 0; padding:10px; border:1px solid #ddd; border-radius:10px; background:#fafafa; }
.temp-box { margin:10px 0; padding:14px; border:2px solid #aac; border-radius:10px; background:#f8f8ff; }
input[type=range] { width:100%; }
button, select { margin:5px; padding:6px; }
.value { font-weight:bold; }
</style>
</head>

<body>
<div class="card">
<h1>Weather Station</h1>

<p><b>Mode:</b> <span id="modeText"></span></p>

<button onclick="toggleMode()">Switch mode</button>

<div id="calibrationPanel">
<h2>Calibration</h2>

<label>Forecast modules:</label>
<select id="modules" onchange="setModules()">
<option>1</option><option>2</option><option>3</option><option>4</option><option>5</option>
</select>

<h3>Actual temperature calibration</h3>

<div class="temp-box">
  <b>Servo 0: current temperature</b>

  <p>+30°C position: <span class="value" id="plus30Val"></span></p>
  <input type="range" id="plus30Slider" min="0" max="0"
    oninput="moveTempCalib('plus30', this.value)">

  <p>0°C position: <span class="value" id="zeroVal"></span></p>
  <input type="range" id="zeroSlider" min="0" max="0"
    oninput="moveTempCalib('zero', this.value)">

  <p>-30°C position: <span class="value" id="minus30Val"></span></p>
  <input type="range" id="minus30Slider" min="0" max="0"
    oninput="moveTempCalib('minus30', this.value)">
</div>

<h3>Other servos</h3>
<div id="servos"></div>

<button onclick="saveSettings()">Save settings</button>
</div>

<div id="stationPanel">
<h2>Weather mode</h2>

<p><b>Forecast mode:</b> <span id="forecastModeText"></span></p>
<p><b>Mode source:</b> <span id="forecastModeSource"></span></p>
<p><b>Physical switch (GPIO13):</b> <span id="forecastSwitchRaw"></span></p>
<p><b>Hourly interval:</b> <span id="forecastIntervalText"></span></p>
<p><b>Weather data time:</b> <span id="actualWeatherTime"></span></p>
<p><b>Last station update:</b> <span id="lastStationUpdateTime"></span></p>
<p><b>Temperature reference:</b> <span id="forecastTempReference"></span></p>

<label>Hourly interval:</label>
<select id="forecastIntervalSelect" onchange="setForecastInterval()">
<option value="1">1 hour</option>
<option value="2">2 hours</option>
<option value="3">3 hours</option>
</select>

<p><b>Temperature:</b> <span id="actualTemp"></span> °C</p>
<p><b>Interpreted weather:</b> <span id="actualWeather"></span></p>
<p><b>Description:</b> <span id="actualWeatherDescription"></span></p>
<p><b>Open-Meteo code:</b> <span id="actualWeatherId"></span></p>
<p><b>Visual cloud score:</b> <span id="visualCloudScore"></span></p>

<div class="temp-box">
  <b>Live weather details</b>
  <p>Coordinates: <span id="coords"></span></p>
  <p>Cloud cover total / low / mid / high:
    <span id="cloudDetails"></span></p>
  <p>Precipitation / rain / showers / snowfall:
    <span id="precipDetails"></span></p>
  <p>Visibility: <span id="visibilityVal"></span> m</p>
  <p>Direct radiation: <span id="radiationVal"></span> W/m²</p>
  <p>Daytime flag: <span id="dayFlag"></span></p>
</div>

<button onclick="updateWeatherNow()">Update weather now</button>

<h3>Forecast shown on modules</h3>
<div id="forecastItems"></div>
</div>

</div>

<script>
let data;

async function loadStatus(){
  let r = await fetch('/status');
  data = await r.json();

  document.getElementById('modeText').innerText =
    data.calibrationMode ? 'Calibration' : 'Weather';

  document.getElementById('calibrationPanel').style.display =
    data.calibrationMode ? 'block':'none';

  document.getElementById('stationPanel').style.display =
    data.calibrationMode ? 'none':'block';

  document.getElementById('modules').value = data.forecastModulesNumber;
  document.getElementById('forecastIntervalSelect').value = data.forecastHourlyIntervalHours;
  document.getElementById('forecastModeText').innerText = data.forecastModeHourly ? 'hourly' : 'daily';
  document.getElementById('forecastModeSource').innerText = data.forecastModeSource;
  document.getElementById('forecastSwitchRaw').innerText =
    data.forecastModeSwitchRaw + (data.forecastModeSwitchRaw ? ' (open / hourly)' : ' (to GND / daily)');
  document.getElementById('forecastIntervalText').innerText = data.forecastHourlyIntervalHours + 'h';
  document.getElementById('actualWeatherTime').innerText = data.actualWeatherTime;
  document.getElementById('lastStationUpdateTime').innerText = data.lastStationUpdateTime;
  document.getElementById('forecastTempReference').innerText =
    data.forecastModeHourly ? `${data.actualTemperature} °C (current)` : `${data.todayMaxTemperature} °C (today max)`;

  document.getElementById('actualTemp').innerText = data.actualTemperature;
  document.getElementById('actualWeather').innerText = data.actualWeatherMain;
  document.getElementById('actualWeatherDescription').innerText = data.actualWeatherDescription;
  document.getElementById('actualWeatherId').innerText = data.actualWeatherId;
  document.getElementById('visualCloudScore').innerText = data.actualVisualCloudScore;
  document.getElementById('coords').innerText = data.weatherLatitude + ', ' + data.weatherLongitude;
  document.getElementById('cloudDetails').innerText =
    `${data.actualCloudCover}% / ${data.actualCloudCoverLow}% / ${data.actualCloudCoverMid}% / ${data.actualCloudCoverHigh}%`;
  document.getElementById('precipDetails').innerText =
    `${data.actualPrecipitation} / ${data.actualRain} / ${data.actualShowers} / ${data.actualSnowfall}`;
  document.getElementById('visibilityVal').innerText = data.actualVisibility;
  document.getElementById('radiationVal').innerText = data.actualDirectRadiation;
  document.getElementById('dayFlag').innerText = data.actualIsDay ? 'day' : 'night';

  drawTempCalibration();
  drawServos();
  drawForecastItems();
}

function drawTempCalibration(){
  setupTempSlider('plus30Slider', 'plus30Val', data.actualTempPlus30Pos);
  setupTempSlider('zeroSlider', 'zeroVal', data.actualTempZeroPos);
  setupTempSlider('minus30Slider', 'minus30Val', data.actualTempMinus30Pos);
}

function setupTempSlider(sliderId, labelId, value){
  let slider = document.getElementById(sliderId);
  slider.min = data.min;
  slider.max = data.max;
  slider.value = value;
  document.getElementById(labelId).innerText = value;
}

function drawServos(){
  let box = document.getElementById('servos');
  box.innerHTML='';

  for(let i=1;i<data.servoNumber;i++){
    if(data.isWeatherServo[i]){
      box.innerHTML += `
        <div class="servo-box">
          <b>Servo ${i}: ${data.names[i]}</b><br>
          Current position: <span class="value" id="servoVal${i}">${data.positions[i]}</span><br>
          Full sun: <span class="value" id="sunnyVal${i}">${data.weatherSunnyPositions[i]}</span>
          <input type="range" min="${data.min}" max="${data.max}"
            value="${data.weatherSunnyPositions[i]}"
            oninput="moveWeatherCalib(${i}, 'sunny', this.value)">
          Partly cloudy: <span class="value" id="partlyCloudyVal${i}">${data.weatherPartlyCloudyPositions[i]}</span>
          <input type="range" min="${data.min}" max="${data.max}"
            value="${data.weatherPartlyCloudyPositions[i]}"
            oninput="moveWeatherCalib(${i}, 'partlyCloudy', this.value)">
          Fog: <span class="value" id="fogVal${i}">${data.weatherFogPositions[i]}</span>
          <input type="range" min="${data.min}" max="${data.max}"
            value="${data.weatherFogPositions[i]}"
            oninput="moveWeatherCalib(${i}, 'fog', this.value)">
          <div style="margin-top:8px">
            <button onclick="testWeatherPosition(${i}, 0)">Sun</button>
            <button onclick="testWeatherPosition(${i}, 1)">Snow</button>
            <button onclick="testWeatherPosition(${i}, 2)">Rain</button>
            <button onclick="testWeatherPosition(${i}, 3)">Partly cloudy</button>
            <button onclick="testWeatherPosition(${i}, 4)">Cloudy</button>
            <button onclick="testWeatherPosition(${i}, 5)">Storm</button>
            <button onclick="testWeatherPosition(${i}, 6)">Fog</button>
          </div>
        </div>`;
    } else {
      box.innerHTML += `
        <div class="servo-box">
          <b>Servo ${i}: ${data.names[i]}</b><br>
          Position: <span class="value" id="servoVal${i}">${data.positions[i]}</span>
          <input type="range" min="${data.min}" max="${data.max}"
            value="${data.positions[i]}"
            oninput="move(${i},this.value)">
        </div>`;
    }
  }
}

function drawForecastItems(){
  let box = document.getElementById('forecastItems');
  box.innerHTML = '';

  for(let i = 0; i < data.forecastModulesNumber; i++){
    box.innerHTML += `
      <div class="servo-box">
        <b>Module ${i + 1}</b><br>
        Label: ${data.forecastLabels[i]}<br>
        Temperature: ${data.forecastTemperatures[i]} °C<br>
        Weather: ${data.forecastWeatherNames[i]}<br>
        Description: ${data.forecastWeatherDescriptions[i]}<br>
        Code: ${data.forecastWeatherIds[i]}<br>
        Cloud score: ${data.forecastVisualCloudScores[i]}
      </div>`;
  }
}

async function move(id,val){
  let label = document.getElementById('servoVal' + id);
  if(label) label.innerText = val;

  await fetch(`/servo?id=${id}&value=${val}`);
}

async function moveWeatherCalib(id,point,val){
  let labelId = 'fogVal';
  if(point === 'sunny') labelId = 'sunnyVal';
  if(point === 'partlyCloudy') labelId = 'partlyCloudyVal';
  let label = document.getElementById(labelId + id);
  if(label) label.innerText = val;

  let currentLabel = document.getElementById('servoVal' + id);
  if(currentLabel) currentLabel.innerText = val;

  await fetch(`/weatherServoCalib?id=${id}&point=${point}&value=${val}`);
}

async function testWeatherPosition(id, category){
  await fetch(`/weatherServoTest?id=${id}&category=${category}`);
}

async function moveTempCalib(point,val){
  if(point === 'plus30') document.getElementById('plus30Val').innerText = val;
  if(point === 'zero') document.getElementById('zeroVal').innerText = val;
  if(point === 'minus30') document.getElementById('minus30Val').innerText = val;

  await fetch(`/actualTempCalib?point=${point}&value=${val}`);
}

async function setModules(){
  let v=document.getElementById('modules').value;
  await fetch(`/modules?count=${v}`);
  loadStatus();
}

async function setForecastInterval(){
  let v = document.getElementById('forecastIntervalSelect').value;
  await fetch(`/forecastInterval?hours=${v}`);
  loadStatus();
}

async function toggleMode(){
  await fetch('/mode');
  loadStatus();
}

async function saveSettings(){
  await fetch('/save');
  alert('Saved');
}

async function updateWeatherNow(){
  await fetch('/updateWeather');
  loadStatus();
}

loadStatus();
</script>

</body>
</html>
)rawliteral";

  return html;
}

// ================= HANDLERS =================

void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

void handleStatus() {
  String json = "{";

  json += "\"calibrationMode\":" + String(calibrationMode ? "true" : "false") + ",";
  json += "\"forecastModulesNumber\":" + String(forecastModulesNumber) + ",";
  json += "\"servoNumber\":" + String(servoNumber) + ",";
  json += "\"min\":" + String(MIN_VALUE) + ",";
  json += "\"mid\":" + String(MID_VALUE) + ",";
  json += "\"max\":" + String(MAX_VALUE) + ",";
  json += "\"forecastModeHourly\":" + String(forecastModeHourly ? "true" : "false") + ",";
  json += "\"forecastHourlyIntervalHours\":" + String(forecastHourlyIntervalHours) + ",";
  json += "\"forecastModeSwitchRaw\":" + String(forecastModeSwitchRaw) + ",";
  json += "\"forecastModeSource\":\"" + forecastModeSource + "\",";

  json += "\"actualTempPlus30Pos\":" + String(actualTempPlus30Pos) + ",";
  json += "\"actualTempZeroPos\":" + String(actualTempZeroPos) + ",";
  json += "\"actualTempMinus30Pos\":" + String(actualTempMinus30Pos) + ",";

  json += "\"actualTemperature\":" + String(actualTemperature, 1) + ",";
  json += "\"actualWeatherId\":" + String(actualWeatherId) + ",";
  json += "\"actualWeatherCategory\":" + String(actualWeatherCategory) + ",";
  json += "\"actualWeatherMain\":\"" + actualWeatherMain + "\",";
  json += "\"actualWeatherDescription\":\"" + actualWeatherDescription + "\",";
  json += "\"actualWeatherTime\":\"" + actualWeatherTime + "\",";
  json += "\"lastStationUpdateTime\":\"" + lastStationUpdateTime + "\",";
  json += "\"actualCloudCover\":" + String(actualCloudCover, 1) + ",";
  json += "\"actualCloudCoverLow\":" + String(actualCloudCoverLow, 1) + ",";
  json += "\"actualCloudCoverMid\":" + String(actualCloudCoverMid, 1) + ",";
  json += "\"actualCloudCoverHigh\":" + String(actualCloudCoverHigh, 1) + ",";
  json += "\"actualPrecipitation\":" + String(actualPrecipitation, 2) + ",";
  json += "\"actualRain\":" + String(actualRain, 2) + ",";
  json += "\"actualShowers\":" + String(actualShowers, 2) + ",";
  json += "\"actualSnowfall\":" + String(actualSnowfall, 2) + ",";
  json += "\"actualVisibility\":" + String(actualVisibility, 0) + ",";
  json += "\"actualDirectRadiation\":" + String(actualDirectRadiation, 1) + ",";
  json += "\"actualIsDay\":" + String(actualIsDay) + ",";
  json += "\"actualVisualCloudScore\":" + String(actualVisualCloudScore, 1) + ",";
  json += "\"todayMaxTemperature\":" + String(todayMaxTemperature, 1) + ",";
  json += "\"weatherLatitude\":" + String(weatherLatitude, 6) + ",";
  json += "\"weatherLongitude\":" + String(weatherLongitude, 6) + ",";

  json += "\"positions\":[";
  for (int i = 0; i < servoNumber; i++) {
    if (i) json += ",";
    json += servoPos[i];
  }
  json += "],";

  json += "\"weatherSunnyPositions\":[";
  for (int i = 0; i < servoNumber; i++) {
    if (i) json += ",";
    json += weatherSunnyPos[i];
  }
  json += "],";

  json += "\"weatherPartlyCloudyPositions\":[";
  for (int i = 0; i < servoNumber; i++) {
    if (i) json += ",";
    json += weatherPartlyCloudyPos[i];
  }
  json += "],";

  json += "\"weatherFogPositions\":[";
  for (int i = 0; i < servoNumber; i++) {
    if (i) json += ",";
    json += weatherFogPos[i];
  }
  json += "],";

  json += "\"isWeatherServo\":[";
  for (int i = 0; i < servoNumber; i++) {
    if (i) json += ",";
    json += isWeatherServo(i) ? "true" : "false";
  }
  json += "],";

  json += "\"forecastLabels\":[";
  for (int i = 0; i < forecastModulesNumber; i++) {
    if (i) json += ",";
    json += "\"" + forecastLabels[i] + "\"";
  }
  json += "],";

  json += "\"forecastTemperatures\":[";
  for (int i = 0; i < forecastModulesNumber; i++) {
    if (i) json += ",";
    json += String(forecastTemperatures[i], 1);
  }
  json += "],";

  json += "\"forecastWeatherIds\":[";
  for (int i = 0; i < forecastModulesNumber; i++) {
    if (i) json += ",";
    json += forecastWeatherIds[i];
  }
  json += "],";

  json += "\"forecastWeatherNames\":[";
  for (int i = 0; i < forecastModulesNumber; i++) {
    if (i) json += ",";
    json += "\"" + forecastWeatherNames[i] + "\"";
  }
  json += "],";

  json += "\"forecastWeatherDescriptions\":[";
  for (int i = 0; i < forecastModulesNumber; i++) {
    if (i) json += ",";
    json += "\"" + forecastWeatherDescriptions[i] + "\"";
  }
  json += "],";

  json += "\"forecastVisualCloudScores\":[";
  for (int i = 0; i < forecastModulesNumber; i++) {
    if (i) json += ",";
    json += String(forecastVisualCloudScores[i], 1);
  }
  json += "],";

  json += "\"names\":[";
  for (int i = 0; i < servoNumber; i++) {
    if (i) json += ",";
    json += "\"" + servoName(i) + "\"";
  }
  json += "]";

  json += "}";

  server.send(200, "application/json", json);
}

void handleServo() {
  int id = server.arg("id").toInt();
  int val = server.arg("value").toInt();

  if (isWeatherServo(id)) {
    server.send(400, "text/plain", "Use weather servo calibration endpoint");
    return;
  }

  setServo(id, val);

  server.send(200, "text/plain", "OK");
}

void handleWeatherServoCalibration() {
  if (!server.hasArg("id") || !server.hasArg("point") || !server.hasArg("value")) {
    server.send(400, "text/plain", "Missing id, point or value");
    return;
  }

  int id = server.arg("id").toInt();
  String point = server.arg("point");
  int value = constrain(server.arg("value").toInt(), MIN_VALUE, MAX_VALUE);

  if (!isWeatherServo(id) || id >= servoNumber) {
    server.send(400, "text/plain", "Invalid weather servo");
    return;
  }

  if (point == "sunny") {
    weatherSunnyPos[id] = value;
  } else if (point == "partlyCloudy") {
    weatherPartlyCloudyPos[id] = value;
  } else if (point == "fog") {
    weatherFogPos[id] = value;
  } else {
    server.send(400, "text/plain", "Invalid point");
    return;
  }

  setServo(id, value);
  server.send(200, "text/plain", "OK");
}

void handleWeatherServoTest() {
  if (!server.hasArg("id") || !server.hasArg("category")) {
    server.send(400, "text/plain", "Missing id or category");
    return;
  }

  int id = server.arg("id").toInt();
  int category = constrain(server.arg("category").toInt(), 0, 6);

  if (!isWeatherServo(id) || id >= servoNumber) {
    server.send(400, "text/plain", "Invalid weather servo");
    return;
  }

  int value = mapWeatherCategoryToServo(id, category);
  displayServoOnly(id, value);
  server.send(200, "text/plain", "OK");
}

void handleActualTempCalibration() {
  if (!server.hasArg("point") || !server.hasArg("value")) {
    server.send(400, "text/plain", "Missing point or value");
    return;
  }

  String point = server.arg("point");
  int value = constrain(server.arg("value").toInt(), MIN_VALUE, MAX_VALUE);

  if (point == "plus30") {
    actualTempPlus30Pos = value;
  } else if (point == "zero") {
    actualTempZeroPos = value;
  } else if (point == "minus30") {
    actualTempMinus30Pos = value;
  } else {
    server.send(400, "text/plain", "Invalid point");
    return;
  }

  setServo(SERVO_ACTUAL_TEMP, value);

  server.send(200, "text/plain", "OK");
}

void handleModules() {
  forecastModulesNumber = constrain(server.arg("count").toInt(), 1, 5);
  updateServoNumber();
  applyActiveServos();
  server.send(200, "text/plain", "OK");
}

void handleForecastMode() {
  server.send(200, "text/plain", "Mode is controlled by physical switch");
}

void handleForecastInterval() {
  forecastHourlyIntervalHours = constrain(server.arg("hours").toInt(), 1, 3);
  lastWeatherUpdate = 0;
  server.send(200, "text/plain", "OK");
}

void handleMode() {
  calibrationMode = !calibrationMode;

  if (!calibrationMode) {
    lastWeatherUpdate = 0;
  }

  server.send(200, "text/plain", "OK");
}

void handleSave() {
  saveSettings();
  calibrationMode = false;
  lastWeatherUpdate = 0;
  server.send(200, "text/plain", "OK");
}

void handleUpdateWeather() {
  updateForecastModeFromSwitch();
  bool ok = fetchWeatherData();

  if (ok) {
    lastStationUpdateTime = currentLocalTimeString();
    displayWeatherStation();
    server.send(200, "text/plain", "OK");
  } else {
    server.send(500, "text/plain", "Weather update failed");
  }
}

// ================= WIFI =================

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Connected!");

  Serial.print("ESP IP address: http://");
  Serial.println(WiFi.localIP());

  if (MDNS.begin(MDNS_NAME)) {
    Serial.print("mDNS started: http://");
    Serial.print(MDNS_NAME);
    Serial.println(".local");
  } else {
    Serial.println("mDNS start failed");
  }

  configTime(TZ_INFO, "pool.ntp.org", "time.nist.gov");
  Serial.println("NTP clock sync started");
}

// ================= OPEN-METEO =================

String urlEncodeCity(String value) {
  value.replace(" ", "%20");
  return value;
}

String weatherCodeToMain(int weatherCode) {
  switch (weatherCode) {
    case 0:
      return "Sunny";

    case 1:
    case 2:
      return "Partly cloudy";

    case 3:
      return "Cloudy";

    case 45:
    case 48:
      return "Fog";

    case 51:
    case 53:
    case 55:
    case 56:
    case 57:
    case 61:
    case 63:
    case 65:
    case 66:
    case 67:
    case 80:
    case 81:
    case 82:
      return "Rain";

    case 71:
    case 73:
    case 75:
    case 77:
    case 85:
    case 86:
      return "Snow";

    case 95:
    case 96:
    case 99:
      return "Thunderstorm";

    default:
      return "Unknown";
  }
}

String weatherCodeToDescription(int weatherCode) {
  switch (weatherCode) {
    case 0: return "Clear sky";
    case 1: return "Mainly clear";
    case 2: return "Partly cloudy";
    case 3: return "Overcast";
    case 45: return "Fog";
    case 48: return "Depositing rime fog";
    case 51: return "Light drizzle";
    case 53: return "Moderate drizzle";
    case 55: return "Dense drizzle";
    case 56: return "Light freezing drizzle";
    case 57: return "Dense freezing drizzle";
    case 61: return "Slight rain";
    case 63: return "Moderate rain";
    case 65: return "Heavy rain";
    case 66: return "Light freezing rain";
    case 67: return "Heavy freezing rain";
    case 71: return "Slight snowfall";
    case 73: return "Moderate snowfall";
    case 75: return "Heavy snowfall";
    case 77: return "Snow grains";
    case 80: return "Slight rain showers";
    case 81: return "Moderate rain showers";
    case 82: return "Violent rain showers";
    case 85: return "Slight snow showers";
    case 86: return "Heavy snow showers";
    case 95: return "Thunderstorm";
    case 96: return "Thunderstorm with slight hail";
    case 99: return "Thunderstorm with heavy hail";
    default: return "Unknown conditions";
  }
}

int classifyWeatherCategory() {
  return classifyWeatherCategoryFromValues(
    actualWeatherId,
    actualPrecipitation,
    actualRain,
    actualShowers,
    actualSnowfall,
    actualVisibility,
    actualIsDay,
    actualDirectRadiation,
    actualCloudCoverLow,
    actualCloudCoverMid,
    actualCloudCoverHigh,
    &actualVisualCloudScore
  );
}

bool fetchJsonPayload(const String& url, String& payload);

bool resolveWeatherLocation() {
  if (weatherLocationResolved) {
    return true;
  }

  if (WEATHER_LOCATION_USE_COORDINATES) {
    weatherLatitude = WEATHER_LATITUDE;
    weatherLongitude = WEATHER_LONGITUDE;
    weatherLocationResolved = true;

    Serial.print("Using configured coordinates: ");
    Serial.print(weatherLatitude, 6);
    Serial.print(", ");
    Serial.println(weatherLongitude, 6);
    return true;
  }

  String url = "https://geocoding-api.open-meteo.com/v1/search?name=";
  url += urlEncodeCity(String(WEATHER_CITY));
  url += "&count=1&language=en&format=json";

  if (strlen(WEATHER_COUNTRY) > 0) {
    url += "&countryCode=";
    url += WEATHER_COUNTRY;
  }

  Serial.println("Resolving weather location...");

  String payload;
  if (!fetchJsonPayload(url, payload)) {
    Serial.println("Geocoding request failed");
    return false;
  }

  StaticJsonDocument<2048> doc;
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.print("Geocoding JSON parse error: ");
    Serial.println(error.c_str());
    return false;
  }

  JsonArray results = doc["results"].as<JsonArray>();
  if (results.isNull() || results.size() == 0) {
    Serial.println("Geocoding returned no results");
    return false;
  }

  weatherLatitude = results[0]["latitude"].as<float>();
  weatherLongitude = results[0]["longitude"].as<float>();
  weatherLocationResolved = true;

  Serial.print("Resolved coordinates: ");
  Serial.print(weatherLatitude, 6);
  Serial.print(", ");
  Serial.println(weatherLongitude, 6);

  return true;
}

bool fetchJsonPayload(const String& url, String& payload) {
  WiFiClientSecure client;
  client.stop();
  client.setInsecure();
  client.setTimeout(15000);
  client.setBufferSizes(512, 512);

  HTTPClient http;
  if (!http.begin(client, url)) {
    Serial.println("HTTP begin failed");
    return false;
  }

  http.useHTTP10(true);
  http.setReuse(false);
  http.setTimeout(15000);
  http.addHeader("Accept-Encoding", "identity");

  int httpCode = http.GET();
  Serial.print("HTTP code: ");
  Serial.println(httpCode);

  if (httpCode != HTTP_CODE_OK) {
    Serial.print("HTTP error detail: ");
    Serial.println(http.errorToString(httpCode));
    http.end();
    return false;
  }

  payload = http.getString();
  http.end();
  client.stop();
  yield();

  Serial.print("Payload size: ");
  Serial.println(payload.length());
  return payload.length() > 0;
}

bool updateForecastModeFromSwitch() {
  int rawState = digitalRead(FORECAST_MODE_SWITCH_PIN);
  bool changed = (rawState != forecastModeSwitchRaw);

  if (rawState != forecastModeSwitchRaw) {
    forecastModeSource = "switch";
  }

  forecastModeSwitchRaw = rawState;
  forecastModeHourly = (rawState == HIGH);
  return changed;
}

void applyForecastTemperatureTrend(float referenceTemperature) {
  float deltas[FORECAST_ITEM_COUNT];
  float maxAbsDelta = 0.0;
  int servoMargin = (MAX_VALUE - MIN_VALUE) * FORECAST_TEMP_SERVO_MARGIN_RATIO;
  int safeMin = MIN_VALUE + servoMargin;
  int safeMax = MAX_VALUE - servoMargin;

  for (int i = 0; i < forecastModulesNumber; i++) {
    deltas[i] = forecastTemperatures[i] - referenceTemperature;
    float absDelta = deltas[i] >= 0.0 ? deltas[i] : -deltas[i];
    maxAbsDelta = max(maxAbsDelta, absDelta);
  }

  for (int i = 0; i < forecastModulesNumber; i++) {
    int tempServoChannel = 2 + i * 2;
    int weatherServoChannel = tempServoChannel + 1;

    int tempServoValue = servoPos[tempServoChannel];
    if (maxAbsDelta > 0.01) {
      float ratio = deltas[i] / maxAbsDelta;
      if (ratio >= 0.0) {
        tempServoValue = servoPos[tempServoChannel] +
          ratio * (safeMax - servoPos[tempServoChannel]);
      } else {
        tempServoValue = servoPos[tempServoChannel] +
          ratio * (servoPos[tempServoChannel] - safeMin);
      }
    }

    tempServoValue = constrain(tempServoValue, safeMin, safeMax);
    displayServoOnly(tempServoChannel, tempServoValue);
    delay(200);
    displayServoOnly(weatherServoChannel, mapWeatherCategoryToServo(weatherServoChannel, forecastWeatherCategories[i]));
    delay(200);
  }
}

void displayActualWeatherServoWithBacklashComp(int targetValue) {
  targetValue = constrain(targetValue, MIN_VALUE, MAX_VALUE);
  int fogValue = weatherFogPos[SERVO_ACTUAL_WEATHER];

  if (lastDisplayedActualWeatherServoValue != targetValue && targetValue != fogValue) {
    displayServoOnly(SERVO_ACTUAL_WEATHER, fogValue);
    delay(250);
  }

  displayServoOnly(SERVO_ACTUAL_WEATHER, targetValue);
  lastDisplayedActualWeatherServoValue = targetValue;
}

bool buildHourlyForecast(
  JsonObject hourly,
  String labels[],
  float temperatures[],
  int weatherIds[],
  int weatherCategories[],
  String weatherNames[],
  String weatherDescriptions[],
  float visualCloudScores[]
) {
  JsonArray times = hourly["time"].as<JsonArray>();
  JsonArray jsonTemperatures = hourly["temperature_2m"].as<JsonArray>();
  JsonArray weatherCodes = hourly["weather_code"].as<JsonArray>();
  JsonArray cloudLow = hourly["cloud_cover_low"].as<JsonArray>();
  JsonArray cloudMid = hourly["cloud_cover_mid"].as<JsonArray>();
  JsonArray cloudHigh = hourly["cloud_cover_high"].as<JsonArray>();
  JsonArray isDay = hourly["is_day"].as<JsonArray>();

  if (times.isNull() || jsonTemperatures.isNull() || weatherCodes.isNull()) {
    return false;
  }

  int step = constrain(forecastHourlyIntervalHours, 1, 3);
  int filled = 0;

  clearForecastCache(labels, temperatures, weatherIds, weatherCategories, weatherNames, weatherDescriptions, visualCloudScores);

  for (size_t i = step; i < times.size() && filled < forecastModulesNumber; i += step) {
    labels[filled] = formatIsoTimeToDisplay(times[i].as<String>());
    temperatures[filled] = jsonTemperatures[i].as<float>();
    weatherIds[filled] = weatherCodes[i].as<int>();
    float low = cloudLow.isNull() ? 0.0 : cloudLow[i].as<float>();
    float mid = cloudMid.isNull() ? 0.0 : cloudMid[i].as<float>();
    float high = cloudHigh.isNull() ? 0.0 : cloudHigh[i].as<float>();
    int dayFlag = isDay.isNull() ? 0 : isDay[i].as<int>();

    hourlyForecastCloudLow[filled] = low;
    hourlyForecastCloudMid[filled] = mid;
    hourlyForecastCloudHigh[filled] = high;
    hourlyForecastIsDay[filled] = dayFlag;

    if (cloudLow.isNull() || cloudMid.isNull() || cloudHigh.isNull()) {
      weatherCategories[filled] = classifyWeatherCategoryFromCodeOnly(weatherIds[filled]);
      visualCloudScores[filled] = 0.0;
    } else {
      weatherCategories[filled] = classifyWeatherCategoryFromValues(
        weatherIds[filled],
        0.0,
        0.0,
        0.0,
        0.0,
        10000.0,
        dayFlag,
        0.0,
        low,
        mid,
        high,
        &visualCloudScores[filled]
      );
    }

    // Forecast hourly should not show "partly cloudy" for WMO code 3 (overcast).
    if (weatherIds[filled] == 3 && weatherCategories[filled] < 4) {
      weatherCategories[filled] = 4;
      if (visualCloudScores[filled] < 56.0) {
        visualCloudScores[filled] = 56.0;
      }
    }

    weatherNames[filled] = weatherCategoryName(weatherCategories[filled]);
    weatherDescriptions[filled] = weatherCodeToDescription(weatherIds[filled]);
    filled++;
  }

  return filled == forecastModulesNumber;
}

bool buildDailyForecast(
  JsonObject daily,
  String labels[],
  float temperatures[],
  int weatherIds[],
  int weatherCategories[],
  String weatherNames[],
  String weatherDescriptions[],
  float visualCloudScores[]
) {
  JsonArray times = daily["time"].as<JsonArray>();
  JsonArray maxTemps = daily["temperature_2m_max"].as<JsonArray>();
  JsonArray weatherCodes = daily["weather_code"].as<JsonArray>();

  if (times.isNull() || maxTemps.isNull() || weatherCodes.isNull() || times.size() < 2) {
    return false;
  }

  cachedTodayMaxTemperature = maxTemps[0].as<float>();
  clearForecastCache(labels, temperatures, weatherIds, weatherCategories, weatherNames, weatherDescriptions, visualCloudScores);

  int filled = 0;
  for (size_t i = 1; i < times.size() && filled < forecastModulesNumber; i++) {
    labels[filled] = times[i].as<String>();
    temperatures[filled] = maxTemps[i].as<float>();
    weatherIds[filled] = weatherCodes[i].as<int>();
    weatherCategories[filled] = classifyWeatherCategoryFromCodeOnly(weatherIds[filled]);
    visualCloudScores[filled] = 0.0;
    weatherNames[filled] = weatherCategoryName(weatherCategories[filled]);
    weatherDescriptions[filled] = weatherCodeToDescription(weatherIds[filled]);
    filled++;
  }

  return filled == forecastModulesNumber;
}

bool fetchWeatherData() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected - cannot fetch weather");
    return false;
  }

  if (!resolveWeatherLocation()) {
    return false;
  }

  int requiredHourlyHours = forecastModulesNumber * forecastHourlyIntervalHours + 1;
  int requiredDailyDays = forecastModulesNumber + 1;

  String baseUrl = "https://api.open-meteo.com/v1/forecast?latitude=";
  baseUrl += String(weatherLatitude, 6);
  baseUrl += "&longitude=";
  baseUrl += String(weatherLongitude, 6);
  baseUrl += "&timezone=auto";

  String requestUrl = baseUrl;
  requestUrl += "&current=temperature_2m,weather_code,cloud_cover,cloud_cover_low,cloud_cover_mid,cloud_cover_high,precipitation,rain,showers,snowfall,visibility,direct_radiation,is_day";
  requestUrl += "&hourly=temperature_2m,weather_code,cloud_cover_low,cloud_cover_mid,cloud_cover_high,is_day";
  requestUrl += "&forecast_hours=";
  requestUrl += String(requiredHourlyHours);
  requestUrl += "&daily=weather_code,temperature_2m_max";
  requestUrl += "&forecast_days=";
  requestUrl += String(requiredDailyDays);

  Serial.println("Fetching weather data...");
  Serial.print("Request URL length: ");
  Serial.println(requestUrl.length());
  Serial.print("Free heap before request: ");
  Serial.println(ESP.getFreeHeap());

  String payload;
  if (!fetchJsonPayload(requestUrl, payload)) {
    Serial.println("Weather request failed");
    return false;
  }

  DynamicJsonDocument doc(10240);
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.print("Weather JSON parse error: ");
    Serial.println(error.c_str());
    return false;
  }

  JsonObject current = doc["current"].as<JsonObject>();
  if (current.isNull()) {
    Serial.println("Missing 'current' object in weather response");
    return false;
  }

  actualTemperature = current["temperature_2m"].as<float>();
  actualWeatherId = current["weather_code"].as<int>();
  actualWeatherTime = formatIsoTimeToDisplay(current["time"].as<String>());
  actualCloudCover = current["cloud_cover"].as<float>();
  actualCloudCoverLow = current["cloud_cover_low"].as<float>();
  actualCloudCoverMid = current["cloud_cover_mid"].as<float>();
  actualCloudCoverHigh = current["cloud_cover_high"].as<float>();
  actualPrecipitation = current["precipitation"].as<float>();
  actualRain = current["rain"].as<float>();
  actualShowers = current["showers"].as<float>();
  actualSnowfall = current["snowfall"].as<float>();
  actualVisibility = current["visibility"].as<float>();
  actualDirectRadiation = current["direct_radiation"].as<float>();
  actualIsDay = current["is_day"].as<int>();

  actualWeatherCategory = classifyWeatherCategory();
  actualWeatherMain = weatherCategoryName(actualWeatherCategory);
  actualWeatherDescription = weatherCodeToDescription(actualWeatherId);

  JsonObject hourly = doc["hourly"].as<JsonObject>();
  JsonObject daily = doc["daily"].as<JsonObject>();

  bool hourlyOk = buildHourlyForecast(
    hourly,
    hourlyForecastLabels,
    hourlyForecastTemperatures,
    hourlyForecastWeatherIds,
    hourlyForecastWeatherCategories,
    hourlyForecastWeatherNames,
    hourlyForecastWeatherDescriptions,
    hourlyForecastVisualCloudScores
  );
  bool dailyOk = buildDailyForecast(
    daily,
    dailyForecastLabels,
    dailyForecastTemperatures,
    dailyForecastWeatherIds,
    dailyForecastWeatherCategories,
    dailyForecastWeatherNames,
    dailyForecastWeatherDescriptions,
    dailyForecastVisualCloudScores
  );

  if (!hourlyOk || !dailyOk) {
    Serial.println("Forecast data incomplete");
    return false;
  }

  applySelectedForecastCache();

  Serial.println("Weather data updated:");
  Serial.print("Temperature: ");
  Serial.println(actualTemperature);
  Serial.print("Open-Meteo weather code: ");
  Serial.println(actualWeatherId);
  Serial.print("Interpreted weather: ");
  Serial.println(actualWeatherMain);
  Serial.print("Visual cloud score: ");
  Serial.println(actualVisualCloudScore);

  return true;
}

void displayWeatherStation() {
  int tempServoValue = mapActualTemperatureToServo(actualTemperature);
  int weatherServoValue = mapWeatherCategoryToServo(SERVO_ACTUAL_WEATHER, actualWeatherCategory);
  float referenceTemperature = forecastModeHourly ? actualTemperature : todayMaxTemperature;

  Serial.println("Displaying weather station on servos:");

  Serial.print("Temperature servo: ");
  Serial.println(tempServoValue);

  Serial.print("Weather servo: ");
  Serial.println(weatherServoValue);

  displayServoOnly(SERVO_ACTUAL_TEMP, tempServoValue);
  delay(300);
  displayActualWeatherServoWithBacklashComp(weatherServoValue);
  delay(300);
  applyForecastTemperatureTrend(referenceTemperature);
}

void updateActualWeatherIfNeeded() {
  if (calibrationMode) return;

  unsigned long now = millis();

  if (now - lastWeatherUpdate >= WEATHER_UPDATE_INTERVAL || lastWeatherUpdate == 0) {
    lastWeatherUpdate = now;

    updateForecastModeFromSwitch();
    bool ok = fetchWeatherData();

    if (ok) {
      lastStationUpdateTime = currentLocalTimeString();
      displayWeatherStation();
    } else {
      Serial.println("Weather update failed");
      lastWeatherUpdate = now - WEATHER_UPDATE_INTERVAL + 60000UL;
    }
  }
}

// ================= SETUP =================

void setup() {
  Serial.begin(115200);

  pinMode(FORECAST_MODE_SWITCH_PIN, INPUT_PULLUP);
  loadSettings();
  clearAllForecastCaches();

  faboPWM.begin();
  faboPWM.init(300);
  faboPWM.set_hz(50);

  applyActiveServos();
  forecastModeSwitchRaw = digitalRead(FORECAST_MODE_SWITCH_PIN);
  forecastModeHourly = (forecastModeSwitchRaw == HIGH);
  forecastModeSource = "switch";

  connectWiFi();

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/servo", handleServo);
  server.on("/weatherServoCalib", handleWeatherServoCalibration);
  server.on("/weatherServoTest", handleWeatherServoTest);
  server.on("/actualTempCalib", handleActualTempCalibration);
  server.on("/modules", handleModules);
  server.on("/forecastMode", handleForecastMode);
  server.on("/forecastInterval", handleForecastInterval);
  server.on("/mode", handleMode);
  server.on("/save", handleSave);
  server.on("/updateWeather", handleUpdateWeather);

  server.begin();

  Serial.println("Server started");
}

// ================= LOOP =================

void loop() {
  server.handleClient();
  MDNS.update();

  bool switchChanged = updateForecastModeFromSwitch();
  if (switchChanged && !calibrationMode) {
    Serial.println("Forecast mode switch changed - applying cached forecast");
    applySelectedForecastCache();
    lastStationUpdateTime = currentLocalTimeString();
    displayWeatherStation();
  }

  updateActualWeatherIfNeeded();
}
