#include <DistanceSensor_A02YYUW.h>
#include <TTGO_LoRaWAN.h>
#include "config.h"

#define BATTERY_FORMATTED_MIN 300
#define BATTERY_FORMATTED_MAX 500

DistanceSensor_A02YYUW distanceSensor(&Serial2);
TTGO_LoRaWAN ttgoLoRaWAN;

RTC_DATA_ATTR unsigned long millisecondsSinceLastTransmissionAtSleep = TRANSMISSION_INTERVAL;
RTC_DATA_ATTR unsigned long millisecondsSinceLastErrorTransmissionAtSleep = TRANSMISSION_ERROR_INTERVAL;

unsigned long timeCurrentTransmission = 0;
unsigned long timeCurrentErrorTransmission = 0;

int secondsToSleep = 0;

float currentWaterLevel = 0;
float avgWaterLevel = 0;
RTC_DATA_ATTR float minWaterLevel = 0;
RTC_DATA_ATTR float maxWaterLevel = 0;
RTC_DATA_ATTR unsigned int meassurementsCount = 0;
RTC_DATA_ATTR float waterLevelSum = 0;

RTC_DATA_ATTR DistanceSensor_A02YYUW_MEASSUREMENT_STATUS lastMeassurementError;
RTC_DATA_ATTR int meassurementErrorsCount = 0;

bool canSleep = false;
int retryTransmissionsCount = 0;

static osjob_t startJob;
static osjob_t transmitDataJob;
static osjob_t transmitErrorJob;
static osjob_t retryTransmissionJob;

static const u1_t PROGMEM APPEUI[8] = APP_EUI;
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

static const u1_t PROGMEM DEVEUI[8] = DEV_EUI;
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

static const u1_t PROGMEM APPKEY[16] = APP_KEY;
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

void setup() {
  #if SERIAL_DEBUG_ENABLED
    Serial.begin(115200);
  #endif

  Serial2.begin(9600, SERIAL_8N1, DISTANCE_SENSOR_SERIAL_PIN_RX, DISTANCE_SENSOR_SERIAL_PIN_TX);
  ttgoLoRaWAN.begin();
  
  #if SERIAL_DEBUG_ENABLED
    DEBUG_PRINT("Setup application");
  #endif

  pinMode(DISTANCE_SENSOR_SWITCH_PIN, OUTPUT);
  digitalWrite(DISTANCE_SENSOR_SWITCH_PIN, LOW);


  #if SERIAL_DEBUG_ENABLED
      DEBUG_PRINT("Has slept %i", ttgoLoRaWAN.hasSlept());
  #endif

  os_setCallback(&startJob, start);
}

void loop() {
  os_runloop_once();

  if (canSleep) {
    goDeepSleep();
  }
}

void start(osjob_t* job) {
  updateWaterLevel();

  if (isErrorTransmissionNeeded()) {
    os_setCallback(&transmitErrorJob, transmitError);
  } else if (isTransmissionNeeded()) {
    os_setCallback(&transmitDataJob, transmitData);
  } else {
    canSleep = true;
  }
}

void updateWaterLevel() {
  int currentMeassurementErrorsCount = 0;
  DistanceSensor_A02YYUW_MEASSUREMENT_STATUS meassurementStatus;

  powerOnSensor();

  do {
    ttgoLoRaWAN.waitForCriticalJobs(DISTANCE_SENSOR_MAX_MEASSURE_TIME);

    meassurementStatus = distanceSensor.meassure();
    currentMeassurementErrorsCount++;
  } while (meassurementStatus != DistanceSensor_A02YYUW_MEASSUREMENT_STATUS_OK &&
    currentMeassurementErrorsCount < MEASSUREMENT_MAX_RETRIES);

  powerOffSensor();

  if (meassurementStatus != DistanceSensor_A02YYUW_MEASSUREMENT_STATUS_OK) {
    lastMeassurementError = meassurementStatus;
    meassurementErrorsCount++;

    #if SERIAL_DEBUG_ENABLED
      DEBUG_PRINT("Error updating water level %i", lastMeassurementError);
    #endif

    return;
  }

  currentWaterLevel = getWaterLevel();

  if (currentWaterLevel < minWaterLevel || meassurementsCount == 0) {
    minWaterLevel = currentWaterLevel;
  }

  if (currentWaterLevel > maxWaterLevel) {
    maxWaterLevel = currentWaterLevel;
  }

  waterLevelSum += currentWaterLevel;

  meassurementsCount++;

  avgWaterLevel = waterLevelSum / meassurementsCount;

  #if SERIAL_DEBUG_ENABLED
    DEBUG_PRINT("Water level updated, current = %f, min = %f, max = %f, avg = %f", currentWaterLevel, minWaterLevel, maxWaterLevel, avgWaterLevel);
  #endif
}

float getWaterLevel() {
  int distance = distanceSensor.getDistance();

  #if SERIAL_DEBUG_ENABLED
    DEBUG_PRINT("Distance is %u", distance);
  #endif

  // Prevent return water level percentage < 0%
  if (distance >= DISTANCE_WATER_LEVEL_EMPTY) {
    return 0.0;
  }

  // Prevent return water level percentage > 100%
  if (distance <= DISTANCE_WATER_LEVEL_FULL) {
    return 100.0;
  }

  float distanceWithoutOffset = (distance - DISTANCE_WATER_LEVEL_FULL);
  float waterLevelFullWithoutOffset = (DISTANCE_WATER_LEVEL_EMPTY - DISTANCE_WATER_LEVEL_FULL);

  return (1 - distanceWithoutOffset / waterLevelFullWithoutOffset) * 100;
}

void powerOnSensor() {
  digitalWrite(DISTANCE_SENSOR_SWITCH_PIN, HIGH);

  ttgoLoRaWAN.waitForCriticalJobs(DISTANCE_SENSOR_START_DELAY);
  delay(DISTANCE_SENSOR_START_DELAY);
}

void powerOffSensor() {
  digitalWrite(DISTANCE_SENSOR_SWITCH_PIN, LOW);
}

void resetDistance() {
  minWaterLevel = 0.0;
  maxWaterLevel = 0.0;
  waterLevelSum = 0.0;
  meassurementsCount = 0.0;
}

bool isTransmissionNeeded() {
  unsigned long millisecondsSinceLastTransmission = getMillisecondsSinceLastTransmission();

  #if SERIAL_DEBUG_ENABLED
    DEBUG_PRINT("Transmission is needed: %i. (millisecondsSinceLastTransmission: %ld, TRANSMISSION_INTERVAL: %ld)", 
      millisecondsSinceLastTransmission >= TRANSMISSION_INTERVAL, millisecondsSinceLastTransmission, TRANSMISSION_INTERVAL);
  #endif

  return millisecondsSinceLastTransmission >= TRANSMISSION_INTERVAL;
}

unsigned long getMillisecondsSinceLastTransmission() {
  return millisecondsSinceLastTransmissionAtSleep + millis();
}

bool isErrorTransmissionNeeded() {
  if (meassurementErrorsCount == 0) {
    #if SERIAL_DEBUG_ENABLED
      DEBUG_PRINT("Transmission error is not needed, 0 errors");
    #endif
    return false;
  }

  unsigned long millisecondsSinceLastErrorTransmission = getMillisecondsSinceLastErrorTransmission();

  #if SERIAL_DEBUG_ENABLED
    DEBUG_PRINT("Transmission error is needed: %i. (millisecondsSinceLastErrorTransmission: %ld, TRANSMISSION_ERROR_INTERVAL: %ld)", 
      millisecondsSinceLastErrorTransmission >= TRANSMISSION_ERROR_INTERVAL, millisecondsSinceLastErrorTransmission, TRANSMISSION_ERROR_INTERVAL);
  #endif

  return millisecondsSinceLastErrorTransmission >= TRANSMISSION_ERROR_INTERVAL;
}

unsigned long getMillisecondsSinceLastErrorTransmission() {
  return millisecondsSinceLastErrorTransmissionAtSleep + millis();
}

void transmitData(osjob_t* job) {
  /**
   * 10 bits per water level measurement, 4 (last, min, max, avg) * 10 = 40 bits => 5 bytes
   * 8 bits for battery, value is mapped from [300 - 500] to [0 - 200]
   */
  int batteryVoltageFormatted = getBatteryVoltageFormatted();
  int currentWatterLevelFormatted = formatWaterLevel(currentWaterLevel);
  int avgWatterLevelFormatted = formatWaterLevel(avgWaterLevel);
  int minWatterLevelFormatted = formatWaterLevel(minWaterLevel);
  int maxWatterLevelFormatted = formatWaterLevel(maxWaterLevel);
  byte dataBytes[8];

  dataBytes[0] = currentWatterLevelFormatted >> 2;
  dataBytes[1] = (currentWatterLevelFormatted << 6) | (avgWatterLevelFormatted >> 4);
  dataBytes[2] = (avgWatterLevelFormatted << 4) | (minWatterLevelFormatted >> 6);
  dataBytes[3] = (minWatterLevelFormatted << 2) | (maxWatterLevelFormatted >> 8);
  dataBytes[4] = maxWatterLevelFormatted & 0xFF;
  dataBytes[5] = batteryVoltageFormatted;

  #if SERIAL_DEBUG_ENABLED
    DEBUG_PRINT("TX data");
  #endif
  millisecondsSinceLastTransmissionAtSleep = 0;
  timeCurrentTransmission = millis();

  #if SERIAL_DEBUG_ENABLED
    DEBUG_PRINT("TX data \"%02X%02X%02X%02X%02X%02X%02X%02X\" (%i bytes)", dataBytes[0], dataBytes[1], dataBytes[2], dataBytes[3], dataBytes[4], 
      dataBytes[5], dataBytes[6], dataBytes[7], sizeof(dataBytes));
  #endif

  ttgoLoRaWAN.sendData(LORAWAN_PORT, dataBytes, sizeof(dataBytes), 0, transmitDataCallback);
}

void transmitError(osjob_t* job) {
  byte dataBytes[2];
  int meassurementErrorsCountLimited = meassurementErrorsCount <= LORAWAN_MEASSUREMENT_ERRORS_COUNT_MAX ?  meassurementErrorsCount : LORAWAN_MEASSUREMENT_ERRORS_COUNT_MAX;

  dataBytes[0] = (((lastMeassurementError * -1) -1) << 6) | (meassurementErrorsCountLimited >> 8);
  dataBytes[1] = meassurementErrorsCountLimited & 0xFF;

  #if SERIAL_DEBUG_ENABLED
    DEBUG_PRINT("TX error");
  #endif
  millisecondsSinceLastErrorTransmissionAtSleep = 0;
  timeCurrentErrorTransmission = millis();

  #if SERIAL_DEBUG_ENABLED
    DEBUG_PRINT("TX error \"%02X%02X\" (%i bytes)", dataBytes[0], dataBytes[1], sizeof(dataBytes));
  #endif

  ttgoLoRaWAN.sendData(LORAWAN_ERROR_PORT, dataBytes, sizeof(dataBytes), 0, transmitErrorCallback);
}

int getBatteryVoltageFormatted() {
  int batteryVoltageFormatted = (int)(getBatteryVoltage() * 100) - BATTERY_FORMATTED_MIN;

  if (batteryVoltageFormatted < BATTERY_FORMATTED_MIN) {
    batteryVoltageFormatted = BATTERY_FORMATTED_MIN;
  } else if (batteryVoltageFormatted > BATTERY_FORMATTED_MAX) {
    batteryVoltageFormatted = BATTERY_FORMATTED_MAX;
  }

  return batteryVoltageFormatted;
}

int formatWaterLevel(float waterLevel) {
  return (int) (waterLevel * 10);
}

float getBatteryVoltage() {
  float batteryVoltage = (float)(analogRead(BATTERY_VOLTAGE_PIN)) / BATTERY_VOLTAGE_PIN_SAMPLE_BITS * BATTERY_VOLTAGE_PIN_MULTIPLIER;

  #if SERIAL_DEBUG_ENABLED
    DEBUG_PRINT("Getting battery %.2fV", batteryVoltage)
  #endif
  
  return batteryVoltage;
}

void goDeepSleep() {
  setSecondsToSleep();

  if (!ttgoLoRaWAN.canGoDeepSleep(secondsToSleep)) {
    return;
  }

  updateMillisecondsSinceLastTransmissionAtSleep();
  updateMillisecondsSinceLastErrorTransmissionAtSleep();
  ttgoLoRaWAN.goDeepSleep(secondsToSleep);
}

void setSecondsToSleep() {
  long nextTransmissionTime = TRANSMISSION_INTERVAL - getMillisecondsSinceLastTransmission();

  if ((timeCurrentTransmission == 0) && (nextTransmissionTime < MEASSUREMENT_INTERVAL)) {
    secondsToSleep = (nextTransmissionTime > 0) ? (nextTransmissionTime / 1000) + 1 : 0;
  } else {
    secondsToSleep = MEASSUREMENT_INTERVAL / 1000;
  }
}

void updateMillisecondsSinceLastTransmissionAtSleep() {
  millisecondsSinceLastTransmissionAtSleep += (secondsToSleep * 1000) + millis() - timeCurrentTransmission;
}

void updateMillisecondsSinceLastErrorTransmissionAtSleep() {
  millisecondsSinceLastErrorTransmissionAtSleep += (secondsToSleep * 1000) + millis() - timeCurrentErrorTransmission;

  if (millisecondsSinceLastErrorTransmissionAtSleep > TRANSMISSION_ERROR_INTERVAL) {
    millisecondsSinceLastErrorTransmissionAtSleep = TRANSMISSION_ERROR_INTERVAL;
  }
}

void transmitDataCallback(bool success) {
  if (!success && canRetryTransmission()) {
    #if SERIAL_DEBUG_ENABLED
      DEBUG_PRINT("Data transmission error, retry transmission in %ld seconds", TRANSMISSION_RETRY_TIMEOUT_IN_SECONDS);
    #endif

    canSleep = false;
    os_setTimedCallback(&retryTransmissionJob, os_getTime() + sec2osticks(TRANSMISSION_RETRY_TIMEOUT_IN_SECONDS), retryTransmission);
  } else {
    #if SERIAL_DEBUG_ENABLED
      if (!success) {
        DEBUG_PRINT("Max retry transmissions reached (%i), retry transmissions count %i. Sleep enabled", MAX_RETRY_MAX_TIMES, retryTransmissionsCount);
      }
    #endif

    retryTransmissionsCount = 0;
    resetDistance();
    canSleep = true;
  }
}

void transmitErrorCallback(bool success) {
  if(!success && canRetryTransmission()) {
    #if SERIAL_DEBUG_ENABLED
      DEBUG_PRINT("Error transmission error, retry transmission in %ld seconds", TRANSMISSION_RETRY_TIMEOUT_IN_SECONDS);
    #endif

    canSleep = false;
    retryTransmissionsCount++;
    os_setTimedCallback(&transmitErrorJob, os_getTime() + sec2osticks(TRANSMISSION_RETRY_TIMEOUT_IN_SECONDS), transmitError);
  } else {
    #if SERIAL_DEBUG_ENABLED
      if (!success) {
        DEBUG_PRINT("Max retry transmissions reached (%i), retry transmissions count %i. Sleep enabled", MAX_RETRY_MAX_TIMES, retryTransmissionsCount);
      }
    #endif

    lastMeassurementError = DistanceSensor_A02YYUW_MEASSUREMENT_STATUS_OK;
    retryTransmissionsCount = 0;
    meassurementErrorsCount = 0;
    canSleep = true;
  }
}

bool canRetryTransmission() {
  return retryTransmissionsCount < MAX_RETRY_MAX_TIMES;
}

void retryTransmission(osjob_t* job) {
  retryTransmissionsCount++;

  updateWaterLevel();

  os_setCallback(&transmitDataJob, transmitData);
}
