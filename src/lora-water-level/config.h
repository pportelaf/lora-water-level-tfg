/***** BEGIN distance sensor configuration *****/
#define DISTANCE_SENSOR_SERIAL_PIN_RX 13
#define DISTANCE_SENSOR_SERIAL_PIN_TX 12
#define DISTANCE_SENSOR_SWITCH_PIN 14

// Delay to take the first measurement after power on the sensor
#define DISTANCE_SENSOR_START_DELAY 600

// Maximum time to take a meassurement
#define DISTANCE_SENSOR_MAX_MEASSURE_TIME 300

// The distance between the sensor and the bottom of the water container
#define DISTANCE_WATER_LEVEL_EMPTY 233

// The distance between the sensor and the water when the water container is full
#define DISTANCE_WATER_LEVEL_FULL 78
/***** END distance sensor configuration *****/

/***** BEGIN battery voltage configuration *****/
#define BATTERY_VOLTAGE_PIN 35
/*
 * BATTERY_VOLTAGE_PIN_MULTIPLIER 2 * 3.3 * 1.1 = 7.26
 * 1. Double the real voltage because the battery voltage is halved
 * 2. Multiply by the reference voltage of the ESP32, 3.3V
 * 3. Multiply by the ADC reference voltage of 1100mV
 */
#define BATTERY_VOLTAGE_PIN_MULTIPLIER 7.26
#define BATTERY_VOLTAGE_PIN_SAMPLE_BITS 4095
/***** END battery voltage configuration *****/

/***** BEGIN LMIC configuration *****/
// APP EUI in little endian format. Unused in ChirpStack
#define APP_EUI { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// DEV EUI in little endian format
#define DEV_EUI { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // FILLME

// APP KEY in big endian format
#define APP_KEY { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // FILLME

#define LORAWAN_PORT 1
#define LORAWAN_ERROR_PORT 2

// Maximum value of the error counter
#define LORAWAN_MEASSUREMENT_ERRORS_COUNT_MAX 16383
/***** END LMIC configuration *****/

/***** BEGIN app configuration *****/
// Timeout in seconds for retry transmission when an error occurs
#define TRANSMISSION_RETRY_TIMEOUT_IN_SECONDS 10

#define MAX_RETRY_MAX_TIMES 1

// Interval between transmissions in milliseconds (30 min)
#define TRANSMISSION_INTERVAL 1800000

// Interval between error transmissions in milliseconds (8h)
#define TRANSMISSION_ERROR_INTERVAL 28800000

// Interval between meassurements in milliseconds (5 min)
#define MEASSUREMENT_INTERVAL 300000

// Number of meassurements retries when an error occurs
#define MEASSUREMENT_MAX_RETRIES 8
/***** END app configuration *****/
