#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
// #include <WiFi.h>
#include <BLEDevice.h>
#include <PubSubClient.h>
#include <TinyGPSPlus.h>
#include <esp_sleep.h>
#include <BLEScan.h>
#include <HardwareSerial.h>
#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>

// UARTs initialization
// TinyGPSPlus gps;
// HardwareSerial gpsSerial(0);  // Hardware Serial 0 for GPS
// HardwareSerial simSerial(1);  // Hardware Serial 0 for SIM800L
// TinyGsm modem(simSerial);
// TinyGsmClient client(modem);  // when using GSM
// PubSubClient mqttClient(client);  // when using GSM

// Global Variables (Declared as `extern`)
// extern WiFiClient wifiClient;
extern TinyGsm modem;
extern TinyGsmClient client;
extern PubSubClient mqttClient;
extern TinyGPSPlus gps;
extern HardwareSerial gpsSerial;  // For GSM module
extern HardwareSerial simSerial;  // For GSM module


// WiFiClient wifiClient;  // when using WiFi
// PubSubClient mqttClient(wifiClient);  // when using WiFi

extern bool stopPublishing;  // Variable to stop publishing GPS data

// WiFi Credentials
#define WIFI_SSID "FREE_INTERNET 2.1"
#define WIFI_PASSWORD "paliggenesias_13"


// MQTT Broker Configuration
#define MQTT_BROKER "homenetwork123.duckdns.org" // MQTT broker IP
#define MQTT_PORT 1883
#define MQTT_USERNAME "mqtt_user"
#define MQTT_PASSWORD "mqtt_pass"

// MQTT Topics
#define MQTT_TOPIC_GPS "home/gps_data"
#define MQTT_TOPIC_ITAG "home/itag_status"
#define MQTT_TOPIC_BATTERY "home/battery_status"

// GPRS Settings
#define GPRS_USER ""
#define GPRS_PASS ""
#define APN "internet"

#define uS_TO_S_FACTOR 1000000ULL // Conversion factor for micro seconds to seconds
#define SLEEP_TIME  5 * 60 * uS_TO_S_FACTOR   // Time ESP32 will go to sleep (in seconds)

// Hardware Pins
#define GSM_RX 6   // SIM800 TX -> ESP32 GPIO
#define GSM_TX 7   // SIM800 RX -> ESP32 GPIO
#define GSM_DTR 3  // SIM800 DTR -> ESP32 GPIO GPIO2
#define GPS_RX 20   // GPS 6M NEO TX -> ESP32 RX
#define GPS_TX 21   // GPS 6M NEO RX -> ESP32 TX
#define WAKEUP_PIN 2
#define BATTERY_STATUS_PIN 0 // GPIO0 for battery status via ADC

// Constants
#define SCAN_TIME 5  // Χρόνος σάρωσης BLE (σε δευτερόλεπτα)
#define PUBLISH_INTERVAL 10000  // 10 δευτερόλεπτα
#define GSM_BAUD 115200
#define GPS_BAUD 9600
#define ITAG_MAC_ADDRESS "ff:ff:c2:11:ec:17" // iTag's MAC address


#endif