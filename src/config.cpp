#include <config.h>
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

TinyGPSPlus gps;
HardwareSerial gpsSerial(0);  // Hardware Serial 0 for GPS
HardwareSerial simSerial(1);  // Hardware Serial 0 for SIM800L
TinyGsm modem(simSerial);
TinyGsmClient client(modem);  // when using GSM
PubSubClient mqttClient(client);  // when using GSM

bool stopPublishing = false;  // Variable to stop publishing GPS data

