#include <BLEManager.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <HardwareSerial.h>
#include <PubSubClient.h>
#include <TinyGPSPlus.h>
#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <config.h>

bool scanForBLE() {
    Serial.println("Starting BLE scan...");
  
    BLEDevice::init("");
    BLEScan* pBLEScan = BLEDevice::getScan(); // Create a BLE scan object
    pBLEScan->setActiveScan(true);   // Active scanning for more data
    pBLEScan->setInterval(100);      // Scan interval
    pBLEScan->setWindow(99);         // Scan window (must be <= interval)
    
    BLEScanResults scanResults = pBLEScan->start(SCAN_TIME, false);
    bool iTagDetected = false;
  
    for (int i = 0; i < scanResults.getCount(); i++) {
      BLEAdvertisedDevice device = scanResults.getDevice(i);
  
      // Έλενχος για το αν η συσκευή που βρέθηκε ταιριάζει με το iTag MAC address
      if (device.getAddress().toString() == ITAG_MAC_ADDRESS && device.getRSSI() > -80) {
        Serial.println("iTag detected!");
        Serial.print("Device RSSI: ");
        Serial.println(device.getRSSI());
        iTagDetected = true;
        break;
      }
    }
  
    BLEDevice::deinit();
    return iTagDetected;
  }