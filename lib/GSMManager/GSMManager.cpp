#include <GSMManager.h>
#include <BLEDevice.h>
#include <TinyGPSPlus.h>
#include <esp_sleep.h>
#include <BLEScan.h>
#include <config.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>

HardwareSerial simSerial(1);  // Hardware Serial 0 for SIM800L
TinyGsm modem(simSerial);
TinyGsmClient client(modem);  // when using GSM
PubSubClient mqttClient(client);  // when using GSM

// WiFiClient wifiClient;  // when using WiFi
// PubSubClient mqttClient(wifiClient);  // when using WiFi

void connectToGSM() {
    Serial.println("Connecting to GSM...");
    simSerial.begin(GSM_BAUD, SERIAL_8N1, GSM_RX, GSM_TX);
  
    // Προσπάθεια επανεκκίνησης του modem μέχρι να επιτύχει
    while (!modem.restart()) {
      Serial.println("Failed to restart the modem! Retrying...");
      delay(5000);  // Αναμονή πριν την επόμενη προσπάθεια
    }
  
    Serial.println("Modem restarted successfully.");
  
    // Προσπάθεια σύνδεσης στο GPRS μέχρι να επιτύχει
    while (!modem.gprsConnect(APN, GPRS_USER, GPRS_PASS)) {
      Serial.println("Failed to connect to GPRS. Retrying...");
      delay(5000);  // Αναμονή πριν την επόμενη προσπάθεια
    }
  
    Serial.println("Connected to GPRS.");
}


void wakeUpSIM800L() {
    Serial.println("Waking up SIM800L...");
    simSerial.println("AT");  // Στείλε οποιαδήποτε εντολή για να ξυπνήσει
    delay(400);
    simSerial.println("AT+CSCLK=0");
    delay(1000);
  
    while (simSerial.available()) {
      String response = simSerial.readString();
      Serial.println("SIM800L: " + response);
      if (response.indexOf("OK") != -1) {
        Serial.println("SIM800L woke up");
        return;
      }
    }
    Serial.println("No response from SIM800L. Retrying...");
    // wakeUpSIM800L();
  }