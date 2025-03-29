#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_sleep.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <HardwareSerial.h>

#include <config.h>
#include <BLEManager.h>
#include <GPSManager.h>
#include <GSMManager.h>
#include <MQTTManager.h>
#include <WiFiManager.h>

// // put function declarations here:
void print_wakeup_reason();

// Global Variables
RTC_DATA_ATTR int bootCount = 0;  // counting the times that esp32 wakes up
volatile unsigned long lastLowTime = 0;  // Time when the pin went LOW
volatile bool isCounting = false;        // If the pin is counting

// bool stopPublishing = false;  // Variable to stop publishing GPS data

// Function to handle the interrupt, starts the countdown
void IRAM_ATTR wakeupISR() {
  int pinState = digitalRead(WAKEUP_PIN);

  if (pinState == HIGH) {
    isCounting = false;  // If the pin is HIGH, stop counting
  } else {
    if (!isCounting) {
      lastLowTime = millis();  // If the pin is LOW, start counting
      isCounting = true;
    }
  }
}

// After waking up, the ESP32 resets and starts running from the beginning of the setup() function.
void setup() {
  // Serial communication for debugging
  Serial.begin(115200);
  delay(1000);  //Take some time to open up the Serial Monitor

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  // Setting Serial Communication for GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  pinMode(GSM_DTR, OUTPUT);  // Setting DTR pin as OUTPUT
  digitalWrite(GSM_DTR, HIGH);  // Making DTR pin HIGH

  // simSerial.begin(GSM_BAUD, SERIAL_8N1, GSM_RX, GSM_TX);

  // WiFi connection
  // connectToWiFi();

  // Setting Battery Pin
  pinMode(BATTERY_STATUS_PIN, INPUT);

  // Setting Wakeup Pin
  pinMode(WAKEUP_PIN, INPUT_PULLUP);

  // Add Interrupt to Wakeup_pin (detecting status change)
  attachInterrupt(digitalPinToInterrupt(WAKEUP_PIN), wakeupISR, CHANGE);

  Serial.println("Esp32 is awake!");

  wakeUpSIM800L();

  connectToGSM();

  connectToMQTT();
  
  if (!scanForBLE()) {
    Serial.println("Starting GPS data transmission...");
    unsigned long lastPublishTime = 0;

    while (!stopPublishing) {
      if (millis() - lastPublishTime >= PUBLISH_INTERVAL) {
        // Update GPS data
        while (gpsSerial.available() > 0) {
          gps.encode(gpsSerial.read());
        }
        if (gps.location.isValid()) {
          float latitude = gps.location.lat();
          float longitude = gps.location.lng();
          float speed = gps.speed.kmph();
          
          Serial.print("Latitude: ");
          Serial.print(latitude, 6);
          Serial.print(", Longitude: ");
          Serial.println(longitude, 6);
          Serial.print("Speed: ");
          Serial.print(speed);
    
          // Δημοσίευση δεδομένων μέσω MQTT
          publishITagStatus(true);
          publishGPSData(latitude, longitude, speed);
        } else {
          Serial.println("Invalid GPS data. Wait...");
        }
        lastPublishTime = millis();
      }

      mqttClient.loop();  // Διατήρηση σύνδεσης MQTT

      // Serial.println("is counting: " + String(isCounting));
      // Serial.println("millis: " + String(millis()));
      // Serial.println("lastLowTime: " + String(lastLowTime));
      // Serial.println("sleep time: " + String(SLEEP_TIME));

      // millis() time that esp32 is running
      // lastLowTime time that pin went LOW (need to tilt the device)
      if (isCounting && (millis() - lastLowTime >= SLEEP_TIME / 1000)) {
        Serial.println("Pin remained LOW for 5 minutes. Going to deep sleep...");
        modem.gprsDisconnect();
        esp_deep_sleep_enable_gpio_wakeup(1 << WAKEUP_PIN, ESP_GPIO_WAKEUP_GPIO_HIGH);
        esp_deep_sleep_start();
        Serial.println("This will never be printed");
      }
      delay(500);
    }
    Serial.println("Exited GPS loop. Waiting for new command...");
  }
  else {
    Serial.println("iTag detected. Going to deep sleep...");
    modem.gprsDisconnect();
    esp_deep_sleep_enable_gpio_wakeup(1 << WAKEUP_PIN, ESP_GPIO_WAKEUP_GPIO_HIGH);
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
  }
}

void loop() {
}


void publishBatteryStatus(float batteryPercentage) {
  if (!mqttClient.connected()) {
    connectToMQTT();
  }

  String payload = "{\"battery_percentage\":";
  payload += String(batteryPercentage, 1);
  payload += "}";

  const char* batteryTopic = "home/battery_status";
  if (mqttClient.publish(batteryTopic, payload.c_str())) {
    Serial.println("Battery percentage successfully published via MQTT:");
    Serial.println(payload);
  } else {
    Serial.println("Failed to publish battery percentage via MQTT.");
  }
}

// Calculation of the battery percentage
void batteryPercentage() {
  // Ρυθμίσεις για ανάγνωση μπαταρίας
  const float referenceVoltage = 3.3; // Τάση αναφοράς ESP32-C3
  const float batteryMaxVoltage = 4.2; // Μέγιστη τάση μπαταρίας
  const float batteryMinVoltage = 3.0; // Ελάχιστη τάση μπαταρίας (όταν θεωρείται "άδεια")
  
  int adcValue = analogRead(BATTERY_STATUS_PIN); // Ανάγνωση ADC
  float voltage = (adcValue / 4095.0) * referenceVoltage * 2; // Υπολογισμός τάσης (με διαιρέτη τάσης)
  
  // Μετατροπή τάσης σε ποσοστό
  float percentage = ((voltage - batteryMinVoltage) / (batteryMaxVoltage - batteryMinVoltage)) * 100.0;

  // Περιορισμός ποσοστού στο εύρος 0% - 100%
  if (percentage > 100.0) percentage = 100.0;
  if (percentage < 0.0) percentage = 0.0;
  
  Serial.print("Battery Percentage: ");
  Serial.print(percentage);
  Serial.println("%");

  publishBatteryStatus(percentage);
}



// Function to print the wakeup reason for ESP32
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:     Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1:     Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER:    Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP:      Serial.println("Wakeup caused by ULP program"); break;
    default:                        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

