#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_sleep.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <HardwareSerial.h>
#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>

// put function declarations here:
void publishGPSData(float, float, float);
void print_wakeup_reason();
void publishITagStatus(bool);
void connectToWiFi();
void connectToMQTT();
void batteryPercentage();
bool scanForBLE();
void connectToGSM();
void wakeUpSIM800L();
void callback(char *, byte *, unsigned int);


// #define uS_TO_S_FACTOR 1000000ULL // Conversion factor for micro seconds to seconds
#define SLEEP_TIME  5 * 60 * 1000000ULL   // Time ESP32 will go to sleep (in seconds)

RTC_DATA_ATTR int bootCount = 0;  // counting the times that esp32 wakes up

#define GSM_RX 6   // SIM800 TX -> ESP32 GPIO
#define GSM_TX 7   // SIM800 RX -> ESP32 GPIO
#define GSM_DTR 3  // SIM800 DTR -> ESP32 GPIO GPIO2
#define GPS_RX 20   // GPS 6M NEO TX -> ESP32 RX
#define GPS_TX 21   // GPS 6M NEO RX -> ESP32 TX
#define WAKEUP_PIN 2
#define BATTERY_PIN 0 // GPIO0 for battery status via ADC
#define SCAN_TIME 5  // Χρόνος σάρωσης BLE (σε δευτερόλεπτα)
#define PUBLISH_INTERVAL 10000  // 10 δευτερόλεπτα
#define GSM_BAUD 115200
#define GPS_BAUD 9600
#define ITAG_MAC_ADDRESS "ff:ff:c2:11:ec:17" // iTag's MAC address

// UARTs initialization
TinyGPSPlus gps;
HardwareSerial gpsSerial(0);  // Hardware Serial 0 for GPS
HardwareSerial simSerial(1);  // Hardware Serial 0 for SIM800L

TinyGsm modem(simSerial);
TinyGsmClient client(modem);  // when using GSM
// WiFiClient wifiClient;  // when using WiFi
PubSubClient mqttClient(client);  // when using GSM
// PubSubClient mqttClient(wifiClient);  // when using WiFi

// MQTT Settings
const char* mqttBroker = "homenetwork123.duckdns.org"; // MQTT broker IP
const int mqttPort = 1883; // MQTT Port (1883)
const char* mqttUser = "mqtt_user"; // MQTT Username (optional)
const char* mqttPassword = "mqtt_pass"; // MQTT Password(optional)

// GPRS Settings
const char* GPRS_USER = "";  // Empty if not required
const char* GPRS_PASS = "";  // Empty if not required
const char* APN = "internet";  // "internet" for Vodafone

// WiFi Settings
const char* ssid = "FREE_INTERNET 2.1";     // WiFi SSID
const char* password = "paliggenesias_13";  // WiFi password

// MQTT Topics
const char* gpsTopic = "home/gps_data";
const char* itagTopic = "home/itag_status";
const char* batteryTopic = "home/battery_status";

volatile unsigned long lastLowTime = 0;  // Time when the pin went LOW
volatile bool isCounting = false;        // If the pin is counting

bool stopPublishing = false;  // Variable to stop publishing GPS data

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

  simSerial.begin(GSM_BAUD, SERIAL_8N1, GSM_RX, GSM_TX);

  // WiFi connection
  // connectToWiFi();

  // Setting Battery Pin
  pinMode(BATTERY_PIN, INPUT);

  // Setting Wakeup Pin
  pinMode(WAKEUP_PIN, INPUT_PULLUP);

  // Add Interrupt to Wakeup_pin (detecting status change)
  attachInterrupt(digitalPinToInterrupt(WAKEUP_PIN), wakeupISR, CHANGE);

  Serial.println("ESP32 is awake!");

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
}

void loop() {
}

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

void publishGPSData(float latitude, float longitude, float speed) {

  if (!mqttClient.connected()) {
    connectToMQTT(); // Reconnection if the connection to MQTT is lost
  }
  
  // Creating a JSON message
  String payload = "{\"latitude\":";
  payload += String(latitude, 6);
  payload += ",\"longitude\":";
  payload += String(longitude, 6);
  payload += ",\"speed\":";
  payload += String(speed, 4);
  payload += "}";

  // Publish in the topic
  const char* gpsTopic = "home/gps_data"; // The topic we will publish the GPS data
  if (mqttClient.publish(gpsTopic, payload.c_str())) {
    Serial.println("GPS data published successfully via MQTT:");
    Serial.println(payload);
  } else {
    Serial.println("Failed to publish GPS data via MQTT.");
  }
}

void publishITagStatus(bool status) {
  if (!mqttClient.connected()) {
    connectToMQTT(); // Reconnection if the connection to MQTT is lost
  }
  
  // Creating a JSON message
  String payload = "{\"itag_detected\":";
  payload += (status ? "true" : "false");
  payload += "}";

  // Publish to topic "home/itag_status"
  const char* itagTopic = "home/itag_status";
  if (mqttClient.publish(itagTopic, payload.c_str())) {
    Serial.println("iTag status was successfully published via MQTT:");
    Serial.println(payload);
  } else {
    Serial.println("Failed to publish iTag status via MQTT.");
  }
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
  
  int adcValue = analogRead(BATTERY_PIN); // Ανάγνωση ADC
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

// // Συνάρτηση για σύνδεση στο WiFi
// void connectToWiFi() {
//   WiFi.begin(ssid, password);
  
//   // Περίμενε μέχρι να συνδεθεί στο WiFi
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.println("Σύνδεση στο WiFi...");
//   }
//   Serial.println("Συνδεδεμένο στο WiFi");
// }

void connectToMQTT() {
  mqttClient.setServer(mqttBroker, mqttPort);
  mqttClient.setCallback(callback);

  while (!mqttClient.connected()) {
    
    Serial.println("Connection to MQTT Broker ...");
    if (mqttClient.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT broker");
      mqttClient.subscribe(gpsTopic);  // Subscribe to topic
      mqttClient.subscribe(itagTopic);
      mqttClient.subscribe(batteryTopic);
    } else {
      Serial.print("Failed to connect to MQTT broker. Error: ");
      Serial.println(mqttClient.state());
      delay(2000);
    }
  }
}

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

// Callback function to handle incoming MQTT messages
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("Received MQTT message: ");
  Serial.println(message);

  if (message == "stop") {
    stopPublishing = true;
    Serial.println("Stopping GPS data transmission.");
  }
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
