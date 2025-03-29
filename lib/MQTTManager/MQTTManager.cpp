#include <MQTTManager.h>
#include <BLEDevice.h>
#include <PubSubClient.h>
#include <TinyGPSPlus.h>
#include <esp_sleep.h>
#include <BLEScan.h>
#include <HardwareSerial.h>
#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <config.h>

void connectToMQTT() {
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(callback);
  
    while (!mqttClient.connected()) {
      
      Serial.println("Connection to MQTT Broker ...");
      if (mqttClient.connect("ESP32Client", MQTT_USERNAME, MQTT_PASSWORD)) {
        Serial.println("Connected to MQTT broker");
        mqttClient.subscribe(MQTT_TOPIC_GPS);  // Subscribe to topic
        mqttClient.subscribe(MQTT_TOPIC_ITAG);
        mqttClient.subscribe(MQTT_TOPIC_BATTERY);
      } else {
        Serial.print("Failed to connect to MQTT broker. Error: ");
        Serial.println(mqttClient.state());
        delay(2000);
      }
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