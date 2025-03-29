#include <PubSubClient.h>
#include <MQTTManager.h>
#include <config.h>

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