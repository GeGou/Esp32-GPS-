#include "WiFiManager.h"
#include <WiFi.h>

void connectToWiFi() {
    WiFi.begin("your-SSID", "your-PASSWORD");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to WiFi!");
}
