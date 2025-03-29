#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <PubSubClient.h>

void connectToMQTT();
void publishITagStatus(bool status);
void callback(char* topic, byte* message, unsigned int length);

#endif
