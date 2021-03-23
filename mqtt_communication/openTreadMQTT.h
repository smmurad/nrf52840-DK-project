/* MQTT library for C++ Server Application 
 *
 * Author: Eivind Jølsgard (based on Bloms nRF-dongle legacy layer)
 * Date: Autumn 2020
 * */

#ifndef MQTT_H
#define MQTT_H

#include <stdint-gcc.h>
#include <stdbool.h>

typedef struct topic{
	uint16_t id;
	uint16_t id_pub; 
	unsigned char* name;
	bool subscribed;
} topic_t;

typedef struct mqtt_message{
	topic_t topic;
	uint8_t* message;
	uint32_t message_length; 
} mqtt_message_t;


void vMQTTTask(void *pvParameters);

#endif //MQTT_H