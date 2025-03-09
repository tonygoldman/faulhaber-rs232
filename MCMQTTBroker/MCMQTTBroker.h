#ifndef MC_MQTTBROKER_H
#define MC_MQTTBROKER_H

/*--------------------------------------------------------------------
 * Implements a generic MQTT broker
 * needs an instance of a WiFiClient to be provided and will try to estabish 
 * the connetection to the defined broker
 * proved a method to publish items and to register for items
 *
 *
 *-------------------------------------------------------------------*/
 
//--- includes ---

#define WiFiTypeNone 0
#define WiFiTypeNINA 1
#define WiFiTypeS3   2

//#define MyBoardType WiFiTypeNone
//#define MyBoardType WiFiTypeNINA
#define MyBoardType WiFiTypeS3

#include <stdint.h>

#if (MyBoardType == WiFiTypeS3)
#include <WiFiS3.h>
#elif (MyBoardType == WiFiTypeNINA)
#include <WiFiNINA.h>
#elif (MyBoardType == WiFiTypeNone)
#include <SPI.h>
#include <Ethernet.h>
#endif

#include <PubSubClient.h>
#include "MC_Helpers.h"

//--- defines ----
#define TopicLength 40

typedef struct MQTTSubTopic {
	MQTTSubTopic *next;
	char topic[TopicLength];
	tfunction_holder OnRxCb;
} MQTTSubTopic;

typedef struct MQTTBrokerData {
	PubSubClient *thisClient;
  char clientName[TopicLength];
	char willName[TopicLength];
	MQTTSubTopic *FirstTopic;
	MQTTSubTopic *LastTopic;
} MQTTBrokerData;

void MCMQTT_Init(MQTTBrokerData *);
void MCMQTT_SetClientName(MQTTBrokerData *,const char *);
void MCMQTT_RegisterBroker(MQTTBrokerData *,PubSubClient *, const char *);
void MCMQTT_Reconnect(MQTTBrokerData *); 
void MCMQTT_RegisterSubTopic(MQTTBrokerData *, MQTTSubTopic *);
void MCMQTT_Publish(MQTTBrokerData *, char*, char*, bool);

void MCMQTT_Update(MQTTBrokerData *);
	

	

#endif