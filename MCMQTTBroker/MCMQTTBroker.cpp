/*-------------------------------------------------------------------
 * MCMQTTBroker.cpp
 * implements a gerneic MQTT broker access for MCRemoteDrives
 *
 * 2024-07-21 AW
 *
 *-------------------------------------------------------------------*/
 
//--- includes ---
 
#include "MCMQTTBroker.h"

#define DEBUG_ONRX		0x0001
#define DEBUG_REGMC  	0x0002
#define DEBUG_PUBLISH 0x0004

//--- definitions ---

#define DEBUG_MQTTBroker (DEBUG_ONRX)
#define LED_PIN LED_BUILTIN


char clientSubTopic[] = "MCMulti/ledControl";     //payload[0] will control/set LED
char clientPubTopic[] = "MCMulti/ledState";       //payload[0] will have ledState value
char WillTopic[] = "/Will";

//--- LED handling -----
int ledState = 0;  //store the stae of the led to be able to report it

extern MQTTBrokerData myMQTTData;

//--- implementation ---

/*----------------------------------------------------------------------
 * non class methods
 * bool bool identifyTopic(char *RxTopic,char *Reference)
 * int32_t extractValue(byte* payload, unsigned int length)
 * void OnRxTopic(char* topic, byte* payload, unsigned int length) 
 *
 *--------------------------------------------------------------------------*/


/*------------------------------------------------------------------------
 * bool identifyTopic(char *RxTopic,char *Reference)
 * more or less a string compare only
 * check wether this is the received topic
 *
 * parameters
 * - the received topic
 * - a topic expected
 *
 * 2024-07-21 AW
 *
 *---------------------------------------------------------------------------*/

bool identifyTopic(char *RxTopic,char *Reference)
{
  if(strcmp(RxTopic,Reference) == 0)
    return true;
  else
    return false; 
}

//------------------------------------------------------------------------
//--- implementation of MCMQTTBroker

void OnRxTopic();

/*------------------------------------------------------------------------
 * void OnRxTopic(char* topic, byte* payload, unsigned int length) 
 * called by the PubSubClient when a tocis has been received
 *
 * parameters
 * - char pointer to the topic
 * - byte pointer to the payload
 * - length of the playload
 *
 * 2024-07-21 AW
 *
 *---------------------------------------------------------------------------*/

void OnRxTopic(char* topic, byte* payload, unsigned int length) 
{
  #if (DEBUG_MQTTBroker & DEBUG_ONRX)
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) 
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
	#endif

  if(identifyTopic(topic,clientSubTopic))
  {
    // Switch on the LED if 1 was received as first character
    if ((char)payload[0] == '1') 
    {
      ledState = 1;
			#if ((MyBoardType == WiFiTypeNINA) || (MyBoardType == WiFiTypeS3))
			//on Nano Every the LED pin is shared with the ET SPI
      digitalWrite(LED_PIN, HIGH); 
			#else
			Serial.println("No LED available!");
      #endif			
    } 
    else 
    {
      ledState = 0;
			#if ((MyBoardType == WiFiTypeNINA) || (MyBoardType == WiFiTypeS3))
			//on Nano Every the LED pin is shared with the ET SPI
      digitalWrite(LED_PIN, LOW); 
			#else
			Serial.println("No LED available!");
			#endif
    }
		{
			char LedUpdate[10]; 
      itoa(ledState, LedUpdate, 10);
      myMQTTData.thisClient->publish(clientPubTopic, LedUpdate,true);
		}
  }
	else
	{
		MQTTSubTopic *next = myMQTTData.FirstTopic;
		
	  while(next)
		{
			if(identifyTopic(topic,next->topic))
			{
        #if (DEBUG_MQTTBroker & DEBUG_ONRX)
				Serial.print("match found: ");
				Serial.println(next->topic);
        #endif
				//now call the regsitered handler
				if(next->OnRxCb.callback != NULL)
					next->OnRxCb.callback(next->OnRxCb.op,(void *)payload, length);
			}
		  next = next->next;
		}
	}
}

/*------------------------------------------------------------------------
 * void MCMQTT_Init(MQTTBrokerData *me)
 * init the instance of the struct and copy it to the local static pointer
 *
 * 2024-07-23 AW
 *
 *------------------------------------------------------------------------*/


void MCMQTT_Init(MQTTBrokerData *me)
{
  me->FirstTopic = NULL;
  me->LastTopic = NULL;
}
 /*------------------------------------------------------------------------
 * void MCMQTT_ConnectToBroker()
 *
 * 2024-07-21 AW
 *
 *-----------------------------------------------------------------------*/

void MCMQTT_RegisterBroker(MQTTBrokerData *me, PubSubClient *client, const char * serverName)
{
	me->thisClient = client;
  client->setServer(serverName, 1883);
  client->setCallback(OnRxTopic);
}


/*------------------------------------------------------------------------
 * void MCMQTT_SetClientName(myMQTTData *me, char *myName)
 * no behavior here so far
 *
 * parameters
 * - the WiFiclient to be used
 * - a name to be used as part of all pub/sub topics
 *
 * 2024-07-21 AW
 *
 *---------------------------------------------------------------------------*/
 
void MCMQTT_SetClientName(MQTTBrokerData *me, const char *myName)
{
	  uint8_t srcIter = 0;
	  uint8_t destIter = 0;
	
	  strcpy(me->clientName, myName);
	
	  while(myName[srcIter] != 0)
			me->willName[destIter++] = myName[srcIter++];
		
		srcIter = 0;
		while(WillTopic[srcIter] != 0)
			me->willName[destIter++] = WillTopic[srcIter++];

		me->willName[destIter++] = 0;
}

/*------------------------------------------------------------------------
 * void MCMQTT_Reconnect() 
 * handle a reconnection to the broker if necessary
 *
 * parameters
 *
 * 2024-07-21 AW
 *
 *---------------------------------------------------------------------------*/

void MCMQTT_Reconnect(MQTTBrokerData *me) 
{
	
  // Loop until we're reconnected
  while (!(me->thisClient->connected())) 
  {    
		Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    //if (client.connect(clientId.c_str(), mqttUsername, mqttPassword)) 
		
    if(me->thisClient->connect(me->clientName,NULL,NULL,me->willName,1,true,"is Offline"))
    {
			MQTTSubTopic *nextTopic = me->FirstTopic;
			
      Serial.println("connected");
      // ... and resubscribe
			// the globa topic for this client
      me->thisClient->subscribe(clientSubTopic);

      while(nextTopic != NULL)
			{
				me->thisClient->subscribe(nextTopic->topic);
				Serial.print("register: ");
				Serial.println(nextTopic->topic);
				
				nextTopic = nextTopic->next;
			}

      me->thisClient->publish(me->willName,"is Online",true);
    } 
    else 
    {
      Serial.print("failed, rc=");
      Serial.print(me->thisClient->state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/*-----------------------------------------------------------------------------
 * void MCMQTT_RegisterSubTopic(MQTTBrokerData *me, MQTTSubTopic *topicData)
 * add a single new entry of topic description to the clients management list
 *
 * 2024-07-22 AW
 *
 *-----------------------------------------------------------------------------*/

void MCMQTT_RegisterSubTopic(MQTTBrokerData *me, MQTTSubTopic *topicData)
{
	if(me->FirstTopic == NULL)
	{
		me->FirstTopic = topicData;
		me->LastTopic = topicData;
	}
	else
	{
		me->LastTopic->next = topicData;
		me->LastTopic = topicData;
	}
  me->LastTopic->next = NULL;	
}

/*--------------------------------------------------------------------------------
 * void MCMQTT_Publish(char* topic, char* payload)
 * publish the given payload under the given topic to the broker we are regsitered to
 *
 * 2024-07-23 AW
 *
 *-------------------------------------------------------------------------------*/

void MCMQTT_Publish(MQTTBrokerData *me, char *topic, char *payload, bool retain)
{
	#if(DEBUG_MQTTBroker & DEBUG_PUBLISH)
	Serial.print("Publish: ");
	Serial.print(topic);
	Serial.print(" >>");
	Serial.println(payload);
	#endif
  me->thisClient->publish(topic, payload,retain);	
}

/*--------------------------------------------------------------------------------
 * void MCMQTT_Update(MQTTBrokerData *me)
 * simp,y call the loop() of the mqtt client component
 *
 * 2024-07-24 AW
 *
 *-------------------------------------------------------------------------------*/

void MCMQTT_Update(MQTTBrokerData *me)
{
	me->thisClient->loop();
}