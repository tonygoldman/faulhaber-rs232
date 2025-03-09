/*-----------------------------------------------------------------
 * MCRemoteControlled.cpp
 * implements the class to control a MCDrive remotely via MQTT
 *
 * Has to be connected to the MCMQTTBroker explicitely and will publish#
 * act values and respond to commands via callback
 *
 * 2024-07-21 AW
 *
 *--------------------------------------------------------------------*/
 
 //--- inlcudes ---
 #include "MCRemoteControlled.h"
 #include "MC_Helpers.h"
 
 //--- defines ---
 
#define DEBUG_ONRX		  0x0001
#define DEBUG_REGNODE	  0x0002
#define DEBUG_PUBLISH		0x0004

#define DoAutoUpdate true


#define DEBUG_MCREMOTE (DEBUG_ONRX | DEBUG_REGNODE)

 //--- globals ---
 
const char *SubTPos = "TPos";
const char *SubTSpeed = "TSpeed";
const char *SubCommand = "Command";

 
const char *PubActPos = "ActPos";
const char *PubActSpeed = "ActSpeed";
const char *PubActVoltage = "ActVoltage";
const char *PubActTemp = "MotorTemp";
const char *PubStatusWord = "StatusW";
const char *PubDriveState = "DriveState";
	
 //--- implementation ---
 
 /*------------------------------------------------------------------------
  * MCRemoteControlled::MCRemoteControlled(uint8_t NodeId, char *NodeName, int8_t HomingMethod)
	*
	* initialize the MCDrive and the topic list
	*
	* 2024-07-21 AW
	*
	*--------------------------------------------------------------------------------------*/
 
 MCRemoteControlled::MCRemoteControlled(MQTTBrokerData *client, uint8_t NodeId, const char *NodeName, int8_t HomingMethod, const char *MQTTClientName)
{
	driveName = NodeName;
	aDrive.SetNodeId(NodeId);
	
	clientData = client;
	
	DriveAHomingMethod = HomingMethod;	
	
	//fill the subscription list
	ConCatTopic(Topics[eRemoteTopicTPos].topic,MQTTClientName, NodeName, SubTPos);
	ConCatTopic(Topics[eRemoteTopicTSpeed].topic,MQTTClientName, NodeName, SubTSpeed);
	ConCatTopic(Topics[eRemoteTopicCommand].topic, MQTTClientName, NodeName, SubCommand);

	Topics[eRemoteTopicTPos].next = NULL;
	Topics[eRemoteTopicTPos].OnRxCb.op = (void *)this;
	Topics[eRemoteTopicTPos].OnRxCb.callback = (tfunction_pointer_t)MCRemoteControlled::OnMsgRxTPos;

	Topics[eRemoteTopicTSpeed].next =NULL;
	Topics[eRemoteTopicTSpeed].OnRxCb.op = (void *)this;
	Topics[eRemoteTopicTSpeed].OnRxCb.callback = (tfunction_pointer_t)MCRemoteControlled::OnMsgRxTSpeed;

	Topics[eRemoteTopicCommand].next = NULL;
	Topics[eRemoteTopicCommand].OnRxCb.op = (void *)this;
	Topics[eRemoteTopicCommand].OnRxCb.callback = (tfunction_pointer_t)MCRemoteControlled::OnMsgRxCommand;
	
	//preset push topics
	ConCatTopic(TopicActPos, MQTTClientName, NodeName, PubActPos);
	ConCatTopic(TopicActSpeed, MQTTClientName, NodeName, PubActSpeed);
	ConCatTopic(TopicActVoltage, MQTTClientName, NodeName, PubActVoltage);
	ConCatTopic(TopicActTemp, MQTTClientName, NodeName, PubActTemp);
	ConCatTopic(TopicStatusWord, MQTTClientName, NodeName, PubStatusWord);
	ConCatTopic(TopicDriveState, MQTTClientName, NodeName, PubDriveState);
}

/*-----------------------------------------------------------------------------
 * ConnectToMsgHandler(MsgHandler *Handler)
 * 
 * connect this instance of the MCTestCaycle to the MsgHandler in charge
 *
 * 2024-07-21 AW
 *
 *---------------------------------------------------------------------------*/

void MCRemoteControlled::ConnectToMsgHandler(MsgHandler *Handler)
{
	aDrive.Connect2MsgHandler(Handler);
}

/*--------------------------------------------------------------------------------
 * void MCRemoteControlled::RegisterTopicsAtBroker()
 *
 * register the topics of this instance at the broker
 *
 * 2024-07-21 AW
 *
 *---------------------------------------------------------------------------------*/

void MCRemoteControlled::RegisterTopicsAtBroker()
{
  MCMQQT_RegisterSubTopic(clientData, &(Topics[eRemoteTopicTPos]));	
  MCMQQT_RegisterSubTopic(clientData, &(Topics[eRemoteTopicTSpeed]));	
  MCMQQT_RegisterSubTopic(clientData, &(Topics[eRemoteTopicCommand]));	
	
	#if(DEBUG_MCREMOTE & DEBUG_REGNODE) 
	Serial.print("TPos@: ");
	Serial.println(TopicActPos);
	#endif

}

/*------------------------------------------------------------------------
 * int32_t extractValue(byte* payload, unsigned int length)
 * returns a numeric value out of the payload
 *
 * parameters
 * - the paylod of the received topic
 * - the length of the payload
 *
 * 2024-07-21 AW
 *
 *---------------------------------------------------------------------------*/

int32_t MCRemoteControlled::ExtractValue(char* payload, unsigned int length)
{
  uint8_t idx = 0;
  int32_t value = 0;
  
  if(payload[idx] == '-')
    idx++;
    
  while(idx < length)
  {
    value = value * 10;
    value = value + (payload[idx] - '0');
    idx++;
  }
  if(payload[0] == '-')
    value = -1 * value;

  return value;
}
/*-------------------------------------------------------------------------------
 * void MCRemoteControlled::OnRxTPos(char *payload)
 * the handler for procesing received target pos
 *
 * 2024-07-21 AW
 *
 *-----------------------------------------------------------------------------*/

void MCRemoteControlled::OnRxTPos(char *payload, unsigned int length)
{
  TPos = ExtractValue(payload, length);

	#if(DEBUG_MCREMOTE & DEBUG_ONRX) 
	Serial.print(driveName);
	Serial.print(">> TPos: ");
	Serial.println(TPos,DEC);
	#endif
}

/*-------------------------------------------------------------------------------
 * void MCRemoteControlled::OnRxTSpeed(char *payload)
 * the handler for procesing received target pos
 *
 * 2024-07-21 AW
 *
 *-----------------------------------------------------------------------------*/

void MCRemoteControlled::OnRxTSpeed(char *payload, unsigned int length)
{
  TSpeed = ExtractValue(payload, length);

	#if(DEBUG_MCREMOTE & DEBUG_ONRX) 
	Serial.print(driveName);
	Serial.print(">> TSpeed: ");
	Serial.println(TSpeed,DEC);
	#endif
}

/*-------------------------------------------------------------------------------
 * void MCRemoteControlled::OnRxCommand(char *payload)
 * the handler for procesing received target pos
 *
 * 2024-07-21 AW
 *
 *-----------------------------------------------------------------------------*/

void MCRemoteControlled::OnRxCommand(char *payload, unsigned int length)
{
	NextCommand = (MCRemoteCommands)ExtractValue(payload, length);

	#if(DEBUG_MCREMOTE & DEBUG_ONRX) 
	Serial.print(driveName);
	Serial.print(">> Command: ");
	Serial.println(NextCommand,DEC);
	#endif
}

/*------------------------------------------------------------------------------
 * void MCRemoteControlled::ConCatTopic(char *topic,const char *BrokerPrefix,const char *NodeName, const char *TopicName)
 *
 * copy the three componets into the topic field to form a meaningfull topic
 *
 * 2024-07-21 AW
 *
 *--------------------------------------------------------------------------------*/

void MCRemoteControlled::ConCatTopic(char *topic, const char *ClientName, const char *DriveName, const char *TopicName)
{
	uint8_t iter = 0;
	uint8_t iterTopic = 0;

	//start with the Prefix
  while((ClientName[iter] != 0) && (iterTopic < TopicLength-1))
    topic[iterTopic++] = ClientName[iter++];

	topic[iterTopic++] = '/';

	//add the NodeName
  iter = 0;
  while((DriveName[iter] != 0) && (iterTopic < TopicLength-1))
    topic[iterTopic++] = DriveName[iter++];
	
	topic[iterTopic++] = '/';
	
	//end with the topic itself
  iter = 0;
  while((TopicName[iter] != 0) && (iterTopic < TopicLength-1))
    topic[iterTopic++] = TopicName[iter++];
	
	//add a 0
	if(iterTopic < TopicLength)
	  topic[iterTopic] = 0;	
	else
		topic[TopicLength-1] = 0;
}

/*------------------------------------------------------------------------------
 * void MCRemoteControlled::Update(uint32_t time)
 *
 * update the drive behavior based on the receioved commands
 *
 * 2024-07-21 AW
 *
 *--------------------------------------------------------------------------------*/

void MCRemoteControlled::MCRemoteControlled::Update(uint32_t time)
{
	
	currentMillis = time;
	aDrive.SetActTime(currentMillis);
	
	DriveCommStates NodeState = aDrive.CheckComState();
   
  if((NodeState == eMCError) || (NodeState == eMCTimeout))
  {
    //Serial.println("Main: Reset NodeA State");
    aDrive.ResetComState();
    //should be avoided in the end
    actDriveStep = eMCRemoteNone;
  }
	
  switch(actDriveStep)
  {
    case eMCRemoteNone:
      //do nothing - that's the idle state
      if(NextCommand > eMCRemoteNone)
      {
        //start handling of the request
        actDriveStep = NextCommand;
        //reset request again          
        NextCommand = eMCRemoteNone;
        isAutoUpdate = false;
        Serial.print("Start: ");
        Serial.println((uint16_t)actDriveStep,DEC);
        //actDriveStep = eMCRemoteNone;
      }
      else
      {
        //no request pending
        //check for timeout of auto-update
        if(lastUpdateAt + updateEvery < currentMillis)
        {
          lastUpdateAt = currentMillis;
					#if DoAutoUpdate
          actDriveStep = eMCRemoteUpdateActValues;
          isAutoUpdate = true;
					#endif
        }
      }
      break;
    case eMCRemoteUpdateStatus:
      //get a copy of the drive status
      if((aDrive.UpdateDriveStatus()) == eMCDone)
      {
        //switch back to idle state
        actDriveStep = eMCRemoteNone;
				
        Serial.println("Main: Status updated");
      }
      break;
    case eMCRemoteDisable:
      //disable the drive
      if((aDrive.DisableDrive()) == eMCDone)
      {
        char payload[20];

				strcpy(payload, "Disabled");
        //switch back to idle state
        actDriveStep = eMCRemoteNone;

        Serial.println("Main: Drive disabled");
				MCMQQT_Publish(clientData, TopicDriveState, payload, true);
      }
      break;
    case eMCRemoteEnable:
      //enable the drive
      if((aDrive.EnableDrive()) == eMCDone)
      {
        char payload[20];

				strcpy(payload, "Enabled");
        //switch back to idle state
        actDriveStep = eMCRemoteNone;

        Serial.println("Main: Drive enabled");
				MCMQQT_Publish(clientData, TopicDriveState, payload, true);
      }
      break;
    case eMCRemoteHalt:
      //move at speed
      if((aDrive.MoveAtSpeed(0)) == eMCDone)
      {
        //switch back to idle state
        actDriveStep = eMCRemoteNone;

        Serial.println("Main: PV @ 0");
      }
      break;
    case eMCRemoteMoveSpeed:
      //move at speed
      if((aDrive.MoveAtSpeed(TSpeed)) == eMCDone)
      {
        //switch back to idle state
        actDriveStep = eMCRemoteNone;

        Serial.print("Main: PV @ ");
        Serial.println(TSpeed);
      }
      break;
    case eMCRemoteMoveHome:
      //move to pos
      if((aDrive.StartAbsMove(0,false)) == eMCDone)
      {
        //switch back to idle state
        actDriveStep = eMCRemoteNone;

        Serial.println("Main: Moved to 0");
      }
      break;
    case eMCRemoteMoveAbs:
      //move to pos
      if((aDrive.StartAbsMove(TPos,false)) == eMCDone)
      {
        //switch back to idle state
        actDriveStep = eMCRemoteNone;

        Serial.print("Main: Moved to ");
        Serial.println(TPos);
      }
      break;
    case eMCRemoteMoveRel:
      //move to pos
      if((aDrive.StartRelMove(TPos,false)) == eMCDone)
      {
        //switch back to idle state
        actDriveStep = eMCRemoteNone;

        Serial.print("Main: Moved by ");
        Serial.println(TPos);
      }
      break;
    case eMCRemoteCheckInPos:
      //wait for pos
      if(aDrive.IsInPos() == eMCDone)
      {
        //switch back to idle state
        actDriveStep = eMCRemoteNone;

        Serial.println("Main: Drive is in Pos");
      }
      break;
    case eMCRemoteUpdateActValues:
      //Update ActValues
      if(aDrive.UploadParamterList(ListOfValues,MCRemoteNumActValues) == eMCDone)
      {
        char payload[20];
        //switch back to idle state
        actDriveStep = eMCRemoteNone;

        if(!isAutoUpdate)
          Serial.println("Main: updates pos/speed");
        
        //pos				
        itoa((int32_t)(ListOfValues[0].value), payload, 10);
				MCMQQT_Publish(clientData, TopicActPos, payload, true);

        //speed
        itoa((int32_t)(ListOfValues[1].value), payload, 10);
				MCMQQT_Publish(clientData, TopicActSpeed, payload, true);
				
        //supply
        itoa((uint16_t)(ListOfValues[2].value), payload, 10);
				MCMQQT_Publish(clientData, TopicActVoltage, payload, true);
				
        //temp
        itoa((int16_t)(ListOfValues[3].value), payload, 10);
				MCMQQT_Publish(clientData, TopicActTemp, payload, true);

      }
      break;
    default:
      //switch back to idle state
      actDriveStep = eMCRemoteNone;
      Serial.println("Main: Unexpected command");
      break;
  }	
}

