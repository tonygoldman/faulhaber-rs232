#ifndef MC_REMOTE_H
#define MC_REMOTE_H

/*--------------------------------------------------------------------
 * a class to implement remote control of a MCDrive via MCMQTTBroker
 *
 * 2024-07-21 AW 
 *
 *-------------------------------------------------------------------*/
 
//--- includes ---

#include <stdint.h>
#include <MCDrive.h>
#include <MCMQTTBroker.h>

//--- defines ---


const uint8_t MCRemoteNumActValues = 4;

const uint8_t NumSubTopics = 3;

enum MCRemoteSubTopicIdx {
	eRemoteTopicTPos,
	eRemoteTopicTSpeed,
	eRemoteTopicCommand
};

typedef enum MCRemoteCommands {
	eMCRemoteNone = 0,
	eMCRemoteUpdateStatus = 1,
	eMCRemoteUpdateActValues = 2,
	eMCRemoteDisable = 3,
	eMCRemoteEnable = 4,
	eMCRemoteMoveAbs = 5,
	eMCRemoteMoveHome = 6,
	eMCRemoteMoveRel = 7,
	eMCRemoteCheckInPos = 8,	
	eMCRemoteMoveSpeed = 9,
	eMCRemoteHalt= 10,	
	eMCRemoteGoHome = 11
} MCRemoteCommands; 


//--- definition ---

class MCRemoteControlled {
	public:
		MCRemoteControlled(MQTTBrokerData *, uint8_t, const char *, int8_t, const char *);
	  void ConnectToMsgHandler(MsgHandler *);
	  
	  void RegisterTopicsAtBroker();
	  void Update(uint32_t);
				
		static void OnMsgRxTPos(void *op,void *p, uint32_t len) {
			((MCRemoteControlled *)op)->OnRxTPos((char *)p, len);
		};
		
		static void OnMsgRxTSpeed(void *op,void *p, uint32_t len) {
			((MCRemoteControlled *)op)->OnRxTSpeed((char *)p, len);
		};
		
		static void OnMsgRxCommand(void *op,void *p, uint32_t len) {
			((MCRemoteControlled *)op)->OnRxCommand((char *)p, len);
		};
		
		uint32_t updateEvery = 2000;

	private:
	  void ConCatTopic(char *,const char *,const char *, const char *);

  	void OnRxTPos(char *, unsigned int);
		void OnRxTSpeed(char *, unsigned int);
		void OnRxCommand(char *, unsigned int);
	
	  char TopicActPos[TopicLength];
    char TopicActSpeed[TopicLength];
    char TopicActVoltage[TopicLength];
    char TopicActTemp[TopicLength];
    char TopicStatusWord[TopicLength];
		char TopicDriveState[TopicLength];

	  int32_t ExtractValue(char*, unsigned int);
	
	  const char *driveName;	
	
	  MQTTSubTopic Topics[NumSubTopics];
	  MQTTBrokerData *clientData;
	
	  int8_t DriveAHomingMethod;

	  MCDrive aDrive;
    
    uint32_t lastUpdateAt = 0;	
		bool isAutoUpdate;
    uint32_t currentMillis;
	
	  int32_t TPos;
	  int32_t ActPos;
	  int32_t TSpeed;
	  int32_t ActSpeed;
	  
	  int16_t ActMotorTemp;
	  uint16_t SupplyVoltage;
	
	  uint16_t ControlWord;
	  uint16_t StatusWord;
	
	  MCRemoteCommands NextCommand = eMCRemoteNone;	
		MCRemoteCommands actDriveStep = eMCRemoteNone;
		
		MCDriveParameter ListOfValues[MCRemoteNumActValues] = {{0x6063, 0, 0, 4},  //act pos
                                    {0x606c, 0, 0, 4},  //act speed
																		{0x2325, 7, 0 ,2},	//act motor supply
																		{0x2326, 3, 0, 2}}; //act motor temp
};

#endif
