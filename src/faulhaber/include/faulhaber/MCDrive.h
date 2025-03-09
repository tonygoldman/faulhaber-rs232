#ifndef MCDRIVE_H
#define MCDRIVE_H

/*--------------------------------------------------------------
 * class MCDrive
 * offers hich level commands to interact with a servo drive
 *
 * 2020-07-17 AW Frame
 *
 *-------------------------------------------------------------*/
 
//--- inlcudes ----
 
#include "faulhaber/MCNode.h"
#include <stdint.h>

//--- service define ---


typedef enum DriveCommStates {
	eMCIdle,
	eMCWaiting,
	eMCBusy,
	eMCDone,
	eMCError,
	eMCTimeout
} DriveCommStates;

typedef struct MCDriveParameter {
	uint16_t index;
	uint8_t subIndex;
	uint32_t value;
	uint8_t length;
} MCDriveParameter;
	
class MCDrive {
	public:
		MCDrive();
		void Connect2MsgHandler(MsgHandler *);
		void SetNodeId(uint8_t);
		void SetActTime(uint32_t);

		DriveCommStates CheckComState();
		void ResetComState(); 
		void SetTORetryMax(uint8_t);
		void SetBusyRetryMax(uint8_t);
		
		DriveCommStates UpdateDriveStatus();		
		DriveCommStates SendReset();

		DriveCommStates EnableDrive();
		DriveCommStates DisableDrive();
		DriveCommStates StopDrive();

	  DriveCommStates ReadObject(uint16_t idx, uint8_t subIdx, uint8_t *dataPtr);
    DriveCommStates ReadObject(uint16_t idx, uint8_t subIdx, uint16_t *dataPtr);
	  DriveCommStates ReadObject(uint16_t idx, uint8_t subIdx, uint32_t *dataPtr);
	
	  DriveCommStates DownloadParamterList(MCDriveParameter *, uint8_t);
	  DriveCommStates UploadParamterList(MCDriveParameter *, uint8_t);
	
  	DriveCommStates WriteObject(uint16_t idx, uint8_t subIdx, uint8_t value);
  	DriveCommStates WriteObject(uint16_t idx, uint8_t subIdx, uint16_t value);
  	DriveCommStates WriteObject(uint16_t idx, uint8_t subIdx, uint32_t value);

		DriveCommStates SetOpMode(int8_t);
		DriveCommStates SetProfile(uint32_t, uint32_t, uint32_t, int16_t);		
		
		DriveCommStates StartAbsMove(int32_t, bool);
		DriveCommStates StartRelMove(int32_t, bool);
	
  	DriveCommStates ConfigureHoming(int8_t);
		DriveCommStates DoHoming(uint16_t);

    DriveCommStates MoveAtSpeed(int32_t);
		
		DriveCommStates IsInPos();
		DriveCommStates IsHomingFinished();
		
		CWCommStates GetNodeState();
		SDOCommStates GetSDOState();
		
		uint8_t GetAccessStep();
		
		uint16_t GetSW();
		int8_t GetOpMode();
		CWCommStates GetCWAccess();
		
		bool IsLive();
		uint16_t GetLastError();
			
		MCNode ThisNode;

	private:
		void OnTimeOut();
		DriveCommStates Wait4Status(uint16_t, uint16_t);
		DriveCommStates MovePP(int32_t,bool, bool);
		
		DriveCommStates MCDriveRxTxState = eMCIdle;
		
		uint8_t AccessStep = 0;
		
		int8_t OpModeRequested;
		int8_t OpModeReported;		
		
		SDOCommStates SDOAccessState = eSDOIdle;
		CWCommStates CWAccessState = eCWIdle;

		uint8_t TORetryCounter = 0;
		uint8_t TORetryMax = 1;
		uint8_t BusyRetryCounter = 0;
		uint8_t BusyRetryMax = 1;
		
		bool isTimerActive = false;

		uint32_t actTime;

		bool isLive = false;
};

#endif
