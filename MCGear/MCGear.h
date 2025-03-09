#ifndef MCGEAR_H
#define MCGEAR_H

/*--------------------------------------------------------------
 * class MCGear
 * offers hich level commands to interact with a partly autoated GH
 *
 * 2020-07-17 AW Frame
 * 2024-05-04    Removed ref to timer
 * 2024-05-05    Adapt to Gear
 *
 *-------------------------------------------------------------*/
 
//--- inlcudes ----
 
#include <MCNode.h>
#include <stdint.h>

//--- service define ---


typedef enum DriveCommStates {
	eMCIdle,
	eMCWaiting,
	eMCBusy,
	eMCDone,
	eMCError,
	eMCTimeout,
}
 DriveCommStates;

	
class MCGear {
	public:
		MCGear();
		void Connect2MsgHandler(MsgHandler *);
		void SetNodeId(uint8_t);
		void SetActTime(uint32_t);

		DriveCommStates CheckComState();
		void ResetComState(); 
		void SetTORetryMax(uint8_t);
		void SetBusyRetryMax(uint8_t);
		
		DriveCommStates UpdateDriveStatus();		
		DriveCommStates SendReset();
	
	  DriveCommStates SwitchToGear(int32_t);
	  //DriveCommStates UpdateActGear();
	  DriveCommStates UpdateActGear(uint32_t *);
	
	  DriveCommStates ReadObject(uint16_t idx, uint8_t subIdx, uint8_t *dataPtr);
    DriveCommStates ReadObject(uint16_t idx, uint8_t subIdx, uint16_t *dataPtr);
	  DriveCommStates ReadObject(uint16_t idx, uint8_t subIdx, uint32_t *dataPtr);
	
  	DriveCommStates WriteObject(uint16_t idx, uint8_t subIdx, uint8_t value);
  	DriveCommStates WriteObject(uint16_t idx, uint8_t subIdx, uint16_t value);
  	DriveCommStates WriteObject(uint16_t idx, uint8_t subIdx, uint32_t value);

		DriveCommStates EnableDrive();
		DriveCommStates DisableDrive();
		DriveCommStates StopDrive();
	
	  DriveCommStates IsInPos();
		
		CWCommStates GetNodeState();
		SDOCommStates GetSDOState();
		
		uint8_t GetAccessStep();
		
		uint16_t GetSW();
		uint8_t GetOpMode();
		CWCommStates GetCWAccess();
		
		bool IsLive();
		uint16_t GetLastError();
		
		//hander to be registered at the OsTimer
		static void OnTimeOutCb(void *p) {
			((MCGear *)p)->OnTimeOut();
		};
	
		MCNode ThisNode;

	private:
		void OnTimeOut();
		DriveCommStates Wait4Status(uint16_t, uint16_t);
		
		DriveCommStates GearRxTxState = eMCIdle;
		
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
