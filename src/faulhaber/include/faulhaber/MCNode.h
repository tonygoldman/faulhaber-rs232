#ifndef MCNODE_H
#define MCNODE_H

/*--------------------------------------------------------------
 * class MCNode
 * handles writes to the CW and the asynch received SW
 * handles also the boot MSG and the reset command
 *
 * 2020-05-24 AW Frame
 * 2024-05-07 AW remove ref to timer
 *
 *-------------------------------------------------------------*/
 
//--- inlcudes ----

#include "faulhaber/MsgHandler.h"
#include "faulhaber/SDOHandler.h"
#include <stdint.h>

//--- service define ---

//define all Tx Msg types
//ResetRequest WriteCwRequest
#define MCNodeTxMsg UART_Msg

const uint8_t MaxDeviceNameLen = 32;

typedef struct __attribute__((packed)) CwSwMsg {
   uint8_t u8Prefix  : 8;
   uint8_t u8Len     : 8;
   uint8_t u8NodeNr  : 8;
   MCMsgCommands u8Cmd : 8;
   uint16_t Payload;
   uint8_t CRC;
   uint8_t u8Suffix;
} CwSwMsg;

typedef struct __attribute__((packed)) ResetReqMsg {
   uint8_t u8Prefix  : 8;
   uint8_t u8Len     : 8;
   uint8_t u8NodeNr  : 8;
   MCMsgCommands u8Cmd : 8;
   uint8_t CRC;
   uint8_t u8Suffix;
} ResetReqMsg;

//no need for CW and SW - these are plain 16 bit payload

typedef enum CWCommStates {
	eCWIdle,
	eCWWaiting,
	//eCWBusy,
	eCWDone,
	eCWError,
	eCWRetry,
	eCWTimeout,
	eCWRxResponse,
	eCWWait4SW
}
 CWCommStates;

	
class MCNode {
	public:
		MCNode();
		void Connect2MsgHandler(MsgHandler *);
		void SetNodeId(uint8_t);
		void SetActTime(uint32_t);
		
		CWCommStates UpdateComStateBySDO();
		void ResetComState(); 
		void ResetSDOState();
		void SetTORetryMax(uint8_t);
		void SetBusyRetryMax(uint8_t);

		CWCommStates SendCw(uint16_t,uint32_t);
		CWCommStates PullSW(uint32_t);

		CWCommStates SendReset();
						
		SDOCommStates ReadSDO(unsigned int, unsigned char);
		SDOCommStates WriteSDO(unsigned int, unsigned char, uint32_t *,unsigned char);
		SDOCommStates GetSDOState();

		unsigned long GetObjValue();

		bool IsLive();
		uint16_t GetLastError();

		uint16_t StatusWord;
		uint16_t ControlWord;

		static void OnSysMsgRxCb(void *op,void *p) {
			((MCNode *)op)->OnRxHandler((MCMsg *)p);
		};
			
	private:
		void OnRxHandler(MCMsg *);

		CwSwMsg CwMsgBuffer;
		ResetReqMsg ResetReqBuffer;
		
		CWCommStates RxTxState = eCWIdle;
		CWCommStates CWAccessState = eCWIdle;
		CWCommStates SWAccessState = eCWIdle;
		
		SDOCommStates SDOAccessState = eSDOIdle;

		uint8_t AccessStep = 0;
		
		uint8_t Channel = InvalidSlot;
		int16_t NodeId = invalidNodeId;

		uint16_t EMCYCode;
		uint8_t DeviceName[MaxDeviceNameLen];
		uint8_t firstCWAccess = 1;
		
		bool hasMsgHandlerLocked = false;
		
		SDOHandler RWSDO;

		MsgHandler *Handler;
		MCNodeTxMsg TxMsg;

		uint32_t actTime;


		uint8_t TORetryCounter = 0;
		uint8_t TORetryMax = 1;

		uint8_t BusyRetryCounter = 0;
		uint8_t BusyRetryMax = 1;

		uint32_t CWSentAt;
		uint32_t SWRxAt;

		bool isLive = false;
};
 

#endif
