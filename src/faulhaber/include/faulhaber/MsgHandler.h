#ifndef MSG_HANDLER_H
#define MSG_HANDLER_H

/*--------------------------------------------------------------------
 * interface for class MsgHandler
 * is used to send requests via Uart and to receive and validate
 * Msg from Uart. Depending on their serive they will be distributed
 *
 * 2020-05-14 AW Frame
 * 2020-11-18    Done
 *
 *-------------------------------------------------------------------*/
 
//--- includes ---

#include "faulhaber/MCUart.h"
#include <stdint.h>

typedef enum MCMsgCommands {
	eBootMsg = 0,
	eSdoReadReq = 1,
	eSdoWriteReq = 2,
	eSdoError = 3,
	eCtrlWord = 4,
	eStatusWord = 5,
	eLoggingReq = 6,
	eEmergencyMsg = 7,
	eInvalidCmdCode = 0xFF
} MCMsgCommands;

typedef struct __attribute__((packed)) MCMsgHdr {
   uint8_t u8Prefix  : 8;
   uint8_t u8Len     : 8;
   uint8_t u8NodeNr  : 8;
   MCMsgCommands u8Cmd : 8;
   uint8_t u8UserDataStart[3];
} MCMsgHdr;

typedef union MCMsg {
   MCMsgHdr Hdr;
   UART_Msg Raw;
} MCMsg;

const uint8_t MsgHandler_MaxNodes = 4;
const int16_t invalidNodeId = -1;
const uint8_t InvalidSlot = 0xff;


class MsgHandler {
	public:
		MsgHandler();
		void Open(const char*, uint32_t);
		void Update(uint32_t);
		uint8_t RegisterNode(uint8_t);
		void UnRegisterNode(uint8_t);
		int8_t GetNodeId(uint8_t);
		bool SendMsg(uint8_t, MCMsg *);
		void Register_OnRxSDOCb(uint8_t,pfunction_holder *);
		void Register_OnRxSysCb(uint8_t,pfunction_holder *);
		void ResetMsgHandler();
		
		bool LockHandler();
		void UnLockHandler();
				
		static void OnMsgRxCb(void *op,void *p) {
			((MsgHandler *)op)->OnRxHandler((MCMsg *)p);
		};

	private:
		void OnRxHandler(MCMsg *);
		uint8_t FindNode(uint8_t);
		bool IsCrcOk(const UART_Msg *);
		uint8_t CalcCRC(const uint8_t *,int);
		bool isLocked = false;
		
		MCUart Uart;
		//a buffer to be used, if the interface is blocked
		MCMsg TxMsg[MsgHandler_MaxNodes];
		bool TxMsgPending[MsgHandler_MaxNodes];
		int16_t nodeId[MsgHandler_MaxNodes];
		pfunction_holder OnRxSDOCb[MsgHandler_MaxNodes];
		pfunction_holder OnRxSysCb[MsgHandler_MaxNodes];	
		
		uint32_t actTime;
		uint32_t lockTime;
};


#endif
