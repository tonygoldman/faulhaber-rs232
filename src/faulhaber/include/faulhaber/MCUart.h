#ifndef UART_H
#define UART_H

//---------------------------------------------------------------------
//  includes

#include "faulhaber/MC_Helpers.h"
#include <stdint.h>
#include <libserial/SerialStream.h>

//---------------------------------------------------------------------
//  local definitions

#define MaxMsgTime 3  //3 sm @ 115200 baud


const unsigned int UART_MAX_MSG_SIZE = 64;
const unsigned int UART_MIN_MSG_SIZE = 6;

typedef struct __attribute__((packed)) UART_MsgHdr {
   uint8_t u8Prefix  : 8;
   uint8_t u8Len     : 8;
   uint8_t u8NodeNr  : 8;
} UART_MsgHdr;

typedef union UART_Msg {
   uint8_t u8Data[UART_MAX_MSG_SIZE];
   UART_MsgHdr Hdr;
} UART_Msg;

//define the enum with the Comm states

typedef enum UartStates {
	eUartNotReady,
	eUartOperating,
	eUartTimeout
} UartStates;


//--- class definition ---

class MCUart
{
	public:
		MCUart();
		~MCUart();
		void Open(const char*, uint32_t);
		void ReOpen(uint32_t);
		void Update(uint32_t);
		void Register_OnRxCb(pfunction_holder *);
		short CheckStatus();
		short WriteMsg(UART_Msg *);
		void Stop();
		void Start(uint32_t baud = 115200);
		void ResetUart();
		
		static void OnTimeOutCb(void *p) {
			((MCUart *)p)->OnTimeOut();
		};
	
	private:
		uint8_t rxIdx = 0;
		uint8_t rxSize = 0;
		uint32_t BaudRate = 115200;
		//an Arduino wouldn't need a TX buffer - it's part of the Serial
		//on an bare bone embedded it would be needed though for
		//sending via TXE interrupt
		UART_Msg RxMsg;
		UART_Msg TxMsg;
		
		pfunction_holder OnRxCb;
	
		void OnTimeOut();
		int TimerHandle = -1;
		uint32_t To_Threshold;
		bool isTimerActive = false;
		UartStates state;

		std::shared_ptr<LibSerial::SerialStream> serial_stream_;
};

#endif
