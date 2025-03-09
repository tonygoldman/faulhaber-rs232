
//---------------------------------------------------------------------
//  includes

#include <MCUart.h>

//---------------------------------------------------------------------
//  local definitions

#define DEBUG_OPEN 		0x0001
#define DEBUG_RXCHAR 	0x0002
#define DEBUG_TXFRAME	0x0004
#define DEBUG_RXFRAME	0x0008
#define DEBUG_TO			0x0010
#define DEBUG_ERROR		0x0020
#define DEBUG_RXERROR   0x0040

#define FORCE_TxAtBuf0 1
 

const uint8_t MsgSuffix = 0x45;	// == E
const uint8_t MsgPrefix = 0x53;   // == S

const uint16_t MsgTimeout = MaxMsgTime; // timeout in ms

//#define DEBUG_UART (DEBUG_TO | DEBUG_ERROR | DEBUG_OPEN | DEBUG_RXERROR | DEBUG_TXFRAME | DEBUG_RXFRAME)
#define DEBUG_UART (DEBUG_TO | DEBUG_ERROR | DEBUG_OPEN | DEBUG_RXERROR)


//--- implementations ---

/*----------------------------------------------------------
 * MCUart(unsigned long Baudrate)
 * constructor of this class
 * reset all members and open the interface with the
 * requested speed
 *
 * 
 * 2020-05-10 AW Header
 * 2020-11-18    Done
 * 
 * --------------------------------------------------------*/

MCUart::MCUart()
{
	rxIdx = 0;
	rxSize = 0;
	state = eUartNotReady;
}

/*----------------------------------------------------------
 * Open(unsigned long)
 * explicitely open the interface
 * 
 * 2020-05-15 AW Frame
 * 2020-11-18    Done
 * 
 * ---------------------------------------------------------*/
 void MCUart::Open(uint32_t baud = 115200)
 {
	BaudRate = baud;
	rxIdx = 0;
	rxSize = 0;

	#if(DEBUG_UART & DEBUG_OPEN)
	Serial.print("UART: Open @ Speed: ");
	Serial.println(BaudRate, DEC);
	#endif

	Serial1.begin(BaudRate);

	//while(!Serial1)
	//	;
	//Serial 1 seems to need some additional time to be really ready
	delay(1000);
	for(uint16_t i = 0; i < 10;i++)
		Serial1.write((char)0);
	Serial1.flush();
	state = eUartOperating;
}

/*----------------------------------------------------------
 * Reopen(unsigned long)
 * change the BR by waitinf for any ongoing transmission, closing
 * and reopening the interface
 * 
 * 2020-05-15 AW Frame
 * 2020-11-18    Done
 * 
 * ---------------------------------------------------------*/
 void MCUart::ReOpen(uint32_t baud = 115200)
 {
	BaudRate = baud;
	rxIdx = 0;
	rxSize = 0;
	
	Serial1.flush();
	Serial1.end();
	state = eUartNotReady;

	Serial1.begin(BaudRate);
	state = eUartOperating;
}

/*----------------------------------------------------------
 * ResetUart()
 * reset the timeout and whatever has been receives so far
 * 
 * 2020-07-25 AW Frame
 * 2020-11-18    Done
 * 2024-05-04    remove reference to timer
 * 
 * ---------------------------------------------------------*/
void MCUart::ResetUart()
{
	rxIdx = 0;
	rxSize = 0;
	state = eUartOperating;
}

/*----------------------------------------------------------
 * Update()
 * is to be called in each cycle an will collect the Rx data
 * 
 * 2020-05-13 AW Frame
 * 2020-11-18    Done
 * 2024-05-04    remove reference to timer
 * 
 * ---------------------------------------------------------*/
 
 void MCUart::Update(uint32_t actTime)
 {
	bool store = true;

	if(state == eUartOperating)
	{
		if(Serial1.available())
		{
			#if(DEBUG_UART & DEBUG_RXCHAR)
		  Serial.print("UART chunk: >");
			#endif
			
			while(Serial1.available())
			{
				//read the first char
				uint8_t inChar = (uint8_t)Serial1.read();
				//now add it to the buffer if applicable
				if(rxIdx < UART_MAX_MSG_SIZE)
				{
					if(rxIdx == 0)
					{
						rxSize = UART_MIN_MSG_SIZE;
						if(inChar ==  MsgPrefix)
						{
							To_Threshold = actTime + MsgTimeout;
							isTimerActive = true;
						}
						else
							store = false;
					}
					else if (rxIdx == 1)
					{
						rxSize = inChar + 2;
					}	
					
					//now store or not store the char
					if(store)
					{
						#if(DEBUG_UART & DEBUG_RXCHAR)
						Serial.print(inChar, HEX);
						Serial.print(" ");
						#endif

						To_Threshold = actTime + MsgTimeout;
		
						RxMsg.u8Data[rxIdx++] = inChar;
						//check for finished
						if(rxIdx == rxSize)
						{
							//all characters received
							rxIdx = 0;
							
							isTimerActive = false;

							if(inChar == MsgSuffix)
							{
								 #if(DEBUG_UART & DEBUG_RXFRAME)
								 Serial.print("UART: rx: ");
								 for(uint8_t i = 0; i < rxSize;i++)
								 {
										Serial.print(RxMsg.u8Data[i],HEX);
										Serial.print(".");
								 }
								 Serial.println("#");
								 #endif
								 if(OnRxCb.callback != NULL)
										OnRxCb.callback(OnRxCb.op,(void *)&RxMsg);
							}
							#if(DEBUG_UART & DEBUG_RXFRAME)
							else
							{
							  Serial.print("UART: wrong Rx end!");
							}
							#endif
						}  
					}
					else
					{
						#if(DEBUG_UART & DEBUG_RXERROR)
						Serial.print("!");
						Serial.print(inChar, HEX);
						#endif
					}
				}
				else
				{
					//overflow
					rxIdx = 0;
					rxSize = 0;
				}
			}
    }
		
		if((isTimerActive) && (To_Threshold < actTime))	
		{	
			OnTimeOut();
			isTimerActive = false;
			//once again set a time-out which has to elaps before new
			//messages are to be handeled
			To_Threshold = actTime + MsgTimeout;
			state = eUartTimeout;
		}
	}
	else
	{
		//we are in TO state
		if(To_Threshold < actTime)
		{
			#if(DEBUG_UART & DEBUG_TO)
			Serial.print("UART: revocered from TO!");
			#endif
			//elapsed
			state = eUartOperating;
		}
	}
 }

/*----------------------------------------------------------
 * Register_onRxCb(function_holder *cb)
 * store the function and object pointer for the callback
 * called in case of a successful Rx
 * 
 * 2020-05-10 AW Header
 * 2020-11-18    Done
 * 
 * --------------------------------------------------------*/

void MCUart::Register_OnRxCb(pfunction_holder *Cb)
{
	OnRxCb.callback = Cb->callback;
	OnRxCb.op = Cb->op;
}

/*----------------------------------------------------------
 * CheckStatus()
 * Check whether the Tx is idle
 * returns true if idle, false if not
 * 
 * 2020-05-10 AW Header
 * 2020-11-18    Done
 * 
 * --------------------------------------------------------*/

short MCUart::CheckStatus()
{
	return Serial1.availableForWrite();	
}

/*----------------------------------------------------------
 * WriteMsg(UART_Msg *)
 * hand over a message to be sent. Will be copied into a
 * transfer buffer so the call will likely returnb efore the
 * message was fully sent
 * 
 * 2020-05-10 AW Header
 * 2020-11-18    Done
 * 
 * --------------------------------------------------------*/

short MCUart::WriteMsg(UART_Msg *Msg)
{
bool status = false;

	uint16_t len = (uint16_t)Msg->Hdr.u8Len + 2;
	uint16_t size = Serial1.availableForWrite();
	
	#if(DEBUG_UART & DEBUG_TXFRAME)
	Serial.print("UART: ");
	Serial.print(size);
	Serial.println(" buffer");
	#endif
	
	//on an R4 Wifi the Serial1.availableForWrite() is reportet to 0 but it does transmit
	#if FORCE_TxAtBuf0
	if(size == 0)
    size = len;
  #endif

	if((len <= size) && (state == eUartOperating))
	{		 
		status = true;
		
		// copy to transient buffer
		for(uint8_t i=0;i<len;i++)
		{
			((uint8_t *)&TxMsg)[i] = ((uint8_t *)Msg)[i];	
		}

		//add prefix and postfix to the frame
		TxMsg.u8Data[0] = MsgPrefix;
		TxMsg.u8Data[len - 1] = MsgSuffix;
		
		#if(DEBUG_UART & DEBUG_TXFRAME)
		Serial.print("UART Tx: len: ");
		Serial.print(len, DEC);
		Serial.print(">>");

		for(uint8_t i=0;i<len;i++)
		{
			Serial.print((char)(TxMsg.u8Data[i]), HEX);
			Serial.print(".");
		}
		Serial.println("#");
		#endif
		
		Serial1.write((char *)(TxMsg.u8Data),TxMsg.Hdr.u8Len +2);
	}
	#if(DEBUG_UART & DEBUG_TXFRAME)
	else
	{
		Serial.println("UART: busy ");
		Serial.print("TxLen: ");
		Serial.print(len, DEC);
		Serial.print(" << ");

		for(uint8_t i=0;i<len;i++)
		{
			Serial.print((char)(Msg->u8Data[i]), HEX);
			Serial.print(".");
		}
		Serial.println("!!");
	}
	#endif
	return status;
}

/*----------------------------------------------------------
 * OnTimeOut()
 * handler to be registered at the timer and to be started
 * when a new Rx is started
 * Will reset the frame after the time-out
 * 
 * 2020-05-10 AW Header
 * 2020-11-18    Done
 * 
 * --------------------------------------------------------*/

void MCUart::OnTimeOut()
{
	TimerHandle = -1;
	rxIdx = 0;
	rxSize = 0;
	#if(DEBUG_UART & DEBUG_TO)
	Serial.println("UART TO");
	#endif
}
