/*-------------------------------------------------------------------
 * MsgHandler.cpp
 * Implementation of the MsgHandler Class
 * used for validation and distribution of messages
 *
 * 2020-05-14 AW Frame
 * 2020-11-18    Done
 *
 *-------------------------------------------------------------------*/
 
//--- includes ---
 
#include <MsgHandler.h>
#include <MC_Helpers.h>
#include <stdint.h>

#define DEBUG_ONRX		0x0001
#define DEBUG_REGNODE	0x0002
#define DEBUG_TXMSG		0x0004
#define DEBUG_ULCK    0x0008

//--- definitions ---

#define DEBUG_MSGHandler (DEBUG_ULCK)

const uint16_t MsgHandlerMaxLeaseTime = (2 * MaxMsgTime + 2);


//--- implementation ---

/*------------------------------------------------------
 * MsgHandler()
 * constuctor. Register the callback at my instance of
 * the Uart
 * 
 * 2020-05-15 AW Rev A
 * 
 * ----------------------------------------------------*/
 
MsgHandler::MsgHandler()
{
	pfunction_holder Cb;
	//register Cb
	Cb.callback = (pfunction_pointer_t)MsgHandler::OnMsgRxCb;
	Cb.op = (void *)this;
	Uart.Register_OnRxCb(&Cb);
	//now set default values for no node regsitered
	for(int16_t i = 0; i < MsgHandler_MaxNodes;i++)
	{
		nodeId[i] = invalidNodeId;
		TxMsgPending[i] = false;
	}
	isLocked = false;
}

/*------------------------------------------------------
 * Open(Msg)
 * Open the serial interface at the set rate
 * 
 * 2020-05-15 AW Rev A
 * 
 * ----------------------------------------------------*/
 
void MsgHandler::Open(uint32_t baudrate)
{
	//open the interface and set speed of UART
	Uart.Open(baudrate);	
}

/*------------------------------------------------------
 * Update()
 * needed to call the Update of the underlying Uart as there
 * is no real interrupt driven Rx or Tx here
 * If the Msghandler has been locked for a too long time
 * it will e unlocked here to give the system a chance to recover
 * 
 * 2020-05-15 AW Rev A
 * 
 * ----------------------------------------------------*/
 
void MsgHandler::Update(uint32_t timeNow)
{
	actTime = timeNow;
	Uart.Update(actTime);
	
	if(isLocked && (actTime - lockTime > MsgHandlerMaxLeaseTime))
	{
		UnLockHandler();
		#if(DEBUG_MSGHandler & DEBUG_ULCK)
		Serial.println("Msg: unlocked");
		#endif
	}
}

/*------------------------------------------------------
 * ResetMsgHandler()
 * to be called, when to upper layers run into a TO
 * does not delete pending messages
 * MsgHandler itself doesn't really have states do be reset
 * so it's a call to reset the Uart only
 * 
 * 2020-07-25 AW Rev A
 * 
 * ----------------------------------------------------*/
void MsgHandler::ResetMsgHandler()
{
	Uart.ResetUart();
}


/*------------------------------------------------------
 * LockHandler()
 * try to set the lock flag
 * no direct consequence here, caller would have to deal with the
 * result
 * 
 * 2020-11-07 AW Rev A
 * 
 * ----------------------------------------------------*/
bool MsgHandler::LockHandler()
{
	if(isLocked)
		//is allready locked
		return false;
	else
	{
		isLocked = true;
		lockTime = actTime;
	}	
	return true;

}


/*------------------------------------------------------
 * UnLockHandler()
 * unlock the handler - to allow for others to access it
 * 
 * 
 * 2020-11-07 AW Rev A
 * 
 * ----------------------------------------------------*/
void MsgHandler::UnLockHandler()
{
	isLocked = false;
}


/*------------------------------------------------------
 * OnRxHandler(Msg)
 * react to a received Msg
 * first check the CRC. If valid use a switch case
 * to call the registered handler
 * 
 * 2020-05-15 AW Rev A
 * 
 * ----------------------------------------------------*/
 
void MsgHandler::OnRxHandler(MCMsg *RxMsg)
{
uint8_t NodeHandle = FindNode(RxMsg->Hdr.u8NodeNr);

	if((NodeHandle != InvalidSlot) && IsCrcOk((UART_Msg *)RxMsg))
	{
		MCMsgCommands cmd = RxMsg->Hdr.u8Cmd;
		
		switch(cmd)
		{
			case eBootMsg:
			case eCtrlWord:
			case eStatusWord:
			case eEmergencyMsg:
				#if(DEBUG_MSGHandler & DEBUG_ONRX)
				Serial.print("MSG: Rx Sys CMD: ");
				Serial.println(cmd, HEX);
				#endif
				if(OnRxSysCb[NodeHandle].callback != NULL)
					OnRxSysCb[NodeHandle].callback(OnRxSysCb[NodeHandle].op,(void *)RxMsg);
				break;
			case eSdoReadReq:
			case eSdoWriteReq:
			case eSdoError:
				#if(DEBUG_MSGHandler & DEBUG_ONRX)
				Serial.println("MSG: Rx SDO Response");
				#endif
				if(OnRxSDOCb[NodeHandle].callback != NULL)
					OnRxSDOCb[NodeHandle].callback(OnRxSDOCb[NodeHandle].op,(void *)RxMsg);
				break;
			default:
				break;
		}
	}
	//now after having received a message try to resend any pening Tx message
	for(int i = 0; i < MsgHandler_MaxNodes;i++)
	{
		if(TxMsgPending[i])
		{
			//try to send
			//clear the flag if successfull
			if(SendMsg(i,&(TxMsg[i])))
				TxMsgPending[i] = false;
			break;
		}
	}

}

/*----------------------------------------------------------
 * char FindNode(char)
 * find the NodeHandle in the vector of adresses
 * 
 * 2020-05-16 AW Header
 * 
 * --------------------------------------------------------*/
uint8_t MsgHandler::FindNode(uint8_t NodeId)
{
	uint8_t i = 0;
	uint8_t slot = InvalidSlot;
	while(i < MsgHandler_MaxNodes)
	{
		if((uint8_t)nodeId[i] == NodeId)
			slot = i;
		i++;
	}
	return slot;	
}

/*----------------------------------------------------------
 * uint8_t RegisterNode(char)
 * try to register a node with it's node ID.
 * the handerl shall be used as a regference for furhter calls
 * 
 * 2020-05-16 AW Header
 * 
 * --------------------------------------------------------*/

uint8_t MsgHandler::RegisterNode(uint8_t thisNodeId)
{
	uint8_t i = 0;
	uint8_t slot = InvalidSlot;
	while(i < MsgHandler_MaxNodes)
	{
		if(nodeId[i] == invalidNodeId)
		{
			slot = i;
			break;
		}
		i++;
	}
	if(slot != InvalidSlot)
		nodeId[slot]=(int16_t)thisNodeId;
	
	#if(DEBUG_MSGHandler & DEBUG_REGNODE)
	Serial.print("Msg: Reg ");
	Serial.print(thisNodeId, DEC);
	Serial.print(" at ");
	Serial.println(slot);
	#endif
	
	return slot;
}

/*----------------------------------------------------------
 *  int8_t MsgHandler::GetNodeId(uint8_t)
 *  read the nodeId of a regsitered node back
 * 
 * 2020-11-01 AW
 * ----------------------------------------------------------*/ 

int8_t MsgHandler::GetNodeId(uint8_t NodeHandle)
{
int8_t thisNodeId = -1;

	if(NodeHandle < MsgHandler_MaxNodes)
	{
		thisNodeId = (uint8_t) nodeId[NodeHandle];
	}
	return thisNodeId;
}
		

/*----------------------------------------------------------
 * void UnRegisterNode(char)
 * remove the entry for a given node
 * 
 * 2020-05-16 AW Header
 * 
 * --------------------------------------------------------*/

void MsgHandler::UnRegisterNode(uint8_t NodeHandle)
{
	if(NodeHandle < MsgHandler_MaxNodes)
	{
		nodeId[NodeHandle] = invalidNodeId;
		OnRxSDOCb[NodeHandle].callback = NULL;
		OnRxSDOCb[NodeHandle].op = NULL;
		OnRxSysCb[NodeHandle].callback = NULL;
		OnRxSysCb[NodeHandle].op = NULL;
		
	}
}
		
/*----------------------------------------------------------
 * SendMsg(char NodeHandle, MCMsg *TxMsg)
 * if possible send the Msg directly
 * this can be done without copying the Msg into a buffer,
 * as MCUart doest copy
 * 
 * 2020-05-10 AW Header
 * 
 * --------------------------------------------------------*/

bool MsgHandler::SendMsg(uint8_t NodeHandle, MCMsg *NewTxMsg)
{
	bool returnValue = false;

	UART_Msg *ThisMsg = (UART_Msg *)NewTxMsg;
	
	#if(DEBUG_MSGHandler & DEBUG_TXMSG)
	Serial.print("Msg: Msg 4 Node ");
	Serial.print(nodeId[NodeHandle], DEC);
	#endif
	
	if(NodeHandle < MsgHandler_MaxNodes)
	{
		returnValue = true;
		//add Node-Id
		ThisMsg->Hdr.u8NodeNr = (uint8_t) nodeId[NodeHandle];
		//and CRC
		ThisMsg->u8Data[ThisMsg->Hdr.u8Len] = CalcCRC((const uint8_t *)&(ThisMsg->u8Data[1]), ThisMsg->Hdr.u8Len-1);

		//write or store
		if(!(Uart.WriteMsg(ThisMsg)))
		{
			// try to store
			if(TxMsgPending[NodeHandle])
			{
				//there is already a stored message for this one
				returnValue = false;

				#if(DEBUG_MSGHandler & DEBUG_TXMSG)
				Serial.println("Msg:  full!");
				#endif
			}
			else
			{
				//store this message
				//will result in a return of true so the calling
				//instance will consider the TX to be successfull
				TxMsgPending[NodeHandle] = true;
				for(uint8_t i=0;i<=ThisMsg->Hdr.u8Len;i++)
					TxMsg[NodeHandle].Raw.u8Data[i] = ThisMsg->u8Data[i];	
				
				#if(DEBUG_MSGHandler & DEBUG_TXMSG)
				Serial.println("Msg: stored");
				#endif
			}
		}
		else
		{
			//return of the Uart was true - so successfull TX
			#if(DEBUG_MSGHandler & DEBUG_TXMSG)
			Serial.println("Msg: sent");
			#else
			;
			#endif
		}
	}
	return returnValue;
}

/*----------------------------------------------------------
 * Register_onRxCb(function_holder *cb)
 * store the function and object pointer for the callback
 * called in case of a successful Rx
 * 
 * 2020-05-10 AW Header
 * 
 * --------------------------------------------------------*/

void MsgHandler::Register_OnRxSDOCb(uint8_t NodeHandle, pfunction_holder *Cb)
{
	if(NodeHandle < MsgHandler_MaxNodes)
	{
		OnRxSDOCb[NodeHandle].callback = Cb->callback;
		OnRxSDOCb[NodeHandle].op = Cb->op;
	}
}

/*----------------------------------------------------------
 * Register_onRxCb(function_holder *cb)
 * store the function and object pointer for the callback
 * called in case of a successful Rx
 * 
 * 2020-05-10 AW Header
 * 
 * --------------------------------------------------------*/

void MsgHandler::Register_OnRxSysCb(uint8_t NodeHandle, pfunction_holder *Cb)
{
	if(NodeHandle < MsgHandler_MaxNodes)
	{
		OnRxSysCb[NodeHandle].callback = Cb->callback;
		OnRxSysCb[NodeHandle].op = Cb->op;
	}
}

/*----------------------------------------------------------
 * bool IsCrcOk(UART_Msg *)
 * check the CRC of a Msg against the CRC which can be calculated
 * based on the message
 * 
 * 2020-05-10 AW Header
 * 
 * --------------------------------------------------------*/

bool MsgHandler::IsCrcOk(const UART_Msg *Msg)
{
	return (Msg->u8Data[Msg->Hdr.u8Len] == CalcCRC(&(Msg->u8Data[1]), Msg->Hdr.u8Len-1)); 	
}
/*----------------------------------------------------------
 * char CalcCRC(char *buffer,int len)
 * calculate the CRC of a given buffer
 * 
 * 2020-05-10 AW Header
 * 
 * --------------------------------------------------------*/

uint8_t MsgHandler::CalcCRC(const uint8_t *buffer,int len)
{
  uint8_t calcCRC = 0xFF;
  for(uint8_t i = 0;i<len;i++)
  {
    calcCRC = calcCRC ^((uint8_t *)buffer)[i];
    for(uint8_t j = 0;j < 8;j++)
    {
      if(calcCRC & 0x01)
        calcCRC = (calcCRC >> 1) ^ 0xd5;
      else
        calcCRC = (calcCRC >> 1);
    }
  }
return calcCRC;
}
