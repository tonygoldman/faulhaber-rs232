/*---------------------------------------------------
 * MCNode.cpp
 * implements the class to handel NMT and access
 * to the CW/SW
 * MCNode does not implement the CiA 402 state machine
 *
 * 2020-05-24 AW Frame
 * 2024-05-07 AW remove ref to timer
 *
 *--------------------------------------------------------------*/
 
//--- includes ---

#include <cstdio>
#include "faulhaber/MCNode.h"


//--- local defines ---

#define DEBUG_RXMSG		0x0001
#define DEBUG_TO			0x0002
#define DEBUG_ERROR		0x0004
#define	DEBUG_TXCW		0x0008
#define DEBUG_RXSW		0x0010
#define DEBUG_RXEMCY	0x0020
#define DEBUG_RESET		0x0040

//#define DEBUG_NODE (DEBUG_TO | DEBUG_ERROR | DEBUG_RXSW | DEBUG_TXCW) 
#define DEBUG_NODE (DEBUG_TO | DEBUG_ERROR) 

typedef struct EMCYMsg {
   uint8_t u8Prefix  : 8;
   uint8_t u8Len     : 8;
   uint8_t u8NodeNr  : 8;
   MCMsgCommands u8Cmd : 8;
   uint16_t ErrorCode;
   uint8_t ErrorRegister;
   uint16_t FaulhaberErrorReg;
   uint8_t Reserved[3];
   uint8_t CRC;
   uint8_t u8Suffix;
} EMCYMsg;

typedef struct CwMsgResponse {
   uint8_t u8Prefix  : 8;
   uint8_t u8Len     : 8;
   uint8_t u8NodeNr  : 8;
   MCMsgCommands u8Cmd : 8;
   uint8_t Error;
   uint8_t CRC;
   uint8_t u8Suffix;
} CwMsgResponse;

const uint16_t CwRespTimeOut = 5;  //20;
const uint16_t MaxSWResponseDelay = 20; //50;

//--- public functions ---

/*---------------------------------------------------------------------
 * MCNode::MCNode()
 * as of now there is noting to intialized when created
 * 
 * 2020-11-21 AW Done
 * ------------------------------------------------------------------*/

MCNode::MCNode()
{
	;
}

/*-------------------------------------------------------------------
 * void SetNodeId(uint8_t ThisNodeId)
 * Set the NodeId for this instance. Needs to be called before the
 * Msghandler can be registered
 * 
 * 2020-11-21 AW Done
 * -----------------------------------------------------------------*/

void MCNode::SetNodeId(uint8_t ThisNodeId)
{
	NodeId = (int16_t)ThisNodeId;
}

/*--------------------------------------------------------------------
 * void Connect2MsgHandler(MsgHandler *ThisHandler)
 * In each system there is a single Msghandler but there can be
 * multiple drives. So the different instances of the MCNode
 * need to be connected to the instance of the Msghandler by calling
 * this method.
 * Directly calls the Handler->RegisterNode method and when successful
 * registers its callback for System messages plus calls the init()
 * method of its embedded SDOHandler instance to also connect it to the
 * Msghandler.
 * RxTxState is initialized too
 * 
 * 2020-11-21 AW Done
 * ------------------------------------------------------------------*/

void MCNode::Connect2MsgHandler(MsgHandler *ThisHandler)
{
	Handler = ThisHandler;
	//register the nodeId
	if((Channel = Handler->RegisterNode((uint8_t)NodeId)) != InvalidSlot)
	{
		pfunction_holder Cb;
		//then regsiter the SysRxhandler
		Cb.callback = (pfunction_pointer_t)MCNode::OnSysMsgRxCb;
		Cb.op = (void *)this;
		Handler->Register_OnRxSysCb(Channel,&Cb);

		//then register this objects SDOHandler as the SDO handler of this node
		RWSDO.init(Handler, Channel);
		RxTxState = eCWIdle;
	}
}

/*-------------------------------------------------------------------
 * void SetActTime(uint32_t time)
 * If no HW-timer is used this method needs to called cyclically
 * with the latest millis() value to check for any time-outs.
 * Does the same update for the embedded SDOhandler.
 * 
 * 2020-11-21 AW Done
 * -----------------------------------------------------------------*/

void MCNode::SetActTime(uint32_t time)
{
	actTime = time;
	RWSDO.SetActTime(time);
}

/*-------------------------------------------------------------------
 * CWCommStates UpdateComStateBySDO()
 * Check the SDO ComState which will also update the ComState of then
 * Node if requried and return the ComState
 * 
 * 2020-11-21 AW Done
 * -----------------------------------------------------------------*/

CWCommStates MCNode::UpdateComStateBySDO()
{
	//check SDO status will force the RxTxState into eCWerror or eCWTimeout
	//if SDO failed. Otherwise we will simply read the actual RxTxState

	//check the SDOState
	if(RWSDO.GetComState() == eSDOError)
		RxTxState = eCWError;
	if(RWSDO.GetComState() == eSDOTimeout)
		RxTxState = eCWTimeout;

	return RxTxState;
}

/*------------------------------------------------------------------
 * void ResetSDOState()
 * Reset the ComState of the embedded SDOHandler and reset its reflected
 * local state too
 * 
 * 2020-11-21 AW Done
 * ----------------------------------------------------------------*/
 
void MCNode::ResetSDOState()
{
	RWSDO.ResetComState();
	SDOAccessState = eSDOIdle;	
}

/*--------------------------------------------------------------------
 * void ResetComState()
 * Reset the ComState of the MCNode
 * Is the only way to recover from an eCWError state.
 * Will reset all Com related states and will unlock an still locked
 * MsgHandler.
 * Does the reset of the embedded SDOHandler too.
 * 
 * 2020-11-21 AW Done
 * ------------------------------------------------------------------*/


void MCNode::ResetComState()
{
	RxTxState = eCWIdle;
	CWAccessState = eCWIdle;
	SWAccessState = eCWIdle;
	
	TORetryCounter = 0;
	BusyRetryCounter = 0;
	AccessStep = 0;
	ResetSDOState();
	
	if(hasMsgHandlerLocked)
	{
		Handler->UnLockHandler();
		hasMsgHandlerLocked = false;
	}

	
}

/*------------------------------------------------------------------
 * void SetTORetryMax(uint8_t value)
 * Configre a different number of TO from which the Node could  try to 
 * recover autoamtically until the eCWError state is reached.
 * Default is within the class definition.
 * 
 * 2020-11-21 AW Done
 * ----------------------------------------------------------------*/

void MCNode::SetTORetryMax(uint8_t value)
{
	TORetryMax = value;
}

/*------------------------------------------------------------------
 * void SetBusyRetryMax(uint8_t value)
 * When the Msghandler or embedded SDO handler is busy an attemted 
 * access ends up busy and will retry automatically. Define a different
 * number of retrys until the access will fail.
 * Default is within the class definition.
 * 
 * 2020-11-21 AW Done
 * ----------------------------------------------------------------*/

void MCNode::SetBusyRetryMax(uint8_t value)
{
	BusyRetryMax = value;
}

/*------------------------------------------------------------------
 * CWCommStates MCNode::SendCw(uint16_t Data, uint32_t maxSWDelay = MaxSWResponseDelay)
 * Send the ControlWord which is given as a parameter. 
 * The actual send request is executed only, if the requested CW is different
 * then the last which has been stored in the class or if it
 * is the first attempt to send a CW.
 * Any CW send service has to answered by the drive, which would be
 * received by the OnRxHandler.
 * After a request was accepted by the Msghandler the drive ends up in eCWWaiting.
 * If no response is received after CwRespTimeOut/2 the request will be re-sent.
 * When the response was received and an STatusWord response is expected
 * the Node will end up in eCWDone and will pull a StatusWord via SDO serive
 * periodically with the period time <> 0 give as maxSWDelay.
 * It's the responsibility of the user to check the received Statusword and
 * stopp calling SendCW if the expected result has been receeived and the
 * method ended up in eCWDone.
 * Needs a call to ResetComState to switch back to eCWIdle.
 * 
 * As any access to the MsgHandler SendCw will lock the Msghandler and
 * will only unlock it the actual service failed.
 * Successful servie will unlock in OnRxHandler().
 * 
 * 2020-11-21 AW Done
 * ----------------------------------------------------------------*/

CWCommStates MCNode::SendCw(uint16_t Data, uint32_t maxSWDelay = MaxSWResponseDelay)
{
	bool doSend = (Data != ControlWord) || firstCWAccess;
			
	if((CWAccessState == eCWRetry) && ((CWSentAt + CwRespTimeOut) < actTime) )
		doSend = true;
	
	//necessary to retrigger the access to the CW in a chain
	if(doSend)
	{
		if(CWAccessState == eCWDone)
			CWAccessState = eCWIdle;
	}
	else
	{
		//no need to send or resend
		if(CWAccessState == eCWIdle)
			CWAccessState = eCWRxResponse;
	}
		
	#if(DEBUG_NODE & DEBUG_TXCW)
	static CWCommStates LastCWState;
	static SDOCommStates LastSDOState;

	if(CWAccessState != LastCWState)
	{
		LastCWState = CWAccessState;
		std::printf("Node: Send CW: New CWState ");
		std::printf("%d\n", CWAccessState);
	}
	if(SDOAccessState != LastSDOState)
	{
		LastSDOState = SDOAccessState;
		std::printf("Node: Send CW: New SDOState ");
		std::printf("%d\n", SDOAccessState);
	}
	#endif

	//we do send only, when available
	switch(CWAccessState)
	{
		case eCWWaiting:
			if((CWSentAt + CwRespTimeOut/2) < actTime)
			{
				CWAccessState = eCWRetry;
				doSend = true;
			}
			#if(DEBUG_NODE & DEBUG_TXCW)
			std::printf("W");
			#endif
			
			//no break here
		case eCWRetry:		
		case eCWIdle:
			if((hasMsgHandlerLocked = Handler->LockHandler()))
			{				 
				if (doSend)
				{
					CwMsgBuffer.u8Len = 6;
					CwMsgBuffer.u8NodeNr = (uint8_t)NodeId;
					CwMsgBuffer.u8Cmd = eCtrlWord;
					CwMsgBuffer.Payload = Data;

					#if(DEBUG_NODE & DEBUG_TXCW)
					std::printf("Node: Send CW: ");
					std::printf("%X", Data);
					std::printf(" --> ");
					#endif

					if(Handler->SendMsg(Channel,(MCMsg *)&CwMsgBuffer))
					{
						CWAccessState = eCWWaiting;
						ControlWord = Data;
						BusyRetryCounter = 0;
						firstCWAccess = false;
						
						//register a timeout handler
						#if(DEBUG_NODE & DEBUG_TXCW)
						std::printf("Node: CW Waiting\n");
						#endif

						//the timer will generate a TO condition, when the response in not received in time
						CWSentAt = actTime;

					}
					else
					{
						
						Handler->UnLockHandler();
						hasMsgHandlerLocked = false;
						
						BusyRetryCounter++;
						if(BusyRetryCounter > BusyRetryMax)
						{
							CWAccessState = eCWError;
							
							#if(DEBUG_NODE & DEBUG_ERROR)
							std::printf("Node: CW Busy!!\n");
							#endif
						}
						else
						{
							CWAccessState = eCWRetry;

							#if(DEBUG_NODE & DEBUG_TXCW)
							std::printf("Node: CW Busy --> retry\n");
							#endif
						}
					}  // end of sent successfully or not
				} // end of do only if new data
			}
			break;
		case eCWRxResponse:
			//waiting is handled in eCWDone
			CWAccessState = eCWDone;
			Handler->UnLockHandler();
			hasMsgHandlerLocked = false;

			//define time now as the start of the waiting time for SW
			SWRxAt = actTime;
			break;
		case eCWDone:
			//if we stay in this state pull the SW from time to time
			//only if a non zero waiting time is set
			if(maxSWDelay > 0)
			{
				if(SWRxAt < (actTime - maxSWDelay))
				{
					//we might be waiting for a response but the original access
					//to the SW is finished
					//trigger a pull on the SW
					CWAccessState = eCWWait4SW;		
				}
			}
			break;
		case eCWWait4SW:
			if(SDOAccessState == eSDODone)
			{
				StatusWord = (uint16_t)RWSDO.GetObjValue();
				SWRxAt = actTime;
				//this access should not be necessary
				SDOAccessState = RWSDO.GetComState();
				CWAccessState = eCWDone;
								
				#if(DEBUG_NODE & DEBUG_TXCW)
				std::printf("Node: Send CW: SW pulled ");
				std::printf("%X\n", StatusWord);
				#endif
			}
			else			
			{
				#if(DEBUG_NODE & DEBUG_TXCW)
				if(SDOAccessState == eSDOIdle)
					std::printf("Node: Send CW: SW Request \n");
				#endif

				SDOAccessState = RWSDO.ReadSDO(0x6041, 0x00);
			}
			break;	
	}  // end of switch
	
	//map the CWAccessState to the universal RxTxState
	RxTxState = CWAccessState;
	
	//check the SDOState too
  UpdateComStateBySDO();

	return RxTxState;
}

/* --------------------------------------------------------------
 * PullSW()
 * Is intended to force a cyclic update of the StatusWord in methods
 * relying on the contents of the SW
 * The periodic time is the singel parameter here.
 * Uses the SDO service to read the StatusWord.
 * Any user has to poll the contents of the local copy of the 
 * StatusWord and if as expected stop calling this method as soon
 * as it ended up in eCWDone.
 * Needs a call to ResetComState to switch back to eCWIdle.
 * 
 * 2020-07-17 AW
 * ------------------------------------------------------------*/
 
CWCommStates MCNode::PullSW(uint32_t maxSWDelay)
{	
	switch(SWAccessState)
	{
		case eCWIdle:
			//define time now as the start of the waiting time for SW
			SWRxAt = actTime;
			SWAccessState = eCWWait4SW;
			break;
		case eCWDone:
			//if we stay in this state pull the SW from time to time
			//only if a non zero waiting time is set
			if((SWRxAt < (actTime - maxSWDelay)) && (maxSWDelay > 0))
			{
				//we might be waiting for a response but the original access
				//to the SW is finished
				//trigger a pull on the SW
				#if(DEBUG_NODE & DEBUG_RXSW)
				std::printf("Node: Pull SW: Time over: pull again\n");
				#endif
				SWAccessState = eCWWait4SW;		
			}
			break;
		case eCWWait4SW:
			if(SDOAccessState == eSDODone)
			{
				//could have a debug option to only print changed responses
				StatusWord = (uint16_t)RWSDO.GetObjValue();
				SWRxAt = actTime;
				//this access should not be necessary
				SDOAccessState = RWSDO.GetComState();
				SWAccessState = eCWDone;
								
				#if(DEBUG_NODE & DEBUG_RXSW)
				std::printf("Node: Pull SW: SW pulled ");
				std::printf("%X\n", StatusWord);
				#endif
			}
			else			
			{
				#if(DEBUG_NODE & DEBUG_RXSW)
				if(SDOAccessState == eSDOIdle)
					std::printf("Node: Pull SW: Send SW Request \n");
				#endif

				SDOAccessState = RWSDO.ReadSDO(0x6041, 0x00);
			}
			break;	
	}

	//map the CWAccessState to the universal RxTxState
	RxTxState = SWAccessState;
	
	//check the SDOState too
  UpdateComStateBySDO();

	return RxTxState;
}

/*------------------------------------------------------------------
 * CWCommStates SendReset()
 * Send a ResetNode message to the drive.
 * 
 * 2020-11-21 AW untested
 * ----------------------------------------------------------------*/

CWCommStates MCNode::SendReset()
{
	//we do send only, when available
	//CWAccessState is used here too
	switch(CWAccessState)
	{
		case eCWIdle:
		case eCWRetry:
			//must not send if Msghandler not available
			if((hasMsgHandlerLocked = Handler->LockHandler()))
			{				 
				ResetReqBuffer.u8Len = 6;
				ResetReqBuffer.u8NodeNr = (uint8_t)NodeId;
				ResetReqBuffer.u8Cmd = eBootMsg;

				if(Handler->SendMsg(Channel,(MCMsg *)&ResetReqBuffer))
				{
					CWAccessState = eCWDone;
					//directly unlock the Msghandler - no response expected
					Handler->UnLockHandler();
					isLive = false;

					BusyRetryCounter = 0;
					
					#if(DEBUG_NODE & DEBUG_RESET)
					std::printf("Node: RESET sent\n");
					#endif
					
				}
				else
				{					
					Handler->UnLockHandler();
					BusyRetryCounter++;
					if(BusyRetryCounter > BusyRetryMax)
					{
						CWAccessState = eCWError;
						
						#if(DEBUG_NODE & DEBUG_ERROR)
						std::printf("Node: RESET Busy!!\n");
						#endif
					}
					else
					{
						CWAccessState = eCWRetry;

						#if(DEBUG_NODE & DEBUG_RESET)
						std::printf("Node: RESET Busy --> retry\n");
						#endif
					}
					
				}
			}
		break;
	}
	return CWAccessState;
}

/*------------------------------------------------------------------
 * bool IsLive()
 * Check whether a boot Msg of the drive has been received
 * Please note: in net-mode of multiple drives no boot messages
 * will be sent at all.
 * 
 * 2020-11-21 AW Done
 * ----------------------------------------------------------------*/

bool MCNode::IsLive()
{
	return isLive;
}
		
/*------------------------------------------------------------------
 * uint16_t GetLastError()
 * Read the last EMCY code. EMCY service is not available in net mode.
 * 
 * 2020-11-21 AW Done
 * ----------------------------------------------------------------*/

uint16_t MCNode::GetLastError()
{
	return EMCYCode;
}

/*------------------------------------------------------------------
 * SDOCommStates ReadSDO(unsigned int Idx, unsigned char SubIdx)
 * Provide access to the SDO serive of the built-in SDOHandler.
 * 
 * 2020-11-21 AW Done
 * ----------------------------------------------------------------*/

SDOCommStates MCNode::ReadSDO(unsigned int Idx, unsigned char SubIdx)
{
	return RWSDO.ReadSDO(Idx,SubIdx);
}

/*------------------------------------------------------------------
 * DOCommStates WriteSDO(unsigned int Idx, unsigned char SubIdx, uint32_t * pData,unsigned char len)
 * Provide access to the SDO serive of the built-in SDOHandler.
 * 
 * 2020-11-21 AW Done
 * ----------------------------------------------------------------*/

SDOCommStates MCNode::WriteSDO(unsigned int Idx, unsigned char SubIdx, uint32_t * pData,unsigned char len)
{
	return RWSDO.WriteSDO(Idx,SubIdx,pData,len);
}

/*------------------------------------------------------------------
 * unsigned long GetObjValue()
 * Provide access to the SDO serive of the built-in SDOHandler.
 * 
 * 2020-11-21 AW Done
 * ----------------------------------------------------------------*/

unsigned long MCNode::GetObjValue()
{
	return RWSDO.GetObjValue();
}
//--------------------------------------------------------------------
// --- private functions ---
//--------------------------------------------------------------------

		
/*------------------------------------------------------------------
 * void OnRxHandler(MCMsg *Msg)
 * React to any received SysMsg. This callback willb e regsitered at the
 * MsgHandler and will deal with any received SysMsg like boot, EMCY and the
 * responses to SendCW. Alsom deals with asynch Rx of a StatusWord in
 * non net systems.
 * 
 * 2020-11-21 AW Done
 * ----------------------------------------------------------------*/

void MCNode::OnRxHandler(MCMsg *Msg)
{
	MCMsgCommands Cmd = Msg->Hdr.u8Cmd;
	
	switch(Cmd)
	{
		case eBootMsg:
			#if(DEBUG_NODE & DEBUG_RXMSG)
			std::printf("Node: Rx Boot\n");
			#endif
			
			isLive = true;
			
			RWSDO.ResetComState();
			ResetComState();
				
			break;
		case eCtrlWord:
			//mus be the response - check for ok
			//there mnight be a transient moment where an response is recevied while being 
			//in eCWRetry
			if((CWAccessState == eCWWaiting) || (CWAccessState == eCWRetry))
			{
				if(((CwMsgResponse *)Msg)->Error == 0)
				{
					#if(DEBUG_NODE & DEBUG_TXCW)
					std::printf("Node: Rx CW ok -->CWRxResponse\n");
					#endif
					firstCWAccess = 0;
					CWAccessState = eCWRxResponse;
				}
				else
				{
					#if(DEBUG_NODE & DEBUG_ERROR)
					std::printf("Node: Rx CW error\n");
					#endif
					
					//wrong answer
					CWAccessState = eCWError;
				}
			}
			else
			{
				#if(DEBUG_NODE & DEBUG_ERROR)
				std::printf("Node: Rx CW why? ");
				#endif
				//wrong state
				CWAccessState = eCWError;
			}			
			break;
		case eStatusWord:
			//does contain valuable data
			//can be received at anytime
			StatusWord = ((CwSwMsg *)Msg)->Payload;
			SWRxAt = actTime;
			
			#if(DEBUG_NODE & DEBUG_RXSW)
			std::printf("Node: Rx SW ");
			std::printf("%X\n", StatusWord);
			#endif
			break;
		case eEmergencyMsg:
			//does contain valuable data
			//can be received at anytime
			EMCYCode = ((EMCYMsg *)Msg)->ErrorCode;
			
			#if(DEBUG_NODE & DEBUG_RXEMCY)
			std::printf("Node: Rx EMCY ");
			std::printf("%X\n", EMCYCode);
			#endif
			break;
		default:
			//unexpected message
			#if(DEBUG_NODE & DEBUG_ERROR)
			std::printf("Node: Rx wrong cmd!");
			std::printf("%X\n", StatusWord);
			#endif
			
			RxTxState = eCWError;
			break;
	}
}

/*------------------------------------------------------------------
 * SDOCommStates CheckSDOState()
 * Read-acces to the ComState of the built-in SDOHandler
 * 
 * 2020-11-21 AW Done
 * ----------------------------------------------------------------*/

SDOCommStates MCNode::GetSDOState()
{
	//check the SDOState
	return RWSDO.GetComState();
}