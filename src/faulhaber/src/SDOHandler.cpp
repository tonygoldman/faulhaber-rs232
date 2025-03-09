/*---------------------------------------------------
 * SDOHandler.cpp
 * handels R/W access to deive parameters via SDO services
 * does itself no interpreation
 *
 * 2020-05-24 AW Frame
 * 2024-05-07 AW remove ref to timer
 *
 *--------------------------------------------------------------*/
 
//--- includes ---

#include "faulhaber/SDOHandler.h"
#include <cstdio>

#define DEBUG_RXMSG     0x0001
#define DEBUG_WREQ      0x0002
#define DEBUG_RREQ      0x0004
#define DEBUG_ERROR     0x0008
#define DEBUG_TO        0x0010
#define DEBUG_UPDATETime 0x0020
#define DEBUG_BUSY      0x8000

//#define DEBUG_SDO (DEBUG_TO | DEBUG_ERROR | DEBUG_RXMSG | DEBUG_WREQ | DEBUG_RREQ | DEBUG_BUSY)
#define DEBUG_SDO (DEBUG_TO | DEBUG_ERROR)

//--- implementation ---

const unsigned int SDORespTimeOut = (4 * MaxMsgTime);

//--- public calls ---

/*---------------------------------------------------
 * SDOHandler()
 * init the instance by at least initializing the RxLen
 * 
 * 2020-11-18 AW Done
 *---------------------------------------------------*/
 
SDOHandler::SDOHandler()
{
    RxLen = 0;
}

/*-------------------------------------------------------
 * void init(MsgHandler *,uint8_t)
 * create a functor to register this instance of a SDOHandler
 * at the Msghander which is referred to.
 * The SDOHandler will store the pointer to the Msghandler for further
 * use. Needs to be given the handle under which the node is registered
 * at the MsgHandler and uses this to finally regsiter the Call-back for
 * SDO messages
 * 
 * 2020-11-18 AW Done
 * ---------------------------------------------------------------*/

void SDOHandler::init(MsgHandler *ThisHandler, uint8_t Handle)
{
    pfunction_holder Cb;
    //register Cb
    Cb.callback = (pfunction_pointer_t)SDOHandler::OnSDOMsgRxCb;
    Cb.op = (void *)this;
    
    Handler = ThisHandler;
    Channel = Handle;
    Handler->Register_OnRxSDOCb(Channel,&Cb);
    SDORxTxState = eSDOIdle;    
}

/*---------------------------------------------------------------
 * SDOCommStates GetComState()
 * return the state of either the Rx or Tx of an SDO
 * 
 * 2020-11-18 AW Done
 * -------------------------------------------------------------*/
 
SDOCommStates SDOHandler::GetComState()
{
    return SDORxTxState;
}

/*--------------------------------------------------------------
 * void SetTORetryMax(uint8_t)
 * An attempted transfer can run into a timeout either while trying to send
 * or by waiting for a response. The Time-out counter is incremented if 
 * a consecutive occurs. the call here cen be used to modify the
 * default max value for the number of time-outs in a row
 * 
 * 2020-11-18 AW Done
 * --------------------------------------------------------------*/ 

void SDOHandler::SetTORetryMax(uint8_t value)
{
    TORetryMax = value;
}

/*--------------------------------------------------------------
 * void SetBusyRetryMax(uint8_t)
 * An attempted transfer can fail because the Msghandler and Uart are blocked
 * either while trying to send a request. The Time-out counter is incremented if 
 * a consecutive occurs. the call here cen be used to modify the
 * default max value for the number of time-outs in a row
 * 
 * 2020-11-18 AW Done
 * --------------------------------------------------------------*/
 
void SDOHandler::SetBusyRetryMax(uint8_t value)
{
    BusyRetryMax = value;
}

/*----------------------------------------------
 * void SDOHandler::ResetComState()
 * to be called after each interaction to 
 * move the SDORxTxState from eDone to eIdle
 * 
 * 2020-10-16 AW inital
 * ---------------------------------------------*/

void SDOHandler::ResetComState()
{
    SDORxTxState = eSDOIdle;
    TORetryCounter = 0;
    BusyRetryCounter = 0;
    //Handler should not be reset, as it could be used by different
    //instances of the Drive
    //Handler->ResetMsgHandler();    
    //Handler can be unlocked if it is still denoted to be locked
    
    if(hasMsgHandlerLocked)
    {
        Handler->UnLockHandler();
        hasMsgHandlerLocked = false;
    }
}
/*-------------------------------------------------------------
 * SDOCommStates ReadSDO(uint16_t Idx, uint8_t SubIdx)
 * Try to read a drive parameter identified by its Idx and SubIdx.
 * Is using a step sequence of sendig a request, waiting for an answer
 * and giving it a retry if not sucessfull.
 * Reception fo the responses is via the OnRxHandler().
 * SDORxTxState is used to handle the steps and as the central feedback
 * ReadSDO will end up in eDone state to indicate the requested value
 * has been received and can be read.
 * So actuall reading the value will reset the communication state to eIdle.
 * 
 * As any access to the MsgHandler ReadSDO will lock the Msghandler and
 * will only unlock it the actual service failed.
 * Successful servie will unlock in OnRxHandler().
 * 
 * 2020-11-18 AW Done
 * -------------------------------------------------------------*/

SDOCommStates SDOHandler::ReadSDO(uint16_t Idx, uint8_t SubIdx)
{
    switch(SDORxTxState)
    {
        case eSDOIdle:
        case eSDORetry:
            //fill header
            RxRqMsg.u8Len = 7;
            RxRqMsg.u8Cmd = eSdoReadReq;
            RxRqMsg.Idx = Idx;
            RxRqMsg.SubIdx = SubIdx;

            if((hasMsgHandlerLocked = Handler->LockHandler()))
            {
                //try to send the data
                if(Handler->SendMsg(Channel,(MCMsg *)&RxRqMsg))
                {
                    SDORxTxState = eSDOWaiting;

                    BusyRetryCounter = 0;
                    
                    #if(DEBUG_SDO & DEBUG_RREQ)
                    std::printf("SDO: N %d RxReq ok: %X --> eSDOWaiting\n", Handler->GetNodeId(Channel), Idx);
                    #endif
                    
                    //register a timeout handler
                    RequestSentAt = actTime;
                    isTimerActive = true;
                }
                else
                {
                    Handler->UnLockHandler();
                    hasMsgHandlerLocked = false;

                    //didn't work
                    BusyRetryCounter++;
                    if(BusyRetryCounter > BusyRetryMax)
                    {
                        SDORxTxState = eSDOError;
                        #if(DEBUG_SDO & DEBUG_ERROR)
                        std::printf("SDO: N %d RxReq failed --> eError\n", Handler->GetNodeId(Channel));
                        #endif
                    }
                    else
                    {
                        SDORxTxState = eSDORetry;
                        #if(DEBUG_SDO & DEBUG_RREQ & DEBUG_BUSY)
                        std::printf("SDO: N %d RxReq busy --> eRetry\n", Handler->GetNodeId(Channel));
                        #endif
                    }
                }
            }
            break;
    }
    return SDORxTxState;
}

/*-------------------------------------------------------------
 * SDOCommStates WriteSDO(uint16_t Idx, uint8_t SubIdx,uint32_t *Data,uint8_t len)
 * Try to write a drive parameter identified by its Idx and SubIdx.
 * Additional parameters are the value itself and the size of the parameter in bytes.
 * Is using a step sequence of sendig a request, waiting for an answer
 * and giving it a retry if not sucessfull.
 * Reception fo the responses is via the OnRxHandler().
 * SDORxTxState is used to handle the steps and as the central feedback
 * WriteSDO will end up in eDone state to indicate the requested value
 * has been sent. Nedes to be reset explictily by calling ResetComState().
 * 
 * As any access to the MsgHandler WriteSDO will lock the Msghandler and
 * will only unlock it the actual service failed.
 * Successful servie will unlock in OnRxHandler().
 * 
 * 2020-11-18 AW Done
 * -------------------------------------------------------------*/

SDOCommStates SDOHandler::WriteSDO(uint16_t Idx, uint8_t SubIdx, uint32_t *Data,uint8_t len)
{
    switch(SDORxTxState)
    {
        case eSDOIdle:
        case eSDORetry:
        //fill header
            TxRqMsg.u8Len = 7 + len;
            TxRqMsg.u8Cmd = eSdoWriteReq;
            TxRqMsg.Idx = Idx;
            TxRqMsg.SubIdx = SubIdx;
            
            //copy the data into the message
          //on Cortex this has to be done byte wise as u8UserData is not aligned to 32 bits
            TxRqMsg.u8UserData[0] = ((uint8_t *)Data)[0];			
            TxRqMsg.u8UserData[1] = ((uint8_t *)Data)[1];			
            TxRqMsg.u8UserData[2] = ((uint8_t *)Data)[2];			
            TxRqMsg.u8UserData[3] = ((uint8_t *)Data)[3];			
            
            if((hasMsgHandlerLocked = Handler->LockHandler()))
            {				 
                //send the data
                if(Handler->SendMsg(Channel,(MCMsg *)&TxRqMsg))
                {
                    SDORxTxState = eSDOWaiting;
                    BusyRetryCounter = 0;
                    
                    #if(DEBUG_SDO & DEBUG_WREQ)
                    std::printf("SDO: N %d TxReq ok %X\n", Handler->GetNodeId(Channel), Idx);
                    #endif

                    RequestSentAt = actTime;
                    isTimerActive = true;
                }
                else
                {
                    Handler->UnLockHandler();
                    hasMsgHandlerLocked = false;

                    BusyRetryCounter++;
                    if(BusyRetryCounter > BusyRetryMax)
                    {
                        SDORxTxState = eSDOError;
                        #if(DEBUG_SDO & DEBUG_ERROR)
                        std::printf("SDO: N %d TxReq failed\n", Handler->GetNodeId(Channel));
                        #endif
                    }
                    else
                    {
                        SDORxTxState = eSDORetry;
                        #if(DEBUG_SDO & (DEBUG_WREQ | DEBUG_BUSY))
                        std::printf("SDO: N %d TxReq busy\n", Handler->GetNodeId(Channel));
                        #endif
                    }
                }
            }
            break;
    } //end of switch (SDORxTxState)
    return SDORxTxState;
}

/*-----------------------------------------------------
 * uint32_t GetObjValue()
 * Acutally read the last received object value.
 * It is returned as an int32_t and might have to be casted by a caller
 * 
 * 2020-11-18 AW Done
 * -------------------------------------------------------*/

uint32_t SDOHandler::GetObjValue()
{
    uint32_t retValue = RxData.u32;
    
    #if(DEBUG_SDO & DEBUG_RREQ)
    std::printf("SDO: Data fetched from buffer\n");
    #endif

    if(SDORxTxState == eSDODone)
        SDORxTxState = eSDOIdle;
        
    return retValue;    
}
//-------------------------------------------------------------------
//--- private calls ---

/*-------------------------------------------------------------------
 * void OnRxHandler(MCMsg *Msg)
 * The actual handler for any SDO services received by the MsgHandler
 * Checks wheter the received response belongs to any open
 * requenst and will switch these to eDone.
 * Other will transit to eError.
 * 
 * 2020-11-18 AW Done
 * -----------------------------------------------------------------*/

void SDOHandler::OnRxHandler(MCMsg *Msg)
{
    MCMsgCommands Cmd = Msg->Hdr.u8Cmd;
    SDOMaxMsg *SDO = (SDOMaxMsg *)Msg;

    #if(DEBUG_SDO  & DEBUG_RXMSG)
    std::printf("S: Rx in S:%d\n", SDORxTxState);
    if((RxRqMsg.Idx == SDO->Idx) && (RxRqMsg.SubIdx == SDO->SubIdx))
        std::printf(" as expexted\n");
    else
    {
      std::printf("SDO: Rx Idx %X exp: %X\n", RxRqMsg.Idx, SDO->Idx);
      std::printf("SDO: Rx SubIdx %X exp: %X\n", RxRqMsg.SubIdx, SDO->SubIdx);
    }	
    #endif
    
    switch(Cmd)
    {
        case eSdoReadReq:
            //should contain the requested data
            if((RxRqMsg.Idx == SDO->Idx) && (RxRqMsg.SubIdx == SDO->SubIdx) && 
                ((SDORxTxState == eSDOWaiting) || (SDORxTxState == eSDORetry)) )
            {
              #if(DEBUG_SDO  & DEBUG_RXMSG)
              std::printf("S: processing RRx\n --> eSDODone\n");
              #endif
                //correct answer
                //calc the length of the payload
                RxLen = (SDO->u8Len) - 7;
                
                isTimerActive = false;
                
                //on Cortex this has to be done byte wise as u8UserData is not aligned to 32 bits
                RxData.u8[0] = SDO->u8UserData[0];
                RxData.u8[1] = SDO->u8UserData[1];
                RxData.u8[2] = SDO->u8UserData[2];
                RxData.u8[3] = SDO->u8UserData[3];
                
                //switch transfer to eDone state and unlock the 
                //used MsgHandler    
                SDORxTxState = eSDODone;
                Handler->UnLockHandler();
                hasMsgHandlerLocked = false;

                #if(DEBUG_SDO  & DEBUG_RXMSG)
                std::printf("SDO: Rx Idx %X :%X\n", SDO->Idx, RxData.u32);
                #endif
            }
            else
            {
                //wrong answer
                SDORxTxState = eSDOError;
                
                #if(DEBUG_SDO & DEBUG_ERROR)
                std::printf("SDO: Rx Error! Idx: %X. %X >> %d\n", SDO->Idx, SDO->SubIdx, SDORxTxState);
                #endif                
            }
            break;
        case eSdoWriteReq:
            //should be the response only
            if((TxRqMsg.Idx == SDO->Idx) && (TxRqMsg.SubIdx == SDO->SubIdx) && 
                ((SDORxTxState == eSDOWaiting) || (SDORxTxState == eSDORetry)))
            {
              #if(DEBUG_SDO  & DEBUG_RXMSG)
              std::printf("S: processing WRx");
              #endif
                //correct answer
                //switch the state to the eDone and unlock the underlying 
                //MsgHandler
                SDORxTxState = eSDODone;
                Handler->UnLockHandler();
                hasMsgHandlerLocked = false;
                
                isTimerActive = false;

                #if(DEBUG_SDO  & DEBUG_RXMSG)
                std::printf("SDO: Tx Idx %X ok\n", SDO->Idx);
                #endif
            }
            else
            {
                //wrong answer
                SDORxTxState = eSDOError;    
                            
                #if(DEBUG_SDO & DEBUG_ERROR)
                std::printf("SDO: Tx Error! Idx: %X. %X >> %d\n", SDO->Idx, SDO->SubIdx, SDORxTxState);
                #endif                
            }
            break;
        default:
            //what's this? --> transit to eError
            SDORxTxState = eSDOError;
                #if(DEBUG_SDO & DEBUG_ERROR)
                std::printf("SDO: Rx wrong CMD Error!");
                #endif                
            break;
    }
}

/*----------------------------------------------------
 * void SetActTime(uint32_t time)
 * Soft-Update of the internal time in case of no HW timer being used.
 * use the updated time to check whether any of the responses
 * is timed out and call the OnTimeOut() if so.
 * 
 * 2020-11-18 AW Done
 * -----------------------------------------------------------*/

void SDOHandler::SetActTime(uint32_t time)
{
    #if (DEBUG_SDO & DEBUG_UPDATETime)
    std::printf("S: dT:%u\n", time - actTime);
    #endif
    
    actTime = time;
    
    if((isTimerActive) && ((RequestSentAt + SDORespTimeOut) < actTime))    
    {    
        OnTimeOut();
        isTimerActive = false;
    }

}

/*----------------------------------------------------------
 * void OnTimeOut()
 * In case of a time-out either detected by the HW-tiemr or by the 
 * soft-timer swtich the communication either to a retry and increment
 * the retry counter or switch to final state eTimeout.
 * 
 * 2020-11-18 AW Done
 * -------------------------------------------------------------*/

void SDOHandler::OnTimeOut()
{
    #if(DEBUG_SDO & DEBUG_TO)
    std::printf("SDO: Timeout ");
    #endif
    
    if(TORetryCounter < TORetryMax)
    {
        SDORxTxState = eSDORetry;
        TORetryCounter++;
        
        if(hasMsgHandlerLocked)
        {
            Handler->UnLockHandler();
            hasMsgHandlerLocked = false;
        }


        #if(DEBUG_SDO & DEBUG_TO)
        std::printf("retry\n");
        #endif
    }
    else
    {    
        SDORxTxState = eSDOTimeout;
        TORetryCounter = 0;

        #if(DEBUG_SDO & DEBUG_TO)
        std::printf("final\n");
        #endif
    }
}
