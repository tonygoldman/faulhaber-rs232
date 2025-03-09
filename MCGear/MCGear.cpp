/*---------------------------------------------------
 * MCGear.cpp
 * implements the class to handel NMT and access
 * to the CW/SW
 * MCNode does not implement the CiA 402 state machine
 *
 * 2020-05-24 AW Frame
 * 2024-05-04 AW removed ref to timer
 * 2024-05-05 AW MCGear derived from MCDrive
 * 2024-05-09 AW added getter for ActGear
 * 2024-08-18 AW edited some calls to fit to the updated libs
 *--------------------------------------------------------------*/
 
//--- includes ---

#include <MCGear.h>


//--- local defines ---

#define DEBUG_RXMSG		0x0001
#define DEBUG_TO		  0x0002
#define DEBUG_ERROR		0x0004
#define DEBUG_UPDATE	0x0010
#define DEBUG_MoveSpeed	0x0020
#define DEBUG_ENABLE	0x0040
#define DEBUG_DISABLE	0x0080
#define DEBUG_STOP		0x0100
#define DEBUG_MOVEPP	0x0200
#define DEBUG_HOME		0x0400
#define DEBUG_RWPARAM	0x0800
#define DEBUG_PULLSW    0x1000
#define DEBUG_WriteSDO 0x2000

#define DEBUG_DRIVE (DEBUG_TO | DEBUG_ERROR) 

//--- specific objects of the GH module

const uint16_t ProgramConstIndex 	= 0x3010;
const uint8_t SubIdxActGear 			= 0x01;
const uint8_t SubIdxTargetGear 		= 0x02;
const uint8_t SubIdxGearDist 			= 0x03;
const uint8_t SubIdxGearStatus 		= 0x04;

//--- some definition to handle StatusWord and ControlWord of a drive ---

const uint16_t FSM402StatusMask = 0x6F;

const uint16_t FSM402_SwitchOnDisabled = 0x40;
const uint16_t FSM402_Ready2SwitchOn = 0x21;
const uint16_t FSM402_SwitchedOn = 0x23;
const uint16_t FSM402_Enabled = 0x27;
const uint16_t FSM402_Stopped = 0x07;
const uint16_t FSM402_FaultState = 0x08;
const uint16_t FSM402_FaultBit = 0x08;

const uint16_t StatusBit_PP_IsInPos = 0x0400;
const uint16_t StatusBit_PP_Ack = 0x1000;

const uint16_t StatusBit_PV_TargetReached = 0x0400;
const uint16_t StatusBit_PV_n0 = 0x1000;

const uint16_t StatusMask_Homing_Finished = 0x1400;

const uint16_t FSM402ControlMask = 0x000F;
const uint16_t FSM402_QSBit = 0x0004;
const uint16_t FSM402_HaltBit = 0x0100;

const uint16_t PP_StartBit 		= 0x0010;
const uint16_t PP_ImmediateBit 	= 0x0020; 
const uint16_t PP_RelativeBit	= 0x0040;
const uint16_t PP_ChangeOnSetP	= 0x0200;

//--- defines for the time-outs -------

const uint16_t MaxSWResponseDelay = 50;
const uint16_t PullSWCycleTime = 20;

//--- public functions ---

/*---------------------------------------------------------------------
 * MCGear()
 * Not much to be dnone in the intializer
 * 
 * 2020-11-22 AW Done
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

MCGear::MCGear()
{
	;
}

/*---------------------------------------------------------------------
 * void SetNodeId(uint8_t ThisNodeId)
 * Set the NodeId for this instance. Needs to be called before the
 * Msghandler can be registered
 * ThisNode is the MCNode instance local to MCGear
 * 
 * 2020-11-22 AW Done
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

void MCGear::SetNodeId(uint8_t ThisNodeId)
{
	ThisNode.SetNodeId(ThisNodeId);
}

/*---------------------------------------------------------------------
 * void Connect2MsgHandler(MsgHandler *ThisHandler)
 * In each system there is a single Msghandler but there can be
 * multiple drives. So the different instances of the MCNode and their
 * embeddd SDOHandlers need to be connected to the instance of the 
 * Msghandler by calling this method.
 * Also sets a default for this instances ComState
 * ThisNode is the MCNode instance local to MCGear
 * 
 * 2020-11-22 AW Done
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

void MCGear::Connect2MsgHandler(MsgHandler *ThisHandler)
{
	ThisNode.Connect2MsgHandler(ThisHandler);
	
	GearRxTxState = eMCIdle;
}

/*---------------------------------------------------------------------
 * void SetActTime(uint32_t time)
 * If no HW-timer is used this method needs to called cyclically
 * with the latest millis() value to check for any time-outs.
 * Does the same update for the MCNode and embedded SDOhandler.
 * ThisNode is the MCNode instance local to MCGear
 *  
 * 2020-11-22 AW Done
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

void MCGear::SetActTime(uint32_t time)
{
	actTime = time;
	ThisNode.SetActTime(time);
}

/*-------------------------------------------------------------------
 * CWCommStates CheckComState()
 * Check the ComState of the MCNode instance and update the ComState of then
 * Drive itself if requried.
 * Return the ComState
 *
 * 2020-11-22 AW Done
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

/*-------------------------------------------------------------------
 * CWCommStates CheckComState()
 * Check the ComState of the MCNode instance and update the ComState of then
 * Drive itself if requried.
 * Return the ComState
 *
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

DriveCommStates MCGear::CheckComState()
{
	//check  status will force the MCDriveRxTxState into eCWError or eCWTimeout
	//if SDO failed. Otherwise we will simply read the actual MCDriveRxTxState
	CWCommStates NodeState = ThisNode.UpdateComStateBySDO();
	SDOAccessState = ThisNode.GetSDOState();
	
	if(NodeState == eCWTimeout)
		GearRxTxState = eMCTimeout;
	if(NodeState == eCWError)
		GearRxTxState = eMCError;
		
	return GearRxTxState;
}

/*---------------------------------------------------------------------
 * CWCommStates GetNodeState()
 * Return the ComState of the MCNode instance of this drive.
 * 
 * 2020-11-22 AW Done
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

CWCommStates MCGear::GetNodeState()
{
	return ThisNode.UpdateComStateBySDO();
}

/*---------------------------------------------------------------------
 * SDOCommStates GetSDOState()
 * Return the ComState of teh SDOHandler instance of this drive.
 * 
 * 2020-11-22 AW Done
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

SDOCommStates MCGear::GetSDOState()
{
	return ThisNode.GetSDOState();
}

/*---------------------------------------------------------------------
 * uint16_t GetSW()
 * Return the last StatusWord received from the drive.
 * 
 * 2020-11-22 AW Done
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

uint16_t MCGear::GetSW()
{
	return ThisNode.StatusWord;
}

/*---------------------------------------------------------------------
 * uint8_t GetOpMode()
 * Return the last OpMode as updated by the UpdateStatus call
 * 
 * 2024-05-10W MCGear
 *--------------------------------------------------------------------*/

uint8_t MCGear::GetOpMode()
{
	return (uint8_t)OpModeReported;
}

/*---------------------------------------------------------------------
 * CWCommStates GetCWAccess()
 * Return the actual CWAccessState of this drive. this is for debugging
 * any of the step sequences where an access to the CW is used.
 * 
 * 2020-11-22 AW Done
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

CWCommStates MCGear::GetCWAccess()
{
	return CWAccessState;
}

/*---------------------------------------------------------------------
 * uint8_t GetAccessStep()
 * Return the general AccessStep used in the step sequences for the
 * different actions. this is used to debug where we might be stuck.
 * 
 * 2020-11-22 AW Done
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

uint8_t MCGear::GetAccessStep()
{
	return AccessStep;
}

/*---------------------------------------------------------------------
 * void ResetComState()
 * Reset the ComState of the Drive but do the same for teh MCNode instance
 * and reset the AccessSteps to 0 too.
 * Timeout and Retry counters are reset too to have a clean drive.
 * 
 * 2020-11-22 AW Done
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

void MCGear::ResetComState()
{
	GearRxTxState = eMCIdle;
	ThisNode.ResetComState();
	SDOAccessState = eSDOIdle;
	CWAccessState = eCWIdle;
	AccessStep = 0;

	TORetryCounter = 0;
	BusyRetryCounter = 0;
}

/*---------------------------------------------------------------------
 * void SetTORetryMax(uint8_t value)
 * Set a different value for the number of TO the drive can have before
 * the ComState will be eMCError. Default is set in the class definition.
 * 
 * This could be extended to the MCNode instance too but is nor so far.
 * 
 * 2020-11-22 AW Done
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

void MCGear::SetTORetryMax(uint8_t value)
{
	ThisNode.SetTORetryMax(value);
}

/*---------------------------------------------------------------------
 * void SetBusyRetryMax(uint8_t value)
 * Set a different value for the number of subsequent retrys the drive 
 * can have before the ComState will be eMCError. 
 * Default is set in the class definition.
 * 
 * This could be extended to the MCNode instance too but is nor so far.
 *  * 
 * 2020-11-22 AW Done
 *--------------------------------------------------------------------*/

void MCGear::SetBusyRetryMax(uint8_t value)
{
	ThisNode.SetBusyRetryMax(value);
}

//---------------------------------------------------------------------
// ----- real drive behavior ------------------------------------------

/*---------------------------------------------------------------------
 * DriveCommStates UpdateDriveStatus()
 * Update the local copy of the OpMode and the StatusWord.
 * Uses a step sequence based on AccessStep to create the sequence of
 * requests.
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 * 
 * 2020-11-22 AW Done
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

#if 1
//explizite Implentierung - mit spzifischen Testmeldungen

DriveCommStates MCGear::UpdateDriveStatus()
{
	switch(AccessStep)
	{
		case 0:
			switch(SDOAccessState)
			{
				case eSDODone:
					OpModeReported = (uint8_t)ThisNode.GetObjValue();
					AccessStep = 1;
					ThisNode.ResetSDOState();
					SDOAccessState = eSDOIdle;
					
					#if (DEBUG_DRIVE & DEBUG_UPDATE)
					Serial.print("Gear: OpMode pulled ");
					Serial.println(OpModeReported, HEX);
					#endif
					break;
				case eSDOIdle:
				case eSDORetry:
				case eSDOWaiting:
					#if(DDEBUG_DRIVE & DEBUG_UPDATE)
					if(SDOAccessState == eSDOIdle)
						Serial.print("Gear: OpMode Request ");
					else if(SDOAccessState == eSDORetry)
						Serial.print("Gear: OpMode Request Retry");
					#endif
					
					SDOAccessState = ThisNode.ReadSDO(0x6061, 0x00);
					GearRxTxState = eMCWaiting;
					break;
			}								
			break;
		case 1:
			switch(SDOAccessState)
			{
				case eSDODone:
					ThisNode.StatusWord = (uint16_t)ThisNode.GetObjValue();
					AccessStep = 0;
					GearRxTxState = eMCDone;
					ThisNode.ResetSDOState();
					SDOAccessState = eSDOIdle;
					
					#if(DEBUG_DRIVE & DEBUG_UPDATE)
					Serial.print("Gear: SW pulled ");
					Serial.println(ThisNode.StatusWord, HEX);
					#endif
					break;
				case eSDOIdle:
				case eSDORetry:
				case eSDOWaiting:
					#if(DEBUG_DRIVE & DEBUG_UPDATE)
					if(SDOAccessState == eSDOIdle)
						Serial.println("Gear: SW Request ");
					#endif
					
					SDOAccessState = ThisNode.ReadSDO(0x6041, 0x00);
					break;
			}
			break;
	}	
	//always check whether a communication is stuck final 
	return CheckComState();
}

#else
//alternative Implementeirung die vorhandenen Funktionen nutzend

DriveCommStates MCGear::UpdateDriveStatus()
{
	switch(AccessStep)
	{
		case 0:
      if((GearRxTxState = ReadObject(0x6061, 0x00,(uint8_t *)&OpModeReported)) == eMCDone)
			  AccessStep = 1;
					
			break;
		case 1:
      if((GearRxTxState = ReadObject(0x6041, 0x00,(uint16_t *)&(ThisNode.StatusWord))) == eMCDone)
			  AccessStep = 0;

			break;
	}	
	//always check whether a communication is stuck final 
	return CheckComState();
}
#endif

/*---------------------------------------------------------------------
 * DriveCommStates UpdateActGear()
 * Update the local copy of the actual gear we are in
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 * 
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

DriveCommStates MCGear::UpdateActGear(uint32_t *reportedGear)
{
	return ReadObject(ProgramConstIndex,SubIdxActGear,reportedGear);
}

/*---------------------------------------------------------------------
 * DriveCommStates ReadObject(uint16_t idx, uint8_t subIdx, uint8_t *dataPtr)
 * DriveCommStates ReadObject(uint16_t idx, uint8_t subIdx, uint16_t *dataPtr)
 * DriveCommStates ReadObject(uint16_t idx, uint8_t subIdx, uint32_t *dataPtr)
 *
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 * 
 * 2024-05-12 AW MCGear
 *               aus UpdateActGear für 3 Wortbereiten abgeleitet
 *--------------------------------------------------------------------*/

DriveCommStates MCGear::ReadObject(uint16_t idx, uint8_t subIdx, uint8_t *dataPtr)
{	
	switch(SDOAccessState)
	{
		case eSDODone:
			*dataPtr = (uint8_t)ThisNode.GetObjValue();
			ThisNode.ResetSDOState();
			SDOAccessState = eSDOIdle;
			GearRxTxState = eMCDone;
			
			#if (DEBUG_DRIVE & DEBUG_UPDATE)
			Serial.print("Gear: object ");
		  Serial.print(idx,HEX);
		  Serial.print(".");
		  Serial.print(subIdx,HEX);
		  Serial.print(" = ");
			Serial.println(*dataPtr, HEX);
			#endif
			break;
		case eSDOIdle:
		case eSDORetry:
		case eSDOWaiting:
			#if(DEBUG_DRIVE & DEBUG_UPDATE)
			if(SDOAccessState == eSDOIdle)
			Serial.print("Gear: object ");
		  Serial.print(idx,HEX);
		  Serial.print(".");
		  Serial.print(subIdx,HEX);
			Serial.println(" angefragt");
			#endif
			
			SDOAccessState = ThisNode.ReadSDO(idx, subIdx);
			GearRxTxState = eMCWaiting;
			break;				
	}

	//always check whether a communication is stuck final 
	return CheckComState();
}

DriveCommStates MCGear::ReadObject(uint16_t idx, uint8_t subIdx, uint16_t *dataPtr)
{	
	switch(SDOAccessState)
	{
		case eSDODone:
			*dataPtr = (uint16_t)ThisNode.GetObjValue();
			ThisNode.ResetSDOState();
			SDOAccessState = eSDOIdle;
			GearRxTxState = eMCDone;
			
			#if (DEBUG_DRIVE & DEBUG_UPDATE)
			Serial.print("Gear: object ");
		  Serial.print(idx,HEX);
		  Serial.print(".");
		  Serial.print(subIdx,HEX);
		  Serial.print(" = ");
			Serial.println(*dataPtr, HEX);
			#endif
			break;
		case eSDOIdle:
		case eSDORetry:
		case eSDOWaiting:
			#if(DEBUG_DRIVE & DEBUG_UPDATE)
			if(SDOAccessState == eSDOIdle)
			Serial.print("Gear: object ");
		  Serial.print(idx,HEX);
		  Serial.print(".");
		  Serial.print(subIdx,HEX);
			Serial.println(" angefragt");
			#endif
			
			SDOAccessState = ThisNode.ReadSDO(idx, subIdx);
			GearRxTxState = eMCWaiting;
			break;				
	}

	//always check whether a communication is stuck final 
	return CheckComState();
}

DriveCommStates MCGear::ReadObject(uint16_t idx, uint8_t subIdx, uint32_t *dataPtr)
{	
	switch(SDOAccessState)
	{
		case eSDODone:
			*dataPtr = (uint32_t)ThisNode.GetObjValue();
			ThisNode.ResetSDOState();
			SDOAccessState = eSDOIdle;
			GearRxTxState = eMCDone;
			
			#if (DEBUG_DRIVE & DEBUG_UPDATE)
			Serial.print("Gear: object ");
		  Serial.print(idx,HEX);
		  Serial.print(".");
		  Serial.print(subIdx,HEX);
		  Serial.print(" = ");
			Serial.println(*dataPtr, HEX);
			#endif
			break;
		case eSDOIdle:
		case eSDORetry:
		case eSDOWaiting:
			#if(DEBUG_DRIVE & DEBUG_UPDATE)
			if(SDOAccessState == eSDOIdle)
			Serial.print("Gear: object ");
		  Serial.print(idx,HEX);
		  Serial.print(".");
		  Serial.print(subIdx,HEX);
			Serial.println(" angefragt");
			#endif
			
			SDOAccessState = ThisNode.ReadSDO(idx, subIdx);
			GearRxTxState = eMCWaiting;
			break;				
	}

	//always check whether a communication is stuck final 
	return CheckComState();
}

/*---------------------------------------------------------------------
 * DriveCommStates EnableDrive()
 * Enable the drive state machine.
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 
 * 2020-11-22 AW Done
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

DriveCommStates MCGear::EnableDrive()
{
uint16_t StatusWord = ThisNode.StatusWord;
uint16_t ControlWord = ThisNode.ControlWord;
	
	if((StatusWord & FSM402StatusMask) == FSM402_Enabled)
	{
		if((CWAccessState == eCWIdle) || (CWAccessState == eCWDone))
		{
			ThisNode.ResetComState();
			CWAccessState = eCWIdle;
			GearRxTxState = eMCDone;
			
			#if(DEBUG_DRIVE & DEBUG_ENABLE)
			Serial.print("Gear: Enabled SW ");
			Serial.println(StatusWord, HEX);
			#endif
		}
		else
		{
			//CWAccess has to be finshed 
			CWAccessState = ThisNode.SendCw(ControlWord,MaxSWResponseDelay);
		}
	}
	else
	{
		unsigned int newCW;

        if((StatusWord & FSM402StatusMask) == FSM402_Ready2SwitchOn) 
			newCW = (ControlWord & FSM402ControlMask) | 0x07;
		else if((StatusWord & FSM402StatusMask) == FSM402_SwitchedOn) 
			newCW = (ControlWord & FSM402ControlMask) | 0x0F;
		else if((StatusWord & FSM402StatusMask) == FSM402_Stopped) 
			newCW = (ControlWord & FSM402ControlMask) | 0x0F;
		else if((StatusWord & FSM402StatusMask) == FSM402_FaultState) 
			newCW = (ControlWord & FSM402ControlMask) | 0x80; 
		else
		    newCW = (ControlWord & FSM402ControlMask) | 0x06; 	
				
		//send a new request only, if that has not been done last time	
		#if(DEBUG_DRIVE & DEBUG_ENABLE)
		if((CWAccessState == eCWIdle) || (CWAccessState == eCWRetry))
		{
			Serial.print("Gear: Enable ");
			Serial.println(newCW, HEX);
		}
		#endif
			
		//would have to reflect the CWAccessState but that's done
		//inside the SendCW already
		CWAccessState = ThisNode.SendCw(newCW,MaxSWResponseDelay);			
		GearRxTxState = eMCWaiting;
	}
	//always check whether a SDO is stuck final 
	return CheckComState();
}

/*---------------------------------------------------------------------
 * DriveCommStates DisableDrive()
 * Disable the drive state machine.
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 
 * 2020-11-22 AW Done
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

DriveCommStates MCGear::DisableDrive()
{
uint16_t StatusWord = ThisNode.StatusWord;
uint16_t ControlWord = ThisNode.ControlWord;


	if((StatusWord & FSM402StatusMask) == FSM402_SwitchOnDisabled)
	{
		if((CWAccessState == eCWIdle) || (CWAccessState == eCWDone))
		{
			ThisNode.ResetComState();
			CWAccessState = eCWIdle;		
			GearRxTxState = eMCDone;
			
			#if(DEBUG_DRIVE & DEBUG_DISABLE)
			Serial.print("Gear: Disabled SW ");
			Serial.println(StatusWord, HEX);
			#endif
		}
		else
		{
			CWAccessState = ThisNode.SendCw(ControlWord,MaxSWResponseDelay);
		}
		
	}
	else
	{
		uint16_t newCW = (ControlWord & ~FSM402ControlMask);
						
		//send a new request only, if that has not been done last time	
		#if(DEBUG_DRIVE & DEBUG_DISABLE)
		if((CWAccessState == eCWIdle) || (CWAccessState == eCWRetry))
		{
			Serial.print("Gear: Disable CW ");
			Serial.println(ControlWord, HEX);
		}
		#endif		
			
		//would have to reflect the CWAccessState but that's done
		//inside the SendCW already
		CWAccessState = ThisNode.SendCw(newCW,MaxSWResponseDelay);
		GearRxTxState = eMCWaiting;
		
	}
	//always check whether a SDO is stuck final 
	return CheckComState();
}

/*---------------------------------------------------------------------
 * DriveCommStates StopDrive()
 * Switch the drive state machine to QuickStop.
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be reset to eMCIdle after having registered the eMCDone
 *
 * 2020-11-22 AW untested
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

DriveCommStates MCGear::StopDrive()
{
uint16_t StatusWord = ThisNode.StatusWord;
uint16_t ControlWord = ThisNode.ControlWord;

	if( ((StatusWord & FSM402StatusMask) == FSM402_Stopped) || ((StatusWord & FSM402StatusMask) == FSM402_SwitchOnDisabled) ) 
	{		
		if((CWAccessState == eCWIdle) || (CWAccessState == eCWDone))
		{
			ThisNode.ResetComState();
			CWAccessState == eCWIdle;
			
			#if(DEBUG_DRIVE & DEBUG_STOP)
			Serial.print("Gear: Stopped SW ");
			Serial.println(StatusWord, HEX);
			#endif
		}
		else
		{
			CWAccessState = ThisNode.SendCw(ControlWord,MaxSWResponseDelay);	
		}
	}
	else
	{
		//reset the QS bit
		uint16_t newCW = ControlWord & ~FSM402_QSBit;
					
		#if(DEBUG_DRIVE & DEBUG_STOP)
		if((CWAccessState == eCWIdle) || (CWAccessState == eCWRetry))
		{
			Serial.print("Disable ");
			Serial.println(ControlWord, HEX);
		}
		#endif
			
		//would have to reflect the CWAccessState but that's done
		//inside the SendCW already
		CWAccessState = ThisNode.SendCw(newCW,MaxSWResponseDelay);
		GearRxTxState = eMCWaiting;
	}
	//always check whether a SDO is stuck final 
	return CheckComState();
}


/*---------------------------------------------------------------------
 * DriveCommStates SwitchToGear(int32_t TargetGear)
 * Request a specific gear to be set by sending the gear number to 
 * the program constant
 *
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 * 
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

DriveCommStates MCGear::SwitchToGear(int32_t TargetGear)
{
  return WriteObject(ProgramConstIndex, SubIdxTargetGear,(uint32_t)TargetGear);	
}

/*---------------------------------------------------------------------
 * DriveCommStates WriteObject(uint16_t idx, uint8_t subIdx, uint8_t value)
 * DriveCommStates WriteObject(uint16_t idx, uint8_t subIdx, uint16_t value)
 * DriveCommStates WriteObject(uint16_t idx, uint8_t subIdx, uint32_t value)

 * Write to a single object
 *
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 * 
 * 2024-05-05 AW MCGear
 * 2024-05-12    aus SwitchtoGear abgeleitet
 *--------------------------------------------------------------------*/

DriveCommStates MCGear::WriteObject(uint16_t idx, uint8_t subIdx, uint8_t value)
{
	
	if(SDOAccessState == eSDODone)
	{
		ThisNode.ResetComState();
		ThisNode.ResetSDOState();
		SDOAccessState = eSDOIdle;
		GearRxTxState = eMCDone;
		
		#if(DEBUG_DRIVE & DEBUG_WriteSDO)
		Serial.print("Gear: set object ");
		Serial.print(idx,HEX);
		Serial.print(".");
		Serial.print(subIdx,HEX);
		Serial.print(" = ");
		Serial.print(value, HEX);
		Serial.println(" done");
		#endif
	}
	else
	{
		#if(DEBUG_DRIVE & DEBUG_WriteSDO)
		if(SDOAccessState == eSDOIdle)
		{
			Serial.print("Gear: set object ");
			Serial.print(idx,HEX);
			Serial.print(".");
			Serial.print(subIdx,HEX);
			Serial.print(" = ");
			Serial.print(value, HEX);
			Serial.println(" requested");
		}
		#endif
	
		SDOAccessState = ThisNode.WriteSDO(idx, subIdx,(uint32_t *)&value,1);
		GearRxTxState = eMCWaiting;
	}

	//always check whether a SDO is stuck final 
	return CheckComState();				
}

DriveCommStates MCGear::WriteObject(uint16_t idx, uint8_t subIdx, uint16_t value)
{
	
	if(SDOAccessState == eSDODone)
	{
		ThisNode.ResetComState();
		ThisNode.ResetSDOState();
		SDOAccessState = eSDOIdle;
		GearRxTxState = eMCDone;
		
		#if(DEBUG_DRIVE & DEBUG_WriteSDO)
		Serial.print("Gear: set object ");
		Serial.print(idx,HEX);
		Serial.print(".");
		Serial.print(subIdx,HEX);
		Serial.print(" = ");
		Serial.print(value, HEX);
		Serial.println(" done");
		#endif
	}
	else
	{
		#if(DEBUG_DRIVE & DEBUG_WriteSDO)
		if(SDOAccessState == eSDOIdle)
		{
			Serial.print("Gear: set object ");
			Serial.print(idx,HEX);
			Serial.print(".");
			Serial.print(subIdx,HEX);
			Serial.print(" = ");
			Serial.print(value, HEX);
			Serial.println(" requested");
		}
		#endif
	
		SDOAccessState = ThisNode.WriteSDO(idx, subIdx,(uint32_t *)&value,2);
		GearRxTxState = eMCWaiting;
	}

	//always check whether a SDO is stuck final 
	return CheckComState();				
}

DriveCommStates MCGear::WriteObject(uint16_t idx, uint8_t subIdx, uint32_t value)
{
	
	if(SDOAccessState == eSDODone)
	{
		ThisNode.ResetComState();
		ThisNode.ResetSDOState();
		SDOAccessState = eSDOIdle;
		GearRxTxState = eMCDone;
		
		#if(DEBUG_DRIVE & DEBUG_WriteSDO)
		Serial.print("Gear: set object ");
		Serial.print(idx,HEX);
		Serial.print(".");
		Serial.print(subIdx,HEX);
		Serial.print(" = ");
		Serial.print(value, HEX);
		Serial.println(" done");
		#endif
	}
	else
	{
		#if(DEBUG_DRIVE & DEBUG_WriteSDO)
		if(SDOAccessState == eSDOIdle)
		{
			Serial.print("Gear: set object ");
			Serial.print(idx,HEX);
			Serial.print(".");
			Serial.print(subIdx,HEX);
			Serial.print(" = ");
			Serial.print(value, HEX);
			Serial.println(" requested");
		}
		#endif
	
		SDOAccessState = ThisNode.WriteSDO(idx, subIdx,(uint32_t *)&value,4);
		GearRxTxState = eMCWaiting;
	}

	//always check whether a SDO is stuck final 
	return CheckComState();				
}

/*---------------------------------------------------------------------
 * DriveCommStates IsInPos()
 * Check the StatusWord of the drive and test it for the target reached
 * bit being set. Will update the Statusword cyclically using the internal
 * Wait4Status.
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be rest to eMCIdle after having registered the eMCDone
 * 
 * 2020-11-22 AW Done
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

DriveCommStates MCGear::IsInPos()
{				
	return Wait4Status(StatusBit_PP_IsInPos,PullSWCycleTime);	
}

/*---------------------------------------------------------------------
 * bool IsLive()
 * Check whether a boot Msg of the drive has been received
 * Please note: in net-mode of multiple drives no boot messages
 * will be sent at all.
 *
 * 2020-11-22 AW untested
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

bool MCGear::IsLive()
{
	return ThisNode.IsLive();
}
		
/*---------------------------------------------------------------------
 * uint16_t GetLastError()
 * Read the last EMCY code. EMCY service is not available in net mode.
 * 
 * 2020-11-22 AW Done
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

uint16_t MCGear::GetLastError()
{
	return ThisNode.GetLastError();
}

//-------------------------------------------------------------------
//---- private functions --------
//-------------------------------------------------------------------

/*---------------------------------------------------------------------
 * void OnTimeOut()
 * Handler for whatever TO might be implemented during complex actions
 * As of now not used.
 * 
 * 2020-11-22 AW untested
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

void MCGear::OnTimeOut()
{
	#if(DEBUG_DRIVE & DEBUG_TO)
	Serial.print("Gear: Timeout ");
	#endif
}

/*---------------------------------------------------------------------
 * DriveCommStates Wait4Status(uint16_t mask, uint16_t CycleTime)
 * Check the StatusWord of the drive for the given pattern.
 * Will only return with eMCDone when the pattern is found. Will update
 * the StatusWord cyclically.
 * --> will report eMCWaiting while busy
 * --> will report eMCDone when finished
 * --> needs to be reset to eMCIdle after having registered the eMCDone
 * 
 * 2020-11-22 AW Done
 * 2024-05-05 AW MCGear
 *--------------------------------------------------------------------*/

DriveCommStates MCGear::Wait4Status(uint16_t mask, uint16_t CycleTime)
{	
	if((ThisNode.StatusWord & mask) == mask)
	{
		if( ((CWAccessState == eCWIdle) || (CWAccessState == eCWDone)) &&
			(SDOAccessState == eSDOIdle))
		{
			ThisNode.ResetComState();
			CWAccessState = eCWIdle;
			GearRxTxState = eMCDone;
				
			#if(DEBUG_DRIVE & DEBUG_PULLSW)
			Serial.print("Gear: mask ");
			Serial.print(mask, HEX);
			Serial.println(" found");
			#endif
		}
		else
		{
			CWAccessState = ThisNode.PullSW(CycleTime);			
		}
	}
	else
	{						
		#if(DEBUG_DRIVE & DEBUG_PULLSW)
		if((CWAccessState == eCWIdle) || (CWAccessState == eCWRetry))
		{
			Serial.println("Gear: pull SW: ");
		}
		#endif
			
		CWAccessState = ThisNode.PullSW(CycleTime);
		GearRxTxState = eMCWaiting;			
	}
	//always check whether a SDO is stuck final 
	return CheckComState();					
}
