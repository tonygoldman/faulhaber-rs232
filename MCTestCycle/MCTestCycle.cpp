/*-------------------------------------------
 * MCTestCycle.cpp
 *
 * implements s impley test cayle to execute a continous communication
 * with a single MCDrive
 *
 * 2024-07-14 AW started
 *------------------------------------------------------*/
 
 //--- inlcudes ---
 #include "MCTestCycle.h"
 
 //--- defines ---

#define AutoReset 0

int8_t DefaultTargetOpModes[] = {1,3,6,8,9,10,-1};

/*-----------------------------------------------------------------------------
 * MCTestCycle(uint8_t NodeId, char *NodeName, int8_t HomingMethod)
 * 
 * init a new instance of the test cycle
 *
 * 2024-07-14 AW
 *
 *---------------------------------------------------------------------------*/

MCTestCycle::MCTestCycle(uint8_t NodeId, char *NodeName, int8_t HomingMethod)
{
	  Name = NodeName;
	  aDrive.SetNodeId(NodeId);
	
	  deltaSpeed = minSpeed/2;
    deltaAccDec = minAccDec;
		DriveAHomingMethod = HomingMethod;


	  driveStep = 0;
    
	  TargetOpModes = DefaultTargetOpModes;
    OpModeIdx = 0;
    TargetOpModesCount = 7;

    actAcc = maxAcc;
    actDec = maxDec;
    actSpeed = minSpeed;
}

/*-----------------------------------------------------------------------------
 * ConnectToMsgHandler(MsgHandler *Handler)
 * 
 * connect this instance of the MCTestCaycle to the MasHandler in charge
 *
 * 2024-07-21 AW
 *
 *---------------------------------------------------------------------------*/

void MCTestCycle::ConnectToMsgHandler(MsgHandler *Handler)
{
	aDrive.Connect2MsgHandler(Handler);
}


/*-----------------------------------------------------------------------------
 * EnableDebug()
 * 
 * enable debug messages via Serial
 *
 * 2024-07-20 AW
 *
 *---------------------------------------------------------------------------*/

void MCTestCycle::EnableDebug()
{
	printDebug = true;
}

/*-----------------------------------------------------------------------------
 * DisableDebug()
 * 
 * disable debug messages via Serial
 *
 * 2024-07-20 AW
 *
 *---------------------------------------------------------------------------*/

void MCTestCycle::DisableDebug()
{
  printDebug = false;
}

/*-----------------------------------------------------------------------------
 * PrintNodeState(uint16_t NodeState)
 * 
 * print this drives node state inline
 *
 * 2024-07-20 AW
 *
 *---------------------------------------------------------------------------*/

void MCTestCycle::PrintNodeState(uint16_t NodeState)
{
  switch(NodeState)
  {
    case eMCIdle:
      Serial.print("MCIdle");
      break;
    case eMCWaiting:
      Serial.print("MCWaiting");
      break;
    case eMCBusy:
      Serial.print("MCBusy");
      break;
    case eMCDone:
      Serial.print("MCDone");
      break;
    case eMCError:
      Serial.print("MCError");
      break;
    case eMCTimeout:
      Serial.print("MCTimeout");
      break;
    default:
      Serial.print("unknown");
      break;
  }
}

/*-----------------------------------------------------------------------------
 * PrintlnNodeState(uint16_t NodeState)
 * 
 * print this drives node state and end the line
 *
 * 2024-07-20 AW
 *
 *---------------------------------------------------------------------------*/

void MCTestCycle::PrintlnNodeState(uint16_t NodeState)
{
  switch(NodeState)
  {
    case eMCIdle:
      Serial.println("MCIdle");
      break;
    case eMCWaiting:
      Serial.println("MCWaiting");
      break;
    case eMCBusy:
      Serial.println("MCBusy");
      break;
    case eMCDone:
      Serial.println("MCDone");
      break;
    case eMCError:
      Serial.println("MCError");
      break;
    case eMCTimeout:
      Serial.println("MCTimeout");
      break;
    default:
      Serial.println("unknown");
      break;
  }
}


/*-----------------------------------------------------------------------------
 * ResetTurns()
 * 
 * reset the counter
 *
 * 2024-07-20 AW
 *
 *---------------------------------------------------------------------------*/

void MCTestCycle::ResetTurns()
{
	Turns = 0;
}

/*-----------------------------------------------------------------------------
 * GetTurns()
 * 
 * return the number of cycles since the last reset
 *
 * 2024-07-20 AW
 *
 *---------------------------------------------------------------------------*/

uint32_t MCTestCycle::GetTurns()
{
	return Turns;
}

/*-----------------------------------------------------------------------------
 * MCTestCycle(uint32_t currentMillis)
 * 
 * update the cycle and report the step counter
 *
 * 2024-07-14 AW
 * 2024-07-20 AW add a return value
 *
 *---------------------------------------------------------------------------*/

uint8_t MCTestCycle::DoCycle(uint32_t currentMillis)
{	
   aDrive.SetActTime(currentMillis);
   //check the status of the MsgHandler here too

   switch(driveStep)
   {
      case 0:
        //first get a copy of the drive status
        if((aDrive.UpdateDriveStatus()) == eMCDone)
        {
					if(printDebug)
					{
            Serial.println("------------------------------------------------");
					  Serial.print(Name);
            Serial.print(" : Nodestate updated - OpMode: ");
            Serial.print(aDrive.GetOpMode(),DEC);
            Serial.print(" SW: ");
            Serial.println(aDrive.GetSW(),HEX);
            Serial.println("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");

            Serial.print(Name);
            Serial.println(" :  -->1 Disable");
					}
          driveStep = 1;

          #if AutoReset
          aDrive.ResetComState();
          #endif
        }
        break;
      case 1:
        //disable the drive first
        if((aDrive.DisableDrive()) == eMCDone)
        {
					if(printDebug)
					{
            Serial.print(Name);
            Serial.println(" : -->2");
					}
          driveStep = 2;

          #if AutoReset
          aDrive.ResetComState();
          #endif
        }
        break;
      case 2:
        //first get a copy of the drive status
        if((aDrive.UpdateDriveStatus()) == eMCDone)
        {
					
					if(printDebug)
					{
            Serial.print(Name);
            Serial.print(" : Nodestate updated - OpMode: ");
            Serial.print(aDrive.GetOpMode(),DEC);
            Serial.print(" SW: ");
            Serial.println(aDrive.GetSW(),HEX);
            
            Serial.print(Name);
            Serial.println(" : -->3 wait");
					}
          driveStep = 3;

          stepTime = currentMillis;
 
          #if AutoReset
          aDrive.ResetComState();
          #endif
        }
        break;
      case 3:
        //wait some time
       if(currentMillis > (stepTime + 2000))
       {
					if(printDebug)
					{
            Serial.print(Name);
            Serial.println(" : -->4: Enable");
					}
          driveStep = 4;
       }
       break;
      case 4:
        //enable next
        if((aDrive.EnableDrive()) == eMCDone)
        {
					if(printDebug)
					{
            Serial.print(Name);
            Serial.println(" : -->5:");
					}
          driveStep = 5;

          #if AutoReset
          aDrive.ResetComState();
          #endif
        }
        break;
      case 5:
        //first get a copy of the drive status
        if((aDrive.UpdateDriveStatus()) == eMCDone)
        {
					if(printDebug)
					{
            Serial.print(Name);
            Serial.print(" : Nodestate updated - OpMode: ");
            Serial.print(aDrive.GetOpMode(),DEC);
            Serial.print(" SW: ");
            Serial.println(aDrive.GetSW(),HEX);

						Serial.print(Name);
            Serial.print(" : -->6 wait");
					}
          driveStep = 6;

          stepTime = currentMillis;
          incrementTime = currentMillis;

          #if AutoReset
          aDrive.ResetComState();
          #endif
        }
        break;
      case 6:
        //wait some time
       if(currentMillis > (stepTime + 2000))
       {
					if(printDebug)
					{
            Serial.print(Name);
            Serial.println(" : -->7: SetOpMode");
					}
          driveStep = 7;

          #if AutoReset
          aDrive.ResetComState();
          #endif
       }
       else if(currentMillis > (incrementTime + 100))
       {
					if(printDebug)
					{
            Serial.print(".");
					}
          incrementTime = currentMillis;
       }
       break;
      case 7:
        if((aDrive.SetOpMode(TargetOpModes[OpModeIdx])) == eMCDone)
      	{
					if(printDebug)
					{
            Serial.print(Name);
            Serial.print(" : OpMode set to :");
            Serial.println(TargetOpModes[OpModeIdx]);
            OpModeIdx++;
            if(OpModeIdx == TargetOpModesCount)
            {
              OpModeIdx = 0;
              Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
            }

            Serial.print(Name);
            Serial.println(" : -->8: try homing");
					}
					driveStep = 8;

          #if AutoReset
          aDrive.ResetComState();
          #endif
        }
        break;
      case 8:
        //config homing
        if((aDrive.ConfigureHoming(DriveAHomingMethod)) == eMCDone)
        {
					if(printDebug)
					{
            Serial.print(Name);
            Serial.println(" : -->9: Do Homing");          
					}
          driveStep = 9;

          StepRetries = 0;
          stepTime = currentMillis;
          incrementTime = currentMillis;

          #if AutoReset
          aDrive.ResetComState();
          #endif
					}
        break;
      case 9:
        //start homing
        //configuration is to already be enabled
        if((aDrive.DoHoming(0)) == eMCDone)
        {
					if(printDebug)
					{
            Serial.print(Name);
            Serial.println(" : -->10: PV 100");
					}
          driveStep = 10;

          #if AutoReset
          aDrive.ResetComState();
          #endif
					}
        else
        {
          StepRetries++;
          //print a line
          if(currentMillis > (incrementTime + 100))
          {
					  if(printDebug)
					  {
              //Serial.print(".");
              Serial.print("SW access state : ");
              Serial.print(aDrive.GetCWAccess());
              Serial.print(" SDO access: ");
              Serial.print(aDrive.GetSDOState());
              Serial.print(" SW: ");
              Serial.println(aDrive.GetSW(),HEX);
						}
            incrementTime = currentMillis;
          }
          if(StepRetries > MaxRetries)
          {
					  if(printDebug)
					  {
              Serial.print(Name);
              Serial.println(" : -->8: Restart Homing");          
						}
						driveStep = 8;
						
            #if AutoReset
            aDrive.ResetComState();
            #endif
          }
        }     
        break; 
      case 10:
        //move at speed
        if((aDrive.MoveAtSpeed(100)) == eMCDone)
        {
					if(printDebug)
					{
            Serial.print(Name);
            Serial.println(" : -->11 delay");
					}
					driveStep = 11;

          stepTime = currentMillis;
          incrementTime = currentMillis;

          #if AutoReset
          aDrive.ResetComState();
          #endif
        }
        break;
      case 11:
        //wait some time
       if(currentMillis > (stepTime + 2000))
       {
					if(printDebug)
					{
            Serial.print(Name);
            Serial.println(" : -->12: PV-100");
					}
					driveStep = 12;
       }
       else if(currentMillis > (incrementTime + 100))
       {
					if(printDebug)
					{
            Serial.print(".");
					}
          incrementTime = currentMillis;
       }
       break;
      case 12:
        //move at speed
        if((aDrive.MoveAtSpeed(-100)) == eMCDone)
        {
					if(printDebug)
					{
            Serial.print(Name);
            Serial.println(" n: -->13: delay");
					}
					driveStep = 13;

          stepTime = currentMillis;
          incrementTime = currentMillis;

          #if AutoReset
          aDrive.ResetComState();
          #endif
        }
        break;
      case 13:
        //wait some time
        if(currentMillis > (stepTime + 2000))
        {
					if(printDebug)
					{
            Serial.print(Name);
            Serial.println(" : -->14: PP@50000");
					}
          driveStep = 14;
        }
        else if(currentMillis > (incrementTime + 100))
        {
					if(printDebug)
					{
            Serial.print(".");
					}
          incrementTime = currentMillis;
        }
        break;
      case 14:
        //move to 0
        if((aDrive.StartAbsMove(50000,false)) == eMCDone)
        {
					if(printDebug)
					{
            Serial.print(Name);
            Serial.println(" : -->15 - wait for bein there");
					}
					driveStep = 15;
          StepRetries = 0;
          
					#if AutoReset
          aDrive.ResetComState();
          #endif
        }
        break;
      case 15:
        //wait for pos
        if(aDrive.IsInPos() == eMCDone)
        {
					if(printDebug)
					{
            Serial.print(Name);
            Serial.println(" : -->16: PP@0");
					}
					driveStep = 16;
          
					#if AutoReset
          aDrive.ResetComState();
          #endif
        }
        else
        {
          StepRetries++;
          if(StepRetries > MaxRetries)
          {
					  if(printDebug)
					  {
              Serial.print(Name);
              Serial.println(" n: -->14: Restart AbsMove");          
					  }
					  driveStep = 14;
            
						#if AutoReset
            aDrive.ResetComState();
            #endif
          }
        }     
        break;
      case 16:
        //move to 0
        if((aDrive.StartAbsMove(0,false)) == eMCDone)
        {
					if(printDebug)
					{
            Serial.print(Name);
            Serial.println(" n: -->17 - wait for being there");
          }
					
					driveStep = 17;
          
					StepRetries = 0;

          #if AutoReset
          aDrive.ResetComState();
          #endif
        }
        break;
      case 17:
        if(aDrive.IsInPos() == eMCDone)
        {
					if(printDebug)
					{
            Serial.print(Name);
            Serial.println(" : -->18 - change profile");
					}
          driveStep = 18;
          
					//#if AutoReset
          aDrive.ResetComState();
          //#endif
        }
        else
        {
          StepRetries++;
          if(StepRetries > MaxRetries)
          {
					  if(printDebug)
					  {
              Serial.print(Name);
              Serial.println(" n: -->16: Restart AbsMove");          
						}
						driveStep = 16;
            
						#if AutoReset
            aDrive.ResetComState();
            #endif
          }
        }     
        break;
      case 18:
        if((aDrive.SetProfile(actAcc,actDec,actSpeed,0)) == eMCDone)
        {
					if(printDebug)
					{
            Serial.print(Name);
            Serial.print(" : Loop -->0 @");
            Serial.println(actSpeed, DEC);
          }  
					driveStep = 0;
					Turns++;
          
					actSpeed += deltaSpeed;
          
					if((actSpeed <= minSpeed) || (actSpeed >= maxSpeed))
            deltaSpeed = deltaSpeed * (-1);
          
				  #if AutoReset
          aDrive.ResetComState();
          #endif
        }
        break;
   }
	 return driveStep;
 }

/*-----------------------------------------------------------------------------
 * ResetComState()
 * test and reset the ComState of this MCDrive is necessary
 *
 * 2024-07-20 AW
 *
 *---------------------------------------------------------------------------*/

void MCTestCycle::ResetComState()
{
	
	DriveCommStates NodeState = aDrive.CheckComState();
	
  if((NodeState == eMCError) || (NodeState == eMCTimeout))
  {
    Serial.print(Name);
    Serial.print(" : : Nodestate == ");
    PrintlnNodeState(NodeState);

    Serial.print(Name);
    Serial.println(" : Reset Node State");
    aDrive.ResetComState();
    //should be avoided in the end
    driveStep = 0;
   }
}
