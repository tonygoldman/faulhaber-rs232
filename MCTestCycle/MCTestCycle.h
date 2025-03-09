#ifndef MC_TESTCYCLE_H
#define MC_TESTCYCLE_H

/*-----------------------------------------------------
 * implements a clas which uses a single instance of an MCDrive
 * o test the communication implemented there
 *
 * 2024-07-14 AW
 *
 *-----------------------------------------------------*/
 
//---includes ---

#include <stdint.h>

#include <MCNode.h>
#include <MCDrive.h>

const uint32_t maxAcc = 2500;
const uint32_t minAccDec = 250;
const uint32_t maxDec = 2500;
const uint32_t maxSpeed = 3000;
const uint32_t minSpeed = 500;
const uint16_t MaxRetries = 200;


class MCTestCycle {
	public:
		MCTestCycle(uint8_t, char *, int8_t);
	
	  void ConnectToMsgHandler(MsgHandler *);
	
	  uint8_t DoCycle(uint32_t);
    void EnableDebug();
	  void DisableDebug();
	
	  void PrintNodeState(uint16_t);
    void PrintlnNodeState(uint16_t);

	  void ResetComState();
	
	  uint32_t GetTurns();
	  void ResetTurns();
	
		int8_t DriveAHomingMethod;
		int8_t *TargetOpModes;
	
    uint32_t deltaSpeed;
    uint32_t deltaAccDec;
	
	private:
  	MCDrive aDrive;

    char *Name;
	
	  uint8_t driveStep;
  	uint16_t StepRetries;
    uint32_t stepTime;

	  uint32_t Turns = 0;  
	  bool printDebug = false;
    
    uint8_t OpModeIdx;
    uint8_t TargetOpModesCount;

	  uint32_t incrementTime;
    uint32_t actAcc;
    uint32_t actDec;
    uint32_t actSpeed;
};

#endif