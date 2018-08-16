#ifndef __DRIVER_JUDGE_H
#define __DRIVER_JUDGE_H
#include "User_typedefs.h"

#define JUDGESYSTEM_PACKSIZE 		150u		//裁判系统包大小
#define JUDGESYSTEM_FRAMEHEADER    	0xA5        //帧头

#define JudgeInfoType_RobotStatus     0x0001
#define JudgeInfoType_RobotHurt 	  0x0002
#define JudgeInfoType_ShootData       0x0003
#define JudgeInfoType_PowerHeatData   0x0004
#define JudgeInfoType_RFIDCard        0x0005
#define JudgeInfoType_GameResult      0x0006
#define JudgeInfoType_GetBuff         0x0007
#define JudgeInfoType_RobotPos        0x0008
#define JudgeInfoType_JudgeSendData   0x0100

//整个数据帧的长度，FrameHeader(5-Byte)+CmdID(2-Byte)+Data(n-Byte)+FrameTail(2-Byte, CRC16, 整包校验)
#define JudgeInfoLength_RobotStatus     17
#define JudgeInfoLength_RobotHurt 	    10
#define JudgeInfoLength_ShootData       15
#define JudgeInfoLength_PowerHeatData   29
#define JudgeInfoLength_RFIDCard        11
#define JudgeInfoLength_GameResult      11
#define JudgeInfoLength_GetBuff         12
#define JudgeInfoLength_RobotPos        25
#define JudgeInfoLength_JudgeSendData   22







extern Judge_RobotStatus_t Judge_RobotStatus;
extern Judge_RobotHurt_t Judge_RobotHurt;
extern Judge_ShootData_t Judge_ShootData;
extern Judge_PowerHeatData_t Judge_PowerHeatData;
extern Judge_RFIDCard_t Judge_RFIDCard;
extern Judge_GameResult_t Judge_GameResult;
extern Judge_GetBuff_t Judge_GetBuff;
extern Judge_RobotPos_t Judge_RobotPos;
extern Judge_SendData_t Judge_SendData;

void JudgeSystem_Init(void);
void Judge_getInfo(uint16_t dataLength);
void Judge_sendInfo(void);


#endif
