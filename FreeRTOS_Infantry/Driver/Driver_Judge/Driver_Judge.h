#ifndef __DRIVER_JUDGE_H
#define __DRIVER_JUDGE_H
#include <string.h>
#include "stm32f4xx.h"
#pragma anon_unions

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


//比赛机器人状态，频率 10Hz 推送 CmdID 0x0001
typedef struct{
	union {
		uint8_t dataBuff[8];
		struct{
			uint16_t gameRemianTime;		//当前阶段剩余时间，单位 s
			uint8_t gameStatus;				//当前比赛状态
			uint8_t robotLevel;				//机器人当前等级
			uint16_t remainHP;				//机器人当前血量
			uint16_t maxHP;					//机器人满血量
		};
	}data;
	uint8_t infoUpdateFlag;
}Judge_RobotStatus_t;


//伤害数据，受到攻击伤害实时推送 CmdID 0x0002
typedef struct{
	union {
		uint8_t dataBuff[1];
		struct{
			uint8_t armorType : 4;		//受到伤害的装甲板ID号
			uint8_t hurtType : 4;		//伤害类型
		};
	}data;
	uint8_t infoUpdateFlag;
}Judge_RobotHurt_t;


//实时射击数据 CmdID 0x0003
typedef struct{
	union {
		uint8_t dataBuff[6];
		__packed struct{
			uint8_t bulletType;		//弹丸类型
			uint8_t bulletFreq;		//弹丸射频，单位：发每秒
			float bulletSpeed;		//弹丸射速，单位：米每秒
		};
	}data;
	uint8_t infoUpdateFlag;
}Judge_ShootData_t;

//实时功率和热量数据，50Hz 频率周期发送 CmdID 0x0004
typedef struct{
	union {
		uint8_t dataBuff[20];
		struct{
			float chassisVolt;
			float chassisCurrent;
			float chassisPower;
			float chassisPowerBuffer;
			uint16_t shooter17mm_Heat;
			uint16_t shooter42mm_Heat;
		};
	}data;
	uint8_t infoUpdateFlag;
}Judge_PowerHeatData_t;


//实时场地交互数据，检测到 IC 卡时，10Hz 频率推送 CmdID 0x0005
typedef struct{
	union {
		uint8_t dataBuff[2];
		struct{
			uint8_t cardType;		//卡类型
			uint8_t cardIndex;		//卡索引号，可用于区分不同区域
		};
	}data;
	uint8_t infoUpdateFlag;
}Judge_RFIDCard_t;


//比赛结果数据，比赛结束时推送 CmdID 0x0006
typedef struct{
	uint8_t winner;			//比赛结果
}Judge_GameResult_t;


//获得 buff CmdID 0x0007
typedef struct{
	union {
		uint8_t dataBuff[2];
		struct{
			uint8_t buffType;		//Buff 类型
			uint8_t buffAddition;	//加成百分比（比如 10 代表加成 10%）
		};
	}data;
	uint8_t infoUpdateFlag;
}Judge_GetBuff_t;

//机器人位置和枪口朝向信息，50Hz 频率周期发送 CmdID 0x0008
typedef struct{
	union {
		uint8_t dataBuff[16];
		struct{
			float x;			//x位置坐标
			float y;			//y位置坐标
			float z;			//z位置坐标
			float gunYaw;		//枪口朝向角度
		};
	}data;
	uint8_t infoUpdateFlag;
}Judge_RobotPos_t;

//参赛队自定义数据，用于显示在操作界面，限频 10Hz CmdID 0x0100
typedef struct{
	union {
		uint8_t dataBuff[13];
		struct{
			float data1;
			float data2;
			float data3;
			uint8_t mask;
		};
	}data;
	uint8_t infoUpdateFlag;
	
}Judge_SendData_t;




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
