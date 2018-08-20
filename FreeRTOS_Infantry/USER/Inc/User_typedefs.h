#ifndef __USER_TYPEDEFS_H
#define __USER_TYPEDEFS_H

#include <string.h>
#include "Driver_PID.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#pragma anon_unions


typedef enum
{
	COMMON	= 1,//一般模式
    SUPPLY 	= 2,//补给模式
 HIEROGRAM 	= 3,//神符模式
}Game_Mode_State;

typedef enum
{
	Remote_1	= 1,//遥控模式(跟随云台)
    Remote_2 	= 2,//遥控模式(不跟随云台)
    Keyboard 	= 3,//键鼠模式
	Ctrl_OFF	= 4,//失能控制
}Ctrl_Mode_State;

typedef enum
{
	UP		    = 1,//加速
    Down 	    = 2,//减速
    Normal   	= 3,//正常
}Speed_Mode_State;

typedef struct 
{
	 pid_t	     Out;
     pid_t		 In;

}PID_Struct;
typedef struct 
{
	 uint32_t        FrameRate;
      uint8_t        Offline;
}FrameRate_Struct;

typedef struct 
{
	    float        Acc_X;
		float        Acc_Y;
		float        Acc_Z;
	
		float        Mag_X;
		float        Mag_Y;
		float        Mag_Z;
	
	  int16_t        Gyr_X;
	  int16_t        Gyr_Y;
	  int16_t        Gyr_Z;
	
		float        Yaw;
		float        Roll;
		float        Pitch;
	
		float		 Q[4];
	
		float        Target_Yaw;
		float        Target_Roll;
		float        Target_Pitch;
		
		pid_t	     Chassis_PID;
   PID_Struct		 Yaw_PID;
   PID_Struct		 Pitch_PID;

	 uint32_t        FrameRate;
      uint8_t        Offline;
		
}Gyro_GY955_Struct;


typedef struct
{
		 int16_t		X;
		 int16_t		Y;
		uint16_t        Angle;
		 int16_t        Distance[6];
		uint16_t        Error;
		uint32_t        FrameRate;
         uint8_t        Offline;

}UWB_Struct;


typedef struct 
{
		uint16_t 		Real_Angle;	
		 int16_t 		Real_Current;			
		 int16_t 		Real_Speed;			 
    	
		uint16_t 		Target_Angle;	
		 int16_t 		Target_Current;	
		 int16_t		Target_Speed;		
		uint32_t        IntegralLimit;		
				
	       pid_t		PID;
		uint32_t        FrameRate;
         uint8_t        Offline;
	
}Motor3508_Param;

typedef struct
{
		uint16_t 		Real_Angle;	
		 int16_t 		Real_Current;				

		 int16_t 		Target_Angle;	
		 int16_t 		Target_Current;		
		uint32_t        IntegralLimit;		

	    uint32_t        FrameRate;
         uint8_t        Offline;
	  PID_Struct		PID;
	  
}Motor6623_Param;

typedef struct 
{
		
	    uint16_t 		Real_Angle;	
		 int16_t 		Real_Current;			
		 int16_t 		Real_Speed;	
	
		 int16_t   		Last_Angle;
		 int16_t   		Now_Angle; 
		 int16_t   		Error_Angle;
         int32_t   		All_error_Angle;
	
		uint32_t 		Target_Angle;	
		 int16_t 		Target_Current;		
		 int16_t		Target_Speed;			
		uint32_t        IntegralLimit;		

	  PID_Struct		PID;
	  
		uint32_t        FrameRate;
         uint8_t        Offline;   
	
}Motor2006_Param;



typedef struct
{
    Motor3508_Param 	LF;
    Motor3508_Param 	RF;
    Motor3508_Param	 	LB;
    Motor3508_Param		RB;
	
	Gyro_GY955_Struct	Chassis_Gyro;
    float 				TargetVX;
    float			    TargetVY;
    float 				TargetOmega;		
    float 				TargetABSAngle;	
}ChassisParam_Struct;



typedef struct
{
    Motor6623_Param		Pitch;
	Motor6623_Param		Yaw;
	Gyro_GY955_Struct	Cloud_Gyro;
}CloudParam_Struct;

typedef struct
{
	union 
	{
		uint8_t dataBuff[8];
		struct
		{
			float ChassisVolt;
			float ChassisCurrent;
			float ChassisPower;
			float ChassisPowerBuffer;
		};
	}data;
	
}Current_Meter_Struct;

typedef struct
{
	 uint8_t	 Flag;//射击标志
	uint16_t	 Time;//连发时间
	 uint8_t	 Freq;//连发频率
	 uint8_t	  Num;//可发数量

}Shoot_Struct;


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


#endif
