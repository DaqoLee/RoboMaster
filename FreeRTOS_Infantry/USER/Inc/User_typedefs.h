#ifndef __USER_TYPEDEFS_H
#define __USER_TYPEDEFS_H

#include <string.h>
#include "Driver_PID.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#pragma anon_unions


typedef enum
{
	COMMON	= 1,//һ��ģʽ
    SUPPLY 	= 2,//����ģʽ
 HIEROGRAM 	= 3,//���ģʽ
}Game_Mode_State;

typedef enum
{
	Remote_1	= 1,//ң��ģʽ(������̨)
    Remote_2 	= 2,//ң��ģʽ(��������̨)
    Keyboard 	= 3,//����ģʽ
	Ctrl_OFF	= 4,//ʧ�ܿ���
}Ctrl_Mode_State;

typedef enum
{
	UP		    = 1,//����
    Down 	    = 2,//����
    Normal   	= 3,//����
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
	 uint8_t	 Flag;//�����־
	uint16_t	 Time;//����ʱ��
	 uint8_t	 Freq;//����Ƶ��
	 uint8_t	  Num;//�ɷ�����

}Shoot_Struct;


//����������״̬��Ƶ�� 10Hz ���� CmdID 0x0001
typedef struct{
	union {
		uint8_t dataBuff[8];
		struct{
			uint16_t gameRemianTime;		//��ǰ�׶�ʣ��ʱ�䣬��λ s
			uint8_t gameStatus;				//��ǰ����״̬
			uint8_t robotLevel;				//�����˵�ǰ�ȼ�
			uint16_t remainHP;				//�����˵�ǰѪ��
			uint16_t maxHP;					//��������Ѫ��
		};
	}data;
	uint8_t infoUpdateFlag;
}Judge_RobotStatus_t;


//�˺����ݣ��ܵ������˺�ʵʱ���� CmdID 0x0002
typedef struct{
	union {
		uint8_t dataBuff[1];
		struct{
			uint8_t armorType : 4;		//�ܵ��˺���װ�װ�ID��
			uint8_t hurtType : 4;		//�˺�����
		};
	}data;
	uint8_t infoUpdateFlag;
}Judge_RobotHurt_t;


//ʵʱ������� CmdID 0x0003
typedef struct{
	union {
		uint8_t dataBuff[6];
		__packed struct{
			uint8_t bulletType;		//��������
			uint8_t bulletFreq;		//������Ƶ����λ����ÿ��
			float bulletSpeed;		//�������٣���λ����ÿ��
		};
	}data;
	uint8_t infoUpdateFlag;
}Judge_ShootData_t;

//ʵʱ���ʺ��������ݣ�50Hz Ƶ�����ڷ��� CmdID 0x0004
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


//ʵʱ���ؽ������ݣ���⵽ IC ��ʱ��10Hz Ƶ������ CmdID 0x0005
typedef struct{
	union {
		uint8_t dataBuff[2];
		struct{
			uint8_t cardType;		//������
			uint8_t cardIndex;		//�������ţ����������ֲ�ͬ����
		};
	}data;
	uint8_t infoUpdateFlag;
}Judge_RFIDCard_t;


//����������ݣ���������ʱ���� CmdID 0x0006
typedef struct{
	uint8_t winner;			//�������
}Judge_GameResult_t;


//��� buff CmdID 0x0007
typedef struct{
	union {
		uint8_t dataBuff[2];
		struct{
			uint8_t buffType;		//Buff ����
			uint8_t buffAddition;	//�ӳɰٷֱȣ����� 10 ����ӳ� 10%��
		};
	}data;
	uint8_t infoUpdateFlag;
}Judge_GetBuff_t;

//������λ�ú�ǹ�ڳ�����Ϣ��50Hz Ƶ�����ڷ��� CmdID 0x0008
typedef struct{
	union {
		uint8_t dataBuff[16];
		struct{
			float x;			//xλ������
			float y;			//yλ������
			float z;			//zλ������
			float gunYaw;		//ǹ�ڳ���Ƕ�
		};
	}data;
	uint8_t infoUpdateFlag;
}Judge_RobotPos_t;

//�������Զ������ݣ�������ʾ�ڲ������棬��Ƶ 10Hz CmdID 0x0100
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
