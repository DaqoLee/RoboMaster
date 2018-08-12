#ifndef __DRIVER_JUDGE_H
#define __DRIVER_JUDGE_H
#include <string.h>
#include "stm32f4xx.h"
#pragma anon_unions

#define JUDGESYSTEM_PACKSIZE 		150u		//����ϵͳ����С
#define JUDGESYSTEM_FRAMEHEADER    	0xA5        //֡ͷ

#define JudgeInfoType_RobotStatus     0x0001
#define JudgeInfoType_RobotHurt 	  0x0002
#define JudgeInfoType_ShootData       0x0003
#define JudgeInfoType_PowerHeatData   0x0004
#define JudgeInfoType_RFIDCard        0x0005
#define JudgeInfoType_GameResult      0x0006
#define JudgeInfoType_GetBuff         0x0007
#define JudgeInfoType_RobotPos        0x0008
#define JudgeInfoType_JudgeSendData   0x0100

//��������֡�ĳ��ȣ�FrameHeader(5-Byte)+CmdID(2-Byte)+Data(n-Byte)+FrameTail(2-Byte, CRC16, ����У��)
#define JudgeInfoLength_RobotStatus     17
#define JudgeInfoLength_RobotHurt 	    10
#define JudgeInfoLength_ShootData       15
#define JudgeInfoLength_PowerHeatData   29
#define JudgeInfoLength_RFIDCard        11
#define JudgeInfoLength_GameResult      11
#define JudgeInfoLength_GetBuff         12
#define JudgeInfoLength_RobotPos        25
#define JudgeInfoLength_JudgeSendData   22


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
