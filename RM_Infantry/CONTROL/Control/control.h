#ifndef __CONTROL_H
#define __CONTROL_H
#include "stm32f4xx_hal.h"
#include "Driver_PID.h"
#include "Driver_Judge.h"
#include "Driver_DBUS.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#pragma anon_unions

#define INFANTRY          4    //1�ź�4�Ų����в���

#if  INFANTRY==4
#define MEDIAN_PITCH      2400 //Pith���м��е�Ƕ�
#define MEDIAN_YAW        4000 //Yaw���м��е�Ƕ�
#define MEDIAN_ROLL				10
#elif INFANTRY==1
#define MEDIAN_PITCH      4000 //Pith���м��е�Ƕ�
#define MEDIAN_YAW        4400 //Yaw���м��е�Ƕ�
#define MEDIAN_ROLL				10
#endif
#define PITCH_RANGE		   28  		//Pitch�Ƕȷ�Χ
#define YAW_RANGE		   180		//Yaw�Ƕȷ�Χ
#define	YAW_GYRO_RANGE	   200      //�����ǽǶ��޷�

#define RAMMER_NUM		   8		//���̿�����
#define REDUCTION_RATIO	   (36/1)	//����������ٱ�
#define RAMMER_ERROR	   16000	//����תһȦ��ƫ��

#define M2006_Xianfu  8000
#define M3508_Xianfu  8000
#define M6623_Xianfu  6000


enum{
	COMMON	= 1,//һ��ģʽ
    SUPPLY 	= 2,//����ģʽ
 HIEROGRAM 	= 3,//���ģʽ
};

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
	   uint32_t         IntegralLimit;		
	      pid_t			PID;
	      float			KP;					
		  float         KI;					
	      float			KD;					   
	
          float         radian;
		  float         angle;
	
		  float 		Target_angle; 
		  float 		Target_radian;
	 uint32_t        FrameRate;
      uint8_t        Offline;	
	
}Gyro_Struct;

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
	
		   float		KP;					
		   float        KI;					
	       float		KD;					
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
  
	       float		KP;					
		   float        KI;					
	       float		KD;					
	
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
    
		   float		Angle_KP;					
		   float        Angle_KI;					
		   float		Angle_KD;					
		
		   float		Speed_KP;					
		   float        Speed_KI;					
		   float		Speed_KD;	

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
	    Gyro_Struct     Gyro;

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


extern Current_Meter_Struct   Current_Meter;
extern ChassisParam_Struct   ChassisParam;
extern CloudParam_Struct	  CloudParam;
extern Motor2006_Param		  M2006;
extern FrameRate_Struct Judge;
extern FrameRate_Struct Remote;
extern FrameRate_Struct Cur_Meter;
extern Shoot_Struct Shoot;
extern	uint8_t Mode;
extern	float Mou_x,Mou_y;
extern uint16_t 	First_Yaw_Angle,First_Pitch_Angle;
extern uint8_t	Flag1_1Hz, Flag1_10Hz, Flag1_20Hz, Flag1_50Hz,
				Flag2_1Hz, Flag2_10Hz, Flag2_20Hz, Flag2_50Hz,
				Flag3_1Hz, Flag3_10Hz, Flag3_20Hz, Flag3_50Hz;
void Mode_Set(void);
void Frame_Rate_Statistics(void);
void Filter(int16_t Val,float *Value,float Rate);
#endif

