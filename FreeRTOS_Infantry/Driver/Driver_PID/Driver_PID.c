#include "Driver_PID.h"
#include "User_Code.h"

#define NB 0
#define NM 1
#define NS 2
#define ZO 3
#define PS 4
#define PM 5
#define PB 6

float E_Rule[7]={-60,-40,-20,0,20,40,60};
float EC_Rule[7]={-15,-10,-5,0,5,10,15};
float KP_Rule[7]={10,15,25,30,0,0,0};
float KD_Rule[7]={5,10,15,20,0,0,0};
uint8_t KP[7][7]=
//{
//	{PB,PB,PB,PB,PM,ZO,ZO},
//	{PB,PB,PB,PM,PM,ZO,ZO},
//	{PB,PM,PM,PS,ZO,NS,NM},
//	{PM,PM,PS,ZO,NS,NM,NM},
//	{PS,PS,ZO,NM,NM,NM,NB},
//	{ZO,ZO,ZO,NM,NB,NB,NB},
//	{ZO,NS,NB,NB,NB,NB,NB}
//};
{
	{3,3,3,3,3,3,3},
	{2,2,2,2,1,2,2},
	{1,1,1,1,1,1,1},
	{1,1,0,1,0,1,1},
	{0,0,1,0,0,1,0},
	{0,1,0,1,0,0,2},
	{3,3,3,3,3,3,3}
};

uint8_t KD[7][7]=
//{
//	{PB,PB,PB,PB,PM,ZO,ZO},
//	{PB,PB,PB,PM,PM,ZO,ZO},
//	{PB,PM,PM,PS,ZO,NS,NM},
//	{PM,PM,PS,ZO,NS,NM,NM},
//	{PS,PS,ZO,NM,NM,NM,NB},
//	{ZO,ZO,ZO,NM,NB,NB,NB},
//	{ZO,NS,NB,NB,NB,NB,NB}
//};
{
	{3,3,3,2,2,2,2},
	{2,2,2,1,1,1,1},
	{1,1,2,1,1,2,1},
	{1,1,0,1,0,1,1},
	{1,1,0,0,0,1,1},
	{2,2,1,0,1,1,1},
	{3,3,3,3,2,3,2}
};

void abs_limit(float *a, float ABS_MAX){
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}


/*参数初始化--------------------------------------------------------------*/
static void pid_param_init(
    pid_t *pid, 
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
    
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    	
}

/*中途更改参数设定(调试)------------------------------------------------------------*/

static void pid_reset(pid_t	*pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/*
 *@bref. calculate delta PID and position PID
 *@param[in] set： target
 *@param[in] real	measure
 */

float pid_calc(pid_t* pid, float get, float set)
{
		
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
		return 0;
	if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
		return 0;
    
    if(pid->pid_mode == POSITION_PID) //位置式p
    {
/************************************过零点时比较权重挑最近的路到目标值***********************************/
		if(pid==&CloudParam.Pitch.PID.Out||pid==&CloudParam.Yaw.PID.Out||pid==&ChassisParam.Chassis_Gyro.Chassis_PID)
		{
			if(pid->err[NOW]<0)
				pid->err[NOW]=ABS(pid->err[NOW])>ABS(8191-ABS(pid->err[NOW]))?8191-ABS(pid->err[NOW]):pid->err[NOW];
			else if(pid->err[NOW]>0)
				pid->err[NOW]=ABS(pid->err[NOW])>ABS(8191-ABS(pid->err[NOW]))?ABS(pid->err[NOW])-8191:pid->err[NOW];
	    }
		else if(pid==&M2006.PID.Out)
		{
			if(pid->err[NOW]<0)
				pid->err[NOW]=ABS(pid->err[NOW])>ABS(Rammer_Max_Angle-ABS(pid->err[NOW]))?Rammer_Max_Angle-ABS(pid->err[NOW]):pid->err[NOW];
			else if(pid->err[NOW]>0)
				pid->err[NOW]=ABS(pid->err[NOW])>ABS(Rammer_Max_Angle-ABS(pid->err[NOW]))?ABS(pid->err[NOW])-Rammer_Max_Angle:pid->err[NOW];
		}
		else if(pid==&CloudParam.Cloud_Gyro.Yaw_PID.Out)
		{
			if(pid->err[NOW]<0)
				pid->err[NOW]=ABS(pid->err[NOW])>ABS(360-ABS(pid->err[NOW]))?360-ABS(pid->err[NOW]):pid->err[NOW];
			else if(pid->err[NOW]>0)
				pid->err[NOW]=ABS(pid->err[NOW])>ABS(360-ABS(pid->err[NOW]))?ABS(pid->err[NOW])-360:pid->err[NOW];
		}
/*********************************************************************************************************/		
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
		
        abs_limit(&(pid->iout), pid->IntegralLimit);

		pid->pos_out = pid->pout + pid->iout + pid->dout;
			
        abs_limit(&(pid->pos_out), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
    }
    else if(pid->pid_mode == DELTA_PID)//增量式P
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
        
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
		
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;	//update last time
    }
    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
	
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
	
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
//	
}


/*
 *@bref. calculate delta PID and position PID
 *@param[in] set： target
 *@param[in] real	measure
 */

float fuzzy_pid_calc(pid_t* pid, float get, float set)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
	pid->erc[NOW] = pid->err[NOW]-pid->err[LAST];
	
	
    if(pid->pid_mode == POSITION_PID) //位置式p
    {
/************************************过零点时比较权重挑最近的路到目标值***********************************/
		if(pid==&CloudParam.Pitch.PID.Out||pid==&CloudParam.Yaw.PID.Out||pid==&ChassisParam.Chassis_Gyro.Chassis_PID)
		{
			if(pid->err[NOW]<0)
				pid->err[NOW]=ABS(pid->err[NOW])>ABS(8191-ABS(pid->err[NOW]))?8191-ABS(pid->err[NOW]):pid->err[NOW];
			else if(pid->err[NOW]>0)
				pid->err[NOW]=ABS(pid->err[NOW])>ABS(8191-ABS(pid->err[NOW]))?ABS(pid->err[NOW])-8191:pid->err[NOW];
	    }
		else if(pid==&M2006.PID.Out)
		{
			if(pid->err[NOW]<0)
				pid->err[NOW]=ABS(pid->err[NOW])>ABS(Rammer_Max_Angle-ABS(pid->err[NOW]))?Rammer_Max_Angle-ABS(pid->err[NOW]):pid->err[NOW];
			else if(pid->err[NOW]>0)
				pid->err[NOW]=ABS(pid->err[NOW])>ABS(Rammer_Max_Angle-ABS(pid->err[NOW]))?ABS(pid->err[NOW])-Rammer_Max_Angle:pid->err[NOW];
		}
		else if(pid==&CloudParam.Cloud_Gyro.Yaw_PID.Out)
		{
			if(pid->err[NOW]<0)
				pid->err[NOW]=ABS(pid->err[NOW])>ABS(360-ABS(pid->err[NOW]))?360-ABS(pid->err[NOW]):pid->err[NOW];
			else if(pid->err[NOW]>0)
				pid->err[NOW]=ABS(pid->err[NOW])>ABS(360-ABS(pid->err[NOW]))?ABS(pid->err[NOW])-360:pid->err[NOW];
		}
/*********************************************************************************************************/		
		Fuzzy_KP(pid);
		Fuzzy_KD(pid);
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
		
        abs_limit(&(pid->iout), pid->IntegralLimit);

		pid->pos_out = pid->pout + pid->iout + pid->dout;
			
        abs_limit(&(pid->pos_out), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
    }
    else if(pid->pid_mode == DELTA_PID)//增量式P
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
        
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
		
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;	//update last time
    }	
	
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
	
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
	
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];	
	
	return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
}

void Fuzzy_KP(pid_t* pid)
{
	static uint8_t Flag_E,Flag_EC,N;


	float E_Fuzzy[2]={0,0};
	float EC_Fuzzy[2]={0,0};
	float KP_Fuzzy[7]={0,0,0,0,0,0,0};
	
	if(pid->err[NOW]<E_Rule[0])
	{
		E_Fuzzy[0]=1.0;
		Flag_E=0;
	}
	else if(pid->err[NOW]>=E_Rule[0]&&pid->err[NOW]<E_Rule[1])
	{
		E_Fuzzy[0]=(E_Rule[1]-pid->err[NOW])/(E_Rule[1]-E_Rule[0]);
		Flag_E=0;
	}
	else if(pid->err[NOW]>=E_Rule[1]&&pid->err[NOW]<E_Rule[2])
	{
		E_Fuzzy[0]=(E_Rule[2]-pid->err[NOW])/(E_Rule[2]-E_Rule[1]);
		Flag_E=1;
	}
	else if(pid->err[NOW]>=E_Rule[2]&&pid->err[NOW]<E_Rule[3])
	{
		E_Fuzzy[0]=(E_Rule[3]-pid->err[NOW])/(E_Rule[3]-E_Rule[2]);
		Flag_E=2;
	}
	else if(pid->err[NOW]>=E_Rule[3]&&pid->err[NOW]<E_Rule[4])
	{
		E_Fuzzy[0]=(E_Rule[4]-pid->err[NOW])/(E_Rule[4]-E_Rule[3]);
		Flag_E=3;
	}
	else if(pid->err[NOW]>=E_Rule[4]&&pid->err[NOW]<E_Rule[5])
	{
		E_Fuzzy[0]=(E_Rule[5]-pid->err[NOW])/(E_Rule[5]-E_Rule[4]);
		Flag_E=4;
	}
	else if(pid->err[NOW]>=E_Rule[5]&&pid->err[NOW]<E_Rule[6])
	{
		E_Fuzzy[0]=(E_Rule[6]-pid->err[NOW])/(E_Rule[6]-E_Rule[5]);
		Flag_E=5;
	}
	else
	{
		E_Fuzzy[0]=0;
		Flag_E=5;
	}
	
	E_Fuzzy[1]=1.0f-E_Fuzzy[0];
	
	
	if(pid->erc[NOW]<EC_Rule[0])
	{
		EC_Fuzzy[0]=1.0;
		Flag_EC=0;
	}
	else if(pid->erc[NOW]>=EC_Rule[0]&&pid->erc[NOW]<EC_Rule[1])
	{
		EC_Fuzzy[0]=(EC_Rule[1]-pid->erc[NOW])/(EC_Rule[1]-EC_Rule[0]);
		Flag_EC=0;
	}
	else if(pid->erc[NOW]>=EC_Rule[1]&&pid->erc[NOW]<EC_Rule[2])
	{
		EC_Fuzzy[0]=(EC_Rule[2]-pid->erc[NOW])/(EC_Rule[2]-EC_Rule[1]);
		Flag_EC=1;
	}
	else if(pid->erc[NOW]>=EC_Rule[2]&&pid->erc[NOW]<EC_Rule[3])
	{
		EC_Fuzzy[0]=(EC_Rule[3]-pid->erc[NOW])/(EC_Rule[3]-EC_Rule[2]);
		Flag_EC=2;
	}
	else if(pid->erc[NOW]>=EC_Rule[3]&&pid->erc[NOW]<EC_Rule[4])
	{
		EC_Fuzzy[0]=(EC_Rule[4]-pid->erc[NOW])/(EC_Rule[4]-EC_Rule[3]);
		Flag_EC=3;
	}
	else if(pid->erc[NOW]>=EC_Rule[4]&&pid->erc[NOW]<EC_Rule[5])
	{
		EC_Fuzzy[0]=(EC_Rule[5]-pid->erc[NOW])/(EC_Rule[5]-EC_Rule[4]);
		Flag_EC=4;
	}
	else if(pid->erc[NOW]>=EC_Rule[5]&&pid->erc[NOW]<EC_Rule[6])
	{
		EC_Fuzzy[0]=(EC_Rule[6]-pid->erc[NOW])/(EC_Rule[6]-EC_Rule[5]);
		Flag_EC=5;
	}
	else
	{
		EC_Fuzzy[0]=0;
		Flag_EC=5;
	}
	
	EC_Fuzzy[1]=1.0f-EC_Fuzzy[0];	
	
	N=KP[Flag_E][Flag_EC];
	KP_Fuzzy[N]+=E_Fuzzy[0]*EC_Fuzzy[0];
	
	N=KP[Flag_E][Flag_EC+1];
	KP_Fuzzy[N]+=E_Fuzzy[0]*EC_Fuzzy[1];

	N=KP[Flag_E+1][Flag_EC];
	KP_Fuzzy[N]+=E_Fuzzy[1]*EC_Fuzzy[0];

	N=KP[Flag_E+1][Flag_EC+1];
	KP_Fuzzy[N]+=E_Fuzzy[1]*EC_Fuzzy[1];


	
	
	pid->p=(KP_Fuzzy[0]*KP_Rule[0]+KP_Fuzzy[1]*KP_Rule[1]+KP_Fuzzy[2]*KP_Rule[2]+KP_Fuzzy[3]*KP_Rule[3]+KP_Fuzzy[4]*KP_Rule[4]+KP_Fuzzy[5]*KP_Rule[5]+KP_Fuzzy[6]*KP_Rule[6]);
	
}

void Fuzzy_KD(pid_t* pid)
{
	static uint8_t Flag_E,Flag_EC,N;


	float E_Fuzzy[2]={0,0};
	float EC_Fuzzy[2]={0,0};
	float KD_Fuzzy[7]={0,0,0,0,0,0,0};
	
	if(pid->err[NOW]<E_Rule[0])
	{
		E_Fuzzy[0]=1.0;
		Flag_E=0;
	}
	else if(pid->err[NOW]>=E_Rule[0]&&pid->err[NOW]<E_Rule[1])
	{
		E_Fuzzy[0]=(E_Rule[1]-pid->err[NOW])/(E_Rule[1]-E_Rule[0]);
		Flag_E=0;
	}
	else if(pid->err[NOW]>=E_Rule[1]&&pid->err[NOW]<E_Rule[2])
	{
		E_Fuzzy[0]=(E_Rule[2]-pid->err[NOW])/(E_Rule[2]-E_Rule[1]);
		Flag_E=1;
	}
	else if(pid->err[NOW]>=E_Rule[2]&&pid->err[NOW]<E_Rule[3])
	{
		E_Fuzzy[0]=(E_Rule[3]-pid->err[NOW])/(E_Rule[3]-E_Rule[2]);
		Flag_E=2;
	}
	else if(pid->err[NOW]>=E_Rule[3]&&pid->err[NOW]<E_Rule[4])
	{
		E_Fuzzy[0]=(E_Rule[4]-pid->err[NOW])/(E_Rule[4]-E_Rule[3]);
		Flag_E=3;
	}
	else if(pid->err[NOW]>=E_Rule[4]&&pid->err[NOW]<E_Rule[5])
	{
		E_Fuzzy[0]=(E_Rule[5]-pid->err[NOW])/(E_Rule[5]-E_Rule[4]);
		Flag_E=4;
	}
	else if(pid->err[NOW]>=E_Rule[5]&&pid->err[NOW]<E_Rule[6])
	{
		E_Fuzzy[0]=(E_Rule[6]-pid->err[NOW])/(E_Rule[6]-E_Rule[5]);
		Flag_E=5;
	}
	else
	{
		E_Fuzzy[0]=0;
		Flag_E=5;
	}
	
	E_Fuzzy[1]=1.0f-E_Fuzzy[0];
	
	
	if(pid->erc[NOW]<EC_Rule[0])
	{
		EC_Fuzzy[0]=1.0;
		Flag_EC=0;
	}
	else if(pid->erc[NOW]>=EC_Rule[0]&&pid->erc[NOW]<EC_Rule[1])
	{
		EC_Fuzzy[0]=(EC_Rule[1]-pid->erc[NOW])/(EC_Rule[1]-EC_Rule[0]);
		Flag_EC=0;
	}
	else if(pid->erc[NOW]>=EC_Rule[1]&&pid->erc[NOW]<EC_Rule[2])
	{
		EC_Fuzzy[0]=(EC_Rule[2]-pid->erc[NOW])/(EC_Rule[2]-EC_Rule[1]);
		Flag_EC=1;
	}
	else if(pid->erc[NOW]>=EC_Rule[2]&&pid->erc[NOW]<EC_Rule[3])
	{
		EC_Fuzzy[0]=(EC_Rule[3]-pid->erc[NOW])/(EC_Rule[3]-EC_Rule[2]);
		Flag_EC=2;
	}
	else if(pid->erc[NOW]>=EC_Rule[3]&&pid->erc[NOW]<EC_Rule[4])
	{
		EC_Fuzzy[0]=(EC_Rule[4]-pid->erc[NOW])/(EC_Rule[4]-EC_Rule[3]);
		Flag_EC=3;
	}
	else if(pid->erc[NOW]>=EC_Rule[4]&&pid->erc[NOW]<EC_Rule[5])
	{
		EC_Fuzzy[0]=(EC_Rule[5]-pid->erc[NOW])/(EC_Rule[5]-EC_Rule[4]);
		Flag_EC=4;
	}
	else if(pid->erc[NOW]>=EC_Rule[5]&&pid->erc[NOW]<EC_Rule[6])
	{
		EC_Fuzzy[0]=(EC_Rule[6]-pid->erc[NOW])/(EC_Rule[6]-EC_Rule[5]);
		Flag_EC=5;
	}
	else
	{
		EC_Fuzzy[0]=0;
		Flag_EC=5;
	}
	
	EC_Fuzzy[1]=1.0f-EC_Fuzzy[0];	
	
	N=KP[Flag_E][Flag_EC];
	KD_Fuzzy[N]+=E_Fuzzy[0]*EC_Fuzzy[0];

	N=KP[Flag_E][Flag_EC+1];
	KD_Fuzzy[N]+=E_Fuzzy[0]*EC_Fuzzy[1];

	N=KP[Flag_E+1][Flag_EC];
	KD_Fuzzy[N]+=E_Fuzzy[1]*EC_Fuzzy[0];

	N=KP[Flag_E+1][Flag_EC+1];
	KD_Fuzzy[N]+=E_Fuzzy[1]*EC_Fuzzy[1];

	pid->d=(KD_Fuzzy[0]*KD_Rule[0]+KD_Fuzzy[1]*KD_Rule[1]+KD_Fuzzy[2]*KD_Rule[2]+KD_Fuzzy[3]*KD_Rule[3]+KD_Fuzzy[4]*KD_Rule[4]+KD_Fuzzy[5]*KD_Rule[5]+KD_Fuzzy[6]*KD_Rule[6]);
	
}

/*pid总体初始化-----------------------------------------------------------------*/

void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;
//	pid->f_cal_pid = pid_calc;	
//	pid->f_cal_sp_pid = pid_sp_calc;	//addition
		
    /*init pid param */
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
	
}
