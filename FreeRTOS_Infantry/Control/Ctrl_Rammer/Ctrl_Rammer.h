#ifndef __CTRL_RAMMER_H
#define __CTRL_RAMMER_H
#include "User_Code.h"

extern const int32_t 	Rammer_Max_Angle;//����תһȦ�������ۼƵ����ֵ
extern const int32_t	St_Serve;//תһ�������
extern Shoot_Struct 	  Shoot;
extern Motor2006_Param   M2006;
void M2006_PID_Set(void);
void Rammer_Angle(void);
void Rammer_Param_Set(void);//����
#endif

