#ifndef __CTRL_RAMMER_H
#define __CTRL_RAMMER_H
#include "User_typedefs.h"

#define RAMMER_NUM		   8		//���̿�����
#define REDUCTION_RATIO	   (36/1)	//����������ٱ�
#define RAMMER_ERROR	   16000	//����תһȦ��ƫ��

extern Motor2006_Param   M2006;
void M2006_PID_Set(void);
void Rammer_Angle(void);
#endif

