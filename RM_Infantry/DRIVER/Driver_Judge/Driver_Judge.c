
#define __DRIVER_GLOBALS
#include "Driver_Judge.h"
#include "user_can.h"
#include "control.h"

uint8_t JudgeDataBuffer[JudgeBufferLength];
extGameRobotState_t Judge_RobotState;
extRobotHurt_t	Judge_RobotHurt;
extShootData_t Judge_RobotShootData;
extPowerHeatData_t Judge_RobotPowerHeatData;
extRfidDetect_t Judge_RobotRfidDetect;
extGetBuff_t Judge_RobotGetBuff;
extGameRobotPos_t Judge_RobotPos;
uint16_t wExpected ;
unsigned char pchMessage[22];
FrameRate_Struct Judge;
uint8_t Judge_Power_Limit;
//uint16_t wCRC_;
//uint8_t chData;
//uint16_t dwLength_;
//uint16_t chData_;
//uint16_t wCRC_Tab;
/***********************************    ↓    DJI提供的CRC校检函数   ↓  ***********************************/
//crc8 generator polynomial:G(x)=x8+x5+x4+1
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
{
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};


unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8)
{
    unsigned char ucIndex;
    while (dwLength--)
    {
        ucIndex = ucCRC8^(*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return(ucCRC8);
}


/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2)) 
        return 0;
    ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
    return ( ucExpected == pchMessage[dwLength-1] );
}


/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2)) 
        return;
    ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
    pchMessage[dwLength-1] = ucCRC;
}

uint16_t CRC_INIT = 0xffff;

const uint16_t wCRC_Table[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};


/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == 0)
    {
        return 0xFFFF;
    }
    while(dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^wCRC_Table[ ((uint16_t)(wCRC)^(uint16_t)(chData))& 0x00ff] ;
    }
    return wCRC;
}


/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    //uint16_t wExpected = 0;
		wExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}


/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return;
    }
    wCRC = Get_CRC16_Check_Sum ( (uint8_t *)pchMessage, dwLength-2, CRC_INIT );
    pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff);
    pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);
}
/***********************************    ↑    DJI提供的CRC校检函数   ↑  ***********************************/   
  

/**
  * @brief  裁判系统初始化
  * @param  void
  * @retval void
  */
void Judge_InitConfig(void)
{
    InfantryJudge.RealVoltage = 25.2F;
    InfantryJudge.RealCurrent = 0;
    InfantryJudge.LastBlood = 1500;
    InfantryJudge.LastShotSpeed = 23.5;
//    InfantryJudge.LastShotTick = 0;
    JudgeFrameCounter = 0;
    JudgeFrameRate = 0;
#if INFANTRY == 7
    InfantryJudge.ShootNum = 0;
    InfantryJudge.BulletUseUp = 0;
    InfantryJudge.ShootFail = 0;
#endif
}
/**
  * @brief  裁判解析函数
  * @param  huart用于获取接收总帧长度
  * @retval void
  */
//uint32_t time;
void Analysis_Judge(UART_HandleTypeDef* huart)
{
		FormatTrans FT;
		uint8_t offset,n;
		offset=JudgeBufferLength-huart->hdmarx->Instance->NDTR;
		for(n=0;n<offset;)
		{
				if(JudgeDataBuffer[n]==0xa5)
				{
					Judge.FrameRate++;
						switch(JudgeDataBuffer[n+5])
						{
								case 0x01:
										if(Verify_CRC16_Check_Sum(JudgeDataBuffer+n,JudgeFrameLength_1)==1)
										{
												Judge_RobotState.stageRemianTime=JudgeDataBuffer[n+7]|(JudgeDataBuffer[n+8]<<8);
												Judge_RobotState.gameProgress=JudgeDataBuffer[n+9];
												Judge_RobotState.robotLevel=JudgeDataBuffer[n+10];
												Judge_RobotState.remainHP=JudgeDataBuffer[n+11]|(JudgeDataBuffer[n+12]<<8);
												Judge_RobotState.maxHP=JudgeDataBuffer[n+13]|(JudgeDataBuffer[n+14]<<8);
												n+=JudgeFrameLength_1;
										}
										else
										{
												n++;
										}
									break;
								case 0x02:
									if(Verify_CRC16_Check_Sum(JudgeDataBuffer+n,JudgeFrameLength_2)==1)
									{
												Judge_RobotHurt.armorType=JudgeDataBuffer[n+7]&0x07;
												Judge_RobotHurt.hurtType=JudgeDataBuffer[n+7]>>3;
												n+=JudgeFrameLength_2;
									}
									else
									{
											n++;
									}
									break;
								case 0x03:
										if(Verify_CRC16_Check_Sum(JudgeDataBuffer+n,JudgeFrameLength_3)==1)
										{
												Judge_RobotShootData.bulletType=JudgeDataBuffer[n+7];
												Judge_RobotShootData.bulletFreq=JudgeDataBuffer[n+8];
												FT.U[0]=JudgeDataBuffer[n+9];
												FT.U[1]=JudgeDataBuffer[n+10];
												FT.U[2]=JudgeDataBuffer[n+11];
												FT.U[3]=JudgeDataBuffer[n+12];
												Judge_RobotShootData.bulletSpeed=FT.F;
												n+=JudgeFrameLength_3;
										}
										else
										{
												n++;
										}
									break;
								case 0x04:
										if(Verify_CRC16_Check_Sum(JudgeDataBuffer+n,JudgeFrameLength_4)==1)
										{
												FT.U[0]=JudgeDataBuffer[n+7];
												FT.U[1]=JudgeDataBuffer[n+8];
												FT.U[2]=JudgeDataBuffer[n+9];
												FT.U[3]=JudgeDataBuffer[n+10];
												Judge_RobotPowerHeatData.chassisVolt=FT.F;
												FT.U[0]=JudgeDataBuffer[n+11];
												FT.U[1]=JudgeDataBuffer[n+12];
												FT.U[2]=JudgeDataBuffer[n+13];
												FT.U[3]=JudgeDataBuffer[n+14];
												Judge_RobotPowerHeatData.chassisCurrent=FT.F;
												FT.U[0]=JudgeDataBuffer[n+15];
												FT.U[1]=JudgeDataBuffer[n+16];
												FT.U[2]=JudgeDataBuffer[n+17];
												FT.U[3]=JudgeDataBuffer[n+18];
												Judge_RobotPowerHeatData.chassisPower=FT.F;
												FT.U[0]=JudgeDataBuffer[n+19];
												FT.U[1]=JudgeDataBuffer[n+20];
												FT.U[2]=JudgeDataBuffer[n+21];
												FT.U[3]=JudgeDataBuffer[n+22];
												Judge_RobotPowerHeatData.chassisPowerBuffer=FT.F;
												Judge_RobotPowerHeatData.shooterHeat0=JudgeDataBuffer[n+23]|(JudgeDataBuffer[n+24]<<8);
												Judge_RobotPowerHeatData.shooterHeat1=JudgeDataBuffer[n+25]|(JudgeDataBuffer[n+26]<<8);
												n+=JudgeFrameLength_4;
												
												if(Judge_RobotPowerHeatData.chassisPowerBuffer<30)
													Judge_Power_Limit=1;
												else if(Judge_RobotPowerHeatData.chassisPowerBuffer<15)
													Judge_Power_Limit=2;
												else if(Judge_RobotPowerHeatData.chassisPowerBuffer>50)
													Judge_Power_Limit=0;
											}
										else
										{
											n++;
										}
									break;
								case 0x05:
										if(Verify_CRC16_Check_Sum(JudgeDataBuffer+n,JudgeFrameLength_5)==1)
										{
												Judge_RobotRfidDetect.cardType=JudgeDataBuffer[n+7];
												Judge_RobotRfidDetect.cardIdx=JudgeDataBuffer[n+8];
												n+=JudgeFrameLength_5;
										}
										else
										{
												n++;
										}
									break;
								case 0x06:
										if(Verify_CRC16_Check_Sum(JudgeDataBuffer+n,JudgeFrameLength_6)==1)
										{
												n+=JudgeFrameLength_6;
										}
										else
										{
												n++;
										}
									break;
								case 0x07:
										if(Verify_CRC16_Check_Sum(JudgeDataBuffer+n,JudgeFrameLength_7)==1)
										{
												Judge_RobotGetBuff.buffType=JudgeDataBuffer[n+7];
												Judge_RobotGetBuff.buffAddition=JudgeDataBuffer[n+8];
												n+=JudgeFrameLength_7;
										}
										else
										{
												n++;
										}
									break;
								case 0x08:
										if(Verify_CRC16_Check_Sum(JudgeDataBuffer+n,JudgeFrameLength_8)==1)
										{
												FT.U[0]=JudgeDataBuffer[n+7];
												FT.U[1]=JudgeDataBuffer[n+8];
												FT.U[2]=JudgeDataBuffer[n+9];
												FT.U[3]=JudgeDataBuffer[n+10];
												Judge_RobotPos.x=FT.F;
												FT.U[0]=JudgeDataBuffer[n+11];
												FT.U[1]=JudgeDataBuffer[n+12];
												FT.U[2]=JudgeDataBuffer[n+13];
												FT.U[3]=JudgeDataBuffer[n+14];
												Judge_RobotPos.y=FT.F;
												FT.U[0]=JudgeDataBuffer[n+15];
												FT.U[1]=JudgeDataBuffer[n+16];
												FT.U[2]=JudgeDataBuffer[n+17];
												FT.U[3]=JudgeDataBuffer[n+18];
												Judge_RobotPos.z=FT.F;	
												FT.U[0]=JudgeDataBuffer[n+19];
												FT.U[1]=JudgeDataBuffer[n+20];
												FT.U[2]=JudgeDataBuffer[n+21];
												FT.U[3]=JudgeDataBuffer[n+22];
												Judge_RobotPos.yaw=FT.F;
												n+=JudgeFrameLength_8;
											}
										else
										{
												n++;
										}
									break;
								default:
										n++;
						}
				}
				else
				{
						n++;
				}
				
		}
//		time=test_Time();
}

 /*
  * @brief 上传数据到操作界面
  * @param dwLength 帧长 Data1~3和mask是用户数据
  * @retval None
  */
void Transmit_Judge(uint32_t dwLength,float Data1,float Data2,float Data3,uint8_t mask)
{
static  uint8_t Flag;
	Flag++;
	FormatTrans FT;
	pchMessage[0]=0xA5;
	pchMessage[1]=0x0D;
	pchMessage[2]=0x00;
	pchMessage[3]=Flag;
	pchMessage[4]=Get_CRC8_Check_Sum(pchMessage,4,CRC8_INIT);
	pchMessage[5]=0x00;
	pchMessage[6]=0x01;
	FT.F=Data1;
	pchMessage[7]=FT.U[0];
	pchMessage[8]=FT.U[1];
	pchMessage[9]=FT.U[2];
	pchMessage[10]=FT.U[3];
	
	FT.F=Data2;
	pchMessage[11]=FT.U[0];
	pchMessage[12]=FT.U[1];
	pchMessage[13]=FT.U[2];
	pchMessage[14]=FT.U[3];
	
	FT.F=Data3;
	pchMessage[15]=FT.U[0];
	pchMessage[16]=FT.U[1];
	pchMessage[17]=FT.U[2];
	pchMessage[18]=FT.U[3];
	
	pchMessage[19]=mask;
	Append_CRC16_Check_Sum(pchMessage,dwLength);

	HAL_UART_Transmit_DMA(&huart3,pchMessage,dwLength);
}
