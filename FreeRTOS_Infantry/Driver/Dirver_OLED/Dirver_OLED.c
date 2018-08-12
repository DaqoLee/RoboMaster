/************************************************************************************
*  Copyright (c), 2014, HelTec Automatic Technology co.,LTD.
*            All rights reserved.
*
* Http:    www.heltec.cn
* Email:   cn.heltec@gmail.com
* WebShop: heltec.taobao.com
*
* File name: OLED_I2C.c
* Project  : HelTec.uvprij
* Processor: STM32F103C8T6
* Compiler : MDK fo ARM
* 
* Author : С��
* Version: 1.00
* Date   : 2014.4.8
* Email  : hello14blog@gmail.com
* Modification: none
* 
* Description:128*64�����OLED��ʾ�������ļ����������ڻ����Զ���(heltec.taobao.com)��SD1306����IICͨ�ŷ�ʽ��ʾ��
*
* Others: none;
*
* Function List:
*	1. void I2C_Configuration(void) -- ����CPU��Ӳ��I2C
* 2. void I2C_WriteByte(uint8_t addr,uint8_t data) -- ��Ĵ�����ַдһ��byte������
* 3. void WriteCmd(unsigned char I2C_Command) -- д����
* 4. void WriteDat(unsigned char I2C_Data) -- д����
* 5. void OLED_Init(void) -- OLED����ʼ��
* 6. void OLED_SetPos(unsigned char x, unsigned char y) -- ������ʼ������
* 7. void OLED_Fill(unsigned char fill_Data) -- ȫ�����
* 8. void OLED_CLS(void) -- ����
* 9. void OLED_ON(void) -- ����
* 10. void OLED_OFF(void) -- ˯��
* 11. void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize) -- ��ʾ�ַ���(�����С��6*8��8*16����)
* 12. void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N) -- ��ʾ����(������Ҫ��ȡģ��Ȼ��ŵ�codetab.h��)
* 13. void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]) -- BMPͼƬ
*
* History: none;
*
*************************************************************************************/
/////////////////////////////////////////////////
//��Text6ʱ0x06*n����nΪ�������ַ�22����
///////////////////////////////////////////////
#include "Dirver_OLED.h"
#include "codetab.h"
#include "BSP_I2C.h"

I2C_HandleTypeDef* user_i2c;

void I2C_Configuration(I2C_HandleTypeDef* _i2c_)
{
		user_i2c=_i2c_;
}

void I2C_WriteByte(uint8_t addr,uint8_t data)
{

}

void WriteCmd(unsigned char I2C_Command)//д����
{
	uint8_t temp[2]={0x00,I2C_Command};
	HAL_I2C_Master_Transmit(user_i2c,OLED_ADDRESS,temp,2,0xffff);
}

void WriteDat(unsigned char I2C_Data)//д����
{
	uint8_t temp[2]={0x40,I2C_Data};
	HAL_I2C_Master_Transmit(user_i2c,OLED_ADDRESS,temp,2,0xffff);
}

void OLED_Init(void)
{
	//HAL_StatusTypeDef flag;
	I2C_Configuration(&hi2c2);
	uint8_t Cmd[]={0x00,0xAE,0x20,0x10,0xb0,0xc8,0x00,0x10,\
								 0x40,0x81,0xff,0xa1,0xa6,0xa8,0x3F,\
								 0xa4,0xd3,0x00,0xd5,0xf0,0xd9,0x22,\
								 0xda,0x12,0xdb,0x20,0x8d,0x14,0xaf};

	HAL_Delay(100);//�������ʱ����Ҫ
	HAL_I2C_Master_Transmit(user_i2c,OLED_ADDRESS,(uint8_t*)Cmd,29,0xffff);
								 
	OLED_Fill(0xff);
    OLED_Fill(0x00);//����
								 
								 
   Play_OLED_Chassis_Init();
}

void OLED_SetPos(unsigned char x, unsigned char y,uint8_t flag) //������ʼ������
{ 
//		WriteCmd(0xb0+y);
//		WriteCmd(((x&0xf0)>>4)|0x10);
//		WriteCmd((x&0x0f)|0x01);
		uint8_t temp[5]={0x00};
		temp[1]=0xb0+y;
		temp[2]=(x&0x0f)|0x00;
		temp[3]=((x&0xf0)>>4)|0x10;
		temp[4]=0x04;
		
//		if(flag)
//		{
//				//temp[2]=(x&0xf0>>4)|0x10;
//				
//				temp[3]=((x&0x0f)|0x01);
//			  temp[4]=0x04;
//		}
		//temp[3]=((x&0x0f)|0x01);
		HAL_I2C_Master_Transmit(user_i2c,OLED_ADDRESS,temp,temp[4],0xffff);
}

void OLED_Fill(unsigned char fill_Data)//ȫ�����
{
	unsigned char m,n,temp[4];
	for(m=0;m<8;m++)
	{
		temp[0]=0x00;//����
		temp[1]=0xb0+m;//ҳ����ʼ��ַ
		temp[2]=0x00;//low column start address��4λ	
		temp[3]=0x00;//high column start address��4λ
		HAL_I2C_Master_Transmit(user_i2c,OLED_ADDRESS,(uint8_t*)temp,4,0xffff);
		for(n=0;n<128;n++)
		{
			WriteDat(fill_Data);
		}
	}
}

void OLED_CLS(void)//����
{
	OLED_Fill(0x00);
}

//--------------------------------------------------------------
// Prototype      : void OLED_ON(void)
// Calls          : 
// Parameters     : none
// Description    : ��OLED�������л���
//--------------------------------------------------------------
void OLED_ON(void)
{
		//���õ�ɱ�  0x8D
		//������ɱ�  0x14
		//OLED����   0xAF
		uint8_t temp[5]={0x00,0x8D,0x14,0x00,0xAF};
		HAL_I2C_Master_Transmit(user_i2c,OLED_ADDRESS,temp,5,0xffff);
}

//--------------------------------------------------------------
// Prototype      : void OLED_OFF(void)
// Calls          : 
// Parameters     : none
// Description    : ��OLED���� -- ����ģʽ��,OLED���Ĳ���10uA
//--------------------------------------------------------------
void OLED_OFF(void)
{
		//���õ�ɱ�  0x8D
		//�رյ�ɱ�  0x10
		//OLED����    0xAE
		uint8_t temp[5]={0x00,0x8D,0x10,0x00,0xAE};
		HAL_I2C_Master_Transmit(user_i2c,OLED_ADDRESS,temp,5,0xffff);
}

//--------------------------------------------------------------
// Prototype      : void OLED_ShowChar(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize)
// Calls          : 
// Parameters     : x,y -- ��ʼ������(x:0~127, y:0~7); ch[] -- Ҫ��ʾ���ַ���; TextSize -- �ַ���С(1:6*8 ; 2:8*16)
// Description    : ��ʾcodetab.h�е�ASCII�ַ�,��6*8��8*16��ѡ��
//--------------------------------------------------------------
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], _TextSize_ TextSize)
{
		unsigned char c = 0,i = 0,j = 0;
		switch(TextSize)
		{
				case Text6:
				{
						while(ch[j] != '\0')
						{
							c = ch[j] - 32;
							if(x > 126)
							{
									x = 0;
									y++;
							}
							OLED_SetPos(x,y,0);
							for(i=0;i<6;i++)
								WriteDat(F6x8[c][i]);
							x += 6;
							j++;
						}
				}break;
				case Text8:
				{
						while(ch[j] != '\0')
						{
								c = ch[j] - 32;
								if(x > 120)
								{
										x = 0;
										y++;
								}
								OLED_SetPos(x,y,1);
								for(i=0;i<8;i++)
									WriteDat(F8X16[c*16+i]);
								OLED_SetPos(x,y+1,1);
								for(i=0;i<8;i++)
									WriteDat(F8X16[c*16+i+8]);
								x += 8;
								j++;
						}
				}break;
		}
}

//--------------------------------------------------------------
// Prototype      : void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N)
// Calls          : 
// Parameters     : x,y -- ��ʼ������(x:0~127, y:0~7); N:������codetab.h�е�����
// Description    : ��ʾcodetab.h�еĺ���,16*16����
//--------------------------------------------------------------
//void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N)
//{
//	unsigned char wm=0;
//	unsigned int  adder=32*N;
//	OLED_SetPos(x , y,0);
//	for(wm = 0;wm < 16;wm++)
//	{
//		WriteDat(F16x16[adder]);
//		adder += 1;
//	}
//	OLED_SetPos(x,y + 1,1);
//	for(wm = 0;wm < 16;wm++)
//	{
//		WriteDat(F16x16[adder]);
//		adder += 1;
//	}
//}

//--------------------------------------------------------------
// Prototype      : void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);
// Calls          : 
// Parameters     : x0,y0 -- ��ʼ������(x0:0~127, y0:0~7); x1,y1 -- ���Խ���(������)������(x1:1~128,y1:1~8)
// Description    : ��ʾBMPλͼ
//--------------------------------------------------------------
//void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[])
//{
//	unsigned int j=0;
//	unsigned char x,y;

//  if(y1%8==0)
//		y = y1/8;
//  else
//		y = y1/8 + 1;
//	for(y=y0;y<y1;y++)
//	{
//		OLED_SetPos(x0,y,0);
//    for(x=x0;x<x1;x++)
//		{
//			WriteDat(BMP[j++]);
//		}
//	}
//}
///////////////////////////////////////////////////////
////����ʾ����ת�����ַ�����ʾ�ڸ�����x��yλ�����ַ���С��6*8
////signedDataΪʱ0Ϊ�޷�����ʾ
////////////////////////////////////////////////////////
void OLED_ShowData_16(uint8_t x,uint8_t y,uint16_t data,uint8_t signedData)
{
		char str[6]="     0";
		uint8_t Miriade,Thousand,Hundred,Ten,Single;//��ǧ���٣�ʮ
		int16_t temp=0;
		if(signedData)
		{
				if(data&0x8000)//Ϊ����
				{
						temp=data;
						temp=-temp;
						str[0]='-';
						data=temp;//ȥ�����λ
				}
		}
		Miriade=data/10000;
		Thousand=(data%10000)/1000;//�õ�ǧλ
		Hundred=(data%1000)/100;
		Ten=(data%100)/10;
		Single=(data%10);
		if(Miriade==0)
		{
				str[1]=' ';
				if(Thousand==0)
				{
						str[2]=' ';	
						if(Hundred==0)
						{
								str[3]=' ';
								if(Ten==0)
								{
										str[4]=' ';
										str[5]='0'+Single;
								}
								else
								{
										str[4]='0'+Ten;
										str[5]='0'+Single;
								}
						}
						else
						{
								str[3]='0'+Hundred;
								str[4]='0'+Ten;
								str[5]='0'+Single;
						}
				}
				else 
				{
						str[2]='0'+Thousand;
						str[3]='0'+Hundred;
						str[4]='0'+Ten;
						str[5]='0'+Single;
				}
		}
		else 
		{
				str[1]='0'+Miriade;
				str[2]='0'+Thousand;
				str[3]='0'+Hundred;
				str[4]='0'+Ten;
				str[5]='0'+Single;
		}
		OLED_ShowStr(x,y,(uint8_t*)str,Text6);
}


void Play_OLED_Chassis_Init()
{
	OLED_Fill(0xff);
	OLED_Fill(0x00);//����
	OLED_ShowStr(0*6,0,(unsigned char*)"Power_Max",Text6);
	OLED_ShowStr(0*6,4,(unsigned char*)"Pbuff_Min",Text6);
}


void Play_OLED_Chassis()
{
//	OLED_ShowData_16(8*6,0,Judge_RobotPowerHeatData.chassisVolt,1);
//	OLED_ShowData_16(8*6,2,Judge_RobotPowerHeatData.chassisCurrent*1000,1);
//	OLED_ShowData_16(8*6,4,Judge_RobotPowerHeatData.chassisPower,1);
//	OLED_ShowData_16(2*6,6,ChassisParam.LB.Real_Current,1);
//	OLED_ShowData_16(10*6,6,ChassisParam.LB.Target_Current,1);
	

}




