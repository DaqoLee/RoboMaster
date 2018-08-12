#ifndef __DRIVER_OLED_H
#define	__DRIVER_OLED_H
#include "stm32f4xx_hal.h"

#define OLED_ADDRESS	0x78 //ͨ������0R����,������0x78��0x7A������ַ -- Ĭ��0x78
extern I2C_HandleTypeDef* user_i2c;
typedef enum
{
		Text6=0x01,//����Ϊ6*8
		Text8			//	����Ϊ8*8
}_TextSize_;

void I2C_Configuration(I2C_HandleTypeDef* _i2c_);
void I2C_WriteByte(uint8_t addr,uint8_t data);
void WriteCmd(unsigned char I2C_Command);
void WriteDat(unsigned char I2C_Data);
void OLED_Init(void);
void OLED_SetPos(unsigned char x, unsigned char y,uint8_t flag);//������ʼ������;
void OLED_Fill(unsigned char fill_Data);
void OLED_CLS(void);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], _TextSize_ TextSize);
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N);
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);
void OLED_ShowData_16(uint8_t x,uint8_t y,uint16_t data,uint8_t signedData);
void Play_OLED_Chassis(void);
void Play_OLED_Chassis_Init(void);


#endif
