
#define  __DBUS_GLOBALS

#include "Driver_DBUS.h"
#include "stm32f4xx_hal_uart.h"
#define ClearShackTick      50      //����ʱ��
    
uint8_t Key_Test;
FrameRate_Struct Remote;
void User_DMAInit(UART_HandleTypeDef* user_usartX)
{

}

/**
  * @brief  DBUS��ʼ��
  * @param  void
  * @retval void
  */
void DBUS_InitConfig(void)
{
	DBUSConnectStatus = Lost;
	DBUSFrameRate = 0;
	DBUSFrameCounter = 0;
	DBUS_ReceiveData.ch1 = 0;
	DBUS_ReceiveData.ch2 = 0;
	DBUS_ReceiveData.ch3 = 0;
	DBUS_ReceiveData.ch4 = 0;
	DBUS_ReceiveData.keyBoard.key_code = 0;
	DBUS_ReceiveData.keyBoard.jumpkey_code = 0;
}


/**
  * @brief  DBUS���ݽ���
  * @param  void
  * @retval void
  */
void DBUS_DataDecoding(void)
{
	LASTDBUS_ReceiveData = DBUS_ReceiveData;
	
	DBUS_ReceiveData.ch1 = (DBUSBuffer[0] | DBUSBuffer[1]<<8) & 0x07FF;
	DBUS_ReceiveData.ch1 -= 1024;
	DBUS_ReceiveData.ch2 = (DBUSBuffer[1]>>3 | DBUSBuffer[2]<<5 ) & 0x07FF;
	DBUS_ReceiveData.ch2 -= 1024;
	DBUS_ReceiveData.ch3 = (DBUSBuffer[2]>>6 | DBUSBuffer[3]<<2 | DBUSBuffer[4]<<10) & 0x07FF;
	DBUS_ReceiveData.ch3 -= 1024;
	DBUS_ReceiveData.ch4 = (DBUSBuffer[4]>>1 | DBUSBuffer[5]<<7) & 0x07FF;		
	DBUS_ReceiveData.ch4 -= 1024;
	
	DBUS_ReceiveData.switch_left = ( (DBUSBuffer[5] >> 4)& 0x000C ) >> 2;
	DBUS_ReceiveData.switch_right =  (DBUSBuffer[5] >> 4)& 0x0003 ;
	
	DBUS_ReceiveData.mouse.x = DBUSBuffer[6] | (DBUSBuffer[7] << 8);	//x axis
	DBUS_ReceiveData.mouse.y = DBUSBuffer[8] | (DBUSBuffer[9] << 8);
	DBUS_ReceiveData.mouse.z = DBUSBuffer[10]| (DBUSBuffer[11] << 8);
	
	DBUS_ReceiveData.mouse.press_left 	= DBUSBuffer[12];	// is pressed?
	DBUS_ReceiveData.mouse.press_right 	= DBUSBuffer[13];
	
	DBUS_ReceiveData.keyBoard.key_code 	= DBUSBuffer[14] | DBUSBuffer[15] << 8; //key borad code
}



/**
  * @brief  �ж�һ�����Ƿ񱻰���
  * @param  Ҫ�����жϵİ������ַ���д
  * @retval 1 ����        0 δ����
  */
uint8_t DBUS_CheckPush(uint16_t Key)
{
    if(DBUS_ReceiveData.keyBoard.key_code & Key)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/**
  * @brief  �ж�һ�����̼��Ƿ�����������
  * @param  Ҫ�����жϵİ������ַ���д
  * @retval 1 ����        0 δ����
  */
uint8_t DBUS_CheckJumpKey(uint16_t Key)
{
    if(DBUS_ReceiveData.keyBoard.jumpkey_code & Key)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/**
  * @brief  �ж�һ�������Ƿ�����������
  * @param  Ҫ�����жϵİ������ַ���д
  * @retval 1 ����        0 δ����
  */
uint8_t DBUS_CheckJumpMouse(uint8_t Key)
{
    if(Key)
    {
        return DBUS_ReceiveData.mouse.jumppress_left;
    }
    else
    {
        return DBUS_ReceiveData.mouse.jumppress_right;
    }
}




