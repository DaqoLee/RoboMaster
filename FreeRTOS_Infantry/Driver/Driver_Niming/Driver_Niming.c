#include "Driver_Niming.h"
UART_HandleTypeDef* user_huart;
uint32_t user_Time=0;
uint8_t send_buf[32]= {0};
/*��ȡ����ָ��*/
void Init_Ninming(UART_HandleTypeDef* _huart_)
{
    user_huart=_huart_;
}
/**/
uint16_t user_UART_WaitOnFlagUntilTimeout(void)
{
    return	(user_huart->Instance->SR)&(user_TI_Succeed);
}

//����1����1���ַ�
//c:Ҫ���͵��ַ�
void usart1_send_char(uint8_t c)
{
    /*
    	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������
    	USART_SendData(USART1,c);
    	*/
    user_Time = HAL_GetTick();
    while((user_UART_WaitOnFlagUntilTimeout() != user_TI_Succeed)&&((user_Time-HAL_GetTick())<user_Timeout));//ѭ������ֱ�����ͳɹ����б׶�

    if((user_Time-HAL_GetTick())<user_Timeout)
    {
        user_huart->Instance->DR = (c & (uint16_t)0x01FFU);
        //printf("\n���ͳɹ�\n");
    }
    else
    {
        printf("\n��ʱ����ʧ��\n");
    }


}
//�������ݸ�����������λ�����(V2.6�汾)
//fun:������. 0XA0~0XAF
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
void usart1_niming_report(uint8_t fun, uint8_t *data, uint8_t len)
{

    uint8_t i;
//    HAL_StatusTypeDef flag;
    if(len > 28)return;	//���28�ֽ�����
#if NIMING_4_22
    send_buf[len + 4] = 0;	//У��������
    send_buf[0] = 0xaa;	//֡ͷ
    send_buf[1] = 0xaa;
    send_buf[2] = fun;	//������
    send_buf[3] = len;	//���ݳ���
    for(i = 0; i < len; i++)send_buf[4+ i] = data[i];			//��������
    for(i = 0; i < len + 4; i++)send_buf[len + 4] += send_buf[i];	//����У���
    for(i = 0; i < len + 5; i++)usart1_send_char(send_buf[i]);	//�������ݵ�����1
#else
    send_buf[len + 3] = 0;	//У��������
    send_buf[0] = 0x88;	//֡ͷ
    send_buf[1] = fun;	//������
    send_buf[2] = len;	//���ݳ���
    for(i = 0; i < len; i++)send_buf[3+ i] = data[i];			//��������
    for(i = 0; i < len + 3; i++)send_buf[len + 3] += send_buf[i];	//����У���
    for(i = 0; i < len + 4; i++)usart1_send_char(send_buf[i]);	//�������ݵ�����1
#endif
//		flag=HAL_UART_Transmit_DMA(huartX,send_buf,len+4);//���Ͳ�Ҫ����DMA���Ͳ�Ȼ���͵����ݶ�������
//		huartX->gState=HAL_UART_STATE_READY;

}
//���ͼ��ٶȴ��������ݺ�����������
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
//temp ��ʵ�¶�ֵӦΪtemp/100�õ��������֣�temp%100�õ�С������
//ע�ⲻ����ʾС��
void Send_PC_Data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz)
{
    //u8 tbuf[12];
    uint8_t tbuf[12];



    tbuf[0] = (aacx >> 8) & 0XFF; /////////////��1����  ����λ1
    tbuf[1] = aacx & 0XFF;


    tbuf[2] = (aacy >> 8) & 0XFF; /////////////��2����  ����λ2
    tbuf[3] = aacy & 0XFF;
    tbuf[4] = (aacz >> 8) & 0XFF; /////////////��3����  ����λ3
    tbuf[5] = aacz & 0XFF;
    tbuf[6] = (gyrox >> 8) & 0XFF; ////////////��4����  ����λ4
    tbuf[7] = gyrox & 0XFF;
    tbuf[8] = (gyroy >> 8) & 0XFF; ////////////��5����  ����λ5
    tbuf[9] = gyroy & 0XFF;
    tbuf[10] = (gyroz >> 8) & 0XFF; ///////////��6����  ����λ6
    tbuf[11] = gyroz & 0XFF;

    //�����¶�ֵ
//    tbuf[12] = (temp >> 8) & 0xff; ////////////��7����  ����λ7
//    tbuf[13] = temp & 0xff;

    //usart1_niming_report(0XA1,tbuf,12);//�Զ���֡,0XA1
    usart1_niming_report(0xa1, tbuf, 12); //����һ��ֵ
}
//ͨ������1�ϱ���������̬���ݸ�����
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
//roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
//pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
//yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
void usart1_report_imu(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw)
{
    uint8_t tbuf[28];
    uint8_t i;
    for(i = 0; i < 28; i++)tbuf[i] = 0; //��0
    tbuf[0] = (aacx >> 8) & 0XFF;
    tbuf[1] = aacx & 0XFF;
    tbuf[2] = (aacy >> 8) & 0XFF;
    tbuf[3] = aacy & 0XFF;
    tbuf[4] = (aacz >> 8) & 0XFF;
    tbuf[5] = aacz & 0XFF;
    tbuf[6] = (gyrox >> 8) & 0XFF;
    tbuf[7] = gyrox & 0XFF;
    tbuf[8] = (gyroy >> 8) & 0XFF;
    tbuf[9] = gyroy & 0XFF;
    tbuf[10] = (gyroz >> 8) & 0XFF;
    tbuf[11] = gyroz & 0XFF;
    tbuf[18] = (roll >> 8) & 0XFF;
    tbuf[19] = roll & 0XFF;
    tbuf[20] = (pitch >> 8) & 0XFF;
    tbuf[21] = pitch & 0XFF;
    tbuf[22] = (yaw >> 8) & 0XFF;
    tbuf[23] = yaw & 0XFF;
    usart1_niming_report(0XAF, tbuf, 28); //�ɿ���ʾ֡,0XAF
}


