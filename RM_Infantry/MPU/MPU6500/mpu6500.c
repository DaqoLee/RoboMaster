#include "mpu6500.h"
int16_t accel_x, accel_y, accel_z, groy_x, groy_y, groy_z;
MPU_value mpu_value;
unsigned char BUF[6];       //�������ݻ�����
uint8_t mpu6500_buf[20];
uint8_t Init_MPU6500(void)
{
    if( MPU6500_Read_Reg(WHO_AM_I) == 0x70)			//��ȷ��ȡ��6500�ĵ�ַ
    {
        HAL_Delay(10);
        MPU6500_Write_Reg(PWR_MGMT_1, 0X80);   		//��Դ����,��λMPU6500
        HAL_Delay(10);
        MPU6500_Write_Reg(0X68, 0X07); //�����ǡ����ٶȼơ��¶ȼƸ�λ
        HAL_Delay(10);
        MPU6500_Write_Reg(PWR_MGMT_1, 0X01);   		//ѡ��ʱ��Դ
        HAL_Delay(10);
        MPU6500_Write_Reg(USER_CTRL, 0X30);
        HAL_Delay(10);
        MPU6500_Write_Reg(SMPLRT_DIV, 0X00);				//������1000/(1+0)=1000HZ
        HAL_Delay(10);
        MPU6500_Write_Reg(GYRO_CONFIG, 0X10);  		//�����ǲ�����Χ 0X10 ����1000��
        HAL_Delay(10);
        MPU6500_Write_Reg(CONFIG, 0X04);						//��ͨ�˲��� 0x02 20hz (9.9ms delay) fs=1khz
        HAL_Delay(10);
        MPU6500_Write_Reg(ACCEL_CONFIG, 0x10); 		//���ٶȼƲ�����Χ 0X10 ����8g
        HAL_Delay(10);
        MPU6500_Write_Reg(0X1D, 0x04);		//���ٶȼ�����1khz �˲���20hz (99.8ms delay)
        HAL_Delay(10);
        MPU6500_Write_Reg(PWR_MGMT_2, 0X00);   		//ʹ�ܼ��ٶȼƺ�������
		__HAL_SPI_ENABLE(&hspi5);  //ʹ�� SPI5
        SPI5_Read_Write_Byte(0xff);//��������
				return 0;
    }
		return 1;

}
void MPU6500_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t temp1 = reg & 0x7f;
    spi_CS = 0;
    HAL_SPI_TransmitReceive(&hspi5, &temp1, &temp1, 1, 100);
    HAL_SPI_TransmitReceive(&hspi5, &value, &value, 1, 100);
    spi_CS = 1;
}
//---------------------------------------------------------------//
//SPI��ȡ
//reg: addr
uint8_t MPU6500_Read_Reg(uint8_t reg)
{
    uint8_t reg_val = reg | 0x80;
    spi_CS = 0; //	MPU9250_CS=0;  //ƬѡMPU9250
	HAL_SPI_TransmitReceive(&hspi5,&reg_val,&reg_val,1,100);
	reg_val=0xff;
	HAL_SPI_TransmitReceive(&hspi5,&reg_val,&reg_val,1,100);
    spi_CS = 1; //	MPU9250_CS=1;  //ʧ��MPU9250
    return	reg_val;
}
//uint8_t spi_Mag_read(uint8_t reg)
//{
//    uint8_t data;

//    MPU6500_Write_Reg(MPU6500_I2C_SLV4_REG, reg | 0x80);
//    MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x80);
//    data = MPU6500_Read_Reg(MPU6500_I2C_SLV4_DI);
//    MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
//    return data;
//}
//void spi_Mag_write(uint8_t reg, uint8_t value)
//{
//    MPU6500_Write_Reg(I2C_SLV0_ADDR , MPU9250_AK8963_ADDR); //���ô����Ƶ�ַ,mode: write
//    MPU6500_Write_Reg(I2C_SLV0_REG , reg); //set reg addr
//    MPU6500_Write_Reg(I2C_SLV0_DO , value); //send value
//}
short Read_MPU6500(void)
{
    uint8_t T[2] = {0x41, 0};
    short	raw;
    float temp;
    T[0] = MPU6500_Read_Reg(0x41);
    T[1] = MPU6500_Read_Reg(0x42);
    raw = ((uint16_t)T[0] << 8) | T[1];
    temp = 36.53 + ((double)raw) / 340;

    return temp * 100;
}




////////////////////////////���Զ�ȡԭʼ����////////////////////////////////////////////////////////////////////////////////
/*
 * ��������SPI1_Read_Write_Byte
 * ����  ����дһ���ֽ�
 * ����  ��TxData:Ҫд����ֽ�
 * ���  ����ȡ�����ֽ�
 */
uint8_t SPI5_Read_Write_Byte(uint8_t TxData)
{
    uint8_t retry = 0;
    while (__HAL_SPI_GET_FLAG(&hspi5, SPI_FLAG_TXE) == RESET) 	//���ָ����SPI��־λ�������:���ͻ���ձ�־λ
    {
        retry++;
        if(retry > 250)	return 0;
    }
    hspi5.Instance->DR = TxData; 																//ͨ������SPIx����һ������
    retry = 0;

    while (__HAL_SPI_GET_FLAG(&hspi5, SPI_FLAG_RXNE) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
    {
        retry++;
        if(retry > 250) return 0;
    }
    return hspi5.Instance->DR; 															//����ͨ��SPIx������յ�����
}
/**
  * @brief ��ȡ���ٶȼ������ǵ�ԭʼ����
  * @param None
  * @retval None
  */
void MPU6500_ReadValueRaw(void)
{
    uint8_t i;
//    int b[6];
    spi_CS = 0; 																	//ʹ��SPI����

    SPI5_Read_Write_Byte(ACCEL_XOUT_H | 0x80); 				//�Ӽ��ٶȼƵļĴ�����ʼ���ж�ȡ�����Ǻͼ��ٶȼƵ�ֵ//���Ͷ�����+�Ĵ�����

    for(i	=	0; i	<	14; i++)														//һ����ȡ14�ֽڵ�����
    {
        mpu6500_buf[i]	=	SPI5_Read_Write_Byte(0xff);	//����0xff,��Ϊslave��ʶ��
    }
    //,,,,,
    accel_x = BYTE16(uint16_t, mpu6500_buf[0],  mpu6500_buf[1]);
    accel_y = BYTE16(uint16_t, mpu6500_buf[2],  mpu6500_buf[3]);
    accel_z = BYTE16(uint16_t, mpu6500_buf[4],  mpu6500_buf[5]);
    groy_x = BYTE16(uint16_t, mpu6500_buf[8],  mpu6500_buf[9]);
    groy_y = BYTE16(uint16_t, mpu6500_buf[10],  mpu6500_buf[11]);
    groy_z = BYTE16(uint16_t, mpu6500_buf[12],  mpu6500_buf[13]);
//		b[0]=MPU6500_Acc_Raw.x;
//		b[1]=MPU6500_Acc_Raw.y;
//		b[2]=MPU6500_Acc_Raw.z;
//		b[3]=MPU6500_Gyro_Raw.x;
//		b[4]=MPU6500_Gyro_Raw.y;
//		b[5]=MPU6500_Gyro_Raw.z;
    //printf("\nMPU6500_Acc_Raw.x=%d\n",b[0]);

//		printf("\nMPU6500_Acc_Raw.x=%d\n",b);
//
//		printf("\nMPU6500_Acc_Raw.x=%d\n",b);
//
//		mpu6500_tempreature_temp	=	BYTE16(s16, mpu6500_buf[6],  mpu6500_buf[7]);
//		mpu6500_tempreature	=	(float)(35000+((521+mpu6500_tempreature_temp)*100)/34); // ԭ����ĸΪ340�����ڷ���*100����������1000����
//		mpu6500_tempreature = mpu6500_tempreature/1000;


    spi_CS = 1;  	  //��ֹSPI����
}

////////////////////////////���Զ�ȡԭʼ����////////////////////////////////////////////////////////////////////////////////

