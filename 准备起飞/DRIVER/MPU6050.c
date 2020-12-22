#include <stdio.h>
#include <math.h>
#include "MPU6050.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"

int x, y, z, X, Y, Z;

void identify_HMC5883L(void)
{
    u8 ID_A = 0, ID_B = 0, ID_C = 0;

    IIC_Init();

    //�����, ����Ҫ�ȶ�һ��, Ȼ�����������
    IIC_Start();
    IIC_Send_Byte(HMC5883L_Write_Address);
    IIC_Wait_Ack();
    IIC_Send_Byte(0x0A);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(HMC5883L_Read_Address);
    IIC_Wait_Ack();
    ID_A = IIC_Read_Byte(0);
    printf("\r\nhhhhh%chhh\r\n", ID_A);
    delay_us(2);

    //	IIC_Start();
    //	IIC_Send_Byte(HMC5883L_Write_Address);
    //	IIC_Wait_Ack();
    //	IIC_Send_Byte(0x0A);
    //	IIC_Wait_Ack();
    //	IIC_Start();
    //	IIC_Send_Byte(HMC5883L_Read_Address);
    //	IIC_Wait_Ack();
    //	ID_A = IIC_Read_Byte(0);
    //	delay_us(10);
    //	ID_B = IIC_Read_Byte(0);
    //	delay_us(10);
    //	ID_C = IIC_Read_Byte(1);

    IIC_Start();
    IIC_Send_Byte(HMC5883L_Write_Address);
    IIC_Wait_Ack();
    IIC_Send_Byte(0x0A);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(HMC5883L_Read_Address);
    IIC_Wait_Ack();
    ID_A = IIC_Read_Byte(0);
    printf("\r\nhhhhh%chhh\r\n", ID_A);
    delay_us(2);

    IIC_Start();
    IIC_Send_Byte(HMC5883L_Write_Address);
    IIC_Wait_Ack();
    IIC_Send_Byte(0x0B);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(HMC5883L_Read_Address);
    IIC_Wait_Ack();
    ID_B = IIC_Read_Byte(0);
    printf("\r\nhhhhh%chhh\r\n", ID_B);
    delay_us(2);

    IIC_Start();
    IIC_Send_Byte(HMC5883L_Write_Address);
    IIC_Wait_Ack();
    IIC_Send_Byte(0x0C);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(HMC5883L_Read_Address);
    IIC_Wait_Ack();
    ID_C = IIC_Read_Byte(0);
    printf("\r\nhhhhh%chhh\r\n", ID_C);
    delay_us(2);
    IIC_Stop();

    printf("\r\n\r\nHHHHHHHHHHHHHHHHHHHA: %c, B: %c, C: %cHHHHHHHHHHHHHHHHHH\r\n\r\n", ID_A, ID_B, ID_C);

    if (ID_A == 'H' && ID_B == '4' && ID_C == '3')
    {
        printf("\r�豸HMC5773L���ɹ�!\r\n\r");
    }
    else
    {
        printf("\r����!�޷�ʶ���豸HMC5773L!\r\n\r");
    }
}

void HMC5883L_Init()
{
    IIC_Init();
    IIC_Start();
    IIC_Send_Byte(HMC5883L_Write_Address); //�����ַ,ѡ��HMC5883L
    IIC_Wait_Ack();

    IIC_Send_Byte(0x00); //ѡ�����üĴ���A
    IIC_Wait_Ack();
    IIC_Send_Byte(0x18); //����Ĵ���A�Ĳ���
    IIC_Wait_Ack();

    IIC_Send_Byte(0x01); //ѡ�����üĴ���B
    IIC_Wait_Ack();
    IIC_Send_Byte(0x20); //����Ĵ���B�Ĳ���(����),1.3Ga=0X20; 1.9Ga-0X40; 8.1-0XE0
    IIC_Wait_Ack();

    IIC_Send_Byte(0x02); //ѡ��ģʽ�Ĵ���
    IIC_Wait_Ack();
    IIC_Send_Byte(0x00); //����ģʽ: 0x01:��һ����ģʽ; 0x00:��������ģʽ
    IIC_Stop();
}

float Get_Current_Angle(int *mx, int *my, int *mz)
{
    u8 i;
    u8 a[6]; //��Ŵ�HMC5883L����������
    //float Curent_Angle;
    short Curent_Angle;
    //HMC5883L_Init();

    IIC_Start();
    IIC_Send_Byte(HMC5883L_Write_Address); //���ģʽ(�����CPU)
    IIC_Wait_Ack();

    IIC_Send_Byte(0x03); //�����ַָ��0x03׼����ȡx������
    IIC_Wait_Ack();

    IIC_Start();
    IIC_Send_Byte(HMC5883L_Read_Address); //����ģʽ(�����CPU)
    IIC_Wait_Ack();

    for (i = 0; i < 6; i++) //��ȡ����
    {
        if (i == 5) //���һ�����ݲ���ҪAck�ź�,����nAck
        {
            a[i] = IIC_Read_Byte(0);
        }
        else
        {
            a[i] = IIC_Read_Byte(1);
            ;
        }
    }
    IIC_Stop();

    //��X,Y,Z�ĸߵ�λ�ϲ�
    x = a[0];
    x = x << 8;
    x = x | a[1];
    //	  printf("x=%d", x);
    z = a[2];
    z = z << 8;
    z = z | a[3];
    //		printf("z=%d", z);

    y = a[4];
    y = y << 8;
    y = y | a[5];

    //printf("x=%x y=%x z=%x\r\n",x, y, z);
    //	printf("%f\r\n", atan2( (double)((int16_t)y ),(double)((int16_t)x))*(180/3.14159265)+180);
    //������ת��Ϊ��������
    if (x > 0X7FFF)
    {
        x -= 0XFFFF;
    }
    if (y > 0X7FFF)
    {
        y -= 0XFFFF;
    }
    if (z > 0X7FFF)
    {
        z -= 0XFFFF;
    }
    //	printf("x=%d y=%d z=%d\r\n",x, y, z);

    X = (s16)x; //x����
    Y = (s16)y; //y����
    Z = (s16)z; //z����

    //	printf("\r\nxyz: %d %d %d", x, y, z);
    //	printf("\r\nXYZ: %d %d %d", X, Y, Z);

    *mx = X;
    *my = Y;
    *mz = Z;

    //	float xx,yy,zz;
    //	xx = (8.1*X)/2048;
    //	yy = (8.1*Y)/2048;
    //	zz = (8.1*Z)/2048;
    //	printf("\r\nxx=%f; yy=%f; zz=%f\r\n", xx, yy, zz);

    //  Curent_Angle = (atan2(y,x) * (180 / 3.14159265) + 180);  //ʵ��ˮƽ�Ƕ�(���ô�ƫ�ǹ�ʽ)
    //	printf("%f\r\n",(float)Curent_Angle);
    return Curent_Angle;
}

void self_test()
{
    IIC_Init();
    IIC_Start();
    IIC_Send_Byte(HMC5883L_Write_Address); //�����ַ,ѡ��HMC5883L
    IIC_Wait_Ack();

    IIC_Send_Byte(0x00); //ѡ�����üĴ���A
    IIC_Wait_Ack();
    IIC_Send_Byte(0x71); //����Ĵ���A�Ĳ���, �������ѹ
    IIC_Wait_Ack();

    IIC_Send_Byte(0x01); //ѡ�����üĴ���B
    IIC_Wait_Ack();
    IIC_Send_Byte(0x40); //����Ĵ���B�Ĳ���(����),1.9Ga
    IIC_Wait_Ack();

    IIC_Send_Byte(0x02); //ѡ��ģʽ�Ĵ���
    IIC_Wait_Ack();
    IIC_Send_Byte(0x00); //����ģʽ: 0x01:��һ����ģʽ; 0x00:��������ģʽ
    IIC_Stop();
}

// Offset of angular velocity zero drift
float Gyro_Xout_Offset, Gyro_Yout_Offset, Gyro_Zout_Offset;
// Offset of acceleration zero drift
float Accel_Xout_Offset, Accel_Yout_Offset, Accel_Zout_Offset;

// Get gyroscope bias
void Get_Gyro_Bias(void)
{
    uint16_t i;
    int16_t gyro[3];
    int32_t gyro_x = 0, gyro_y = 0, gyro_z = 0;
    static int16_t count = 0;
    u8 buf[6];

    // Take data 2000 times
    for(i = 0; i < 2000; i++)
    {
        if(!MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf))
        {
            // Get xyz angular velocity
            gyro[0] = ((((int16_t)buf[0])<<8) | buf[1]);
            gyro[1] = ((((int16_t)buf[2])<<8) | buf[3]);
            gyro[2] = ((((int16_t)buf[4])<<8) | buf[5]);
            gyro_x += gyro[0];
            gyro_y += gyro[1];
            gyro_z += gyro[2];
            // Record valid count
            count++;
        }
    }

    // Average the value to get the zero drift offset
    Gyro_Xout_Offset = (float)gyro_x / count;
    Gyro_Yout_Offset = (float)gyro_y / count;
    Gyro_Zout_Offset = (float)gyro_z / count;
}

// Get accelerometer bias
void Get_Accel_Bias(void)
{
    uint32_t i;
    int16_t accel[3]; 
    u8 buf[6];
    float accel_x = 0, accel_y = 0, accel_z = 0;
    static int16_t count2 = 0;

    // Take data 2000 times
    for(i = 0; i < 2000; i++)
    {
        if(!MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf))
        {
            // Get xyz acceleration
            accel[0] = (((int16_t)buf[0])<<8) | buf[1];
            accel[1] = (((int16_t)buf[2])<<8) | buf[3];
            accel[2] = (((int16_t)buf[4])<<8) | buf[5];
            accel_x += accel[0];
            accel_y += accel[1];
            accel_z += accel[2];
            // Record valid count
            count2++;
        }
    }

    // Average the value to get the zero drift offset
    Accel_Xout_Offset = (float)accel_x / count2;
    Accel_Yout_Offset = (float)accel_y / count2;
    Accel_Zout_Offset = (float)accel_z / count2;
}

//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Init(void)
{
    u8 res;
    IIC_Init();                              //��ʼ��IIC����
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); //��λMPU6050
    delay_ms(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); //����MPU6050
    MPU_Set_Gyro_Fsr(3);                     //�����Ǵ�����,��2000dps
    MPU_Set_Accel_Fsr(1);                    //���ٶȴ�����,��4g
    MPU_Set_Rate(50);                        //���ò�����50Hz
    MPU_Write_Byte(MPU_INT_EN_REG, 0X00);    //�ر������ж�
    MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);   //�ر�FIFO

    MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); //I2C��ģʽ�ر�
    MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X82); //INT���ŵ͵�ƽ��Ч,��������·ģʽ,HMC5883Lֱͨ����

    res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if (res == MPU_ADDR) //����ID��ȷ
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); //����CLKSEL,PLL X��Ϊ�ο�
        MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); //���ٶ��������Ƕ�����
        MPU_Set_Rate(50);                        //���ò�����Ϊ50Hz
			Get_Accel_Bias();
			Get_Gyro_Bias();
    }
    else
        return 1;
    return 0;
}
//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3); //���������������̷�Χ
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3); //���ü��ٶȴ����������̷�Χ
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 MPU_Set_LPF(u16 lpf)
{
    u8 data = 0;
    if (lpf >= 188)
        data = 1;
    else if (lpf >= 98)
        data = 2;
    else if (lpf >= 42)
        data = 3;
    else if (lpf >= 20)
        data = 4;
    else if (lpf >= 10)
        data = 5;
    else
        data = 6;
    return MPU_Write_Byte(MPU_CFG_REG, data); //�������ֵ�ͨ�˲���
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 MPU_Set_Rate(u16 rate)
{
    u8 data;
    if (rate > 1000)
        rate = 1000;
    if (rate < 4)
        rate = 4;
    data = 1000 / rate - 1;
    data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data); //�������ֵ�ͨ�˲���
    return MPU_Set_LPF(rate / 2);                     //�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
    u8 buf[2];
    short raw;
    float temp;
    MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    raw = ((u16)buf[0] << 8) | buf[1];
    temp = 36.53 + ((double)raw) / 340;
    return temp * 100;
    ;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
    u8 buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        *gx = ((u16)buf[0] << 8) | buf[1];
        *gy = ((u16)buf[2] << 8) | buf[3];
        *gz = ((u16)buf[4] << 8) | buf[5];
    }
    return res;
    ;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
    u8 buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        *ax = ((u16)buf[0] << 8) | buf[1];
        *ay = ((u16)buf[2] << 8) | buf[3];
        *az = ((u16)buf[4] << 8) | buf[5];
    }
    return res;
    ;
}
//IIC����д
//addr:������ַ
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    u8 i;
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 0); //����������ַ+д����
    if (IIC_Wait_Ack())             //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg); //д�Ĵ�����ַ
    IIC_Wait_Ack();     //�ȴ�Ӧ��
    for (i = 0; i < len; i++)
    {
        IIC_Send_Byte(buf[i]); //��������
        if (IIC_Wait_Ack())    //�ȴ�ACK
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
}
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 0); //����������ַ+д����
    if (IIC_Wait_Ack())             //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg); //д�Ĵ�����ַ
    IIC_Wait_Ack();     //�ȴ�Ӧ��
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 1); //����������ַ+������
    IIC_Wait_Ack();                 //�ȴ�Ӧ��
    while (len)
    {
        if (len == 1)
            *buf = IIC_Read_Byte(0); //������,����nACK
        else
            *buf = IIC_Read_Byte(1); //������,����ACK
        len--;
        buf++;
    }
    IIC_Stop(); //����һ��ֹͣ����
    return 0;
}
//IICдһ���ֽ�
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Byte(u8 reg, u8 data)
{
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR << 1) | 0); //����������ַ+д����
    if (IIC_Wait_Ack())                 //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);  //д�Ĵ�����ַ
    IIC_Wait_Ack();      //�ȴ�Ӧ��
    IIC_Send_Byte(data); //��������
    if (IIC_Wait_Ack())  //�ȴ�ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}
//IIC��һ���ֽ�
//reg:�Ĵ�����ַ
//����ֵ:����������
u8 MPU_Read_Byte(u8 reg)
{
    u8 res;
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR << 1) | 0); //����������ַ+д����
    IIC_Wait_Ack();                     //�ȴ�Ӧ��
    IIC_Send_Byte(reg);                 //д�Ĵ�����ַ
    IIC_Wait_Ack();                     //�ȴ�Ӧ��
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR << 1) | 1); //����������ַ+������
    IIC_Wait_Ack();                     //�ȴ�Ӧ��
    res = IIC_Read_Byte(0);             //��ȡ����,����nACK
    IIC_Stop();                         //����һ��ֹͣ����
    return res;
}

//// Offset of angular velocity zero drift
//float Gyro_Xout_Offset, Gyro_Yout_Offset, Gyro_Zout_Offset;
//// Offset of acceleration zero drift
//float Accel_Xout_Offset, Accel_Yout_Offset, Accel_Zout_Offset;


