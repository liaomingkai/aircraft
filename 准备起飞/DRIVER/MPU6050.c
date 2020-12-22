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

    //很奇怪, 必须要先读一次, 然后才能正常读
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
        printf("\r设备HMC5773L设别成功!\r\n\r");
    }
    else
    {
        printf("\r错误!无法识别设备HMC5773L!\r\n\r");
    }
}

void HMC5883L_Init()
{
    IIC_Init();
    IIC_Start();
    IIC_Send_Byte(HMC5883L_Write_Address); //输入地址,选中HMC5883L
    IIC_Wait_Ack();

    IIC_Send_Byte(0x00); //选中配置寄存器A
    IIC_Wait_Ack();
    IIC_Send_Byte(0x18); //输入寄存器A的参数
    IIC_Wait_Ack();

    IIC_Send_Byte(0x01); //选中配置寄存器B
    IIC_Wait_Ack();
    IIC_Send_Byte(0x20); //输入寄存器B的参数(增益),1.3Ga=0X20; 1.9Ga-0X40; 8.1-0XE0
    IIC_Wait_Ack();

    IIC_Send_Byte(0x02); //选中模式寄存器
    IIC_Wait_Ack();
    IIC_Send_Byte(0x00); //输入模式: 0x01:单一测量模式; 0x00:连续测量模式
    IIC_Stop();
}

float Get_Current_Angle(int *mx, int *my, int *mz)
{
    u8 i;
    u8 a[6]; //存放从HMC5883L读来的数据
    //float Curent_Angle;
    short Curent_Angle;
    //HMC5883L_Init();

    IIC_Start();
    IIC_Send_Byte(HMC5883L_Write_Address); //输出模式(相对于CPU)
    IIC_Wait_Ack();

    IIC_Send_Byte(0x03); //输入地址指针0x03准备读取x轴数据
    IIC_Wait_Ack();

    IIC_Start();
    IIC_Send_Byte(HMC5883L_Read_Address); //输入模式(相对于CPU)
    IIC_Wait_Ack();

    for (i = 0; i < 6; i++) //读取数据
    {
        if (i == 5) //最后一个数据不需要Ack信号,发送nAck
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

    //将X,Y,Z的高低位合并
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
    //将补码转换为正常数字
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

    X = (s16)x; //x分量
    Y = (s16)y; //y分量
    Z = (s16)z; //z分量

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

    //  Curent_Angle = (atan2(y,x) * (180 / 3.14159265) + 180);  //实际水平角度(利用磁偏角公式)
    //	printf("%f\r\n",(float)Curent_Angle);
    return Curent_Angle;
}

void self_test()
{
    IIC_Init();
    IIC_Start();
    IIC_Send_Byte(HMC5883L_Write_Address); //输入地址,选中HMC5883L
    IIC_Wait_Ack();

    IIC_Send_Byte(0x00); //选中配置寄存器A
    IIC_Wait_Ack();
    IIC_Send_Byte(0x71); //输入寄存器A的参数, 加正向电压
    IIC_Wait_Ack();

    IIC_Send_Byte(0x01); //选中配置寄存器B
    IIC_Wait_Ack();
    IIC_Send_Byte(0x40); //输入寄存器B的参数(增益),1.9Ga
    IIC_Wait_Ack();

    IIC_Send_Byte(0x02); //选中模式寄存器
    IIC_Wait_Ack();
    IIC_Send_Byte(0x00); //输入模式: 0x01:单一测量模式; 0x00:连续测量模式
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

//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
u8 MPU_Init(void)
{
    u8 res;
    IIC_Init();                              //初始化IIC总线
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); //复位MPU6050
    delay_ms(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); //唤醒MPU6050
    MPU_Set_Gyro_Fsr(3);                     //陀螺仪传感器,±2000dps
    MPU_Set_Accel_Fsr(1);                    //加速度传感器,±4g
    MPU_Set_Rate(50);                        //设置采样率50Hz
    MPU_Write_Byte(MPU_INT_EN_REG, 0X00);    //关闭所有中断
    MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);   //关闭FIFO

    MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); //I2C主模式关闭
    MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X82); //INT引脚低电平有效,并且是旁路模式,HMC5883L直通主机

    res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if (res == MPU_ADDR) //器件ID正确
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); //设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); //加速度与陀螺仪都工作
        MPU_Set_Rate(50);                        //设置采样率为50Hz
			Get_Accel_Bias();
			Get_Gyro_Bias();
    }
    else
        return 1;
    return 0;
}
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3); //设置陀螺仪满量程范围
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3); //设置加速度传感器满量程范围
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败
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
    return MPU_Write_Byte(MPU_CFG_REG, data); //设置数字低通滤波器
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Rate(u16 rate)
{
    u8 data;
    if (rate > 1000)
        rate = 1000;
    if (rate < 4)
        rate = 4;
    data = 1000 / rate - 1;
    data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data); //设置数字低通滤波器
    return MPU_Set_LPF(rate / 2);                     //自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
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
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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
//IIC连续写
//addr:器件地址
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    u8 i;
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 0); //发送器件地址+写命令
    if (IIC_Wait_Ack())             //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg); //写寄存器地址
    IIC_Wait_Ack();     //等待应答
    for (i = 0; i < len; i++)
    {
        IIC_Send_Byte(buf[i]); //发送数据
        if (IIC_Wait_Ack())    //等待ACK
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
}
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 0); //发送器件地址+写命令
    if (IIC_Wait_Ack())             //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg); //写寄存器地址
    IIC_Wait_Ack();     //等待应答
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 1); //发送器件地址+读命令
    IIC_Wait_Ack();                 //等待应答
    while (len)
    {
        if (len == 1)
            *buf = IIC_Read_Byte(0); //读数据,发送nACK
        else
            *buf = IIC_Read_Byte(1); //读数据,发送ACK
        len--;
        buf++;
    }
    IIC_Stop(); //产生一个停止条件
    return 0;
}
//IIC写一个字节
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 reg, u8 data)
{
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR << 1) | 0); //发送器件地址+写命令
    if (IIC_Wait_Ack())                 //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);  //写寄存器地址
    IIC_Wait_Ack();      //等待应答
    IIC_Send_Byte(data); //发送数据
    if (IIC_Wait_Ack())  //等待ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}
//IIC读一个字节
//reg:寄存器地址
//返回值:读到的数据
u8 MPU_Read_Byte(u8 reg)
{
    u8 res;
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR << 1) | 0); //发送器件地址+写命令
    IIC_Wait_Ack();                     //等待应答
    IIC_Send_Byte(reg);                 //写寄存器地址
    IIC_Wait_Ack();                     //等待应答
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR << 1) | 1); //发送器件地址+读命令
    IIC_Wait_Ack();                     //等待应答
    res = IIC_Read_Byte(0);             //读取数据,发送nACK
    IIC_Stop();                         //产生一个停止条件
    return res;
}

//// Offset of angular velocity zero drift
//float Gyro_Xout_Offset, Gyro_Yout_Offset, Gyro_Zout_Offset;
//// Offset of acceleration zero drift
//float Accel_Xout_Offset, Accel_Yout_Offset, Accel_Zout_Offset;


