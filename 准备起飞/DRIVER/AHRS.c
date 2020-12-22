#include "ahrs.h"

static float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; //四元数数组
static int i = 0;
float Angle_Pitch, Angle_Roll, Angle_Yaw;
float GYRO_X, GYRO_Y, GYRO_Z;

//---------------------------------------------------------------------------------------------------
// Definitions

//#define sampleFreq 1100.0f // sample frequency in Hz; 数据的获取及计算时间可以忽略
#define twoKpDef (2.0f)    // 2 * proportional gain
#define twoKiDef (0.005f)    // 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

//volatile float twoKp = twoKpDef;                                           // 2 * proportional gain (Kp)
//volatile float twoKi = twoKiDef;                                           // 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                 // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions



void AHRS_Time_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // Enable TIM5 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    // Close TIM5
    TIM_DeInit(TIM5);
    // TIM5 configuration. Prescaler is 80, period is 0xFFFF, and counter mode is up
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    // Enable TIM5
    TIM_Cmd(TIM5, ENABLE);
}

float Get_AHRS_Time(void)
{
    float temp = 0;
    static uint32_t now = 0;

    // Get timer count
    now = TIM5->CNT;
    // Clear timer count
    TIM5->CNT = 0;
    // Convert to HZ unit and divided by 2
    temp = (float)now / 2000000.0f;

    return temp;
}

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;                                                  //用于归一化
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3; //提前计算四元数
    float hx, hy, bx, bz;                                             //地理坐标系下的磁强度值
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;             //根据陀螺仪更新的四元数估计出机体坐标系下的加速度器和磁力计的值
    float halfex, halfey, halfez;                                     //根据实际测得的加速度值和磁传感器估计出的叉乘误差
    float qa, qb, qc;                                                 //四元数
    float twoKp = 40.0f;
    float twoKi = 0.02f;
    float halfT;

    if(i < 100)
        i++;
    else
    {
        twoKp = 2.0f;
        twoKi = 0.007f;
    }

    // 如果测量的磁传感器值为0，则调用IMU算法
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    // 仅在加速度计测量有效时才计算反馈（避免加速度计归一化中的NaN）
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement 重力加速度归一化
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement  磁传感器归一化
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // 辅助变量，避免重复算术
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // 地球磁场的参考方向
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
        /*hx，hy，hz是地理坐标系下的磁传感器值，可以由机体坐标系下的mx，my，mz左乘CbR得到
        假设理想情况下的机体能够和当地的地理坐标系处于同一XOY平面，且机头指北，那么此时的磁传感器值应该为bx，0，bz
        再根据bx，0，bz（地理坐标系）右乘CbR得到估计后的磁传感器值halfwx，halfwy，halfwz
        */

        // 估计的重力和磁场方向
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
        /*此时的重力加速度值应该为0,0，1（归一化后），
        那么根据此地理坐标系下的重力加速度值，右乘CbR即可得到此时机体坐标系下的重力加速度估计值
        */

        // 误差是场矢量的估计方向与测量方向之间的叉积之和
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled 使用PI对陀螺仪的结果进行补偿(此处已经用了PI处理了)
        if (twoKi > 0.0f)
        {
            // Ki积分误差
//            integralFBx += twoKi * halfex * (1.0f / sampleFreq);
//            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
//            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            halfT = Get_AHRS_Time();
            integralFBx += twoKi * halfex * halfT;
            integralFBy += twoKi * halfey * halfT;
            integralFBz += twoKi * halfez * halfT;

            // 应用积分反馈
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            // 防止整体缠绕
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // 应用比例反馈
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // 四元数的积分变化率
//    gx *= (0.5f * (1.0f / sampleFreq)); // 预乘公因子
//    gy *= (0.5f * (1.0f / sampleFreq));
		gx *= halfT;
		gy *= halfT;
    gz *= halfT;
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx); //更新四元数

    // 四元数归一化
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    float twoKp = 4.0f;
    float twoKi = 0.00f;
	float halfT;

    if(i < 15)
        i++;
    else
    {
        twoKp = 2.0f;
        twoKi = 0.005f;
    }

    // 仅在加速度计测量有效时才计算反馈（避免加速度计归一化中的NaN）
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 估计的重力方向和垂直于磁通量的向量
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // 误差是估计重力方向与测量重力方向的叉积之和
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);
				
        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f)
        {
					 halfT = Get_AHRS_Time();
            integralFBx += twoKi * halfex * halfT; // integral error scaled by Ki
            integralFBy += twoKi * halfey * halfT;
            integralFBz += twoKi * halfez * halfT;
            gx += integralFBx; // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= halfT; // pre-multiply common factors
    gy *= halfT;
    gz *= halfT;
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx); //更新四元数

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//对 x 开更后求倒数
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//计算欧拉角
void CalcuEulerAngle2(float *pitch, float *roll, float *yaw)
{
    //	float q0 = quat[0], q1 = quat[1], q2 = quat[2], q3 = quat[3];
    //1.正点原子四元数转欧拉角
    //计算得到俯仰角/横滚角/航向角
    //		*pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
    //		*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
    //		*yaw   = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;	//yaw

    //2.根据博文的代码
    //常量
    const double Epsilon = 0.0009765625f;
    const double Threshold = 0.5f - Epsilon;
    const double PI = 3.1415926;
    double TEST = q0 * q2 - q1 * q3;

    //起义姿态,俯仰角为±90°
    if (TEST < -Threshold || TEST > Threshold)
    {
        int sign;

        if (TEST > 0)
        {
            sign = 1;
        }
        else if (TEST < 0)
        {
            sign = -1;
        }
        else
        {
            sign = 0;
        }

        *yaw = -2 * sign * (float)atan2(q1, q0) * 53.7;
        *pitch = sign * (PI / 2.0f) * 57.3;
        *roll = 0.0f;
    }
    else
    {
        *roll = atan2(2 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 57.3;
        *pitch = asin(-2 * (q1 * q3 - q0 * q2)) * 57.3;
        *yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;
    }
}


/*姿态解算函数, 读取传感器数据并进行姿态解算,得到欧拉角
*输出, 隐含在全局变量 Q_angle 和 MPU6050_GYRO_LAST 中
*/
void get_angle()
{
    float pitch = 0.f, roll = 0.f, yaw = 0.f; //欧拉角
    short aacx, aacy, aacz;                   //加速度原始数据
    short gyrox, gyroy, gyroz;                //陀螺仪原始数据
    int mx = 0, my = 0, mz = 0;               //磁力计原始数据
    float ax, ay, az;
    float gx, gy, gz;
    float mxf, myf, mzf;

    /*************加速度及和陀螺仪*************/
    //一定要先初始化MPU6050!!!
    //MPU_Init(); //只需要初始化一次即可

    MPU_Get_Accelerometer(&aacx, &aacy, &aacz); //得到加速度传感器数据
    MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);  //得到陀螺仪数据
//    HMC5883L_Init();                            //每次读数据都要初始化 HMC5883L, 因为设置成了单次读取模式
    Get_Current_Angle(&mx, &my, &mz);           //得到磁力计数据

    //陀螺仪转换成度每秒, 量程为2000度每秒
//    gx = ((gyrox * 2000) / 32768 + 2);
//    gy = (gyroy * 2000) / 32768;
//    gz = (gyroz * 2000) / 32768;

//		gx = (gyrox / 16.384 + 2) / 57.295780f;
//    gy = (gyroy / 16.384) / 57.295780f;
//    gz = (gyroz / 16.384) / 57.295780f;
    float ahrs_deg = 57.295780f;
    GYRO_X = (gyrox - Gyro_Xout_Offset) / 16.384;
    GYRO_Y = (gyroy - Gyro_Yout_Offset) / 16.384;
    GYRO_Z = (gyroz - Gyro_Zout_Offset) / 16.384;
		
		gx = GYRO_X / ahrs_deg;
    gy = GYRO_Y / ahrs_deg;
    gz = GYRO_Z / ahrs_deg;
		
    //加速度计转换成g, 量程为2g
//    ax = (aacx * 2 * 9.8) / 32768 + 0.1f;
//    ay = (aacy * 2 * 9.8) / 32768 - 0.1f;
//    az = (aacz * 2 * 9.8) / 32768 + 0.78566f;

    ax = ((aacx - Accel_Xout_Offset) / 8192);
    ay = ((aacy - Accel_Yout_Offset) / 8192);
    az = ((aacz + (8192 - Accel_Zout_Offset)) / 8192);

    float count = 1090.0f; //增益值
    mxf = (mx) / count;
    myf = (my) / count;
    mzf = (mz) / count;

    //姿态解算
    MahonyAHRSupdate(gx, gy, gz, ax, ay, az, mxf, myf, mzf);

    //四元数转换成欧拉角
    CalcuEulerAngle2(&pitch, &roll, &yaw);
    //记录到结构中, 传递给 PID
    Angle_Roll = roll;
    Angle_Pitch = pitch;
    Angle_Yaw = -yaw; //右旋 Yaw 增大

//	printf("G: %f %f %f\nA: %f %f %f\nM: %f %f %f\n", gx, gy, gz, ax, ay, az, mxf, myf, mzf);
//	printf("Pitch:%f Roll:%f Yaw:%f\n\n", Angle_Pitch,Angle_Roll, Angle_Yaw);
    usart1_report_ahrs((int)ax, (int)ay, (int)az, (int)gx, (int)gy, (int)gz, (int)mxf, (int)myf, (int)mzf,
                       (int)(roll * 100), (int)(pitch * 100), (int)(yaw * 10));
}