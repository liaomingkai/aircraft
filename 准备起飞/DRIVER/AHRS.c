#include "ahrs.h"

static float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; //��Ԫ������
static int i = 0;
float Angle_Pitch, Angle_Roll, Angle_Yaw;
float GYRO_X, GYRO_Y, GYRO_Z;

//---------------------------------------------------------------------------------------------------
// Definitions

//#define sampleFreq 1100.0f // sample frequency in Hz; ���ݵĻ�ȡ������ʱ����Ժ���
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
    float recipNorm;                                                  //���ڹ�һ��
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3; //��ǰ������Ԫ��
    float hx, hy, bx, bz;                                             //��������ϵ�µĴ�ǿ��ֵ
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;             //���������Ǹ��µ���Ԫ�����Ƴ���������ϵ�µļ��ٶ����ʹ����Ƶ�ֵ
    float halfex, halfey, halfez;                                     //����ʵ�ʲ�õļ��ٶ�ֵ�ʹŴ��������Ƴ��Ĳ�����
    float qa, qb, qc;                                                 //��Ԫ��
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

    // ��������ĴŴ�����ֵΪ0�������IMU�㷨
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    // ���ڼ��ٶȼƲ�����Чʱ�ż��㷴����������ٶȼƹ�һ���е�NaN��
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement �������ٶȹ�һ��
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement  �Ŵ�������һ��
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // ���������������ظ�����
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

        // ����ų��Ĳο�����
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
        /*hx��hy��hz�ǵ�������ϵ�µĴŴ�����ֵ�������ɻ�������ϵ�µ�mx��my��mz���CbR�õ�
        ������������µĻ����ܹ��͵��صĵ�������ϵ����ͬһXOYƽ�棬�һ�ͷָ������ô��ʱ�ĴŴ�����ֵӦ��Ϊbx��0��bz
        �ٸ���bx��0��bz����������ϵ���ҳ�CbR�õ����ƺ�ĴŴ�����ֵhalfwx��halfwy��halfwz
        */

        // ���Ƶ������ʹų�����
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
        /*��ʱ���������ٶ�ֵӦ��Ϊ0,0��1����һ���󣩣�
        ��ô���ݴ˵�������ϵ�µ��������ٶ�ֵ���ҳ�CbR���ɵõ���ʱ��������ϵ�µ��������ٶȹ���ֵ
        */

        // ����ǳ�ʸ���Ĺ��Ʒ������������֮��Ĳ��֮��
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled ʹ��PI�������ǵĽ�����в���(�˴��Ѿ�����PI������)
        if (twoKi > 0.0f)
        {
            // Ki�������
//            integralFBx += twoKi * halfex * (1.0f / sampleFreq);
//            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
//            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            halfT = Get_AHRS_Time();
            integralFBx += twoKi * halfex * halfT;
            integralFBy += twoKi * halfey * halfT;
            integralFBz += twoKi * halfez * halfT;

            // Ӧ�û��ַ���
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            // ��ֹ�������
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Ӧ�ñ�������
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // ��Ԫ���Ļ��ֱ仯��
//    gx *= (0.5f * (1.0f / sampleFreq)); // Ԥ�˹�����
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
    q3 += (qa * gz + qb * gy - qc * gx); //������Ԫ��

    // ��Ԫ����һ��
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

    // ���ڼ��ٶȼƲ�����Чʱ�ż��㷴����������ٶȼƹ�һ���е�NaN��
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // ���Ƶ���������ʹ�ֱ�ڴ�ͨ��������
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // ����ǹ������������������������Ĳ��֮��
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
    q3 += (qa * gz + qb * gy - qc * gx); //������Ԫ��

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//�� x ����������
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

//����ŷ����
void CalcuEulerAngle2(float *pitch, float *roll, float *yaw)
{
    //	float q0 = quat[0], q1 = quat[1], q2 = quat[2], q3 = quat[3];
    //1.����ԭ����Ԫ��תŷ����
    //����õ�������/�����/�����
    //		*pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
    //		*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
    //		*yaw   = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;	//yaw

    //2.���ݲ��ĵĴ���
    //����
    const double Epsilon = 0.0009765625f;
    const double Threshold = 0.5f - Epsilon;
    const double PI = 3.1415926;
    double TEST = q0 * q2 - q1 * q3;

    //������̬,������Ϊ��90��
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


/*��̬���㺯��, ��ȡ���������ݲ�������̬����,�õ�ŷ����
*���, ������ȫ�ֱ��� Q_angle �� MPU6050_GYRO_LAST ��
*/
void get_angle()
{
    float pitch = 0.f, roll = 0.f, yaw = 0.f; //ŷ����
    short aacx, aacy, aacz;                   //���ٶ�ԭʼ����
    short gyrox, gyroy, gyroz;                //������ԭʼ����
    int mx = 0, my = 0, mz = 0;               //������ԭʼ����
    float ax, ay, az;
    float gx, gy, gz;
    float mxf, myf, mzf;

    /*************���ٶȼ���������*************/
    //һ��Ҫ�ȳ�ʼ��MPU6050!!!
    //MPU_Init(); //ֻ��Ҫ��ʼ��һ�μ���

    MPU_Get_Accelerometer(&aacx, &aacy, &aacz); //�õ����ٶȴ���������
    MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);  //�õ�����������
//    HMC5883L_Init();                            //ÿ�ζ����ݶ�Ҫ��ʼ�� HMC5883L, ��Ϊ���ó��˵��ζ�ȡģʽ
    Get_Current_Angle(&mx, &my, &mz);           //�õ�����������

    //������ת���ɶ�ÿ��, ����Ϊ2000��ÿ��
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
		
    //���ٶȼ�ת����g, ����Ϊ2g
//    ax = (aacx * 2 * 9.8) / 32768 + 0.1f;
//    ay = (aacy * 2 * 9.8) / 32768 - 0.1f;
//    az = (aacz * 2 * 9.8) / 32768 + 0.78566f;

    ax = ((aacx - Accel_Xout_Offset) / 8192);
    ay = ((aacy - Accel_Yout_Offset) / 8192);
    az = ((aacz + (8192 - Accel_Zout_Offset)) / 8192);

    float count = 1090.0f; //����ֵ
    mxf = (mx) / count;
    myf = (my) / count;
    mzf = (mz) / count;

    //��̬����
    MahonyAHRSupdate(gx, gy, gz, ax, ay, az, mxf, myf, mzf);

    //��Ԫ��ת����ŷ����
    CalcuEulerAngle2(&pitch, &roll, &yaw);
    //��¼���ṹ��, ���ݸ� PID
    Angle_Roll = roll;
    Angle_Pitch = pitch;
    Angle_Yaw = -yaw; //���� Yaw ����

//	printf("G: %f %f %f\nA: %f %f %f\nM: %f %f %f\n", gx, gy, gz, ax, ay, az, mxf, myf, mzf);
//	printf("Pitch:%f Roll:%f Yaw:%f\n\n", Angle_Pitch,Angle_Roll, Angle_Yaw);
    usart1_report_ahrs((int)ax, (int)ay, (int)az, (int)gx, (int)gy, (int)gz, (int)mxf, (int)myf, (int)mzf,
                       (int)(roll * 100), (int)(pitch * 100), (int)(yaw * 10));
}