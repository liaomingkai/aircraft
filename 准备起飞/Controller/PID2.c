#include "PID2.h"
#include "Tim.h"
#include "ahrs.h"
#include "control.h"
#include "ahrs.h"


// 积分限幅
#define INTEGRAL_MAX     200.0f
#define INTEGRAL_MIN     -200.0f

// PID 输出限幅
#define PID_OUTPUT_MAX   800.0f
#define PID_OUTPUT_MIN   -800.0f

// PWM out limit
#define PWM_OUT_MIN      1000
#define PWM_OUT_MAX      2000



//PID 更新时间
float Time_dt;
// 控制姿态电机的PID输出
float PID_Roll, PID_Pitch, PID_Yaw;

// Motor speed
uint16_t Motor_1, Motor_2, Motor_3, Motor_4;


// 遥控器输入的期望姿态角
float Motor_Roll = 0.0;
float Motor_Pitch = 0.0;
float Motor_Thr = 0.0;
float Motor_Yaw = 0.0;

// Remote control median value
float Roll_Mid   = 1500.0;
float Pitch_Mid   = 1500.0;
float Yaw_Mid   = 1500.0;

//#define KP  0.8					//1.3    0.07   0.01  1.0  0.3 0.02
//#define KI  0.3
//#define KD  0.02

// Roll PID parameters
float Roll_Kp        = 2.4;	//1.9;
float Roll_Rate_Kp   = 1.05;	//0.70;
float Roll_Rate_Ti   = 0.2;
float Roll_Rate_Td   = 0.02;
// Pitch PID parameters
float Pitch_Kp       =  2.4;	//2.4;2.0
float Pitch_Rate_Kp  = 	1.0;	//0.70;
float Pitch_Rate_Ti  = 	0.3;
float Pitch_Rate_Td  = 	0.02;
// Yaw PID parameters
float Yaw_Kp         = 0.0;
float Yaw_Rate_Kp    = 0.70;
float Yaw_Rate_Ti    = 0.10;
float Yaw_Rate_Td    = 0.01;

// The sum of euler angle PID error
float Roll_Err_Sum   = 0.0;
float Pitch_Err_Sum  = 0.0;
float Yaw_Err_Sum    = 0.0;
// Last euler angle PID error
float Roll_Err_Last  = 0.0;
float Pitch_Err_Last = 0.0;
float Yaw_Err_Last   = 0.0;


void Motor_Calculate(void)
{
    // 得到PID更新间隔时间
    Time_dt = Get_PID_Time();

    // XYZ轴的PID更新
    PID_Roll_Calculate();
    PID_Pitch_Calculate();
    PID_Yaw_Calculate();
	
		/*
			 1             2
				\	   /|\    /
					\ 	|   /
						\	| /
		----------|----------
						/	| \
					/		|	  \				
				/			|     \
			4               3

		*/
		//传过来的数据： 向右：右倾量为正， 向前飞：前飞量为正， 向右旋转：旋转量为正
		//所以姿态解算传过来的角度：向右横滚：Roll为 正   向前飞：Pitch为 
		
		// Motor speed fusion formula in X mode
    // From 1-4 are: clockwise left front, counterclockwise right front, counterclockwise left rear, clockwise right rear
    Motor_1 = (uint16_t)Limit_PWM(Motor_Thr + PID_Pitch - PID_Roll + PID_Yaw);
    Motor_2 = (uint16_t)Limit_PWM(Motor_Thr + PID_Pitch + PID_Roll - PID_Yaw);
    Motor_3 = (uint16_t)Limit_PWM(Motor_Thr - PID_Pitch + PID_Roll + PID_Yaw);
    Motor_4 = (uint16_t)Limit_PWM(Motor_Thr - PID_Pitch - PID_Roll - PID_Yaw);
    
    // Motor speed safety protection before takeoff
    if(Motor_Thr <= 1050)
    {
				Roll_Err_Sum = 0;
				Pitch_Err_Sum = 0;
				Yaw_Err_Sum = 0;
        Motor_1 = 1000;
        Motor_2 = 1000;
        Motor_3 = 1000;
        Motor_4 = 1000;
    }
}


// PWM output limit
float Limit_PWM(float accelerator)
{
    if(accelerator > PWM_OUT_MAX)
    {
        accelerator = PWM_OUT_MAX;
    }
    else if(accelerator < PWM_OUT_MIN)
    {
        accelerator = PWM_OUT_MIN;
    }
    else
    {
        accelerator = accelerator;
    }

    return accelerator;
}

// Roll PID calculation
void PID_Roll_Calculate(void)
{
	
    float Proportion;
    float Integral;
    float Derivative;
    float Error, Output;
		//需要传来Roll,gy等值
    // Outer ring result is input to the inner ring as the error value
    Error = Roll_Kp * (Angle_Roll - Motor_Roll) + GYRO_X;		//Motor_Ail是接收机传来的期望姿态角，Roll是AHRS算出的实际姿态角
    
		//Error是外环输出
    // The sum of error
    Roll_Err_Sum += Error;		//积分量
    
    // PID calculation
    Proportion = Roll_Rate_Kp * Error;
    Integral   = Roll_Rate_Ti * Roll_Err_Sum * Time_dt;
    Derivative = Roll_Rate_Td * (Error - Roll_Err_Last) / Time_dt;
    
    // Integral limit
    if(Integral > INTEGRAL_MAX)
    {
        Integral = INTEGRAL_MAX;
    }
    if(Integral < INTEGRAL_MIN)
    {
        Integral = INTEGRAL_MIN;
    }
    
    // P + I + D = output
    Output = Proportion + Integral + Derivative;

    // PID output limit
    if(Output > PID_OUTPUT_MAX)
    {
        Output = PID_OUTPUT_MAX;
    }
    if(Output < PID_OUTPUT_MIN)
    {
        Output = PID_OUTPUT_MIN;
    }

    // Record last error
    Roll_Err_Last = Error;
    
    // Output PID value
    PID_Roll = Output;
		
}

// Pitch PID控制
void PID_Pitch_Calculate(void)
{

		float Proportion;
    float Integral;
    float Derivative;
    float Error, Output;

    Error = Pitch_Kp * (Angle_Pitch - Motor_Pitch) + GYRO_Y;

    Pitch_Err_Sum += Error;

    Proportion = Pitch_Rate_Kp * Error;
    Integral   = Pitch_Rate_Ti * Pitch_Err_Sum * Time_dt;
    Derivative = Pitch_Rate_Td * (Error - Pitch_Err_Last) / Time_dt;

    if(Integral > INTEGRAL_MAX)
    {
        Integral = INTEGRAL_MAX;
    }
    if(Integral < INTEGRAL_MIN)
    {
        Integral = INTEGRAL_MIN;
    }
    
    Output = Proportion + Integral + Derivative;

    if(Output > PID_OUTPUT_MAX)
    {
        Output = PID_OUTPUT_MAX;
    }
    if(Output < PID_OUTPUT_MIN)
    {
        Output = PID_OUTPUT_MIN;
    }

    Pitch_Err_Last = Error;
    
    PID_Pitch = Output;
	
}

// Yaw PID 控制
void PID_Yaw_Calculate(void)
{
	
    float Proportion;
    float Integral;
    float Derivative;
    float Error, Output;

    // Yaw 控制不需要外环
    Error = GYRO_Z - Motor_Yaw;
    
    Yaw_Err_Sum += Error;

    Proportion = Yaw_Rate_Kp * Error;
    Integral   = Yaw_Rate_Ti * Yaw_Err_Sum * Time_dt;
    Derivative = Yaw_Rate_Td * (Error - Yaw_Err_Last) / Time_dt;
    
    if(Integral > INTEGRAL_MAX)
    {
        Integral = INTEGRAL_MAX;
    }
    if(Integral < INTEGRAL_MIN)
    {
        Integral = INTEGRAL_MIN;
    }
    
    Output = Proportion + Integral + Derivative;

    if(Output > PID_OUTPUT_MAX)
    {
        Output = PID_OUTPUT_MAX;
    }
    if(Output < PID_OUTPUT_MIN)
    {
        Output = PID_OUTPUT_MIN;
    }

    Yaw_Err_Last = Error;
    
    PID_Yaw = Output;

}


void Motor_Expectation_Calculate(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4)
{
    // 输入信号的限幅
    if(ch1 < 1000) { ch1=1000; }
    if(ch1 > 2000) { ch1=2000; }

    if(ch2 < 1000) { ch2=1000; }
    if(ch2 > 2000) { ch2=2000; }

    if(ch3 < 1000) { ch3=1000; }
    if(ch3 > 2000) { ch3=2000; }

    if(ch4 < 1000) { ch4=1000; }
    if(ch4 > 2000) { ch4=2000; }

    // Three-channel remote control value zero offset processing and range reduction
    // (Throttle channel value is not processed)
    Motor_Roll = (float)((ch1 - Roll_Mid) / 25 );  //-20度~20度
    Motor_Pitch = (float)((ch2 - Pitch_Mid) / 25);
    Motor_Thr = (float)ch3;
    Motor_Yaw = (float)((ch4 - Yaw_Mid) / 25);
		
//		//测试用
//		Motor_Yaw = Angle_Yaw;
//		Motor_Roll = 0;
//		Motor_Pitch = 0;
}



