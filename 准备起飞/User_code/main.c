#include "stm32f4xx.h"
#include "ucos_ii.h"
#include "usart.h"
#include "sys.h"
#include "delay.h"
#include "PWM_Capture.h"
#include "ahrs.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Tim.h"
#include "PID2.h"


int main(void)
{
	delay_init(84);
	uart_init(9600);
	TIM1_Cap_Init(0xFFFF, 84-1);				//应该为arr:16位设置为最大， psc设置来为1MHz
	MY_TIM3_PWM_Init(20000-1, 84-1);		//设置了ARR和PSC  占空比最大2000； 50HZ    84MHZ
	PID_Time_Init();
	AHRS_Time_Init();
	MPU_Init();
	HMC5883L_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	
	
	while(1)
	{
		
		//得到姿态解算的姿态角
		get_angle();
		//printf("AHRS计算得到的姿态Roll: %f Pitch: %f Yaw: %f\r\n", Angle_Roll, Angle_Pitch, Angle_Yaw);
		//printf("GYRO_X: %f GYRO_Y: %f GYRO_Z %f \r\n\r\n", GYRO_X, GYRO_Y, GYRO_Z);
		//得到接收机传来的期望姿态
		Motor_Expectation_Calculate(PWMInCh1, PWMInCh2, PWMInCh3, PWMInCh4);
		//printf("接收机传来的：横滚CH1:%d, 俯仰CH2:%d, 油门CH3: %d, 偏航CH4:%d \r\n", PWMInCh1, PWMInCh2, PWMInCh3, PWMInCh4);
		//printf("期望姿态角：油门：%f 横滚：%f 俯仰：%f 偏航：%f\r\n", Motor_Thr, Motor_Roll, Motor_Pitch, Motor_Yaw);
		
		//PID控制
		Motor_Calculate();
		
		//PID输出，写入电机
		PWM_Output(Motor_1, Motor_2, Motor_3, Motor_4);	
		//printf("Motor_1: %d		Motor_2: %d		Motor_3: %d		Motor_4: %d \r\n", Motor_1, Motor_2, Motor_3, Motor_4);		
	}	
}

