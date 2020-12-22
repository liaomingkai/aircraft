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
	TIM1_Cap_Init(0xFFFF, 84-1);				//Ӧ��Ϊarr:16λ����Ϊ��� psc������Ϊ1MHz
	MY_TIM3_PWM_Init(20000-1, 84-1);		//������ARR��PSC  ռ�ձ����2000�� 50HZ    84MHZ
	PID_Time_Init();
	AHRS_Time_Init();
	MPU_Init();
	HMC5883L_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	
	
	while(1)
	{
		
		//�õ���̬�������̬��
		get_angle();
		//printf("AHRS����õ�����̬Roll: %f Pitch: %f Yaw: %f\r\n", Angle_Roll, Angle_Pitch, Angle_Yaw);
		//printf("GYRO_X: %f GYRO_Y: %f GYRO_Z %f \r\n\r\n", GYRO_X, GYRO_Y, GYRO_Z);
		//�õ����ջ�������������̬
		Motor_Expectation_Calculate(PWMInCh1, PWMInCh2, PWMInCh3, PWMInCh4);
		//printf("���ջ������ģ����CH1:%d, ����CH2:%d, ����CH3: %d, ƫ��CH4:%d \r\n", PWMInCh1, PWMInCh2, PWMInCh3, PWMInCh4);
		//printf("������̬�ǣ����ţ�%f �����%f ������%f ƫ����%f\r\n", Motor_Thr, Motor_Roll, Motor_Pitch, Motor_Yaw);
		
		//PID����
		Motor_Calculate();
		
		//PID�����д����
		PWM_Output(Motor_1, Motor_2, Motor_3, Motor_4);	
		//printf("Motor_1: %d		Motor_2: %d		Motor_3: %d		Motor_4: %d \r\n", Motor_1, Motor_2, Motor_3, Motor_4);		
	}	
}

