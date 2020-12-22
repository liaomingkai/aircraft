#include "Motor.h"
#include "usart.h"
#include "delay.h"


void Moto_Reflash(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM)
{		
	if(MOTO1_PWM>Moto_PwmMax)	MOTO1_PWM = Moto_PwmMax;
	if(MOTO2_PWM>Moto_PwmMax)	MOTO2_PWM = Moto_PwmMax;
	if(MOTO3_PWM>Moto_PwmMax)	MOTO3_PWM = Moto_PwmMax;
	if(MOTO4_PWM>Moto_PwmMax)	MOTO4_PWM = Moto_PwmMax;
	if(MOTO1_PWM<Moto_PwmMin)	MOTO1_PWM = Moto_PwmMin;
	if(MOTO2_PWM<Moto_PwmMin)	MOTO2_PWM = Moto_PwmMin;
	if(MOTO3_PWM<Moto_PwmMin)	MOTO3_PWM = Moto_PwmMin;
	if(MOTO4_PWM<Moto_PwmMin)	MOTO4_PWM = Moto_PwmMin;
	
	//printf("\r\n 电机1：%d  电机2：%d  电机3：%d  电机4：%d  \r\n", MOTO1_PWM, MOTO2_PWM, MOTO3_PWM, MOTO4_PWM);
	TIM_SetCompare1(TIM3,MOTO1_PWM);
	TIM_SetCompare2(TIM3,MOTO2_PWM);
	TIM_SetCompare3(TIM3,MOTO3_PWM);
	TIM_SetCompare4(TIM3,MOTO4_PWM);
}

void PWM_Output(uint16_t DR1,uint16_t DR2,uint16_t DR3,uint16_t DR4)
{
    TIM_SetCompare1(TIM3, DR1);
    TIM_SetCompare2(TIM3, DR2);
    TIM_SetCompare3(TIM3, DR3);
    TIM_SetCompare4(TIM3, DR4);
}

