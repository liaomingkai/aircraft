#include "stm32f4xx.h"

#define Moto_PwmMax 2000		//PWM输出最大值
#define Moto_PwmMin 1000		//PWM输出最小值

void Moto_Reflash(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM);
void PWM_Output(uint16_t DR1,uint16_t DR2,uint16_t DR3,uint16_t DR4);

