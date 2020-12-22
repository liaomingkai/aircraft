#include "stm32F4xx.h"

void PID_Time_Init(void);
float Get_PID_Time(void);

void MY_TIM3_Init(u16 arr, u16 psc);
void MY_TIM3_GPIO_Init(void);
void MY_TIM3_PWM_Init(u16 arr, u16 psc);

void MY_TIM3_Init(u16 arr, u16 psc);
void MY_TIM3_GPIO_Init(void);
void MY_TIM3_PWM_Init(u16 arr, u16 psc);
void TIM1_Cap_Init(u16 arr, u16 psc);
