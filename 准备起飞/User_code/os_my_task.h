#include "stm32f4xx.h"
#include "ucos_ii.h"

//定义任务优先级
#define	 LED_1_PRIO 		12
#define  LED_2_PRIO     15
#define  LED_3_PRIO     17
#define  PWM_OUT_PRIO		18

//分配给任务的堆栈大小
#define TASK_STK_SIZE  128

void LED1(void *pdata);
void LED2(void *pdata);
void LED3(void *pdata);
void LED_GPIO_Init(void);
void MY_TIM3_Init(u16 arr, u16 psc);
void MY_TIM3_GPIO_Init(void);
void MY_TIM3_PWM_Init(u16 arr, u16 psc);
void PWM_Out();
void Delay_ms(u16 time);
void LED_GPIO_Init(void);

