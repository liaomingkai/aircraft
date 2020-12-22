#include "stm32f4xx.h"
#include "ucos_ii.h"
#include "os_my_task.h"

void Delay_ms(u16 time)
{
	u16 i = 0;
	while(time--)
	{
		i = 12000;
		while(i--);
	}
}

void LED_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//�ٶ�50MHZ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;				//���⸴�����
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;  //��ʼ��
	GPIO_Init(GPIOC, &GPIO_InitStructure);
		
	GPIO_ResetBits(GPIOC,GPIO_Pin_10);
  GPIO_ResetBits(GPIOC,GPIO_Pin_11);
	GPIO_ResetBits(GPIOC,GPIO_Pin_12);		
}


void LED1(void *pdata) //��ɫ
{
	SysTick_init();
	while(1)
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_10);
		Delay_ms(500);
		GPIO_ResetBits(GPIOC,GPIO_Pin_10);
		OSTimeDly(1000);
	}
}

void LED2(void *pdata) //��ɫ
{
	SysTick_init();
	while(1)
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_11);
		Delay_ms(500);
		GPIO_ResetBits(GPIOC,GPIO_Pin_11);
		OSTimeDly(1000);
	}
}

void LED3(void *pdata)   //��ɫ
{
	SysTick_init();
	while(1)
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_12);
		Delay_ms(500);
		GPIO_ResetBits(GPIOC,GPIO_Pin_12);
		OSTimeDly(1000);
	}
}


