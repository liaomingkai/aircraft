#include "Tim.h"
#include "PWM_Capture.h"
#include "delay.h"


void PID_Time_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // Enable TIM4 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // Close TIM4
    TIM_DeInit(TIM4);
    // TIM4 configuration. Prescaler is 80, period is 0xFFFF, and counter mode is up
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // Enable TIM4
    TIM_Cmd(TIM4, ENABLE);
}

// Get PID update time
float Get_PID_Time(void)
{
    float temp = 0;
    static uint32_t now = 0;

    // Get timer count
    now = TIM4->CNT;
    // Clear timer count
    TIM4->CNT = 0;
    // Convert to HZ unit
    temp = (float)now / 1000000.0f;

    return temp;
}


void MY_TIM3_Init(u16 arr, u16 psc){
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period  = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  		 //���ϼ���
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	
}

//��ʼ��IO��
void MY_TIM3_GPIO_Init(void){
	
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;					//����
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//�ٶ�50MHZ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;				//���⸴�����
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;					//����
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;  //��ʼ��
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

//PWM�����ʼ������
void MY_TIM3_PWM_Init(u16 arr, u16 psc){
	
	TIM_OCInitTypeDef TIM_OCInitstructure;
	
	MY_TIM3_Init(arr, psc);
	MY_TIM3_GPIO_Init();
	
	//��ʼ��PWM��ģʽ
	/**
	ѡ��PWMģʽ��
		PWM1ģʽ��
					���ϼ���ʱ����ǰ����ֵС�����÷�ֵΪ��Ч��ƽ������Ϊ��Ч��ƽ�����¼��������ϼ����෴
		PWM2ģʽ��
					��PWM1ģʽ�������¼���ʱ��ȫ�෴
	*/
	
	//��ʼ��TIM3 Channel 1 2 3 PWMģʽ
	TIM_OCInitstructure.TIM_OCMode = TIM_OCMode_PWM1;			
	TIM_OCInitstructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitstructure.TIM_OCPolarity = TIM_OCPolarity_High;   //�����ƽΪ�ߣ�����Ч��ƽΪ��
	
	TIM_OC1Init(TIM3, &TIM_OCInitstructure);				//��������ͨ��1���
	TIM_OC2Init(TIM3, &TIM_OCInitstructure);
	TIM_OC3Init(TIM3, &TIM_OCInitstructure);
	TIM_OC4Init(TIM3, &TIM_OCInitstructure);
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);			//ʹ��Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_Cmd(TIM3, ENABLE);
	
//	delay_ms(2000);
//	TIM_SetCompare1(TIM3,2000);
//	TIM_SetCompare2(TIM3,2000);
//	TIM_SetCompare3(TIM3,2000);
//	TIM_SetCompare4(TIM3,2000);
	
	delay_ms(3000);
	
	
//	TIM_SetCompare1(TIM3,1000);
//	TIM_SetCompare2(TIM3,1000);
//	TIM_SetCompare3(TIM3,1000);
//	TIM_SetCompare4(TIM3,1000);
	
	
}






void TIM1_Cap_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM1_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);		//ʱ��TIM1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);		//ʹ��GPIOA
	
	/*****************���ų�ʼ��******************/
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;							//PA8
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//����
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8,  GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);
	
	TIM_DeInit(TIM1);
	
	/*****************��ʱ��TIM1��ʼ��******************/
	TIM_TimeBaseStructure.TIM_Period = arr;							//���ü������Զ���װֵ 	ͨ��Ϊ��0xFFFF
	TIM_TimeBaseStructure.TIM_Prescaler = psc;  					//Ԥ��Ƶ����psc = 1M = 1us
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;		//�ظ���������

		
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //������ʼ��
	TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);		//���TIM1�����жϱ�־
	
	/*****************TIM1���������ʼ��******************/
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_1; 			//cc1s = 01  ͨ��ѡ��
	TIM1_ICInitStructure.TIM_ICPolarity =TIM_ICPolarity_Rising;			//�����ز���
	TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//ӳ�䵽TI1��
	TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;					//����Ƶ��ֱ�Ӳ���
	TIM1_ICInitStructure.TIM_ICFilter = 0x0B;												//IC1F = 000 ���������˲��� ���˲�
	TIM_ICInit(TIM1, &TIM1_ICInitStructure);
	

		
	//ͨ��2
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM1_ICInitStructure.TIM_ICFilter = 0x0B;
	TIM_ICInit(TIM1, &TIM1_ICInitStructure);
		
	//ͨ��3
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM1_ICInitStructure.TIM_ICFilter = 0x0B;
	TIM_ICInit(TIM1, &TIM1_ICInitStructure);
	
	//ͨ��4
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM1_ICInitStructure.TIM_ICFilter = 0x0B;
	TIM_ICInit(TIM1, &TIM1_ICInitStructure);
	
	
 /*****************�жϷ����ʼ��******************/
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn ;			//�����жϣ�TIM1_CC_IRQn; 	TIM1�Ĵ����жϣ�TIM1_TRG_COM_TIM11_IRQn 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;			//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 						//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);                 
	
	/*****************���������ж�******************/
	
	TIM_ITConfig(TIM1,TIM_IT_CC1,ENABLE);
	TIM_ITConfig(TIM1,TIM_IT_CC2,ENABLE);
	TIM_ITConfig(TIM1,TIM_IT_CC3,ENABLE);
	TIM_ITConfig(TIM1,TIM_IT_CC4,ENABLE);

	TIM_Cmd(TIM1, ENABLE ); 
}

// Capture status of channels
unsigned char TIM2CH1_CAPTURE_STA = 1;
unsigned char TIM1CH1_CAPTURE_STA = 1;
unsigned char TIM1CH2_CAPTURE_STA = 1;
unsigned char TIM1CH3_CAPTURE_STA = 1;
unsigned char TIM1CH4_CAPTURE_STA = 1;

// Rising edge/falling edge data
uint16_t TIM1CH1_Rise, TIM1CH1_Fall,
         TIM1CH2_Rise, TIM1CH2_Fall,
         TIM1CH3_Rise, TIM1CH3_Fall,
         TIM1CH4_Rise, TIM1CH4_Fall;

// Overflow processing variable
uint16_t TIM1_T;

/*****************�жϺ���******************/
void TIM1_CC_IRQHandler(void)
{
	// CH1 - AIL - Roll
    // Capture the interrupt
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)
    {
        // Clear interrupt flag
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
        // Capture the rising edge
        if(TIM1CH1_CAPTURE_STA == 1)
        {
            // Get the data of rising edge
            TIM1CH1_Rise = TIM_GetCapture1(TIM1);
            // Change capture status to falling edge
            TIM1CH1_CAPTURE_STA = 0;
            // Set to capture on falling edge
            TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Falling);
        }
        // Capture the falling edge
        else
        {
            // Get the data of falling edge
            TIM1CH1_Fall = TIM_GetCapture1(TIM1);
            // Change capture status to rising edge
            TIM1CH1_CAPTURE_STA = 1;

            // Overflow processing
            if(TIM1CH1_Fall < TIM1CH1_Rise)
            {
                TIM1_T = 65535;
            }
            else
            {
                TIM1_T = 0;
            }

            // Falling edge time minus rising edge time to get high-level time
            PWMInCh1 = TIM1CH1_Fall - TIM1CH1_Rise + TIM1_T;
            // Set to capture on rising edge
            TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Rising);
        }
    }

    // CH2 - ELE - Pitch
    if (TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);

        if(TIM1CH2_CAPTURE_STA == 1)
        {
            TIM1CH2_Rise = TIM_GetCapture2(TIM1);
            TIM1CH2_CAPTURE_STA = 0;
            TIM_OC2PolarityConfig(TIM1, TIM_ICPolarity_Falling);
        }
        else
        {
            TIM1CH2_Fall = TIM_GetCapture2(TIM1);
            TIM1CH2_CAPTURE_STA = 1;
            if(TIM1CH2_Fall < TIM1CH2_Rise)
            {
				        TIM1_T = 65535;
            }
            else
            {
                TIM1_T = 0;
            }
            PWMInCh2 = TIM1CH2_Fall - TIM1CH2_Rise + TIM1_T;
            TIM_OC2PolarityConfig(TIM1, TIM_ICPolarity_Rising);
        }
    }

    // CH3 - THR - Acc
    if (TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);

        if(TIM1CH3_CAPTURE_STA == 1)
        {
            TIM1CH3_Rise = TIM_GetCapture3(TIM1);
            TIM1CH3_CAPTURE_STA = 0;
            TIM_OC3PolarityConfig(TIM1,TIM_ICPolarity_Falling);
        }
        else
        {
            TIM1CH3_Fall = TIM_GetCapture3(TIM1);
            TIM1CH3_CAPTURE_STA = 1;
            if(TIM1CH3_Fall < TIM1CH3_Rise)
            {
                TIM1_T = 65535;
            }
            else
            {
                TIM1_T = 0;
            }
            PWMInCh3 = TIM1CH3_Fall - TIM1CH3_Rise + TIM1_T;
            TIM_OC3PolarityConfig(TIM1, TIM_ICPolarity_Rising);
        }
    }

    // CH4 - RUD -Yaw
    if (TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);

        if(TIM1CH4_CAPTURE_STA == 1)
        {
            TIM1CH4_Rise = TIM_GetCapture4(TIM1);
            TIM1CH4_CAPTURE_STA = 0;
            TIM_OC4PolarityConfig(TIM1, TIM_ICPolarity_Falling);
        }
        else
        {
            TIM1CH4_Fall = TIM_GetCapture4(TIM1);
            TIM1CH4_CAPTURE_STA = 1;
            if(TIM1CH4_Fall < TIM1CH4_Rise)
            {
                TIM1_T = 65535;
            }
            else
            {
                TIM1_T = 0;
            }
            PWMInCh4 = TIM1CH4_Fall - TIM1CH4_Rise + TIM1_T;
            TIM_OC4PolarityConfig(TIM1, TIM_ICPolarity_Rising);
        }
    }
}



