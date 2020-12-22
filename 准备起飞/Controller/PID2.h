#include "stm32F4xx.h"

extern uint16_t Motor_1, Motor_2, Motor_3, Motor_4;
extern float Motor_Roll, Motor_Pitch, Motor_Thr, Motor_Yaw;


void Motor_Calculate(void);
float Limit_PWM(float accelerator);
void PID_Roll_Calculate(void);
void PID_Pitch_Calculate(void);
void PID_Yaw_Calculate(void);
void Motor_Expectation_Calculate(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4);
