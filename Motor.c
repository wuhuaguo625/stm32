#include "Motor.h"

float Kp = 5.0f, Ki = 0.5f, Kd = 0.0f;
float Target_RPM = 0.0f;
float Integral = 0.0f;
float Prev_Error = 0.0f;
int16_t PID_Out = 0;

// PWM 初始化 (PB0 - TIM3_CH3)
void Motor_PWM_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1; 
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_Cmd(TIM3, ENABLE);
}

// 方向引脚初始化 (PA3, PA4, PA5)
void Motor_GPIO_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA, GPIO_Pin_5); // STBY 拉高
}

// 编码器初始化 (PA0, PA1 - TIM2 编码器模式)
void Encoder_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 65535;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_Cmd(TIM2, ENABLE);
}

void Motor_Init(void)
{
    Motor_PWM_Init();
    Motor_GPIO_Init();
    Encoder_Init();
}

void Motor_SetPWM(int16_t duty)
{
    if (duty > 100) duty = 100;
    if (duty < -100) duty = -100;
    
    if (duty >= 0) {
        GPIO_SetBits(GPIOA, GPIO_Pin_3);
        GPIO_ResetBits(GPIOA, GPIO_Pin_4);
        TIM_SetCompare3(TIM3, duty * 10);
    } else {
        GPIO_ResetBits(GPIOA, GPIO_Pin_3);
        GPIO_SetBits(GPIOA, GPIO_Pin_4);
        TIM_SetCompare3(TIM3, -duty * 10);
    }
}

int16_t Motor_GetRPM(void)
{
    static int16_t last_count = 0;
    int16_t current_count = TIM_GetCounter(TIM2);
    int16_t diff = current_count - last_count;
    last_count = current_count;
    return (diff * 600) / 1320; 
}

int16_t Speed_PID_Calc(int16_t target, int16_t actual)
{
    float error = (float)target - (float)actual;
    Integral += error;
    if (Integral > 500.0f) Integral = 500.0f;
    if (Integral < -500.0f) Integral = -500.0f;

    float output = Kp * error + Ki * Integral + Kd * (error - Prev_Error);
    Prev_Error = error;

    if (output > 100.0f) output = 100.0f;
    if (output < -100.0f) output = -100.0f;
    
    return (int16_t)output;
}
