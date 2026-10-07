#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

extern float Kp, Ki, Kd;
extern float Target_RPM;
extern float Integral;
extern float Prev_Error;
extern int16_t PID_Out;

void Motor_Init(void);
void Motor_SetPWM(int16_t duty);
int16_t Motor_GetRPM(void);
int16_t Speed_PID_Calc(int16_t target, int16_t actual);

#endif
