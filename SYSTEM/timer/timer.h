#ifndef __TIMER_H
#define __TIMER_H

#ifndef _PWM_H
#define _PWM_H

#include "sys.h"
#include "stm32f10x.h"


//电机IO口定义
#define IN1_PIN								GPIO_Pin_11           //IN1引脚号 E.11
#define IN2_PIN								GPIO_Pin_10           //IN2引脚号 E.10
#define ENA_PIN								GPIO_Pin_8            //ENA引脚号 B.8  定时器4通道3



void TIM3_PWM_Init(u16 arr, u16 psc);
void TIM4_PWM_Init(u16 arr,u16 psc);
void TIM5_Int_Init(u16 arr,u16 psc);
void POWER_M(void);

#endif
#endif
