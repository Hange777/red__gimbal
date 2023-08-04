//
// Created by 16771 on 2022/12/1.
//

#ifndef ENGINEERING_A_2023_C_BOARD_BSP_PWM_H
#define ENGINEERING_A_2023_C_BOARD_BSP_PWM_H

#include "tim.h"
extern void ECF_PWM_50HZ_Output(TIM_HandleTypeDef *htim, int8_t TIM_Channel, const float Output_Percent);
extern void ECF_HobbyWing_ESC_Init(TIM_HandleTypeDef *htim, int8_t TIM_Channel);
extern void ECF_HobbyWing_ESC_Control(TIM_HandleTypeDef *htim, int8_t TIM_Channel,uint16_t pwm);



#endif //ENGINEERING_A_2023_C_BOARD_BSP_PWM_H
