#ifndef __SERVOS_H
#define __SERVOS_H

#include "tim.h"
#include "stdint.h"

#define PER_ANGLE 0.0003703703703703704f
//#define PER_ANGLE 0.00032573289902280130293159609120521f
void servos_start(TIM_HandleTypeDef *htimx, uint32_t Channel);
void servos_stop(TIM_HandleTypeDef *htimx, uint32_t Channel);
void PWMSetDutyRatio(TIM_HandleTypeDef *htimx, uint32_t Channel, float dutyratio);
void PWMSetCompare(TIM_HandleTypeDef *htimx, uint32_t Channel, uint32_t compare);



#endif

