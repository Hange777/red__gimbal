
/* C板蜂鸣器是挂载在 TIM4 CH3上的，查表可知挂载在APB1总线上
 * APB1频率为84MHz,ARR为重装载值auto reload resister
 * PWM输出频率 = APB1/ARR+1 = (84*10^6) / 21000 = 4000Hz
 * */
#include "bsp_buzzer.h"
#include "main.h"
extern TIM_HandleTypeDef htim4;
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}

void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}