#include "Servos.h"

void servos_start(TIM_HandleTypeDef *htimx, uint32_t Channel)
{
	assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));
	
	HAL_TIM_Base_Start(htimx);
	HAL_TIM_PWM_Start(htimx,Channel);

}
 
void servos_stop(TIM_HandleTypeDef *htimx, uint32_t Channel)
{
	HAL_TIM_PWM_Stop(htimx,Channel);
}

/*
    * @brief ����pwmռ�ձ�
    *
    * @param pwm pwmʵ��
    * @param dutyratio ռ�ձ� 0~1
*/
void PWMSetDutyRatio(TIM_HandleTypeDef *htimx, uint32_t Channel, float dutyratio)
{
    __HAL_TIM_SetCompare(htimx, Channel, dutyratio * (htimx->Instance->ARR));
}
/*
    * @brief ����pwmռ�ձ�
    *
    * @param pwm pwmʵ��
    * @param compare 
*/
void PWMSetCompare(TIM_HandleTypeDef *htimx, uint32_t Channel, uint32_t compare)
{
    __HAL_TIM_SetCompare(htimx, Channel, compare);
}
