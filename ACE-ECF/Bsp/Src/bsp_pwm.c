/************************** Dongguan-University of Technology -ACE**************************
 * @file bsp_pwm.c
 * @brief 
 * @author pansyhou侯文辉 (1677195845lyb@gmail.com)
 * @version 1.0
 * @date 2022-12-03
 * 
 * 
 * @history
 * <table>
 * Date       Version Author Description
 * 2022-12-03   1.0   侯文辉     
 * @verbatim 
 * ==============================================================================
 *  该bsp包含
 *  50hz频率输出的控制与初始化函数
 *  50hz的有好盈电调、普通舵机
 *  
 * ==============================================================================
 * @endverbatim
************************** Dongguan-University of Technology -ACE***************************/


// C板PWM输出口图示
/* DBUS                             PWM
 *          C8     C5        C4     C1
 *      TIM8CH3-TIM8CH1  TIM1CH4-TIM1CH1
 *      5V
 *      GND
 * */


/*  配置如下
 *  TIM1挂载168MHz的总线下，需要168分频，故PSC设为167
 *  向上计数
 *  ARR设置为19999
 *  PWM1
 *  pulse设置为2000
 *  计算得到PWM频率为50Hz（大部分舵机与电调都是50hz，听说好盈可以到200？好盈还要开机程序，诡异
 *  对应的周期为20ms，所以pwm计算的分母为20000
 *  对于舵机，舵机的最小占空比是2.5%，计算一下
 *  20000*0.025=500，所以占空比最低值为500
 *  */

#include "bsp_pwm.h"
#include "freertos.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"


extern TIM_HandleTypeDef htim1;

void ECF_HobbyWing_ESC_Init(TIM_HandleTypeDef *htim, int8_t TIM_Channel);
void ECF_PWM_50HZ_Output(TIM_HandleTypeDef *htim, int8_t TIM_Channel, const float Output_Percent);
void ECF_HobbyWing_ESC_Control(TIM_HandleTypeDef *htim, int8_t TIM_Channel,uint16_t pwm);

/**
 *  50HZ输出频率百分比调整函数
 * @param htim 定时器句柄
 * @param TIM_Channel TIM输出通道
 * @param Output_Percent 输出百分比 0-100
 */

void ECF_PWM_50HZ_Output(TIM_HandleTypeDef *htim, int8_t TIM_Channel, const float Output_Percent) {
    //合法性检查
    assert_param(IS_TIM_INSTANCE(htim->Instance));
    assert_param(IS_TIM_CCX_INSTANCE(htim, TIM_Channel));
    if (Output_Percent < 0 || Output_Percent > 100) {
        return;
    }
    __HAL_TIM_SetCompare(htim, TIM_Channel, Output_Percent * 200);
}


/**
 * 好盈skywalker电调启动程序
 * 原理：油门拉最高然后拉最低
 * @param htim 定时器句柄
 * @param TIM_Channel 定时器channel
 */
void ECF_HobbyWing_ESC_Init(TIM_HandleTypeDef *htim, int8_t TIM_Channel){
    //合法性检查
    assert_param(IS_TIM_INSTANCE(htim->Instance));
    assert_param(IS_TIM_CCX_INSTANCE(htim, TIM_Channel));
    HAL_TIM_Base_Start(&htim);
    HAL_TIM_PWM_Start(&htim,TIM_Channel);
    vTaskDelay(4000);
    __HAL_TIM_SetCompare(htim, TIM_Channel,2000);
    vTaskDelay(4000);
    __HAL_TIM_SetCompare(htim, TIM_Channel,1000);
    vTaskDelay(4000);
}



/**
 * 好盈电调控制函数
 * 注意pwm控制范围在 1200到1400（我怕爆了）
 * @param htim
 * @param TIM_Channel
 * @param pwm
 */
void ECF_HobbyWing_ESC_Control(TIM_HandleTypeDef *htim, int8_t TIM_Channel,uint16_t pwm) {
    //别超油门了
    if (pwm >= 1400) pwm = 1300;
    else if(pwm <= 1190)  pwm=1140;
    __HAL_TIM_SetCompare(htim, TIM_Channel,pwm);
}
