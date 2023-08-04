
/*************************** Dongguan-University of Technology -ACE**************************
 * @file    pid.c
 * @author  郑杰瀚
 * @version V1.0.3
 * @date    2022/11/19
 * @brief
 ******************************************************************************
 * @verbatim
 *  已经把常用的pid模式都加进去了，还有滤波器一些没有实现
 *  使用方法：
 *      先创建一个pid结构体。
 *      初始化：
 *            先使用PidInit，注意模式是在这里设置，用 | 链接
 *            然后使用PidInitMode将每个模式逐个赋值
 *      使用：PidCalculate
 *            pid_clear
 *            User_Fun_Callback_Register
 *  demo：
 *       pid_parameter_t motor6020pid_s;//定义一个pid结构体
 *       PidInit(&motor6020pid_s,20,0,0,Output_Limit | Integral_Limit);//使用输出限幅和积分限幅模式
 *       PidInitMode(&motor6020pid_s,Integral_Limit,1000,0);//积分限幅模式设置
 *	     PidInitMode(&motor6020pid_s,Output_Limit,30000,0);//输出限幅模式设置
 *          while(1)
 *          {
 *             ActualValue = sersor();                     //获取数据
 *             setvalue  = setclaculate();                 //获取数据
 *             PidCalculate(&motor6020pid_s,setvalue,ActualValue);   //计算
 *          }
 *
 * @attention
 *      请确保User_Fun_Callback_Register注册了函数再使用结构体中的自定义函数
 *      死区默认是使用的，如果不使用，不用理会就行，使用了会在死区内清除pid。
 * @version
 * v1.0.1 添加了步进式PID，所有功能待验证
 * V1.0.2 添加了很多注释，Integral_Limit ，Output_Limit ，StepIn模式已验证，Derivative_On_Measurement似乎有点问题
 * V1.0.3 添加了积分和输出滤波，待验证
 ************************** Dongguan-University of Technology -ACE***************************/
#include "pid.h"
#include <string.h>
#define abs(x) ((x) > (0) ? (x) : (-(x)))

/*--------------------函数声明-----------------------*/
static fp32 first_order_filter(first_order_filter_t *first_order_filter_type, fp32 input);
static void f_Separated_Integral(pid_parameter_t *pid);
static void f_Integral_Limit(pid_parameter_t *pid);
static void f_Derivative_On_Measurement(pid_parameter_t *pid);
static void Changing_Integration_Rate(pid_parameter_t *pid);
static void f_Output_Limit(pid_parameter_t *pid);
static void f_StepIn(pid_parameter_t *pid);
/*---------------------------------------------------------*/
/**
 * @brief          PID初始化
 * @param[in]      初始化的pid结构体
 * @param[in]      pid的p
 * @param[in]      pid的i
 * @param[in]      pid的d
 * @param[in]      pid模式 详见pid_mode_e枚举
 * @retval         none
 */
void PidInit(pid_parameter_t *pid, fp32 kp, fp32 ki, fp32 kd, uint32_t mode)
{
    memset(pid, 0, sizeof(pid_parameter_t));
    pid->User_Fun = NULL;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->mode = mode;
}
/**
 * @brief          PID模式初始化
 * @param[in]      初始化的pid结构体
 * @param[in]      要初始化的mode值 详见pid_mode_e枚举
 *                 以下为不同的mode需要填入的值
 *                 NONE                         num1、num2都不填
 *                 Deadzone                     num1 死区范围
 *                 Integral_Limit               num1 积分限幅最大值
 *                 Output_Limit                 num1 输出限幅值 这个是绝对值
 *                 Derivative_On_Measurement    num1 惯性系数
 *                 Separated_Integral           num1 积分分离上限 num2积分分离下限
 *                 ChangingIntegrationRate      num1 为变积分上限 num2为变积分下限
 *                 OutputFilter                 num1 为滤波系数
 *                 DerivativeFilter             num1 为滤波系数
 *                 StepIN                       num1 步进数
 * @retval         none
 */
void PidInitMode(pid_parameter_t *pid, uint32_t mode, fp32 num1, fp32 num2)
{
    switch (mode)
    {
    case NONE:
        return;
    case Integral_Limit:
        pid->max_Ierror = num1;
        return;
    case Derivative_On_Measurement:
        pid->gama = num1;
        return;
    case Separated_Integral:
        pid->threshold_max = num1;
        pid->threshold_min = num2;
        return;
    case Output_Limit:
        pid->max_out = num1;
        return;
    case OutputFilter:
        pid->out_filter.num = num1;
        return;
    case ChangingIntegrationRate:
        pid->errorabsmax = num1;
        pid->errorabsmin = num2;
        return;
    case DerivativeFilter:
        pid->d_filter.num = num1;
        return;
    case Deadzone:
        pid->deadband = num1;
        return;
    case StepIn:
        pid->stepIn = num1;
        return;
    }
    return;
}
/**
 * @brief          PID通用计算
 * @param[in]      待计算的pid结构体
 * @param[in]      设定值
 * @param[in]      实际值
 * @retval         none
 */
fp32 PidCalculate(pid_parameter_t *pid, fp32 SetValue, fp32 ActualValue)
{
    pid->SetValue = SetValue;
    // 步进式pid
    if (pid->mode & StepIn)
        f_StepIn(pid);
    pid->ActualValue = ActualValue;
    pid->error = pid->SetValue - pid->ActualValue;
    pid->Derror = pid->error - pid->LastError;
    if (abs(pid->error) >= pid->deadband) //死区
    {
        pid->Pout = pid->error * pid->Kp;
        pid->Ierror += pid->error;

        // 微分先行
        if (pid->mode & Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        else
            pid->Dout = pid->Kd * pid->Derror;

        // 变积分
        if (pid->mode & ChangingIntegrationRate)
            Changing_Integration_Rate(pid);

        // 积分限幅
        if (pid->mode & Integral_Limit)
            f_Integral_Limit(pid);
        pid->Iout = pid->Ki * pid->Ierror;

        // 积分分离 注意需要放在iout计算后
        if (pid->mode & Separated_Integral)
            f_Separated_Integral(pid);

        // 微分滤波
        if (pid->mode & DerivativeFilter)
            pid->Dout = first_order_filter(&pid->d_filter, pid->Dout);

        pid->out = pid->Pout + pid->Iout + pid->Dout;

        // 输出滤波
        if (pid->mode & OutputFilter)
            pid->out = first_order_filter(&pid->out_filter, pid->out);

        // 输出限幅
        if (pid->mode & Output_Limit)
            f_Output_Limit(pid);
    }
    else
    {
        pid_clear(pid); 
    }

    pid->LastActualValue = pid->ActualValue;
    pid->LastSetValue = pid->SetValue;
    pid->LastDerror = pid->Derror;
    pid->LastError = pid->error;

    return pid->out;
}
/**
 * @brief          PID清除
 * @param[in]      待清除的pid结构体
 * @retval         none
 * @attention      只是清除所有计算的数据，不会清除pid或者模式的数据
 */
void pid_clear(pid_parameter_t *pid)
{
    pid->error = pid->LastError = 0.0f;
    pid->Derror = pid->LastDerror = pid->LastLastDerror = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = pid->Ierror = 0.0f;
    pid->ActualValue = pid->SetValue = pid->LastActualValue = pid->LastSetValue = 0.0f;
    pid->d_filter.last_input = pid->out_filter.last_input = 0;
}
/**
 * @brief          PID回调函数注册
 * @param[in]      待注册的pid结构体
 * @param[in]      将要注册的函数
 * @retval         none
 * @attention      none
 */
void User_Fun_Callback_Register(pid_parameter_t *pid, void (*User_Fun)(struct Pid_parameter_t *))
{
    pid->User_Fun = User_Fun;
}

/*------------------------------应用---------------------------------*/
/**
 * @brief          电机速度环控制算法
 * @param[in]      速度环pid结构体
 * @param[in]      设定速度值
 * @param[in]      实际值
 * @retval         计算值
 */
int16_t motor_speed_control(pid_parameter_t *spid, fp32 setSpeed, fp32 actualSpeed)
{
    return PidCalculate(spid, setSpeed, actualSpeed);
}

/**
 * @brief          电机速度位置环串级控制算法
 * @param[in]      速度环pid结构体
 * @param[in]      位置环pid结构体
 * @param[in]      设定位置值
 * @param[in]      实际位置值
 * @param[in]      实际速度值
 * @retval         计算值
 */
int16_t motor_position_speed_control(pid_parameter_t *speed_pid, pid_parameter_t *position_pid, fp32 setPosition, fp32 actual_position, fp32 actual_speed)
{
    speed_pid->SetValue = PidCalculate(position_pid, setPosition, actual_position); //速度环设定值由位置环处理
    return PidCalculate(speed_pid, speed_pid->SetValue, actual_speed);              //电机输出量
}

/*------------------------------以下为内部函数---------------------------------*/
/**
 * @brief          一阶低通滤波计算
 * @param[in]      低通滤波结构体
 * @param[in]      输入值
 * @retval         输出值
 * @attention      none
 */
fp32 first_order_filter(first_order_filter_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out = first_order_filter_type->input * first_order_filter_type->num + (1 - first_order_filter_type->num) * first_order_filter_type->last_input;
    first_order_filter_type->last_input = first_order_filter_type->out;

    return first_order_filter_type->out;
}
/**
 * @brief          微分先行pid
 * @param[in]      pid结构体
 * @retval         none
 * @attention      似乎存在问题 详见 https://blog.csdn.net/foxclever/article/details/80633275
 */
void f_Derivative_On_Measurement(pid_parameter_t *PID)
{
    fp32 c1, c2, c3, temp;

    temp = PID->gama * PID->Kd + PID->Kp;
    c3 = PID->Kd / temp;
    c2 = (PID->Kd + PID->Kp) / temp;
    c1 = PID->gama * c3;
    PID->Dout = c1 * PID->Dout + c2 * PID->ActualValue + c3 * PID->LastActualValue;
}
/**
 * @brief          步进式pid
 * @param[in]      pid结构体
 * @retval         none
 * @attention      详见 https://blog.csdn.net/foxclever/article/details/81151898
 */
void f_StepIn(pid_parameter_t *pid)
{
    fp32 kFactor = 0.0f;
    //    if ((pid->LastSetValue - pid->SetValue <= pid->stepIn) && (pid->LastSetValue - pid->SetValue >= pid->stepIn))
    //    {
    //        return;
    //    }
    if (abs(pid->LastSetValue - pid->SetValue) <= pid->stepIn)
    {
        return;
    }
    else
    {
        if ((pid->LastSetValue - pid->SetValue) > 0.0f)
        {
            kFactor = -1.0f;
        }
        else if ((pid->LastSetValue - pid->SetValue) < 0.0f)
        {
            kFactor = 1.0f;
        }
        else
        {
            kFactor = 0.0f;
        }
        pid->SetValue = pid->LastSetValue + kFactor * pid->stepIn;
    }
}
/**
 * @brief          积分分离
 * @param[in]      pid结构体
 * @retval         none
 * @attention      当超过阈值的时候吧iout清零就行
 */
void f_Separated_Integral(pid_parameter_t *pid)
{
    if (pid->threshold_min > pid->error && pid->error < pid->threshold_max)
        pid->Iout = 0;
}
/**
 * @brief          积分限幅
 * @param[in]      pid结构体
 * @retval         none
 * @attention      当超过阈值的时候吧iout清零就行
 */
void f_Integral_Limit(pid_parameter_t *pid)
{
    if (pid->Ierror > pid->max_Ierror)
    {
        pid->Ierror = pid->max_Ierror;
    }
    if (pid->Ierror < -(pid->max_Ierror))
    {
        pid->Ierror = -(pid->max_Ierror);
    }
}
/**
 * @brief          变积分系数处理函数，实现一个输出0和1之间的分段线性函数
 * @param[in]      pid结构体
 * @retval         none
 * @attention      当偏差的绝对值小于最小值时，输出为1；当偏差的绝对值大于最大值时，输出为0
 *                 当偏差的绝对值介于最大值和最小值之间时，输出在0和1之间线性变化
 */
void Changing_Integration_Rate(pid_parameter_t *pid)
{
    if (abs(pid->error) <= pid->errorabsmin) //最小值
    {
        return;
    }
    else if (abs(pid->error) > pid->errorabsmax) //最大值
    {
        pid->Ierror = 0.0f;
    }
    else
    {
        pid->Ierror *= ((pid->errorabsmax - abs(pid->error)) / (pid->errorabsmax - pid->errorabsmin));
    }
}
/**
 * @brief          输出限幅
 * @param[in]      pid结构体
 * @retval         none
 * @attention      none
 */
void f_Output_Limit(pid_parameter_t *pid)
{
    if (pid->out > pid->max_out)
    {
        pid->out = pid->max_out;
    }
    if (pid->out < -(pid->max_out))
    {
        pid->out = -(pid->max_out);
    }
}
