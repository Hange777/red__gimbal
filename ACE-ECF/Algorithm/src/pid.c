
/*************************** Dongguan-University of Technology -ACE**************************
 * @file    pid.c
 * @author  ֣���
 * @version V1.0.3
 * @date    2022/11/19
 * @brief
 ******************************************************************************
 * @verbatim
 *  �Ѿ��ѳ��õ�pidģʽ���ӽ�ȥ�ˣ������˲���һЩû��ʵ��
 *  ʹ�÷�����
 *      �ȴ���һ��pid�ṹ�塣
 *      ��ʼ����
 *            ��ʹ��PidInit��ע��ģʽ�����������ã��� | ����
 *            Ȼ��ʹ��PidInitMode��ÿ��ģʽ�����ֵ
 *      ʹ�ã�PidCalculate
 *            pid_clear
 *            User_Fun_Callback_Register
 *  demo��
 *       pid_parameter_t motor6020pid_s;//����һ��pid�ṹ��
 *       PidInit(&motor6020pid_s,20,0,0,Output_Limit | Integral_Limit);//ʹ������޷��ͻ����޷�ģʽ
 *       PidInitMode(&motor6020pid_s,Integral_Limit,1000,0);//�����޷�ģʽ����
 *	     PidInitMode(&motor6020pid_s,Output_Limit,30000,0);//����޷�ģʽ����
 *          while(1)
 *          {
 *             ActualValue = sersor();                     //��ȡ����
 *             setvalue  = setclaculate();                 //��ȡ����
 *             PidCalculate(&motor6020pid_s,setvalue,ActualValue);   //����
 *          }
 *
 * @attention
 *      ��ȷ��User_Fun_Callback_Registerע���˺�����ʹ�ýṹ���е��Զ��庯��
 *      ����Ĭ����ʹ�õģ������ʹ�ã����������У�ʹ���˻������������pid��
 * @version
 * v1.0.1 ����˲���ʽPID�����й��ܴ���֤
 * V1.0.2 ����˺ܶ�ע�ͣ�Integral_Limit ��Output_Limit ��StepInģʽ����֤��Derivative_On_Measurement�ƺ��е�����
 * V1.0.3 ����˻��ֺ�����˲�������֤
 ************************** Dongguan-University of Technology -ACE***************************/
#include "pid.h"
#include <string.h>
#define abs(x) ((x) > (0) ? (x) : (-(x)))

/*--------------------��������-----------------------*/
static fp32 first_order_filter(first_order_filter_t *first_order_filter_type, fp32 input);
static void f_Separated_Integral(pid_parameter_t *pid);
static void f_Integral_Limit(pid_parameter_t *pid);
static void f_Derivative_On_Measurement(pid_parameter_t *pid);
static void Changing_Integration_Rate(pid_parameter_t *pid);
static void f_Output_Limit(pid_parameter_t *pid);
static void f_StepIn(pid_parameter_t *pid);
/*---------------------------------------------------------*/
/**
 * @brief          PID��ʼ��
 * @param[in]      ��ʼ����pid�ṹ��
 * @param[in]      pid��p
 * @param[in]      pid��i
 * @param[in]      pid��d
 * @param[in]      pidģʽ ���pid_mode_eö��
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
 * @brief          PIDģʽ��ʼ��
 * @param[in]      ��ʼ����pid�ṹ��
 * @param[in]      Ҫ��ʼ����modeֵ ���pid_mode_eö��
 *                 ����Ϊ��ͬ��mode��Ҫ�����ֵ
 *                 NONE                         num1��num2������
 *                 Deadzone                     num1 ������Χ
 *                 Integral_Limit               num1 �����޷����ֵ
 *                 Output_Limit                 num1 ����޷�ֵ ����Ǿ���ֵ
 *                 Derivative_On_Measurement    num1 ����ϵ��
 *                 Separated_Integral           num1 ���ַ������� num2���ַ�������
 *                 ChangingIntegrationRate      num1 Ϊ��������� num2Ϊ���������
 *                 OutputFilter                 num1 Ϊ�˲�ϵ��
 *                 DerivativeFilter             num1 Ϊ�˲�ϵ��
 *                 StepIN                       num1 ������
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
 * @brief          PIDͨ�ü���
 * @param[in]      �������pid�ṹ��
 * @param[in]      �趨ֵ
 * @param[in]      ʵ��ֵ
 * @retval         none
 */
fp32 PidCalculate(pid_parameter_t *pid, fp32 SetValue, fp32 ActualValue)
{
    pid->SetValue = SetValue;
    // ����ʽpid
    if (pid->mode & StepIn)
        f_StepIn(pid);
    pid->ActualValue = ActualValue;
    pid->error = pid->SetValue - pid->ActualValue;
    pid->Derror = pid->error - pid->LastError;
    if (abs(pid->error) >= pid->deadband) //����
    {
        pid->Pout = pid->error * pid->Kp;
        pid->Ierror += pid->error;

        // ΢������
        if (pid->mode & Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        else
            pid->Dout = pid->Kd * pid->Derror;

        // �����
        if (pid->mode & ChangingIntegrationRate)
            Changing_Integration_Rate(pid);

        // �����޷�
        if (pid->mode & Integral_Limit)
            f_Integral_Limit(pid);
        pid->Iout = pid->Ki * pid->Ierror;

        // ���ַ��� ע����Ҫ����iout�����
        if (pid->mode & Separated_Integral)
            f_Separated_Integral(pid);

        // ΢���˲�
        if (pid->mode & DerivativeFilter)
            pid->Dout = first_order_filter(&pid->d_filter, pid->Dout);

        pid->out = pid->Pout + pid->Iout + pid->Dout;

        // ����˲�
        if (pid->mode & OutputFilter)
            pid->out = first_order_filter(&pid->out_filter, pid->out);

        // ����޷�
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
 * @brief          PID���
 * @param[in]      �������pid�ṹ��
 * @retval         none
 * @attention      ֻ��������м�������ݣ��������pid����ģʽ������
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
 * @brief          PID�ص�����ע��
 * @param[in]      ��ע���pid�ṹ��
 * @param[in]      ��Ҫע��ĺ���
 * @retval         none
 * @attention      none
 */
void User_Fun_Callback_Register(pid_parameter_t *pid, void (*User_Fun)(struct Pid_parameter_t *))
{
    pid->User_Fun = User_Fun;
}

/*------------------------------Ӧ��---------------------------------*/
/**
 * @brief          ����ٶȻ������㷨
 * @param[in]      �ٶȻ�pid�ṹ��
 * @param[in]      �趨�ٶ�ֵ
 * @param[in]      ʵ��ֵ
 * @retval         ����ֵ
 */
int16_t motor_speed_control(pid_parameter_t *spid, fp32 setSpeed, fp32 actualSpeed)
{
    return PidCalculate(spid, setSpeed, actualSpeed);
}

/**
 * @brief          ����ٶ�λ�û����������㷨
 * @param[in]      �ٶȻ�pid�ṹ��
 * @param[in]      λ�û�pid�ṹ��
 * @param[in]      �趨λ��ֵ
 * @param[in]      ʵ��λ��ֵ
 * @param[in]      ʵ���ٶ�ֵ
 * @retval         ����ֵ
 */
int16_t motor_position_speed_control(pid_parameter_t *speed_pid, pid_parameter_t *position_pid, fp32 setPosition, fp32 actual_position, fp32 actual_speed)
{
    speed_pid->SetValue = PidCalculate(position_pid, setPosition, actual_position); //�ٶȻ��趨ֵ��λ�û�����
    return PidCalculate(speed_pid, speed_pid->SetValue, actual_speed);              //��������
}

/*------------------------------����Ϊ�ڲ�����---------------------------------*/
/**
 * @brief          һ�׵�ͨ�˲�����
 * @param[in]      ��ͨ�˲��ṹ��
 * @param[in]      ����ֵ
 * @retval         ���ֵ
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
 * @brief          ΢������pid
 * @param[in]      pid�ṹ��
 * @retval         none
 * @attention      �ƺ��������� ��� https://blog.csdn.net/foxclever/article/details/80633275
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
 * @brief          ����ʽpid
 * @param[in]      pid�ṹ��
 * @retval         none
 * @attention      ��� https://blog.csdn.net/foxclever/article/details/81151898
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
 * @brief          ���ַ���
 * @param[in]      pid�ṹ��
 * @retval         none
 * @attention      ��������ֵ��ʱ���iout�������
 */
void f_Separated_Integral(pid_parameter_t *pid)
{
    if (pid->threshold_min > pid->error && pid->error < pid->threshold_max)
        pid->Iout = 0;
}
/**
 * @brief          �����޷�
 * @param[in]      pid�ṹ��
 * @retval         none
 * @attention      ��������ֵ��ʱ���iout�������
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
 * @brief          �����ϵ����������ʵ��һ�����0��1֮��ķֶ����Ժ���
 * @param[in]      pid�ṹ��
 * @retval         none
 * @attention      ��ƫ��ľ���ֵС����Сֵʱ�����Ϊ1����ƫ��ľ���ֵ�������ֵʱ�����Ϊ0
 *                 ��ƫ��ľ���ֵ�������ֵ����Сֵ֮��ʱ�������0��1֮�����Ա仯
 */
void Changing_Integration_Rate(pid_parameter_t *pid)
{
    if (abs(pid->error) <= pid->errorabsmin) //��Сֵ
    {
        return;
    }
    else if (abs(pid->error) > pid->errorabsmax) //���ֵ
    {
        pid->Ierror = 0.0f;
    }
    else
    {
        pid->Ierror *= ((pid->errorabsmax - abs(pid->error)) / (pid->errorabsmax - pid->errorabsmin));
    }
}
/**
 * @brief          ����޷�
 * @param[in]      pid�ṹ��
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
