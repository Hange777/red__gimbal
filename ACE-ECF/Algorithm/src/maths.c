#include "maths.h"
#include <stddef.h>
#include "arm_math.h"
#include "list_of_function.h"

float invSqrt(float x) //ƽ�����������㷨
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/*ȡ���������ֵ�ľ���ֵ*/
int16_t max_abs(int16_t x, int16_t y)
{
    if (user_abs(x) >= user_abs(y))
        return user_abs(x);
    else
        return user_abs(y);
}

/*
 *���ܣ��˶�����б�º��������ٶ����ƣ�
 *���룺1.���ٶ����ƶ�Ӧ�ṹ��  2.������ 3.���Ƽ��ٶ�
 *������������
 *���ͣ�16λ����
 */
int16_t motion_acceleration_control(acceleration_control_type_t *acceleration_control, int16_t Input, int16_t Limit)
{
    acceleration_control->Input = Input;
    acceleration_control->acc_limit = Limit;

    acceleration_control->acc_now = acceleration_control->Input - acceleration_control->Last_Input;

    if (user_abs(acceleration_control->acc_now) > acceleration_control->acc_limit)
    {
        acceleration_control->Output = acceleration_control->Last_Input + acceleration_control->acc_now / user_abs(acceleration_control->acc_now) * acceleration_control->acc_limit;
    }

    acceleration_control->Last_Input = acceleration_control->Output;

    return acceleration_control->Output;
}

/*
 *���ܣ�����ѭ�����ƣ�16λ��
 *���룺1.����ֵ  2.���Ʒ���(����)
 *�������޷����ֵ
 *������������ֵ������ +-���Ʒ��� �ķ�Χ��
 */
int16_t loop_restriction_int16(int16_t num, int16_t limit_num)
{
    if (user_abs(num) > limit_num)
    {
        if (num >= 0)
            num -= limit_num;
        else
            num += limit_num;
    }
    return num;
}

/*
 *���ܣ�����ѭ�����ƣ�float��
 *���룺1.����ֵ  2.���Ʒ���(����)
 *�������޷����ֵ
 *������������ֵ������ +-���Ʒ��� �ķ�Χ��
 */
float loop_restriction_float(float num, float limit_num)
{
    if (user_abs(num) > limit_num)
    {
        if (num >= 0)
            num -= limit_num;
        else
            num += limit_num;
    }
    return num;
}

/*ѭ���޷�32*/
float loop_fp32_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//б�º���(���ٶ�����)
void data_accelerated_control(float *input, float acc)
{
    static int16_t last_num = 0;
    int16_t temp;
    temp = *input - last_num;

    if (user_abs(temp) > acc)
        *input = last_num + temp / user_abs(temp) * acc;

    last_num = *input;
}

// �޷��˲�����
float limiting_filter(float new_value, float last_value, float delat_max)
{
    if ((new_value - last_value > delat_max) || (last_value - new_value > delat_max))
        return last_value;
    return new_value;
}

// sin�������
float sin_calculate(float angle)
{
    float sin_angle;

    if (angle >= 0.0f && angle < 90.0f)
        sin_angle = (Trigonometric_Functions[(int)(user_abs(angle) * 10.0f)] / 100.0f);
    else if (angle >= 90.0f && angle < 180.f)
        sin_angle = (Trigonometric_Functions[(int)(user_abs(180.0f - angle) * 10.0f)] / 100.0f);
    else if (angle >= -180.0f && angle < -90.f)
        sin_angle = -(Trigonometric_Functions[(int)(user_abs(180.0f + angle) * 10.0f)] / 100.0f);
    else if (angle >= -90.0f && angle < 0.f)
        sin_angle = -(Trigonometric_Functions[(int)(user_abs(180.0f - (180.0f + angle)) * 10.0f)] / 100.0f);
    else if (angle == 180.f)
        sin_angle = 0.0f;

    return sin_angle;
}

// cos�������
float cos_calculate(float angle)
{
    float cos_angle;

    angle = user_abs(angle);

    if (angle >= 0.0f && angle < 90.0f)
        cos_angle = (Trigonometric_Functions[(int)(user_abs(90.0f - angle) * 10.0f)] / 100.0f);
    else if (angle >= 90.0f && angle < 180.f)
        cos_angle = -(Trigonometric_Functions[(int)(user_abs(angle - 90.0f) * 10.0f)] / 100.0f);
    else if (angle == 180.f)
        cos_angle = -1.0f;

    return cos_angle;
}

float float_min_distance(float target, float actual, float minValue, float maxValue)
{
	if (maxValue < minValue)
    {
        return 0;
    }
	
	target = loop_fp32_constrain(target, minValue,maxValue);
	
	if (user_abs(target - actual) > (maxValue - minValue) / 2.0f)
	{
		if(maxValue - actual < (maxValue - minValue) / 2.0f)
		{
			return maxValue - actual + target - minValue;
		}
		else
		{
			return minValue - actual + target - maxValue;
		}
	}
	else 
	{
		return target - actual;
	}
	
}
