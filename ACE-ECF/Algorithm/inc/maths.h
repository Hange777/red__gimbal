#ifndef __MATHS_H
#define __MATHS_H
#include "struct_typedef.h"

//绝对值
#define user_abs(x) ((x) > (0) ? (x) : (-(x)))

//数字限幅
#define value_limit(val, min, max)   \
    {                                \
        (val) = (val) < (min) ? (min) : (val); \
        (val) = (val) > (max) ? (max) : (val); \
    }
//最大值
#define max(x, y) ((x) > (y) ? (x) : (y))

//运动加速度限制斜坡函数
typedef __packed struct
{
    float Input;      //当前取样值
    float Last_Input; //上次取样值
    float Output;     //输出值
    float acc_now;    //当前加速度
    float acc_limit;  //需要限制的加速度
} acceleration_control_type_t;

int16_t max_abs(int16_t x, int16_t y); //取最大值的绝对值
fp32 invSqrt(fp32 x);                  //平方根倒数
float float_min_distance(float target, float actual, float minValue, float maxValue); //寻最小值
/* 运动控制斜坡函数（加速度限制）（16位） */
int16_t motion_acceleration_control(acceleration_control_type_t *acceleration_control, int16_t Input, int16_t Limit); //运动加速度限制

/* 循环限制 */
int16_t loop_restriction_int16(int16_t num, int16_t limit_num);         // 16位循环限幅
float loop_restriction_float(float num, float limit_num);               //浮点循环限幅
float loop_fp32_constrain(float Input, float minValue, float maxValue); //循环限制（云台角度处理）
float cos_calculate(float angle);
float sin_calculate(float angle);

/* 斜坡函数 */
void data_accelerated_control(float *input, float acc); //加速度限制斜坡函数

/* 限幅滤波 */
float limiting_filter(float new_value, float last_value, float delat_max);

//弧度格式化为-PI~PI
#define RAD_FORMAT(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
