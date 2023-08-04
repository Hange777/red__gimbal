#ifndef __FILTER_H__
#define __FILTER_H__
#include "struct_typedef.h"
#include "arm_math.h"

#define mat arm_matrix_instance_f32 //浮点矩阵结构的矩阵结构体   ( 包含 行数、列数、以及数组(数据) )
#define mat_init arm_mat_init_f32   //浮点矩阵初始化
#define mat_add arm_mat_add_f32     //浮点矩阵加法
#define mat_sub arm_mat_sub_f32     //浮点矩阵减法
#define mat_mult arm_mat_mult_f32   //浮点矩阵乘法
#define mat_trans arm_mat_trans_f32 //浮点矩阵转置
#define mat_inv arm_mat_inverse_f32 //浮点矩阵逆

typedef struct
{
    float raw_value;
    float filtered_value[4];
    mat xhat, xhatminus, z, A, H, AT, HT, Q, R, p, Pminus, k;
} kalman_filter_t;

typedef struct
{
    float raw_value;
    float filtered_value[4];
    float xhat_data[4], xhatminus_data[4], z_data[2], Pminus_data[16], K_data[8];
    float P_data[16];
    float AT_data[16], HT_data[8];
    float A_data[16];
    float H_data[8];
    float Q_data[16];
    float R_data[4];
} kalman_filter_init_t;

typedef struct
{
    float X_last; //上一时刻的最优结果  X(k-|k-1)
    float X_mid;  //当前时刻的预测结果  X(k|k-1)
    float X_now;  //当前时刻的最优结果  X(k|k)
    float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float P_now;  //当前时刻最优结果的协方差  P(k|k)
    float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float kg;     // kalman增益
    float A;      //系统参数
    float B;
    float Q;
    float R;
    float H;
} extKalman_t;

void KalmanCreate(extKalman_t *p, float T_Q, float T_R);
float KalmanFilter(extKalman_t *p, float dat);

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
void kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);

extern kalman_filter_init_t visual_kalman_filter_init;
extern kalman_filter_t visual_kalman_filter;

//滑动均值滤波参数（浮点）
typedef __packed struct
{
    fp32 Input;        //当前取样值
    int32_t count_num; //取样次数
    fp32 Output;       //滤波输出
    fp32 Sum;          //累计总和
    fp32 FIFO[250];    //队列
    int32_t sum_flag;  //已经够250个标志
} sliding_mean_filter_type_t;

//一阶低通滤波参数
typedef __packed struct
{
    fp32 input;      //输入数据
    fp32 last_input; //上次数据
    fp32 out;        //滤波输出的数据
    fp32 num;        //滤波参数
} first_order_filter_type_t;

/* 低通滤波 */
extern float first_order_filter(first_order_filter_type_t *first_order_filter_type, fp32 input);
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 num);

/* 平滑滤波 */
float sliding_mean_filter(sliding_mean_filter_type_t *mean_filter, fp32 Input, int32_t num); //均值滑窗滤波
void sliding_mean_filter_init(sliding_mean_filter_type_t *mean_filter);                      //均值滑窗滤波初始化（可不用，直接定义结构体时给初值）

#endif
