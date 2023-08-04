#include "filter.h"

/*********************************************************************一阶卡尔曼***********************************************************************/
/**
 * @author  Liu heng
 * 一阶卡尔曼滤波器来自RoboMaster论坛
 *   一维卡尔曼滤波器
 *   使用时先定义一个kalman指针，然后调用kalmanCreate()创建一个滤波器
 *   每次读取到传感器数据后即可调用KalmanFilter()来对数据进行滤波
 *          使用示例
 *          extKalman_t p;                  //定义一个卡尔曼滤波器结构体
 *          float SersorData;             //需要进行滤波的数据
 *          KalmanCreate(&p,20,200);      //初始化该滤波器的Q=20 R=200参数
 *          while(1)
 *          {
 *             SersorData = sersor();                     //获取数据
 *             SersorData = KalmanFilter(&p,SersorData);  //对数据进行滤波
 *          }
 */

/**
 * @name   kalmanCreate
 * @brief  创建一个卡尔曼滤波器
 * @param  p:  滤波器
 *         T_Q:系统噪声协方差
 *         T_R:测量噪声协方差
 *
 * @retval none
 * @attention R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
 *		       	反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
 */
void KalmanCreate(extKalman_t *p, float T_Q, float T_R)
{
  p->X_last = (float)0;
  p->P_last = 0;
  p->Q = T_Q;
  p->R = T_R;
  p->A = 1;
  p->B = 0;
  p->H = 1;
  p->X_mid = p->X_last;
}

/**
 * @name   KalmanFilter
 * @brief  卡尔曼滤波器
 * @param  p:  滤波器
 *         dat:待滤波数据
 * @retval 滤波后的数据
 * @attention Z(k)是系统输入,即测量值   X(k|k)是卡尔曼滤波后的值,即最终输出
 *            A=1 B=0 H=1 I=1  W(K)  V(k)是高斯白噪声,叠加在测量值上了,可以不用管
 *            以下是卡尔曼的5个核心公式
 *            一阶H'即为它本身,否则为转置矩阵
 */

float KalmanFilter(extKalman_t *p, float dat)
{
  p->X_mid = p->A * p->X_last;                    // 计算当前时刻的预测结果(1)         x(k|k-1) = A*X(k-1|k-1) + B*U(k) + W(K)
  p->P_mid = p->A * p->P_last + p->Q;             // 计算当前时刻预测结果的协方差(2)    p(k|k-1) = A*p(k-1|k-1)*A' + Q
  p->kg = p->P_mid / (p->P_mid + p->R);           // 计算klaman增益(4)               kg(k) = p(k|k-1)*H' / (H*p(k|k-1)*H'+R)
  p->X_now = p->X_mid + p->kg * (dat - p->X_mid); // 计算当前时刻的最优结果(3)         x(k|k) = X(k|k-1) + kg(k)*(Z(k) - H*X(k|k-1))
  p->P_now = (1 - p->kg) * p->P_mid;              // 计算当前时刻最优结果的协方差(5)    p(k|k) = (I-kg(k)*H) * P(k|k-1)

  p->P_last = p->P_now; // 状态更新
  p->X_last = p->X_now;

  return p->X_now; // 输出预测结果x(k|k)
}

/*******************************************************************二阶卡尔曼***************************************************************/

/**
 * @brief      视觉数据卡尔曼初始化
 * @param[in]
 * @retval
 * @attention
 */
kalman_filter_init_t visual_kalman_filter_init = {0};
kalman_filter_t visual_kalman_filter = {0};
void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
  mat_init(&F->xhat, 4, 1, (float *)I->xhat_data);           // 4*1 特征值(k-1)
  mat_init(&F->xhatminus, 4, 1, (float *)I->xhatminus_data); // 4*1 特征值(k)
  mat_init(&F->z, 2, 1, (float *)I->z_data);                 // 2*1 测量值
  mat_init(&F->Pminus, 4, 4, (float *)I->Pminus_data);       // 4*4 新的协方差矩阵
  mat_init(&F->k, 4, 2, (float *)I->K_data);                 // 4*2 kalman增益
  mat_init(&F->p, 4, 4, (float *)I->P_data);                 // 4*4 协方差矩阵
  mat_init(&F->AT, 4, 4, (float *)I->AT_data);               // 4*4 状态方程
  mat_init(&F->HT, 4, 2, (float *)I->HT_data);               // 4*2
  mat_init(&F->A, 4, 4, (float *)I->A_data);                 // 4*4
  mat_init(&F->H, 2, 4, (float *)I->H_data);                 // 2*4
  mat_init(&F->Q, 4, 4, (float *)I->Q_data);                 // 4*4
  mat_init(&F->R, 2, 2, (float *)I->R_data);                 // 4*4
  mat_trans(&F->A, &F->AT);
  mat_trans(&F->H, &F->HT);
}

// 一阶低通滤波计算
float first_order_filter(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
  first_order_filter_type->input = input;
  first_order_filter_type->out = first_order_filter_type->input * first_order_filter_type->num + (1 - first_order_filter_type->num) * first_order_filter_type->last_input;
  first_order_filter_type->last_input = first_order_filter_type->out;

  return first_order_filter_type->out;
}

// 一阶低通滤波初始化
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 num)
{
  if (first_order_filter_type == NULL)
  {
    return;
  }

  first_order_filter_type->input = 0;
  first_order_filter_type->last_input = 0;
  first_order_filter_type->num = num;
  first_order_filter_type->out = 0;
}

/*
 *功能：滑动均值滤波参数初始化(浮点型)
 *输入：滤波对象结构体
 */
void sliding_mean_filter_init(sliding_mean_filter_type_t *mean_filter)
{
  mean_filter->count_num = 0;
  for (int i = 0; i < 20; i++)
    mean_filter->FIFO[i] = 0.0f;
  mean_filter->Input = 0.0f;
  mean_filter->Output = 0.0f;
  mean_filter->Sum = 0.0f;
  mean_filter->sum_flag = 0;
}

/*
 *功能：滑动均值滤波（浮点型）------抑制小幅度高频噪声
 *传入：1.滤波对象结构体  2.更新值 3.均值数量
 *传出：滑动滤波输出值（250次）
 */
float sliding_mean_filter(sliding_mean_filter_type_t *mean_filter, float Input, int num)
{
  // 更新
  mean_filter->Input = Input;
  mean_filter->FIFO[mean_filter->count_num] = mean_filter->Input;
  mean_filter->count_num++;

  if (mean_filter->count_num == num)
  {
    mean_filter->count_num = 0;
    mean_filter->sum_flag = 1;
  }
  // 求和
  if (mean_filter->sum_flag == 1)
  {
    for (int count = 0; count < num; count++)
    {
      mean_filter->Sum += mean_filter->FIFO[count];
    }
  }
  // 均值
  mean_filter->Output = mean_filter->Sum / num;
  mean_filter->Sum = 0;

  return mean_filter->Output;
}
