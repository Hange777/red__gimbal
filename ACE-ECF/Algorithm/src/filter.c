#include "filter.h"

/*********************************************************************һ�׿�����***********************************************************************/
/**
 * @author  Liu heng
 * һ�׿������˲�������RoboMaster��̳
 *   һά�������˲���
 *   ʹ��ʱ�ȶ���һ��kalmanָ�룬Ȼ�����kalmanCreate()����һ���˲���
 *   ÿ�ζ�ȡ�����������ݺ󼴿ɵ���KalmanFilter()�������ݽ����˲�
 *          ʹ��ʾ��
 *          extKalman_t p;                  //����һ���������˲����ṹ��
 *          float SersorData;             //��Ҫ�����˲�������
 *          KalmanCreate(&p,20,200);      //��ʼ�����˲�����Q=20 R=200����
 *          while(1)
 *          {
 *             SersorData = sersor();                     //��ȡ����
 *             SersorData = KalmanFilter(&p,SersorData);  //�����ݽ����˲�
 *          }
 */

/**
 * @name   kalmanCreate
 * @brief  ����һ���������˲���
 * @param  p:  �˲���
 *         T_Q:ϵͳ����Э����
 *         T_R:��������Э����
 *
 * @retval none
 * @attention R�̶���QԽ�󣬴���Խ���β���ֵ��Q�������ֻ�ò���ֵ
 *		       	��֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ��
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
 * @brief  �������˲���
 * @param  p:  �˲���
 *         dat:���˲�����
 * @retval �˲��������
 * @attention Z(k)��ϵͳ����,������ֵ   X(k|k)�ǿ������˲����ֵ,���������
 *            A=1 B=0 H=1 I=1  W(K)  V(k)�Ǹ�˹������,�����ڲ���ֵ����,���Բ��ù�
 *            �����ǿ�������5�����Ĺ�ʽ
 *            һ��H'��Ϊ������,����Ϊת�þ���
 */

float KalmanFilter(extKalman_t *p, float dat)
{
  p->X_mid = p->A * p->X_last;                    // ���㵱ǰʱ�̵�Ԥ����(1)         x(k|k-1) = A*X(k-1|k-1) + B*U(k) + W(K)
  p->P_mid = p->A * p->P_last + p->Q;             // ���㵱ǰʱ��Ԥ������Э����(2)    p(k|k-1) = A*p(k-1|k-1)*A' + Q
  p->kg = p->P_mid / (p->P_mid + p->R);           // ����klaman����(4)               kg(k) = p(k|k-1)*H' / (H*p(k|k-1)*H'+R)
  p->X_now = p->X_mid + p->kg * (dat - p->X_mid); // ���㵱ǰʱ�̵����Ž��(3)         x(k|k) = X(k|k-1) + kg(k)*(Z(k) - H*X(k|k-1))
  p->P_now = (1 - p->kg) * p->P_mid;              // ���㵱ǰʱ�����Ž����Э����(5)    p(k|k) = (I-kg(k)*H) * P(k|k-1)

  p->P_last = p->P_now; // ״̬����
  p->X_last = p->X_now;

  return p->X_now; // ���Ԥ����x(k|k)
}

/*******************************************************************���׿�����***************************************************************/

/**
 * @brief      �Ӿ����ݿ�������ʼ��
 * @param[in]
 * @retval
 * @attention
 */
kalman_filter_init_t visual_kalman_filter_init = {0};
kalman_filter_t visual_kalman_filter = {0};
void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
  mat_init(&F->xhat, 4, 1, (float *)I->xhat_data);           // 4*1 ����ֵ(k-1)
  mat_init(&F->xhatminus, 4, 1, (float *)I->xhatminus_data); // 4*1 ����ֵ(k)
  mat_init(&F->z, 2, 1, (float *)I->z_data);                 // 2*1 ����ֵ
  mat_init(&F->Pminus, 4, 4, (float *)I->Pminus_data);       // 4*4 �µ�Э�������
  mat_init(&F->k, 4, 2, (float *)I->K_data);                 // 4*2 kalman����
  mat_init(&F->p, 4, 4, (float *)I->P_data);                 // 4*4 Э�������
  mat_init(&F->AT, 4, 4, (float *)I->AT_data);               // 4*4 ״̬����
  mat_init(&F->HT, 4, 2, (float *)I->HT_data);               // 4*2
  mat_init(&F->A, 4, 4, (float *)I->A_data);                 // 4*4
  mat_init(&F->H, 2, 4, (float *)I->H_data);                 // 2*4
  mat_init(&F->Q, 4, 4, (float *)I->Q_data);                 // 4*4
  mat_init(&F->R, 2, 2, (float *)I->R_data);                 // 4*4
  mat_trans(&F->A, &F->AT);
  mat_trans(&F->H, &F->HT);
}

// һ�׵�ͨ�˲�����
float first_order_filter(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
  first_order_filter_type->input = input;
  first_order_filter_type->out = first_order_filter_type->input * first_order_filter_type->num + (1 - first_order_filter_type->num) * first_order_filter_type->last_input;
  first_order_filter_type->last_input = first_order_filter_type->out;

  return first_order_filter_type->out;
}

// һ�׵�ͨ�˲���ʼ��
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
 *���ܣ�������ֵ�˲�������ʼ��(������)
 *���룺�˲�����ṹ��
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
 *���ܣ�������ֵ�˲��������ͣ�------����С���ȸ�Ƶ����
 *���룺1.�˲�����ṹ��  2.����ֵ 3.��ֵ����
 *�����������˲����ֵ��250�Σ�
 */
float sliding_mean_filter(sliding_mean_filter_type_t *mean_filter, float Input, int num)
{
  // ����
  mean_filter->Input = Input;
  mean_filter->FIFO[mean_filter->count_num] = mean_filter->Input;
  mean_filter->count_num++;

  if (mean_filter->count_num == num)
  {
    mean_filter->count_num = 0;
    mean_filter->sum_flag = 1;
  }
  // ���
  if (mean_filter->sum_flag == 1)
  {
    for (int count = 0; count < num; count++)
    {
      mean_filter->Sum += mean_filter->FIFO[count];
    }
  }
  // ��ֵ
  mean_filter->Output = mean_filter->Sum / num;
  mean_filter->Sum = 0;

  return mean_filter->Output;
}
