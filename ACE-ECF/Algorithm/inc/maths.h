#ifndef __MATHS_H
#define __MATHS_H
#include "struct_typedef.h"

//����ֵ
#define user_abs(x) ((x) > (0) ? (x) : (-(x)))

//�����޷�
#define value_limit(val, min, max)   \
    {                                \
        (val) = (val) < (min) ? (min) : (val); \
        (val) = (val) > (max) ? (max) : (val); \
    }
//���ֵ
#define max(x, y) ((x) > (y) ? (x) : (y))

//�˶����ٶ�����б�º���
typedef __packed struct
{
    float Input;      //��ǰȡ��ֵ
    float Last_Input; //�ϴ�ȡ��ֵ
    float Output;     //���ֵ
    float acc_now;    //��ǰ���ٶ�
    float acc_limit;  //��Ҫ���Ƶļ��ٶ�
} acceleration_control_type_t;

int16_t max_abs(int16_t x, int16_t y); //ȡ���ֵ�ľ���ֵ
fp32 invSqrt(fp32 x);                  //ƽ��������
float float_min_distance(float target, float actual, float minValue, float maxValue); //Ѱ��Сֵ
/* �˶�����б�º��������ٶ����ƣ���16λ�� */
int16_t motion_acceleration_control(acceleration_control_type_t *acceleration_control, int16_t Input, int16_t Limit); //�˶����ٶ�����

/* ѭ������ */
int16_t loop_restriction_int16(int16_t num, int16_t limit_num);         // 16λѭ���޷�
float loop_restriction_float(float num, float limit_num);               //����ѭ���޷�
float loop_fp32_constrain(float Input, float minValue, float maxValue); //ѭ�����ƣ���̨�Ƕȴ���
float cos_calculate(float angle);
float sin_calculate(float angle);

/* б�º��� */
void data_accelerated_control(float *input, float acc); //���ٶ�����б�º���

/* �޷��˲� */
float limiting_filter(float new_value, float last_value, float delat_max);

//���ȸ�ʽ��Ϊ-PI~PI
#define RAD_FORMAT(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
