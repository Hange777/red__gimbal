#ifndef MAT_H
#define MAT_H

#include <string.h>

#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32
#define mat arm_matrix_instance_f32 //�������ṹ�ľ���ṹ��   ( ���� �������������Լ�����(����) )
#define mat_init arm_mat_init_f32   //��������ʼ��
#define mat_add arm_mat_add_f32     //�������ӷ�
#define mat_sub arm_mat_sub_f32     //����������
#define mat_mult arm_mat_mult_f32   //�������˷�
#define mat_trans arm_mat_trans_f32 //�������ת��
#define mat_inv arm_mat_inverse_f32 //���������


#endif



