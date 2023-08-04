#ifndef MAT_H
#define MAT_H

#include <string.h>

#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32
#define mat arm_matrix_instance_f32 //浮点矩阵结构的矩阵结构体   ( 包含 行数、列数、以及数组(数据) )
#define mat_init arm_mat_init_f32   //浮点矩阵初始化
#define mat_add arm_mat_add_f32     //浮点矩阵加法
#define mat_sub arm_mat_sub_f32     //浮点矩阵减法
#define mat_mult arm_mat_mult_f32   //浮点矩阵乘法
#define mat_trans arm_mat_trans_f32 //浮点矩阵转置
#define mat_inv arm_mat_inverse_f32 //浮点矩阵逆


#endif



