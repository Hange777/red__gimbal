/**
 * ************************* Dongguan-University of Technology -ACE**************************
 * @file lqr.h
 * @brief 
 * @author 洪张茹
 * @version 1.0
 * @date 2022-12-05
 * 
 * ************************* Dongguan-University of Technology -ACE**************************
 */
#ifndef LQR_H
#define LQR_H

/* Include */
#include "stm32f4xx.h"
#include "cmsis_os.h"
#include <string.h>
#include "mat.h"


/* Define */
#ifndef lqr_abs
#define lqr_abs(x) ((x > 0) ? x : -x)
#endif

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif




/* Private Body */
typedef struct 
{
  uint8_t System_State_Size;     //对应u与input
  uint8_t Control_Size;          //对应Output

  //非线性控制量
  double Control_Variable;
  double Control_Area;      //
	
  double *Input;
  double *Output;
  double *k;
  
  double *target;
	
  void (*User_Func_f)(void);

}LQR_t;



/* Funtion */
//extern void LQR_Calculate(LQR_t *lqr);
void LQR_Init(LQR_t *lqr, uint8_t system_state_size, uint8_t control_size, double *k);
void LQR_Data_Update(LQR_t *lqr, double* system_state);
void LQR_Calculate(LQR_t *lqr);

#endif


