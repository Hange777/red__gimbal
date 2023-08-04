/**
 * ************************* Dongguan-University of Technology -ACE**************************
 * @file lqr.c
 * @brief 
 * @author 洪张茹
 * @version 1.0
 * @date 2022-12-05
 * 
 * ************************* Dongguan-University of Technology -ACE**************************
 */


#include "lqr.h"

/* Funtion */
void LQR_Data_Clear(LQR_t *lqr);


/**
 * ************************* Dongguan-University of Technology -ACE**************************
 * @brief LQR计算
 * @param  lqr              
 * ************************* Dongguan-University of Technology -ACE**************************
 */
void LQR_Calculate(LQR_t *lqr)
{
	int i,j;
	for(i = 0; i < lqr->Control_Size; i++)
	{
		lqr->Output[i] = 0;
		for(j = 0; j < lqr->System_State_Size; j++)
		{
			lqr->Output[i] += lqr->Input[j] * lqr->k[i * lqr->System_State_Size + j];
		}
	}
	
}


/**
 * ************************* Dongguan-University of Technology -ACE**************************
 * @brief  LQR初始化
 * @param  lqr              
 * @param  system_state_size
 * @param  control_size     
 * @param  control_area     
 * ************************* Dongguan-University of Technology -ACE**************************
 */
void LQR_Init(LQR_t *lqr, uint8_t system_state_size, uint8_t control_size, double *k)
{
  lqr->System_State_Size = system_state_size;
  lqr->Control_Size = control_size;
	
  //lqr->Control_Area = control_area;
  if(system_state_size != 0)
  {
    lqr->Input = (double *)user_malloc(sizeof(double) * system_state_size * control_size);
	memset(lqr->Input, 0, sizeof(double) * system_state_size * control_size);
	lqr->k = (double *)user_malloc(sizeof(double) * system_state_size * control_size);
	memset(lqr->k, 0, sizeof(double) * system_state_size * control_size);

  }
  if(control_size != 0)
  {
    lqr->Output = (double *)user_malloc(sizeof(double) * control_size);
	memset(lqr->Output, 0, sizeof(double) * control_size);

  }
  
	  lqr->k = k;
  
}



/**
 * ************************* Dongguan-University of Technology -ACE**************************
 * @brief  LQR数据清除
 * @param  lqr              
 * ************************* Dongguan-University of Technology -ACE**************************
 */
void LQR_Data_Clear(LQR_t *lqr)
{
  memset(lqr->Input, 0, sizeof(double) * lqr->System_State_Size * lqr->Control_Size);
  
  memset(lqr->Output, 0, sizeof(double) * lqr->Control_Size);
}


/**
 * ************************* Dongguan-University of Technology -ACE**************************
 * @brief  LQR数据更新
 * @param  lqr              
 * ************************* Dongguan-University of Technology -ACE**************************
 */
void LQR_Data_Update(LQR_t *lqr, double* system_state)
{
	int i = 0;
	for(; i < lqr->System_State_Size; i++)
	{
		lqr->Input[i] = system_state[i];
	}
}




