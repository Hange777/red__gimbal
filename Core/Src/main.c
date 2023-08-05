/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Servos.h"
#include "config.h"
#include "pid.h"
#include "arm_math.h"
#include "maths.h"
#include "struct_variables.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern TIM_HandleTypeDef htim4;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
system_state_e system_state;
float yaw_angle = Y_OFFSET;
float pitch_angle = X_OFFSET;
pid_parameter_t pid_y;
pid_parameter_t pid_x;
float x=0;
float y=0;
float x_angle=0;
float y_angle=0;
#define x1 9.5f
#define y1 14.7f

float task1list[][2]=
{
//	0,0,
	25,25,
	0,0,
	127,127
};
float* task1list_p = &task1list[0][0];

float task2list[][2]=
{
//	0,0,
	26,0,
	26,26,
	-26,26,
	-26,-25,
	26,-25,
	26,1,
	0,1,
//	x1,y1+0.8,
//	-x1,y1+0.8,
//	-x1,-y1,
//	x1,-y1,
//	x1,y1+0.8,
	127,127
};
float* task2list_p = &task2list[0][0];

volatile uint8_t usart_even = 0;
int autolist[5][2]=
{
	0,0,
	0,0,
	0,0,
	0,0,
	127,127
};
int* autopoint = &autolist[0][0];
float autopointlist[][2]=
{
	0,0,
	0,0,
	0,0,
	0,0,
	0,0,
//	25,25,
	127,127
};
float* autopointpoint = &autopointlist[0][0];
uint8_t usart_rec[40];
uint8_t turn_flag=1;
GPIO_PinState KEY1;
GPIO_PinState KEY2;
GPIO_PinState KEY3;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	system_state = TASK1;
	
  HAL_UART_Receive_IT(&huart1, usart_rec, 28);
	/*-------------------------------------------------------------------------pid--------------------------------------------------------------------------*/
	PidInit(&pid_y,1,0,10,StepIn);
	PidInitMode(&pid_y,StepIn,0.0005f,0);
	pid_y.LastSetValue = 0.025f + PER_ANGLE*Y_OFFSET;
	PidInit(&pid_x,1,0,10,StepIn);
	PidInitMode(&pid_x,StepIn,0.0005f,0);
	pid_x.LastSetValue = 0.025f + PER_ANGLE*X_OFFSET;
	/*-------------------------------------------------------------------------servos--------------------------------------------------------------------------*/
	servos_start(&htim4,TIM_CHANNEL_1);//pitch
	servos_start(&htim4,TIM_CHANNEL_2);//yaw
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	volatile uint32_t count=0;
	volatile int task3_stop = 0;
  while (1)
  {
		static system_state_e last_system_state;
		/*-------------------------------------------------------------------------key_deal--------------------------------------------------------------------------*/
		KEY1 = !HAL_GPIO_ReadPin(KEY_1_GPIO_Port,KEY_1_Pin);
		KEY2 = !HAL_GPIO_ReadPin(KEY_2_GPIO_Port,KEY_2_Pin);
		KEY3 = !HAL_GPIO_ReadPin(KEY_3_GPIO_Port,KEY_3_Pin);
		if(KEY1)
		{
			last_system_state = STOP;
			system_state = TASK1;
		}
		if(KEY3)
		{
			if(system_state == TASK2)
			{
				last_system_state = STOP;
				system_state = TASK3;
			}
			else
			{
				last_system_state = STOP;
				system_state = TASK2;
			}
			
		}
		if(KEY2)
		{
			if(system_state == TASK3)
			{
				task3_stop = 1;
//				servos_stop(&htim4,TIM_CHANNEL_1);//pitch
//				servos_stop(&htim4,TIM_CHANNEL_2);//yaw
			}
			else
			{
				last_system_state = STOP;
				system_state = STOP;
			}
		}
		else
		{
			if(system_state == TASK3)
			{
				task3_stop = 0;
//				servos_start(&htim4,TIM_CHANNEL_1);//pitch
//				servos_start(&htim4,TIM_CHANNEL_2);//yaw
			}
		}
		/*-------------------------------------------------------------------------data_deal--------------------------------------------------------------------------*/
		if(usart_even)
		{
			if(usart_rec[0]=='f'
			 &&usart_rec[1]=='f'
			&&usart_rec[26]=='e'
			&&usart_rec[27]=='e')
			{
//				if(system_state != TASK3)
//				{
					int* j = &autolist[0][0];
					for(int i = 2;i<27;i+=3)
					{
						*j=(usart_rec[i]-48)*100 + (usart_rec[i+1]-48)*10 + (usart_rec[i+2]-48)*1;
						j++;
					}
					
					j = &autolist[0][0];
					float* k = &autopointlist[0][0];
					for(int i = 0;i<4;i++)
					{
						*k++ = CAMERA_Y_GAIN *(-(*j++ * CAM2CAN_Y - 25   )) + CAMERA_Y_OFFSET;
						*k++ = CAMERA_X_GAIN *(-(*j++ * CAM2CAN_X - 25 -4)) + CAMERA_X_OFFSET;
					}
					autopointlist[4][0] = autopointlist[0][0];
					autopointlist[4][1] = autopointlist[0][1];
//				}
			}
			else
			{
				HAL_UART_AbortReceive_IT(&huart1);
				HAL_UART_Receive_IT(&huart1, usart_rec, 28);
			}
			usart_even=0;
		}
		
		
		
	/*-------------------------------------------------------------------------cmd--------------------------------------------------------------------------*/

		count++;
//		turn_flag=1;
		if(last_system_state!=system_state)
		{
			if(last_system_state == STOP )
			{
					servos_start(&htim4,TIM_CHANNEL_1);//pitch
					servos_start(&htim4,TIM_CHANNEL_2);//yaw
			}
			task1list_p = &task1list[0][0];
			task2list_p = &task2list[0][0];
			autopointpoint = &autopointlist[0][0];
			x=25;
			y=25;
			count=100000;
			turn_flag=1;
		}
		last_system_state = system_state;
		switch(system_state)
		{
			case TASK1:
			{
				if(count>=150 && turn_flag)
					{
					if(*task1list_p==127)
					{
						task1list_p = &task1list[0][0];
						turn_flag=0;
//						system_state = TASK2;
						count = 0;
						break;
					}
					y=*task1list_p++;
					x=*task1list_p++;

					count = 0;
				}
				break;
			}
			case TASK2:
			{
				if(count>=150 && turn_flag)
					{
						if(*task2list_p==127)
						{
							task2list_p = &task2list[0][0];
							turn_flag=0;
//							system_state = TASK3;
													count = 0;
						break;
						}
						y=*task2list_p++;
						x=*task2list_p++;

						count = 0;
					}
					break;
			}
			case TASK3:
			{
				static float k = 0;
				static float delat_x = 0;
				if(task3_stop == 0)
				{
					if(count>=350 && turn_flag)
					{
						if(*autopointpoint==127)
						{
							autopointpoint = &autopointlist[0][0];
							turn_flag=0;
	//						system_state = TASK1;
							count = 0;
							break;
						}
						k = (y-*autopointpoint) / (x- *(autopointpoint+1));
						delat_x = (*(autopointpoint+1) - x) / 350.0f;
						autopointpoint++;
						autopointpoint++;
						count = 0;
					}
					if(turn_flag)
					{
						x += delat_x;
						y += delat_x*k;
					}
				}
				else
				{
					count--;
				}
				break;
			}
			case STOP:
			{
				servos_stop(&htim4,TIM_CHANNEL_1);//pitch
				servos_stop(&htim4,TIM_CHANNEL_2);//yaw
				break;
			}
		}
		value_limit(x,-30.0f,30.0f);
		value_limit(y,-30.0f,30.0f);
		/*-------------------------------------------------------------------------yaw--------------------------------------------------------------------------*/
		y*=1.02f;
		y_angle = PI2RAD * atan2(y,100);
		float yaw_control=0.025f + PER_ANGLE * yaw_angle + PER_ANGLE * y_angle;
		float yaw_control_out;
		yaw_control_out = PidCalculate(&pid_y,yaw_control,0);
		PWMSetDutyRatio(&htim4,TIM_CHANNEL_2,yaw_control_out);
		y/=1.02f;
		/*-------------------------------------------------------------------------pitch--------------------------------------------------------------------------*/
		x*=0.95f;
		if(x>-tan((X_OFFSET-PITCH_ZERO_ANGLE)/PI2RAD)*100)
		{
			x_angle = PI2RAD *atan2(tan((X_OFFSET-PITCH_ZERO_ANGLE)/PI2RAD)*100 + x,100);
		}
		else
		{
			float temp_x;
			temp_x=x+tan((X_OFFSET-PITCH_ZERO_ANGLE)/PI2RAD)*100;
			x_angle = PI2RAD *atan2(temp_x,100);
		}
		float pitch_control=0.025f + PER_ANGLE*PITCH_ZERO_ANGLE + PER_ANGLE* x_angle;
		float pitch_control_out;
		pitch_control_out = PidCalculate(&pid_x,pitch_control,0);
		PWMSetDutyRatio(&htim4,TIM_CHANNEL_1,pitch_control_out);
		x/=0.95f;

		

		HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1){
		usart_even = 1;
		while(HAL_OK == HAL_UART_Receive_IT(&huart1, usart_rec, 28));
//		if(usart_rec[0]=='f')
//		{
//			volatile int i = 1;
//			HAL_StatusTypeDef state;
//			do
//			{
//				if(i>=36)
//				{
//					break;
//				}
//				state = HAL_UART_Receive(&huart1,&usart_rec[i],1,0);
//				i++;
//			}
//			while(state == HAL_OK);
//		}
//		while(HAL_OK == HAL_UART_Receive_IT(&huart1, usart_rec, 1));
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
