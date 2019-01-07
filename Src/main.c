/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define PERIOD_MIN 100			// minimum period in milliseconds
#define PERIOD_MAX 600			// maximum period in milliseconds

uint32_t tickNow;						// global access to tick count

typedef enum stateLed1
{
	LED1_R,
	LED1_G,
	LED1_B
} stateLed1_t;
stateLed1_t stateLed1 = LED1_R;

typedef enum stateLed2
{
	LED2_R,
	LED2_G,
	LED2_B
} stateLed2_t;
stateLed2_t stateLed2 = LED2_R;

typedef enum stateLed3
{
	LED3_R,
	LED3_G,
	LED3_B
} stateLed3_t;
stateLed3_t stateLed3 = LED3_R;

typedef enum stateLed4
{
	LED4_R,
	LED4_G,
	LED4_B
} stateLed4_t;
stateLed4_t stateLed4 = LED4_R;

typedef enum stateLed5
{
	LED5_R,
	LED5_G,
	LED5_B
} stateLed5_t;
stateLed5_t stateLed5 = LED5_R;

typedef enum stateLed6
{
	LED6_R,
	LED6_G,
	LED6_B
} stateLed6_t;
stateLed6_t stateLed6 = LED6_R;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint32_t GenerateRandomNumber(void);
void led1Handler(void);
void led2Handler(void);
void led3Handler(void);
void led4Handler(void);
void led5Handler(void);
void led6Handler(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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

  /* USER CODE BEGIN 2 */
	srand(10); // randomize seed

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// get the tick count
		tickNow = HAL_GetTick();
		// service LED 1 state machine
		led1Handler();
		led2Handler();
		led3Handler();
		led4Handler();
		led5Handler();
		led6Handler();
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

// generates a pseudo-random number in the range PERIOD_MIN to PERIOD_MAX
uint32_t GenerateRandomNumber(void)
{
	// take a random number in the range 0 -> 0x7fffffff
	uint32_t x = rand();													
	// convert to the desired scale
	x = (x / (RAND_MAX / (PERIOD_MAX - PERIOD_MIN))) + PERIOD_MIN;
	return x;
}
 // state handler for LED1
void led1Handler(void)
{
	switch(stateLed1)
		{	
			static uint8_t frist = 0;					// first pass flag
			static uint32_t tickWas = 0;			// sysTick at the last transition
			static uint32_t period = 0;				// random period
			
			case LED1_R:
			{
				// if this is the first pass through this state
				if(frist)
				{
					period = GenerateRandomNumber();
					HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, GPIO_PIN_SET);
					tickWas = tickNow;
					frist = 0;
				}
				// otherwise wait until the assigned random period has elapsed and decide which colour is next
				if((tickNow - tickWas) > period)
				{
					HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, GPIO_PIN_RESET);
					if(period < 550)
					{
						stateLed1 = LED1_G;
						frist = 1;
					}
					else
					{
						stateLed1 = LED1_B;
						frist = 1;
					}
				}
			break;
			}
			
			case LED1_G:
			{
				// if this is the first pass through this state
				if(frist)
				{
					period = GenerateRandomNumber();
					HAL_GPIO_WritePin(LED1_G_GPIO_Port, LED1_G_Pin, GPIO_PIN_SET);
					tickWas = tickNow;
					frist = 0;
				}
				// otherwise wait until the assigned random period has elapsed and decide which colour is next
				if((tickNow - tickWas) > period)
				{
					HAL_GPIO_WritePin(LED1_G_GPIO_Port, LED1_G_Pin, GPIO_PIN_RESET);
					if(period < 550)
					{
						stateLed1 = LED1_R;
						frist = 1;
					}
					else
					{
						stateLed1 = LED1_B;
						frist = 1;
					}
				}
				break;
				
			}
			case LED1_B:
			{
				// if this is the first pass through this state
				if(frist)
				{
					period = GenerateRandomNumber();
					HAL_GPIO_WritePin(LED1_B_GPIO_Port, LED1_B_Pin, GPIO_PIN_SET);
					tickWas = tickNow;
					frist = 0;
				}
				// otherwise wait until the assigned random period has elapsed and decide which colour is next
				if((tickNow - tickWas) > period)
				{
					HAL_GPIO_WritePin(LED1_B_GPIO_Port, LED1_B_Pin, GPIO_PIN_RESET);
					if(period < 550)
					{
						stateLed1 = LED1_R;
						frist = 1;
					}
					else
					{
						stateLed1 = LED1_G;
						frist = 1;
					}
				}
				break;
			}
			
			default:
			{
				while(1);
			}
		}
}


void led2Handler(void)
{
	switch(stateLed2)
		{	
			static uint8_t frist = 0;					// first pass flag
			static uint32_t tickWas = 0;			// sysTick at the last transition
			static uint32_t period = 0;				// random period
			
			case LED2_R:
			{
				// if this is the first pass through this state
				if(frist)
				{
					period = GenerateRandomNumber();
					HAL_GPIO_WritePin(LED2_R_GPIO_Port, LED2_R_Pin, GPIO_PIN_SET);
					tickWas = tickNow;
					frist = 0;
				}
				// otherwise wait until the assigned random period has elapsed and decide which colour is next
				if((tickNow - tickWas) > period)
				{
					HAL_GPIO_WritePin(LED2_R_GPIO_Port, LED2_R_Pin, GPIO_PIN_RESET);
					if(period < 550)
					{
						stateLed2 = LED2_G;
						frist = 1;
					}
					else
					{
						stateLed2 = LED2_B;
						frist = 1;
					}
				}
			break;
			}
			
			case LED2_G:
			{
				// if this is the first pass through this state
				if(frist)
				{
					period = GenerateRandomNumber();
					HAL_GPIO_WritePin(LED2_G_GPIO_Port, LED2_G_Pin, GPIO_PIN_SET);
					tickWas = tickNow;
					frist = 0;
				}
				// otherwise wait until the assigned random period has elapsed and decide which colour is next
				if((tickNow - tickWas) > period)
				{
					HAL_GPIO_WritePin(LED2_G_GPIO_Port, LED2_G_Pin, GPIO_PIN_RESET);
					if(period < 550)
					{
						stateLed2 = LED2_R;
						frist = 1;
					}
					else
					{
						stateLed2 = LED2_B;
						frist = 1;
					}
				}
				break;
				
			}
			case LED2_B:
			{
				// if this is the first pass through this state
				if(frist)
				{
					period = GenerateRandomNumber();
					HAL_GPIO_WritePin(LED2_B_GPIO_Port, LED2_B_Pin, GPIO_PIN_SET);
					tickWas = tickNow;
					frist = 0;
				}
				// otherwise wait until the assigned random period has elapsed and decide which colour is next
				if((tickNow - tickWas) > period)
				{
					HAL_GPIO_WritePin(LED2_B_GPIO_Port, LED2_B_Pin, GPIO_PIN_RESET);
					if(period < 550)
					{
						stateLed2 = LED2_R;
						frist = 1;
					}
					else
					{
						stateLed2 = LED2_G;
						frist = 1;
					}
				}
				break;
			}
			
			default:
			{
				while(1);
			}
		}
}

void led3Handler(void)
{
	switch(stateLed3)
		{	
			static uint8_t frist = 0;					// first pass flag
			static uint32_t tickWas = 0;			// sysTick at the last transition
			static uint32_t period = 0;				// random period
			
			case LED3_R:
			{
				// if this is the first pass through this state
				if(frist)
				{
					period = GenerateRandomNumber();
					HAL_GPIO_WritePin(LED3_R_GPIO_Port, LED3_R_Pin, GPIO_PIN_SET);
					tickWas = tickNow;
					frist = 0;
				}
				// otherwise wait until the assigned random period has elapsed and decide which colour is next
				if((tickNow - tickWas) > period)
				{
					HAL_GPIO_WritePin(LED3_R_GPIO_Port, LED3_R_Pin, GPIO_PIN_RESET);
					if(period < 550)
					{
						stateLed3 = LED3_G;
						frist = 1;
					}
					else
					{
						stateLed3 = LED3_B;
						frist = 1;
					}
				}
			break;
			}
			
			case LED3_G:
			{
				// if this is the first pass through this state
				if(frist)
				{
					period = GenerateRandomNumber();
					HAL_GPIO_WritePin(LED3_G_GPIO_Port, LED3_G_Pin, GPIO_PIN_SET);
					tickWas = tickNow;
					frist = 0;
				}
				// otherwise wait until the assigned random period has elapsed and decide which colour is next
				if((tickNow - tickWas) > period)
				{
					HAL_GPIO_WritePin(LED3_G_GPIO_Port, LED3_G_Pin, GPIO_PIN_RESET);
					if(period < 550)
					{
						stateLed3 = LED3_R;
						frist = 1;
					}
					else
					{
						stateLed3 = LED3_B;
						frist = 1;
					}
				}
				break;
				
			}
			case LED3_B:
			{
				// if this is the first pass through this state
				if(frist)
				{
					period = GenerateRandomNumber();
					HAL_GPIO_WritePin(LED3_B_GPIO_Port, LED3_B_Pin, GPIO_PIN_SET);
					tickWas = tickNow;
					frist = 0;
				}
				// otherwise wait until the assigned random period has elapsed and decide which colour is next
				if((tickNow - tickWas) > period)
				{
					HAL_GPIO_WritePin(LED3_B_GPIO_Port, LED3_B_Pin, GPIO_PIN_RESET);
					if(period < 550)
					{
						stateLed3 = LED3_R;
						frist = 1;
					}
					else
					{
						stateLed3 = LED3_G;
						frist = 1;
					}
				}
				break;
			}
			
			default:
			{
				while(1);
			}
		}
}

void led4Handler(void)
{
	switch(stateLed4)
		{	
			static uint8_t frist = 0;					// first pass flag
			static uint32_t tickWas = 0;			// sysTick at the last transition
			static uint32_t period = 0;				// random period
			
			case LED4_R:
			{
				// if this is the first pass through this state
				if(frist)
				{
					period = GenerateRandomNumber();
					HAL_GPIO_WritePin(LED4_R_GPIO_Port, LED4_R_Pin, GPIO_PIN_SET);
					tickWas = tickNow;
					frist = 0;
				}
				// otherwise wait until the assigned random period has elapsed and decide which colour is next
				if((tickNow - tickWas) > period)
				{
					HAL_GPIO_WritePin(LED4_R_GPIO_Port, LED4_R_Pin, GPIO_PIN_RESET);
					if(period < 550)
					{
						stateLed4 = LED4_G;
						frist = 1;
					}
					else
					{
						stateLed4 = LED4_B;
						frist = 1;
					}
				}
			break;
			}
			
			case LED4_G:
			{
				// if this is the first pass through this state
				if(frist)
				{
					period = GenerateRandomNumber();
					HAL_GPIO_WritePin(LED4_G_GPIO_Port, LED4_G_Pin, GPIO_PIN_SET);
					tickWas = tickNow;
					frist = 0;
				}
				// otherwise wait until the assigned random period has elapsed and decide which colour is next
				if((tickNow - tickWas) > period)
				{
					HAL_GPIO_WritePin(LED4_G_GPIO_Port, LED4_G_Pin, GPIO_PIN_RESET);
					if(period < 550)
					{
						stateLed4 = LED4_R;
						frist = 1;
					}
					else
					{
						stateLed4 = LED4_B;
						frist = 1;
					}
				}
				break;
				
			}
			case LED4_B:
			{
				// if this is the first pass through this state
				if(frist)
				{
					period = GenerateRandomNumber();
					HAL_GPIO_WritePin(LED4_B_GPIO_Port, LED4_B_Pin, GPIO_PIN_SET);
					tickWas = tickNow;
					frist = 0;
				}
				// otherwise wait until the assigned random period has elapsed and decide which colour is next
				if((tickNow - tickWas) > period)
				{
					HAL_GPIO_WritePin(LED4_B_GPIO_Port, LED4_B_Pin, GPIO_PIN_RESET);
					if(period < 550)
					{
						stateLed4 = LED4_R;
						frist = 1;
					}
					else
					{
						stateLed4 = LED4_G;
						frist = 1;
					}
				}
				break;
			}
			
			default:
			{
				while(1);
			}
		}
}

void led5Handler(void)
{
	switch(stateLed5)
		{	
			static uint8_t frist = 0;					// first pass flag
			static uint32_t tickWas = 0;			// sysTick at the last transition
			static uint32_t period = 0;				// random period
			
			case LED5_R:
			{
				// if this is the first pass through this state
				if(frist)
				{
					period = GenerateRandomNumber();
					HAL_GPIO_WritePin(LED5_R_GPIO_Port, LED5_R_Pin, GPIO_PIN_SET);
					tickWas = tickNow;
					frist = 0;
				}
				// otherwise wait until the assigned random period has elapsed and decide which colour is next
				if((tickNow - tickWas) > period)
				{
					HAL_GPIO_WritePin(LED5_R_GPIO_Port, LED5_R_Pin, GPIO_PIN_RESET);
					if(period < 550)
					{
						stateLed5 = LED5_G;
						frist = 1;
					}
					else
					{
						stateLed5 = LED5_B;
						frist = 1;
					}
				}
			break;
			}
			
			case LED5_G:
			{
				// if this is the first pass through this state
				if(frist)
				{
					period = GenerateRandomNumber();
					HAL_GPIO_WritePin(LED5_G_GPIO_Port, LED5_G_Pin, GPIO_PIN_SET);
					tickWas = tickNow;
					frist = 0;
				}
				// otherwise wait until the assigned random period has elapsed and decide which colour is next
				if((tickNow - tickWas) > period)
				{
					HAL_GPIO_WritePin(LED5_G_GPIO_Port, LED5_G_Pin, GPIO_PIN_RESET);
					if(period < 550)
					{
						stateLed5 = LED5_R;
						frist = 1;
					}
					else
					{
						stateLed5 = LED5_B;
						frist = 1;
					}
				}
				break;
				
			}
			case LED5_B:
			{
				// if this is the first pass through this state
				if(frist)
				{
					period = GenerateRandomNumber();
					HAL_GPIO_WritePin(LED5_B_GPIO_Port, LED5_B_Pin, GPIO_PIN_SET);
					tickWas = tickNow;
					frist = 0;
				}
				// otherwise wait until the assigned random period has elapsed and decide which colour is next
				if((tickNow - tickWas) > period)
				{
					HAL_GPIO_WritePin(LED5_B_GPIO_Port, LED5_B_Pin, GPIO_PIN_RESET);
					if(period < 550)
					{
						stateLed5 = LED5_R;
						frist = 1;
					}
					else
					{
						stateLed5 = LED5_G;
						frist = 1;
					}
				}
				break;
			}
			
			default:
			{
				while(1);
			}
		}
}

void led6Handler(void)
{
	switch(stateLed6)
		{	
			static uint8_t frist = 0;					// first pass flag
			static uint32_t tickWas = 0;			// sysTick at the last transition
			static uint32_t period = 0;				// random period
			
			case LED6_R:
			{
				// if this is the first pass through this state
				if(frist)
				{
					period = GenerateRandomNumber();
					HAL_GPIO_WritePin(LED6_R_GPIO_Port, LED6_R_Pin, GPIO_PIN_SET);
					tickWas = tickNow;
					frist = 0;
				}
				// otherwise wait until the assigned random period has elapsed and decide which colour is next
				if((tickNow - tickWas) > period)
				{
					HAL_GPIO_WritePin(LED6_R_GPIO_Port, LED6_R_Pin, GPIO_PIN_RESET);
					if(period < 550)
					{
						stateLed6 = LED6_G;
						frist = 1;
					}
					else
					{
						stateLed6 = LED6_B;
						frist = 1;
					}
				}
			break;
			}
			
			case LED6_G:
			{
				// if this is the first pass through this state
				if(frist)
				{
					period = GenerateRandomNumber();
					HAL_GPIO_WritePin(LED6_G_GPIO_Port, LED6_G_Pin, GPIO_PIN_SET);
					tickWas = tickNow;
					frist = 0;
				}
				// otherwise wait until the assigned random period has elapsed and decide which colour is next
				if((tickNow - tickWas) > period)
				{
					HAL_GPIO_WritePin(LED6_G_GPIO_Port, LED6_G_Pin, GPIO_PIN_RESET);
					if(period < 550)
					{
						stateLed6 = LED6_R;
						frist = 1;
					}
					else
					{
						stateLed6 = LED6_B;
						frist = 1;
					}
				}
				break;
				
			}
			case LED6_B:
			{
				// if this is the first pass through this state
				if(frist)
				{
					period = GenerateRandomNumber();
					HAL_GPIO_WritePin(LED6_B_GPIO_Port, LED6_B_Pin, GPIO_PIN_SET);
					tickWas = tickNow;
					frist = 0;
				}
				// otherwise wait until the assigned random period has elapsed and decide which colour is next
				if((tickNow - tickWas) > period)
				{
					HAL_GPIO_WritePin(LED6_B_GPIO_Port, LED6_B_Pin, GPIO_PIN_RESET);
					if(period < 550)
					{
						stateLed6 = LED6_R;
						frist = 1;
					}
					else
					{
						stateLed6 = LED6_G;
						frist = 1;
					}
				}
				break;
			}
			
			default:
			{
				while(1);
			}
		}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
