/**
  ******************************************************************************
  * @file    FreeRTOS/FreeRTOS_ThreadCreation/Src/main.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    25-May-2015
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stm32f1xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "board_config.h"
#if BOARD_SPI_ENABLE
#include "Drivers/SPI.h"
#endif
#if BOARD_CAN_ENABLE
#include "Drivers/can.h"
#endif
#if BOARD_UART_ENABLE
#include "Drivers/uart.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void LED_Thread(void const *argument);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

#if BOARD_SPI_ENABLE
  SPI_Init();
#endif
#if BOARD_CAN_ENABLE
  CAN_Init();
#endif
#if BOARD_UART_ENABLE
  UART_Init();
#endif

	__GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // LED低电平点亮，初始化为熄灭
  GPIO_InitStructure.Pin = GPIO_PIN_9;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  osThreadDef(LED, LED_Thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  if (osThreadCreate(osThread(LED), NULL) == NULL)
  {
    Error_Handler();
  }

	osKernelStart();

	for (;;)
  {
  }
}

static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStructure = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStructure = {0};

  RCC_OscInitStructure.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStructure.HSEState = RCC_HSE_ON;           // 外部8MHz晶振
  RCC_OscInitStructure.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStructure.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStructure.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStructure.PLL.PLLMUL = RCC_PLL_MUL9;       // 8MHz × 9 = 72MHz
  if (HAL_RCC_OscConfig(&RCC_OscInitStructure) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStructure.ClockType = RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStructure.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStructure.AHBCLKDivider = RCC_SYSCLK_DIV1;  // HCLK 72MHz
  RCC_ClkInitStructure.APB1CLKDivider = RCC_HCLK_DIV2;   // PCLK1 36MHz
  RCC_ClkInitStructure.APB2CLKDivider = RCC_HCLK_DIV1;   // PCLK2 72MHz
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStructure, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void SysTick_Handler(void)
{
	HAL_IncTick();
	osSystickHandler();
}

static void LED_Thread(void const *argument)
{
	(void) argument;

	for (;;)
	{
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
    osDelay(500);
	}
}

static void Error_Handler(void)
{
  __disable_irq();
  while (1)
	{
	}
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
	while (1)
	{
	}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
