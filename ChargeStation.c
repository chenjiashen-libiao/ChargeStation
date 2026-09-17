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
#include "UserSource/Drivers/can.h"
#include "UserSource/Drivers/adc.h"
#include "UserSource/Drivers/si4463.h"
#include "UserSource/CAN_CMD.h"
#include "UserSource/charge_control.h"
#include "UserSource/registration.h"
#include "UserSource/wireless_protocol.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const uint16_t g_ver __attribute__((section(".appinfo"))) = BOARD_VERSION;
const uint16_t g_dev_type __attribute__((section(".appinfo"))) = BOARD_DEVICE_TYPE;
static volatile uint8_t wirelessInitFailed;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void LED_Thread(void const *argument);
static void CAN_Rx_Thread(void const *argument);
static void SI4463_Thread(void const *argument);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{ // 初始化系统、外设和任务后启动RTOS
  __enable_irq();
  HAL_Init();
  if (HAL_RCC_DeInit() != HAL_OK)
  {
    Error_Handler();
  }
  SystemClock_Config();

  // 注册信息决定设备合法状态；充电控制先建立安全输出，再启动ADC采样
  Registration_Load();
  CAN_Init();
  ChargeControl_Init();
  ADC_Init();

	__GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // LED低电平点亮，初始化为熄灭
  GPIO_InitStructure.Pin = GPIO_PIN_9;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  // LED显示状态，CAN处理配置，无线处理收发，Charge执行功率控制
  osThreadDef(LED, LED_Thread, osPriorityNormal, 0, 48);
  osThreadDef(CAN_Rx, CAN_Rx_Thread, osPriorityNormal, 0, 768);
  osThreadDef(SI4463, SI4463_Thread, osPriorityRealtime, 0, 416);
  osThreadDef(Charge, ChargeControl_Task, osPriorityNormal, 0, 128);

  if ((osThreadCreate(osThread(LED), NULL) == NULL) ||
    (osThreadCreate(osThread(CAN_Rx), NULL) == NULL) ||
    (osThreadCreate(osThread(SI4463), NULL) == NULL) ||
    (osThreadCreate(osThread(Charge), NULL) == NULL))
  {
    Error_Handler();
  }

	osKernelStart();

	for (;;)
  {
  }
}

static void SystemClock_Config(void)
{ // 使用外部8MHz晶振配置32MHz系统时钟
  RCC_OscInitTypeDef RCC_OscInitStructure = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStructure = {0};

  RCC_OscInitStructure.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStructure.HSEState = RCC_HSE_ON;           // 外部8MHz晶振
  RCC_OscInitStructure.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStructure.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStructure.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStructure.PLL.PLLMUL = RCC_PLL_MUL4;       // 8MHz × 4 = 32MHz
  if (HAL_RCC_OscConfig(&RCC_OscInitStructure) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStructure.ClockType = RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStructure.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStructure.AHBCLKDivider = RCC_SYSCLK_DIV1;  // HCLK 32MHz
  RCC_ClkInitStructure.APB1CLKDivider = RCC_HCLK_DIV2;   // PCLK1 16MHz
  RCC_ClkInitStructure.APB2CLKDivider = RCC_HCLK_DIV1;   // PCLK2 32MHz
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStructure, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

void SysTick_Handler(void)
{ // 维护HAL时基并驱动CMSIS-RTOS系统节拍
	HAL_IncTick();
	osSystickHandler();
}

static void LED_Thread(void const *argument)
{ // 根据无线初始化和工作信道状态闪烁状态灯
	(void) argument;

	for (;;)
	{
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
  osDelay(wirelessInitFailed ? 100 :
    (WirelessProtocol_IsWorkChannel() ? 300 : 500));
	}
}

static void CAN_Rx_Thread(void const *argument)
{ // 周期处理注册、升级和无线配置命令
  (void) argument;
  osDelay(20);
  CAN_CMD_SendSN();

  for (;;)
  {
    CAN_CMD_Process();
    osDelay(1);
  }
}

static void SI4463_Thread(void const *argument)
{ // 初始化无线模块并持续处理固定长收发事件
  (void) argument;

  // 无线初始化失败不阻止其他任务运行，由LED快速闪烁提示故障
  if (!SI4463_Init())
  {
    wirelessInitFailed = 1;
  }
  WirelessProtocol_Init();
  for (;;)
  {
    SI4463_Process();
    WirelessProtocol_Process();
    osDelay(1);
  }
}

static void Error_Handler(void)
{ // 关闭中断并停机，保留故障现场
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
{ // 完整断言模式下停机，保留错误文件和行号参数
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
	while (1)
	{
	}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
