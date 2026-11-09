#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stm32f1xx_hal.h>

/* 功能开关：1=启用，0=禁用 */
#define BOARD_SPI_ENABLE                 0
#define BOARD_UART_ENABLE                0

/* 程序信息：固定写入Flash的0x08004000起始位置 */
#define BOARD_VERSION                     0x0003
#define BOARD_DEVICE_TYPE                 74

/* 注册信息：STM32F103C8末段独占1KB页 */
#define BOARD_UID_ADDRESS                 0x1FFFF7E8
#define BOARD_REGISTRATION_ADDRESS        0x0800E000

/* SI4463模拟SPI */
#define BOARD_SI4463_GPIO_PORT             GPIOB
#define BOARD_SI4463_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()
#define BOARD_SI4463_CSN_PIN               GPIO_PIN_3
#define BOARD_SI4463_MISO_PIN              GPIO_PIN_4
#define BOARD_SI4463_MOSI_IRQ_PIN          GPIO_PIN_5
#define BOARD_SI4463_SCK_PIN               GPIO_PIN_6
#define BOARD_SI4463_IRQN                  EXTI9_5_IRQn

/* 无线地址和射频参数存储 */
#define BOARD_WIRELESS_ADDRESS_STORAGE     0x08011C00
#define BOARD_WIRELESS_ADDRESS_DEFAULT     0x0000EEEE
#define BOARD_RF_HOME_CHANNEL_STORAGE      0x08011C04
#define BOARD_RF_FREQUENCY_STORAGE         0x08011C08
#define BOARD_RF_FREQUENCY_STEP_STORAGE    0x08011C0C
#define BOARD_RF_TX_POWER_STORAGE          0x08011C10

/* 充电采样：三相电流PA3~PA5，输出电压PA0~PA2，输入电压PA6/PA7/PB0 */
#define BOARD_ADC_INSTANCE                 ADC1
#define BOARD_ADC_DMA_INSTANCE             DMA1_Channel1
#define BOARD_ADC_DMA_IRQn                 DMA1_Channel1_IRQn
#define BOARD_ADC_GPIOA_PINS               (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | \
											GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | \
											GPIO_PIN_6 | GPIO_PIN_7)
#define BOARD_ADC_GPIOB_PINS               GPIO_PIN_0

/* SPI配置：默认SPI2，PB12片选，PB13/PB14/PB15通信 */
#define BOARD_SPI_INSTANCE               SPI2
#define BOARD_SPI_CLK_ENABLE()           __HAL_RCC_SPI2_CLK_ENABLE()
#define BOARD_SPI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define BOARD_SPI_CS_PORT                GPIOB
#define BOARD_SPI_CS_PIN                 GPIO_PIN_12
#define BOARD_SPI_SCK_PORT               GPIOB
#define BOARD_SPI_SCK_PIN                GPIO_PIN_13
#define BOARD_SPI_MISO_PORT              GPIOB
#define BOARD_SPI_MISO_PIN               GPIO_PIN_14
#define BOARD_SPI_MOSI_PORT              GPIOB
#define BOARD_SPI_MOSI_PIN               GPIO_PIN_15
#define BOARD_SPI_BAUD_PRESCALER         SPI_BAUDRATEPRESCALER_16
#define BOARD_SPI_CPOL                   SPI_POLARITY_LOW
#define BOARD_SPI_CPHA                   SPI_PHASE_1EDGE

/* CAN配置：STM32F103C8仅有CAN1，默认PA11接收、PA12发送 */
#define BOARD_CAN_INSTANCE               CAN1
#define BOARD_CAN_CLK_ENABLE()           __HAL_RCC_CAN1_CLK_ENABLE()
#define BOARD_CAN_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define BOARD_CAN_REMAP()                ((void)0)
#define BOARD_CAN_RX_PORT                GPIOA
#define BOARD_CAN_RX_PIN                 GPIO_PIN_11
#define BOARD_CAN_TX_PORT                GPIOA
#define BOARD_CAN_TX_PIN                 GPIO_PIN_12
#define BOARD_CAN_PRESCALER              1
#define BOARD_CAN_RX_IRQn                USB_LP_CAN1_RX0_IRQn
#define BOARD_CAN_SCE_IRQn               CAN1_SCE_IRQn

/* UART配置：默认USART1，PA9发送、PA10接收，115200-8-N-1 */
#define BOARD_UART_INSTANCE              USART1
#define BOARD_UART_CLK_ENABLE()          __HAL_RCC_USART1_CLK_ENABLE()
#define BOARD_UART_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define BOARD_UART_REMAP()               ((void)0)
#define BOARD_UART_TX_PORT               GPIOA
#define BOARD_UART_TX_PIN                GPIO_PIN_9
#define BOARD_UART_RX_PORT               GPIOA
#define BOARD_UART_RX_PIN                GPIO_PIN_10
#define BOARD_UART_BAUD_RATE             115200

#endif
