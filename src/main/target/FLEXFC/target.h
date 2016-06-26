/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "FLEXFC"

#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_14
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOB
#define LED0_INVERTED

#define LED1_GPIO   GPIOB
#define LED1_PIN    Pin_13
#define LED1_PERIPHERAL RCC_AHBPeriph_GPIOB
#define LED1_INVERTED

#define USABLE_TIMER_CHANNEL_COUNT 7

#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN CW0_DEG_FLIP

#define ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN CW0_DEG_FLIP

#define LED0
#define LED1

#define USE_USART1
#define USE_USART2
#define USE_USART3
#define SERIAL_PORT_COUNT 3

#ifndef UART1_GPIO
#define UART1_TX_PIN        GPIO_Pin_9  // PA9
#define UART1_RX_PIN        GPIO_Pin_10 // PA10
#define UART1_GPIO          GPIOA
#define UART1_GPIO_AF       GPIO_AF_7
#define UART1_TX_PINSOURCE  GPIO_PinSource9
#define UART1_RX_PINSOURCE  GPIO_PinSource10
#endif

#define UART2_TX_PIN        GPIO_Pin_14 // PA14 / SWCLK
#define UART2_RX_PIN        GPIO_Pin_15 // PA15
#define UART2_GPIO          GPIOA
#define UART2_GPIO_AF       GPIO_AF_7
#define UART2_TX_PINSOURCE  GPIO_PinSource14
#define UART2_RX_PINSOURCE  GPIO_PinSource15

#ifndef UART3_GPIO
#define UART3_TX_PIN        GPIO_Pin_10 // PB10 (AF7)
#define UART3_RX_PIN        GPIO_Pin_11 // PB11 (AF7)
#define UART3_GPIO_AF       GPIO_AF_7
#define UART3_GPIO          GPIOB
#define UART3_TX_PINSOURCE  GPIO_PinSource10
#define UART3_RX_PINSOURCE  GPIO_PinSource11
#endif

#define USE_I2C
#define I2C_DEVICE (I2CDEV_1) // PB6/SCL, PB7/SDA

#define USE_SPI
#define USE_SPI_DEVICE_1 // PA4,5,6,7 on AF5
/*
#define SPI1_GPIO               GPIOA
#define SPI1_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOA
//#define SPI1_NSS_PIN            Pin_4
//#define SPI1_NSS_PIN_SOURCE     GPIO_PinSource4
#define SPI1_SCK_PIN            Pin_5
#define SPI1_SCK_PIN_SOURCE     GPIO_PinSource5
#define SPI1_MISO_PIN           Pin_6
#define SPI1_MISO_PIN_SOURCE    GPIO_PinSource6
#define SPI1_MOSI_PIN           Pin_7
#define SPI1_MOSI_PIN_SOURCE    GPIO_PinSource7
*/
#define MPU6000_CS_GPIO_CLK_PERIPHERAL   RCC_AHBPeriph_GPIOA
#define MPU6000_CS_GPIO       GPIOA
#define MPU6000_CS_PIN        GPIO_Pin_4
#define MPU6000_SPI_INSTANCE  SPI1

#define USE_ADC
#define BOARD_HAS_VOLTAGE_DIVIDER

// TODO - CPU crashes in cleanflight
//#define ADC_INSTANCE                ADC4
//#define ADC_DMA_CHANNEL             DMA2_Channel2
//#define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA2

#define VBAT_ADC_GPIO               GPIOB
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_12
#define VBAT_ADC_CHANNEL            ADC_Channel_3

/*
#define LED_STRIP
#define LED_STRIP_TIMER TIM1

#define USE_LED_STRIP_ON_DMA1_CHANNEL2
#define WS2811_GPIO                     GPIOA
#define WS2811_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOA
#define WS2811_GPIO_AF                  GPIO_AF_6
#define WS2811_PIN                      GPIO_Pin_8
#define WS2811_PIN_SOURCE               GPIO_PinSource8
#define WS2811_TIMER                    TIM1
#define WS2811_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM1
#define WS2811_DMA_CHANNEL              DMA1_Channel2
#define WS2811_IRQ                      DMA1_Channel2_IRQn
#define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC2
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH2_HANDLER
*/

#define BLACKBOX
//#define GPS
//#define GTUNE
#define SERIAL_RX
//#define TELEMETRY
#define USE_SERVOS
#define USE_CLI

#define DEFAULT_RX_FEATURE FEATURE_RX_PPM
#define DEFAULT_FEATURES FEATURE_BLACKBOX

#define SPEKTRUM_BIND
// USART3,
#define BIND_PORT  GPIOB
#define BIND_PIN   Pin_11

#define BRUSHED_MOTORS
#define BRUSHED_MOTORS_PWM_RATE 16000  // default value

#define RX_POWER
#define USE_FPF132X

#define FPF132X_GPIO_PERIPHERAL RCC_AHBPeriph_GPIOA
#define FPF132X_GPIO            GPIOA
#define FPF132X_EN_PIN          GPIO_Pin_11
#define FPF132X_SEL_PIN         GPIO_Pin_12 // High = 5v, Low = 3.3v

#define USE_BC417

#define RX_PPM_BIND_PROCEDURE

#define HARDWARE_BIND_PLUG
#define BINDPLUG_GPIO_PERIPHERAL RCC_AHBPeriph_GPIOB
#define BINDPLUG_PORT  GPIOB
#define BINDPLUG_PIN   Pin_11

