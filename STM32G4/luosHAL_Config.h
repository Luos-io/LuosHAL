/******************************************************************************
 * @file luosHAL_Config
 * @brief This file allow you to configure LuosHAL according to your design
 *        this is the default configuration created by Luos team for this MCU Family
 *        Do not modify this file if you want to ovewrite change define in you project
 * @MCU Family STM32G4
 * @author Luos
 * @version 0.0.0
 ******************************************************************************/
#ifndef _LUOSHAL_CONFIG_H_
#define _LUOSHAL_CONFIG_H_

#include "stm32G4xx_hal.h"

#define DISABLE 0x00
/*******************************************************************************
 * PINOUT CONFIG
 ******************************************************************************/
#ifndef PORT_CLOCK_ENABLE
#define PORT_CLOCK_ENABLE()         do { \
                                    __HAL_RCC_GPIOA_CLK_ENABLE(); \
                                    __HAL_RCC_GPIOB_CLK_ENABLE();\
                                      } while(0U)
#endif

#ifndef PTPA_PIN
#define PTPA_PIN                    GPIO_PIN_8
#else
#if (PTPA_PIN == DISABLE)
#warning PTPA wrong definition
#endif
#endif
#ifndef PTPA_PORT
#define PTPA_PORT                   GPIOA
#else
#if (PTPA_PORT == DISABLE)
#warning PTPA wrong definition
#endif
#endif
#ifndef PTPA_IRQ
#define PTPA_IRQ                    EXTI9_5_IRQn
#else
#if (PTPA_IRQ == DISABLE)
#warning PTPA wrong definition
#endif
#endif

#ifndef PTPB_PIN
#define PTPB_PIN                    GPIO_PIN_13
#else
#if (PTPB_PIN == DISABLE)
#warning PTPB_PIN wrong definition
#endif
#endif
#ifndef PTPB_PORT
#define PTPB_PORT                   GPIOB
#else
#if (PTPB_PORT == DISABLE)
#warning PTPB_PORT wrong definition
#endif
#endif
#ifndef PTPB_IRQ
#define PTPB_IRQ                    EXTI15_10_IRQn
#else
#if (PTPB_IRQ == DISABLE)
#warning PTPB wrong definition
#endif
#endif

#ifndef TX_LOCK_DETECT_PIN
#define TX_LOCK_DETECT_PIN          GPIO_PIN_11
#endif
#ifndef TX_LOCK_DETECT_PORT
#define TX_LOCK_DETECT_PORT         GPIOA
#endif
#ifndef TX_LOCK_DETECT_IRQ
#define TX_LOCK_DETECT_IRQ          EXTI15_10_IRQn
#endif

#ifndef RX_EN_PIN
#define RX_EN_PIN                   GPIO_PIN_14
#else
#if (RX_EN_PIN == DISABLE)
#warning RX_EN_PIN wrong definition
#endif
#endif
#ifndef RX_EN_PORT
#define RX_EN_PORT                  GPIOB
#else
#if (RX_EN_PORT == DISABLE)
#warning RX_EN_PORT wrong definition
#endif
#endif

#ifndef TX_EN_PIN
#define TX_EN_PIN                   GPIO_PIN_15
#else
#if (TX_EN_PIN == DISABLE)
#warning TX_EN_PIN wrong definition
#endif
#endif
#ifndef TX_EN_PORT
#define TX_EN_PORT                  GPIOB
#else
#if (TX_EN_PORT == DISABLE)
#warning TX_EN_PORT wrong definition
#endif
#endif

#ifndef COM_LVL_DOWN_PIN
#define COM_LVL_DOWN_PIN            GPIO_PIN_5
#endif
#ifndef COM_LVL_DOWN_PORT
#define COM_LVL_DOWN_PORT           GPIOA
#endif

#ifndef COM_LVL_UP_PIN
#define COM_LVL_UP_PIN              GPIO_PIN_6
#endif
#ifndef COM_LVL_UP_PORT
#define COM_LVL_UP_PORT             GPIOA
#endif

#ifndef COM_TX_PIN
#define COM_TX_PIN                  GPIO_PIN_9
#else
#if (COM_TX_PIN == DISABLE)
#warning COM_TX_PIN wrong definition
#endif
#endif
#ifndef COM_TX_PORT
#define COM_TX_PORT                 GPIOA
#else
#if (COM_TX_PORT == DISABLE)
#warning COM_TX_PORT wrong definition
#endif
#endif
#ifndef COM_TX_AF
#define COM_TX_AF                   GPIO_AF7_USART1
#else
#if (COM_TX_AF == DISABLE)
#warning COM_TX_AF wrong definition
#endif
#endif

#ifndef COM_RX_PIN
#define COM_RX_PIN                  GPIO_PIN_10
#else
#if (COM_RX_PIN == DISABLE)
#warning COM_RX_PIN wrong definition
#endif
#endif
#ifndef COM_RX_PORT
#define COM_RX_PORT                 GPIOA
#else
#if (COM_RX_PORT == DISABLE)
#warning COM_RX_PORT wrong definition
#endif
#endif
#ifndef COM_RX_AF
#define COM_RX_AF                   GPIO_AF7_USART1
#else
#if (COM_RX_AF == DISABLE)
#warning COM_RX_AF wrong definition
#endif
#endif

#ifndef PINOUT_IRQHANDLER
#define PINOUT_IRQHANDLER(PIN)      HAL_GPIO_EXTI_Callback(PIN)
#endif

/*******************************************************************************
 * COM CONFIG
 ******************************************************************************/
#ifndef LUOS_COM_CLOCK_ENABLE
#define LUOS_COM_CLOCK_ENABLE()     __HAL_RCC_USART1_CLK_ENABLE()
#endif
#ifndef LUOS_COM
#define LUOS_COM                    USART1
#endif
#ifndef LUOS_COM_IRQ
#define LUOS_COM_IRQ                USART1_IRQn
#endif
#ifndef LUOS_COM_IRQHANDLER
#define LUOS_COM_IRQHANDLER()       USART1_IRQHandler()
#endif

/*******************************************************************************
 * FLASH CONFIG
 ******************************************************************************/
#ifndef PAGE_SIZE
#define PAGE_SIZE                   (uint32_t) FLASH_PAGE_SIZE
#endif
#ifndef ADDRESS_LAST_PAGE_FLASH
#define ADDRESS_LAST_PAGE_FLASH ((uint32_t)0x0801F800)
#endif

#endif /* _LUOSHAL_CONFIG_H_ */
