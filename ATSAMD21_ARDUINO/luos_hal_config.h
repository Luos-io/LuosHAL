/******************************************************************************
 * @file luosHAL_Config
 * @brief This file allow you to configure LuosHAL according to your design
 *        this is the default configuration created by Luos team for this MCU Family
 *        Do not modify this file if you want to ovewrite change define in you project
 * @MCU Family ATSAMD21
 * @author Luos
 * @version 0.0.0
 ******************************************************************************/
#ifndef _LUOSHAL_CONFIG_H_
#define _LUOSHAL_CONFIG_H_

#include <Arduino.h>

#define DISABLE 0x00

#define MCUFREQ 48000000
/*******************************************************************************
 * PINOUT CONFIG
 ******************************************************************************/
//GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EIC_Val) | GCLK_CLKCTRL_GEN(0x0) | GCLK_CLKCTRL_CLKEN;      
#ifndef PORT_CLOCK_ENABLE
#define PORT_CLOCK_ENABLE()                                                                               \
  do                                                                                                      \
  {                                                                                                       \
    GCLK->CLKCTRL.reg =                                                                                   \
    (uint16_t)(GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EIC_Val) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN);   \
    PM->APBAMASK.reg |= PM_APBAMASK_EIC;                                                                  \
  } while (0U)
#endif

//PTP pin definition
#ifndef PTPA_PIN
#define PTPA_PIN 15
#endif
#ifndef PTPA_PORT
#define PTPA_PORT 0
#endif
#ifndef PTPA_IRQ
#define PTPA_IRQ  15 //see EXTINT
#endif

#ifndef PTPB_PIN
#define PTPB_PIN 20
#endif
#ifndef PTPB_PORT
#define PTPB_PORT 0
#endif
#ifndef PTPB_IRQ
#define PTPB_IRQ  4 //see EXTINT
#endif

//COM pin definition
#ifndef TX_LOCK_DETECT_PIN
#define TX_LOCK_DETECT_PIN DISABLE
#endif
#ifndef TX_LOCK_DETECT_PORT
#define TX_LOCK_DETECT_PORT DISABLE
#endif
#ifndef TX_LOCK_DETECT_IRQ
#define TX_LOCK_DETECT_IRQ DISABLE
#endif

#ifndef RX_EN_PIN
#define RX_EN_PIN 14
#endif
#ifndef RX_EN_PORT
#define RX_EN_PORT 0
#endif

#ifndef TX_EN_PIN
#define TX_EN_PIN 9
#endif
#ifndef TX_EN_PORT
#define TX_EN_PORT 0
#endif

#ifndef COM_LVL_DOWN_PIN
#define COM_LVL_DOWN_PIN DISABLE
#endif
#ifndef COM_LVL_DOWN_PORT
#define COM_LVL_DOWN_PORT DISABLE
#endif

#ifndef COM_LVL_UP_PIN
#define COM_LVL_UP_PIN DISABLE
#endif
#ifndef COM_LVL_UP_PORT
#define COM_LVL_UP_PORT DISABLE
#endif

#ifndef COM_TX_PIN
#define COM_TX_PIN 10
#endif
#ifndef COM_TX_PORT
#define COM_TX_PORT 0
#endif
#ifndef COM_TX_AF
#define COM_TX_AF MUX_PA10C_SERCOM0_PAD2
#endif
#ifndef COM_TX_POS
#define COM_TX_POS 1 //PAD2
#endif

#ifndef COM_RX_PIN
#define COM_RX_PIN 11 //this pin should pin pull up to vcc if no internal pull up
#endif
#ifndef COM_RX_PORT
#define COM_RX_PORT 0
#endif
#ifndef COM_RX_AF
#define COM_RX_AF MUX_PA11C_SERCOM0_PAD3
#endif
#ifndef COM_RX_POS
#define COM_RX_POS 3 //PAD3
#endif

#ifndef PINOUT_IRQHANDLER
#define PINOUT_IRQHANDLER() EIC_Handler()
#endif

//GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_SERCOM0_CORE_Val) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;     
/*******************************************************************************
 * COM CONFIG
 ******************************************************************************/
#ifndef LUOS_COM_CLOCK_ENABLE
#define LUOS_COM_CLOCK_ENABLE()                                                                                   \
  do                                                                                                              \
  {                                                                                                               \
    GCLK->CLKCTRL.reg =                                                                                           \
    (uint16_t)(GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_SERCOM0_CORE_Val) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN);  \
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;                                                                      \
  } while (0U)
#endif
#ifndef LUOS_COM
#define LUOS_COM SERCOM0
#endif
#ifndef LUOS_COM_IRQ
#define LUOS_COM_IRQ SERCOM0_IRQn
#endif
#ifndef LUOS_COM_IRQHANDLER
#define LUOS_COM_IRQHANDLER() SERCOM0_Handler()
#endif
/*******************************************************************************
 * COM TIMEOUT CONFIG
 ******************************************************************************/
#ifndef LUOS_TIMER_LOCK_ENABLE
#define LUOS_TIMER_LOCK_ENABLE()                                                                              \
  do                                                                                                          \
  {                                                                                                           \
    GCLK->CLKCTRL.reg =                                                                                       \
    (uint16_t)(GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_TCC2_TC3_Val) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN);  \
    PM->APBCMASK.reg |= PM_APBCMASK_TC3;                                                                      \
  } while (0U)
#endif
#ifndef LUOS_TIMER
#define LUOS_TIMER TC3
#endif
#ifndef LUOS_TIMER_IRQ
#define LUOS_TIMER_IRQ TC3_IRQn
#endif
#ifndef LUOS_TIMER_IRQHANDLER
#define LUOS_TIMER_IRQHANDLER() TC3_Handler()
#endif
/*******************************************************************************
 * FLASH CONFIG
 ******************************************************************************/
#ifndef PAGE_SIZE
#define PAGE_SIZE 64
#endif
#ifndef ERASE_SIZE
#define ERASE_SIZE 256
#endif
#ifndef FLASH_SIZE
#define FLASH_SIZE 0x40000
#endif
#ifndef ADDRESS_LAST_PAGE_FLASH
#define ADDRESS_LAST_PAGE_FLASH FLASH_SIZE - (16 * PAGE_SIZE)
#endif

#endif /* _LUOSHAL_CONFIG_H_ */
