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

#include "atsamd21j18a.h"

#define DISABLE 0x00
/*******************************************************************************
 * PINOUT CONFIG
 ******************************************************************************/

#ifndef PORT_CLOCK_ENABLE
#define PORT_CLOCK_ENABLE()         do { \
                                    GCLK_REGS->GCLK_CLKCTRL = \
                                    GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EIC_Val)\
                                    | GCLK_CLKCTRL_GEN(0x0)\
                                    | GCLK_CLKCTRL_CLKEN_Msk;\
                                    PM_REGS->PM_APBAMASK |= PM_APBAMASK_EIC_Msk;\
                                      } while(0U)
#endif

//PTP pin definition
#ifndef PTPA_PIN
#define PTPA_PIN                    8
#endif
#ifndef PTPA_PORT
#define PTPA_PORT                   1
#endif
#ifndef PTPA_MUX
#define PTPA_MUX                    MUX_PB08A_EIC_EXTINT8
#endif
#ifndef PTPA_EDGE
#define PTPA_EDGE                   EIC_CONFIG_SENSE0_Pos
#endif

#ifndef PTPB_PIN
#define PTPB_PIN                    9
#endif
#ifndef PTPB_PORT
#define PTPB_PORT                   1
#endif
#ifndef PTPB_MUX
#define PTPB_MUX                    MUX_PB09A_EIC_EXTINT9
#endif
#ifndef PTPB_EDGE
#define PTPB_EDGE                   EIC_CONFIG_SENSE1_Pos
#endif

//COM pin definition
#ifndef TX_LOCK_DETECT_PIN
#define TX_LOCK_DETECT_PIN          DISABLE
#endif
#ifndef TX_LOCK_DETECT_PORT
#define TX_LOCK_DETECT_PORT         DISABLE
#endif
#ifndef TX_LOCK_DETECT_IRQ
#define TX_LOCK_DETECT_IRQ         DISABLE
#endif

#ifndef RX_EN_PIN
#define RX_EN_PIN                   7
#endif
#ifndef RX_EN_PORT
#define RX_EN_PORT                  0
#endif

#ifndef TX_EN_PIN
#define TX_EN_PIN                   6
#endif
#ifndef TX_EN_PORT
#define TX_EN_PORT                  0
#endif

#ifndef COM_LVL_DOWN_PIN
#define COM_LVL_DOWN_PIN            8
#endif
#ifndef COM_LVL_DOWN_PORT
#define COM_LVL_DOWN_PORT           0
#endif

#ifndef COM_LVL_UP_PIN
#define COM_LVL_UP_PIN              9
#endif
#ifndef COM_LVL_UP_PORT
#define COM_LVL_UP_PORT             0
#endif

#ifndef COM_TX_PIN
#define COM_TX_PIN                  4
#endif
#ifndef COM_TX_PORT
#define COM_TX_PORT                 0
#endif
#ifndef COM_TX_AF
#define COM_TX_AF                   MUX_PA04D_SERCOM0_PAD0
#endif

#ifndef COM_RX_PIN
#define COM_RX_PIN                  5 //this pin should pin pull up to vcc if no internal pull up
#endif
#ifndef COM_RX_PORT
#define COM_RX_PORT                 0
#endif
#ifndef COM_RX_AF
#define COM_RX_AF                   MUX_PA05D_SERCOM0_PAD1
#endif

#ifndef PINOUT_IRQHANDLER
#define PINOUT_IRQHANDLER()        EIC_Handler()
#endif

/*******************************************************************************
 * COM CONFIG
 ******************************************************************************/
#ifndef LUOS_COM_CLOCK_ENABLE
#define LUOS_COM_CLOCK_ENABLE()         do { \
                                    GCLK_REGS->GCLK_CLKCTRL = \
                                    GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_SERCOM0_CORE_Val)\
                                    | GCLK_CLKCTRL_GEN(0x0)\
                                    | GCLK_CLKCTRL_CLKEN_Msk;\
                                    PM_REGS->PM_APBCMASK |= PM_APBCMASK_SERCOM0_Msk;\
                                      } while(0U)
#endif
#ifndef LUOS_COM
#define LUOS_COM                    SERCOM0_REGS
#endif
#ifndef LUOS_COM_IRQ
#define LUOS_COM_IRQ                SERCOM0_IRQn
#endif
#ifndef LUOS_COM_IRQHANDLER
#define LUOS_COM_IRQHANDLER()       SERCOM0_Handler() 
#endif
/*******************************************************************************
 * COM TIMEOUT CONFIG
 ******************************************************************************/
#ifndef LUOS_TIMER_LOCK_ENABLE
#define LUOS_TIMER_LOCK_ENABLE()         do { \
                                    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_TCC2_TC3_Val)\
                                    | GCLK_CLKCTRL_GEN(0x0)\
                                    | GCLK_CLKCTRL_CLKEN_Msk;\
                                    PM_REGS->PM_APBCMASK |= PM_APBCMASK_TC3_Msk;\
                                      } while(0U)
#endif
#ifndef LUOS_TIMER
#define LUOS_TIMER                    TC3_REGS
#endif
#ifndef LUOS_TIMER_IRQ
#define LUOS_TIMER_IRQ                TC3_IRQn
#endif
#ifndef LUOS_TIMER_IRQHANDLER
#define LUOS_TIMER_IRQHANDLER()       TC3_Handler()
#endif
/*******************************************************************************
 * FLASH CONFIG
 ******************************************************************************/
#ifndef PAGE_SIZE
#define PAGE_SIZE                   64
#endif
#ifndef ERASE_SIZE
#define ERASE_SIZE                  256
#endif
#ifndef FLASH_SIZE
#define FLASH_SIZE                  0x40000
#endif
#ifndef ADDRESS_LAST_PAGE_FLASH
#define ADDRESS_LAST_PAGE_FLASH     FLASH_SIZE - (16 * PAGE_SIZE)
#endif

#endif /* _LUOSHAL_CONFIG_H_ */
