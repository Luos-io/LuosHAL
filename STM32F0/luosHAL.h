/******************************************************************************
 * @file luosHAL
 * @brief Luos Hardware Abstration Layer. Describe Low layer fonction
 * @MCU Family STM32FO
 * @author Luos
 * @version 0.0.0
 ******************************************************************************/
#ifndef _LUOSHAL_H_
#define _LUOSHAL_H_

#include <stdint.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LUOS_UUID ((uint32_t *)0x1FFFF7AC)
#define MCUFREQ 48000000

// list of all branches of your configuration.
typedef enum
{
    BRANCH_A,
    BRANCH_B,
    NO_BRANCH // you have to keep this one at the last position
} branch_t;

#define PAGE_SIZE (uint32_t) FLASH_PAGE_SIZE
#define ADDRESS_LAST_PAGE_FLASH ((uint32_t)0x0801F800)
#define ADDRESS_ALIASES_FLASH ADDRESS_LAST_PAGE_FLASH
#define ADDRESS_BOOT_FLAG_FLASH (ADDRESS_LAST_PAGE_FLASH + PAGE_SIZE) - 4

// Pinout Port use
#ifndef USE_PORTA
#define USE_PORTA 1
#endif
#ifndef USE_PORTB
#define USE_PORTB 1
#endif
#ifndef USE_PORTC
#define USE_PORTC 0
#endif
#ifndef USE_PORTD
#define USE_PORTD 0
#endif

// Pinout Com
#ifndef COM_LVL_DOWN_PIN
#define COM_LVL_DOWN_PIN GPIO_PIN_5
#endif
#ifndef COM_LVL_DOWN_PORT
#define COM_LVL_DOWN_PORT GPIOA
#endif

#ifndef COM_LVL_UP_PIN
#define COM_LVL_UP_PIN GPIO_PIN_6
#endif
#ifndef COM_LVL_UP_PORT
#define COM_LVL_UP_PORT GPIOA
#endif

#ifndef PTPA_PIN
#define PTPA_PIN GPIO_PIN_8
#endif
#ifndef PTPA_PORT
#define PTPA_PORT GPIOA
#endif

#ifndef PTPB_PIN
#define PTPB_PIN GPIO_PIN_13
#endif
#ifndef PTPB_PORT
#define PTPB_PORT GPIOB
#endif

#ifndef RX_EN_PIN
#define RX_EN_PIN GPIO_PIN_14
#endif
#ifndef RX_EN_PORT
#define RX_EN_PORT GPIOB
#endif

#ifndef TX_EN_PIN
#define TX_EN_PIN GPIO_PIN_15
#endif
#ifndef TX_EN_PORT
#define TX_EN_PORT GPIOB
#endif

#ifndef COM_TX_PIN
#define COM_TX_PIN GPIO_PIN_9
#endif
#ifndef COM_TX_PORT
#define COM_TX_PORT GPIOA
#endif

#ifndef COM_RX_PIN
#define COM_RX_PIN GPIO_PIN_10
#endif
#ifndef COM_RX_PORT
#define COM_RX_PORT GPIOA
#endif

#ifndef TX_LOCK_DETECT_PIN
#define TX_LOCK_DETECT_PIN GPIO_PIN_11
#endif
#ifndef TX_LOCK_DETECT_PORT
#define TX_LOCK_DETECT_PORT GPIOA
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Function
 ******************************************************************************/
void LuosHAL_Init(void);
void LuosHAL_SetIrqState(uint8_t Enable);
uint32_t LuosHAL_GetSystick(void);
void LuosHAL_ComInit(uint32_t Baudrate);
void LuosHAL_SetTxState(uint8_t Enable);
void LuosHAL_SetRxState(uint8_t Enable);
void LuosHAL_ComRxTimeout(void);
void LuosHAL_ComTxTimeout(void);
void LuosHAL_ComReceive(void);
uint8_t LuosHAL_ComTransmit(uint8_t *data, uint16_t size);
void LuosHAL_SetTxLockDetecState(uint8_t Enable);
uint8_t LuosHAL_GetTxLockState(void);
void LuosHAL_GPIOProcess(uint16_t GPIO);
void LuosHAL_SetPTPDefaultState(branch_t branch);
void LuosHAL_SetPTPReverseState(branch_t branch);
void LuosHAL_PushPTP(branch_t branch);
uint8_t LuosHAL_GetPTPState(branch_t branch);
void LuosHAL_ComputeCRC(uint8_t *data, uint16_t size, uint8_t *crc);
void LuosHAL_FlashWriteLuosMemoryInfo(uint32_t addr, uint16_t size, uint8_t *data);
void LuosHAL_FlashReadLuosMemoryInfo(uint32_t addr, uint16_t size, uint8_t *data);

#endif /* _LUOSHAL_H_ */
