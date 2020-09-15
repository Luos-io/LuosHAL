/******************************************************************************
 * @file luosHAL
 * @brief Luos Hardware Abstration Layer. Describe Low layer fonction
 * @MCU Family STM32FO
 * @author Luos
 * @version 0.0.0
 ******************************************************************************/
#ifndef _HAL_H_
#define _HAL_H_

#include <main.h>

//MCU dependencies
#include "stm32f0xx_ll_usart.h"
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

#define COM_LVL_DOWN_PIN	 		GPIO_PIN_5
#define COM_LVL_DOWN_PORT	 		GPIOA

#define COM_LVL_UP_PIN 				GPIO_PIN_6
#define COM_LVL_UP_PORT 			GPIOA

#define PTPA_PIN 					GPIO_PIN_8
#define PTPA_PORT					GPIOA

#define PTPB_PIN					GPIO_PIN_13
#define PTPB_PORT					GPIOB

#define RX_EN_PIN				 	GPIO_PIN_14
#define RX_EN_PORT					GPIOB

#define TX_EN_PIN					GPIO_PIN_15
#define TX_EN_PORT 					GPIOB

#define COM_TX_PIN 					GPIO_PIN_9
#define COM_TX_PORT					GPIOA

#define COM_RX_PIN 					GPIO_PIN_10
#define COM_RX_PORT 				GPIOA

#define TX_LOCK_DETECT_PIN 			GPIO_PIN_11
#define TX_LOCK_DETECT_PORT			GPIOA

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Function
 ******************************************************************************/
void LuosHAL_Init(void);
void LuosHAL_IrqStatus(uint8_t Enable);
uint32_t LuosHAL_GetSystick(void);
void LuosHAL_ComInit(uint32_t baudrate);
void LuosHAL_TxStatus(uint8_t Enable);
void LuosHAL_RxStatus(uint8_t Enable);
void LuosHAL_TimeoutInit(void);
void LuosHAL_ComRxTimeout(void);
void LuosHAL_ComTxTimeout(void);
void LuosHAL_ComReceive(void);
uint8_t LuosHAL_ComTransmit(uint8_t *data, uint16_t size);
void LuosHAL_SetTxLockStatus(uint8_t status);
uint8_t LuosHAL_GetTxLockStatus(void);
void LuosHAL_GPIOInit(void);
void LuosHAL_GPIOProcess(uint16_t GPIO);
void LuosHAL_PTPDetection(branch_t branch);
void LuosHAL_PTPReverseDetection(branch_t branch);
void LuosHAL_SetPTP(branch_t branch);
uint8_t LuosHAL_GetPTP(branch_t branch);
void LuosHAL_CRCInit(void);
void LuosHAL_ComputeCRC(uint8_t *data, uint16_t size, uint8_t *crc);
void LuosHAL_FlashInit(void);
void LuosHAL_FlashEraseLuosMemoryInfo(void);
void LuosHAL_FlashWriteLuosMemoryInfo(uint32_t addr, uint16_t size, uint8_t *data);
void LuosHAL_FlashReadLuosMemoryInfo(uint32_t addr, uint16_t size, uint8_t *data);

#endif /* _HAL_H_ */
