#ifndef _HAL_H_
#define _HAL_H_

#include <main.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
// list of all branches of your configuration.
typedef enum
{
    BRANCH_A,
    BRANCH_B,
    NO_BRANCH // you have to keep this one at the last position
} branch_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Function
 ******************************************************************************/
#define LUOS_UUID ((uint32_t *)0x1FFFF7AC)

#define MCUFREQ 48000000

#define PAGE_SIZE (uint32_t) FLASH_PAGE_SIZE
#define ERASE_SIZE (uint32_t) NVMCTRL_FLASH_ROWSIZE
#define ADDRESS_ALIASES_FLASH (FLASH_ADDR + FLASH_SIZE - (16*FLASH_PAGE_SIZE))
#define ADDRESS_BOOT_FLAG_FLASH (FLASH_ADDR + FLASH_SIZE) - 4


#define ROBUS_POWER_SENSOR_Pin 			GPIO_PIN_2
#define ROBUS_POWER_SENSOR_GPIO_Port 	GPIOA
#define LED_Pin 						GPIO_PIN_3
#define LED_GPIO_Port 					GPIOA

#define RS485_LVL_DOWN_Pin 				GPIO_PIN_5
#define RS485_LVL_DOWN_GPIO_Port 		GPIOA
#define RS485_LVL_UP_Pin 				GPIO_PIN_6
#define RS485_LVL_UP_GPIO_Port 			GPIOA
#define BTN_Pin 						GPIO_PIN_0
#define BTN_GPIO_Port 					GPIOB
#define ROBUS_PTPB_Pin 					GPIO_PIN_13
#define ROBUS_PTPB_GPIO_Port 			GPIOB
#define ROBUS_RE_Pin 					GPIO_PIN_14
#define ROBUS_RE_GPIO_Port 				GPIOB
#define ROBUS_DE_Pin 					GPIO_PIN_15
#define ROBUS_DE_GPIO_Port 				GPIOB
#define ROBUS_PTPA_Pin 					GPIO_PIN_8
#define ROBUS_PTPA_GPIO_Port 			GPIOA
#define ROBUS_TX_Pin 					GPIO_PIN_9
#define ROBUS_TX_GPIO_Port 				GPIOA
#define ROBUS_RX_Pin 					GPIO_PIN_10
#define ROBUS_RX_GPIO_Port 				GPIOA

#define ROBUS_TX_DETECT_Pin 			GPIO_PIN_11
#define ROBUS_TX_DETECT_Port 			GPIOA

void crc(unsigned char *data, unsigned short size, unsigned char *crc);
void LuosHAL_init(void);
void set_baudrate(unsigned int baudrate);
unsigned char hal_transmit(unsigned char *data, unsigned short size);
void send_poke(branch_t branch);
void hal_delay_ms(int factor);
void hal_disable_irq(void);
void hal_enable_irq(void);
void hal_disable_tx(void);
void hal_disable_rx(void);
void hal_enable_tx(void);
void hal_enable_rx(void);
void hal_wait_transmit_end(void);
void hal_disable_irq(void);
void hal_enable_irq(void);
void HAL_LockTx(uint8_t lock);
void set_PTP(branch_t branch);
void reset_PTP(branch_t branch);
unsigned char get_PTP(branch_t branch);
void reverse_detection(branch_t branch);
char HAL_is_tx_lock(void);
void HAL_LockTx(uint8_t lock);
void node_disable_irq(void);
void node_enable_irq(void);
uint32_t node_get_systick(void);

void write_alias(unsigned short local_id, char *alias);
char read_alias(unsigned short local_id, char *alias);

#endif /* _HAL_H_ */
