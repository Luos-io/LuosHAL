#ifndef _HAL_H_
#define _HAL_H_

#define MCUFREQ 170000000

// list of all branches of your configuration.
typedef enum
{
    BRANCH_A,
    BRANCH_B,
    NO_BRANCH // you have to keep this one at the last position
} branch_t;
#include "context.h"

void crc(unsigned char *data, unsigned short size, unsigned char *crc);
void hal_init(void);
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
void set_PTP(branch_t branch);
void reset_PTP(branch_t branch);
unsigned char get_PTP(branch_t branch);
void reverse_detection(branch_t branch);
char HAL_is_tx_lock(void);

#endif /* _HAL_H_ */
