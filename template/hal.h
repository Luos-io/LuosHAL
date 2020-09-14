#ifndef _HAL_H_
#define _HAL_H_

#include "context.h"

void hal_init(void);

unsigned char hal_transmit(unsigned char *data, unsigned short size);

#endif /* _HAL_H_ */
