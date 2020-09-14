#include "hal.h"

/**
 * \fn void hal_init(void)
 * \brief hardware configuration (clock, communication, DMA...)
 */
void hal_init(void)
{
    // init all things
}

/**
 * \fn unsigned char hal_transmit(unsigned char* data)
 * \brief write a data byte
 *
 * \param data *data bytes to send
 * \param size size of data to send in byte
 *
 * \return error
 */
unsigned char hal_transmit(unsigned char *data, unsigned short size)
{
    if /*hardware ready*/
    {
        // Put data into a DMA => serial interface or do it with a for
    }
    else
    {
        ctx.status.master_write = TRUE;
        return 1;
    }
    return 0;
}
