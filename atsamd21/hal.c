#include "hal.h"
#include "reception.h"
#include "context.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
USART_SERIAL_SETUP serialSetup;
uintptr_t ContextHandler;
uint8_t dataReceive = 0;
volatile uint8_t FlagDataReceive = 0;
volatile uint8_t FlagRxDisable = 0;
/*******************************************************************************
 * Function
 ******************************************************************************/
void HAL_EnableTxLockDetection(void);
void HAL_DisableTxLockDetection(void);
static void Luos_HALComRxIRQHandler(uintptr_t context);
//static void Luos_HALComTxIRQHandler(uintptr_t context);
//static void Luos_HALComTimeoutIRQHandler(void);
static void Luos_HALPTPAIRQHandler(uintptr_t context);
static void Luos_HALPTPBIRQHandler(uintptr_t context);
static void Luos_HALTxDetectIRQHandler(uintptr_t context);

/**
 * \fn void hal_init(void)
 * \brief hardware configuration (clock, communication, DMA...)
 */
void LuosHAL_init(void)
{
    //initialize luos com
    SYSTICK_TimerStart();
    
    //set usart context
    set_baudrate(DEFAULTBAUDRATE);
    SERCOM0_USART_SerialSetup(&serialSetup, 0);
    
    //set receive
    SERCOM0_USART_ReadCallbackRegister(&Luos_HALComRxIRQHandler, 0);
    //SERCOM0_USART_WriteCallbackRegister(&Luos_HALComTxIRQHandler, 0);
    SERCOM0_USART_Read(&dataReceive, 1 );
    
      
	// Enable Reception timeout interrupt
	//LL_USART_SetRxTimeout(USART1, TIMEOUT_VAL * (8 + 1 + 1));
    
    //initialize External Interupt
    EIC_CallbackRegister(EIC_PIN_8, &Luos_HALPTPAIRQHandler, 0);
    EIC_CallbackRegister(EIC_PIN_9, &Luos_HALPTPBIRQHandler, 0);
    EIC_CallbackRegister(EIC_PIN_4, &Luos_HALTxDetectIRQHandler, 0);

    // Setup data direction
    Tx_En_OutputEnable();
    // Setup pull ups pins
    RS485_LVL_UP_Set();
    RS485_LVL_DOWN_InputEnable();

    // Setup PTP lines
    reset_PTP(BRANCH_A);
    reset_PTP(BRANCH_B);
}
/**
 * \fn void USART1_IRQHandler(void)
 * \brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
 *
 */
static void Luos_HALComRxIRQHandler(uintptr_t context)
{
    FlagDataReceive = 1;
    SERCOM0_USART_Read(&dataReceive, 1 );// set pointer receive and size
    ctx.data_cb(&dataReceive); // send reception byte to state machine
}

//static void Luos_HALComTxIRQHandler(uintptr_t context)
//{
//    FlagTxFinish = true;
//}

//static void Luos_HALComTimeoutIRQHandler(void)
//{
//   if ((LL_USART_IsActiveFlag_RTO(USART1) != RESET) && (LL_USART_IsEnabledIT_RTO(USART1) != RESET))
//    {
//        if (ctx.tx_lock)
//        {
//            timeout();
//        }
//        else
//        {
//            //ERROR
//        }
//        LL_USART_ClearFlag_RTO(USART1);
//        LL_USART_SetRxTimeout(USART1, TIMEOUT_VAL * (8 + 1 + 1));
//    }
//}
/**
 * \fn HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 * \brief PTP interrupt management
 */
static void Luos_HALPTPAIRQHandler(uintptr_t context)
{
		ptp_handler(BRANCH_A);
}
static void Luos_HALPTPBIRQHandler(uintptr_t context)
{
		ptp_handler(BRANCH_B);
}
static void Luos_HALTxDetectIRQHandler(uintptr_t context)
{
		//HAL_LockTx(true);
}
/**
 * \fn HAL_is_tx_lock(void)
 * \brief PTP interrupt management
 */
char HAL_is_tx_lock(void)
{
    return ctx.tx_lock;
}
/**
 * \fn HAL_EnableTxLockDetection(void)
 * \brief PTP interrupt management
 */
void HAL_EnableTxLockDetection(void)
{
	EIC_InterruptEnable(EIC_PIN_4);
}
/**
 * \fn HAL_DisableTxLockDetection(void)
 * \brief PTP interrupt management
 */
void HAL_DisableTxLockDetection(void)
{
	EIC_InterruptDisable(EIC_PIN_4);
}
/**
 * \fn HAL_is_tx_lock(void)
 * \brief PTP interrupt management
 */
void HAL_LockTx(uint8_t lock)
{
	ctx.tx_lock = lock;
}
/**
 * \fn HAL_is_tx_lock(void)
 * \brief PTP interrupt management
 */
void set_baudrate(unsigned int baudrate)
{
    serialSetup.baudRate = baudrate;
    serialSetup.dataWidth = USART_DATA_8_BIT;
    serialSetup.parity = USART_PARITY_NONE;
    serialSetup.stopBits = USART_STOP_1_BIT;
}

uint16_t calculate_crc(unsigned char *data, unsigned int data_len)
{
    uint16_t crc = 0xFFFF;
    if (data_len == 0)
        return 0;
    for (unsigned int i = 0; i < data_len; ++i)
    {
        uint16_t dbyte = data[i];
        crc ^= dbyte << 8;
        for (unsigned char j = 0; j < 8; ++j)
        {
            uint16_t mix = crc & 0x8000;
            crc = (crc << 1);
            if (mix)
                crc = crc ^ 0x0007;
        }
    }
    return crc;
}
/**
 * \fn unsigned char crc(unsigned char* data, unsigned char size)
 * \brief generate a CRC
 *
 * \param *data data table
 * \param size data size
 *
 * \return CRC value
 */
void crc(unsigned char *data, unsigned short size, unsigned char *crc)
{
    unsigned short calc;
    if (size > 1)
    {
        calc = calculate_crc(data,(char) size);  //calculate
    }
    else
    {
        calc = calculate_crc(data, 1);    //accumalate
    }
    crc[0] = (unsigned char)calc;
    crc[1] = (unsigned char)(calc >> 8);
}
/**
 * \fn reverse_detection(void)
 * \brief
 */
void set_PTP(branch_t branch)
{
    //remouve multiplexe
    if (branch == BRANCH_A)
    {
        EIC_REGS->EIC_INTFLAG |= (1UL << EIC_PIN_8);//clear IT flag
        EIC_InterruptDisable(EIC_PIN_8);
        PORT_REGS->GROUP[1].PORT_PINCFG[8] &= ~0x1;//desactivate PIN_MUX PTPA
        PTPA_OutputEnable();
        PTPA_Set();
       
    }
    else if (branch == BRANCH_B)
    {
        EIC_REGS->EIC_INTFLAG |= (1UL << EIC_PIN_9);//clear IT flag
        EIC_InterruptDisable(EIC_PIN_9);
        PORT_REGS->GROUP[1].PORT_PINCFG[9] &= ~0x1;//desactivate PIN_MUX PTPB
        PTPB_OutputEnable();
        PTPB_Set();
    }
}
/**
 * \fn reverse_detection(void)
 * \brief
 */
void reset_PTP(branch_t branch)
{
    if (branch == BRANCH_A)
	{
        EIC_REGS->EIC_CONFIG[1] |= EIC_CONFIG_SENSE0_RISE;//IT Rising
		PORT_REGS->GROUP[1].PORT_PINCFG[8] |= 0x1;//activate PIN_MUX PTPA
        EIC_REGS->EIC_INTFLAG |= (1UL << EIC_PIN_8);//clear IT flag
        EIC_InterruptEnable(EIC_PIN_8);
	}
	else if (branch == BRANCH_B)
	{
        EIC_REGS->EIC_CONFIG[1] |= EIC_CONFIG_SENSE1_RISE;//IT Rising
		PORT_REGS->GROUP[1].PORT_PINCFG[9] |= 0x1;//activate PIN_MUX PTPB
        EIC_REGS->EIC_INTFLAG |= (1UL << EIC_PIN_9);//clear IT flag
        EIC_InterruptEnable(EIC_PIN_9);
	}
}
/**
 * \fn reverse_detection(void)
 * \brief
 */
void reverse_detection(branch_t branch)
{
    if (branch == BRANCH_A)
	{
        EIC_REGS->EIC_CONFIG[1] |= EIC_CONFIG_SENSE0_FALL;//IT Falling
		PORT_REGS->GROUP[1].PORT_PINCFG[8] |= 0x1;//activate PIN_MUX PTPA
        EIC_REGS->EIC_INTFLAG |= (1UL << EIC_PIN_8);//clear IT flag
        EIC_InterruptEnable(EIC_PIN_8);
	}
	else if (branch == BRANCH_B)
	{
        EIC_REGS->EIC_CONFIG[1] |= EIC_CONFIG_SENSE1_FALL;//IT Falling
		PORT_REGS->GROUP[1].PORT_PINCFG[9] |= 0x1;//activate PIN_MUX PTPB
        EIC_REGS->EIC_INTFLAG |= (1UL << EIC_PIN_9);//clear IT flag
        EIC_InterruptEnable(EIC_PIN_9);
	}
}

unsigned char get_PTP(branch_t branch)
{
	if (branch == BRANCH_A)
    {
        EIC_REGS->EIC_INTFLAG |= (1UL << EIC_PIN_9);//clear IT flag
        EIC_InterruptDisable(EIC_PIN_9);
        PORT_REGS->GROUP[1].PORT_PINCFG[8] &= ~0x1;//desactivate PIN_MUX PTPB
        PTPA_InputEnable();
        return PTPA_Get();
    }
    else if (branch == BRANCH_B)
    {
        EIC_REGS->EIC_INTFLAG |= (1UL << EIC_PIN_9);//clear IT flag
        EIC_InterruptDisable(EIC_PIN_9);
        PORT_REGS->GROUP[1].PORT_PINCFG[9] &= ~0x1;//desactivate PIN_MUX PTPB
        PTPB_InputEnable();
        return PTPB_Get();
    }
    return 0;
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
    for (unsigned short i = 0; i < size; i++)
    {
        SERCOM0_USART_Write((data + i), 1);
        while((SERCOM0_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk) != SERCOM_USART_INT_INTFLAG_DRE_Msk);
        while((FlagDataReceive != 1)&&(FlagRxDisable == false));
        if (ctx.collision)
        {
            // There is a collision
            ctx.collision = FALSE;
            return 1;
        }  
        FlagDataReceive = 0;
    }
    return 0;
}

void hal_wait_transmit_end(void)
{
    
}

/**
 * \fn void hal_disable_irq(void)
 * \brief disable IRQ
 *
 * \return error
 */
 void hal_disable_irq(void)
 {
	 node_disable_irq();
 }

/**
 * \fn void hal_enable_irq(void)
 * \brief enable IRQ
 *
 * \return error
 */
 void hal_enable_irq(void)
 {
	 node_enable_irq();
 }

/**
 * \fn void hal_enable_rx(void)
 * \brief enable RX hard channel
 *
 * \return error
 */
void hal_enable_rx(void)
{
	Rx_En_Clear();
    FlagRxDisable = false;
}

/**
 * \fn void hal_disable_rx(void)
 * \brief disable RX hard channel
 *
 * \return error
 */
void hal_disable_rx(void)
{
	Rx_En_Set();
    FlagRxDisable = true;
}

/**
 * \fn void hal_enable_tx(void)
 * \brief enable TX hard channel
 *
 * \return error
 */
void hal_enable_tx(void)
{
	Tx_En_Set();
}

/**
 * \fn void hal_disable_tx(void)
 * \brief disable TX hard channel
 *
 * \return error
 */
void hal_disable_tx(void)
{
    //uint8_t data = 0;
	Tx_En_Clear();
    //SERCOM0_USART_Write( &data, 0 );
}
/**
 * \fn void board_enable_irq(void)
 * \brief enable IRQ
 *
 * \return error
 */
void node_enable_irq(void)
{
    __enable_irq();
}

void node_disable_irq(void)
{
    __disable_irq();
}

uint32_t node_get_systick(void)
{
	return SYSTICK_TimerCounterGet();
}

 //******** Alias management ****************


/******************************************************************************
 * @brief
 *   Luos_HALErasePage: Luos HAL general initialisation
 * @Param
 *
 * @Return
 *
 ******************************************************************************/
void Luos_HALEraseFlashPage(void)
{
	NVMCTRL_RowErase(ADDRESS_ALIASES_FLASH);
    NVMCTRL_RowErase(ADDRESS_ALIASES_FLASH + 256);
    NVMCTRL_RowErase(ADDRESS_ALIASES_FLASH + 512);
    NVMCTRL_RowErase(ADDRESS_ALIASES_FLASH + 768);
    
}
/******************************************************************************
 * @brief
 *   Luos_HALErasePage: Luos HAL general initialisation
 * @Param
 *
 * @Return
 *
 ******************************************************************************/
void Luos_HALWriteFlash(uint32_t addr, uint16_t size, uint8_t *data)
{
    // Before writing we have to erase the entire page
    // to do that we have to backup current falues by copying it into RAM
    uint8_t page_backup[16*PAGE_SIZE];
    memcpy(page_backup, (void *)ADDRESS_ALIASES_FLASH, 16*PAGE_SIZE);

    // Now we can erase the page
    Luos_HALEraseFlashPage();

    // Then add input data into backuped value on RAM
    uint32_t RAMaddr = (addr - ADDRESS_ALIASES_FLASH);
    memcpy(&page_backup[RAMaddr], data, size);

    NVMCTRL_PageWrite((uint32_t*)page_backup,  ADDRESS_ALIASES_FLASH);
}
/******************************************************************************
 * @brief
 *   Luos_HALErasePage: Luos HAL general initialisation
 * @Param
 *
 * @Return
 *
 ******************************************************************************/
void Luos_HALReadFlash(uint32_t addr, uint16_t size, uint8_t *data)
{
	memcpy(data, (void *)(addr), size);
}

//******** Alias management ****************
void write_alias(unsigned short local_id, char *alias)
{
	uint32_t addr = ADDRESS_ALIASES_FLASH + (local_id * (MAX_ALIAS_SIZE + 1));
	Luos_HALWriteFlash(addr, 16, (uint8_t*)alias);
}

char read_alias(unsigned short local_id, char *alias)
{
    uint32_t addr = ADDRESS_ALIASES_FLASH + (local_id * (MAX_ALIAS_SIZE + 1));
	Luos_HALReadFlash(addr, 16, (uint8_t*)alias);
	// Check name integrity
	if ((((alias[0] < 'A') | (alias[0] > 'Z')) & ((alias[0] < 'a') | (alias[0] > 'z'))) | (alias[0] == '\0'))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}
