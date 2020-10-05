/******************************************************************************
 * @file luosHAL
 * @brief Luos Hardware Abstration Layer. Describe Low layer fonction 
 * @MCU Family STM32FO
 * @author Luos
 * @version 0.0.0
 ******************************************************************************/
#include "luosHAL.h"

#include <stdbool.h>
#include <string.h>
#include "reception.h"
#include "context.h"

//MCU dependencies this HAL is for family STM32FO you can find
//the HAL stm32cubef0 on ST web site
#include "stm32f0xx_ll_usart.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
CRC_HandleTypeDef hcrc;
GPIO_InitTypeDef GPIO_InitStruct = {0};

/*******************************************************************************
 * Function
 ******************************************************************************/
static void LuosHAL_FlashInit(void);
static void LuosHAL_CRCInit(void);
static void LuosHAL_TimeoutInit(void);
static void LuosHAL_GPIOInit(void);
static void LuosHAL_FlashEraseLuosMemoryInfo(void);
static inline void LuosHAL_ComReceive(void);

/////////////////////////Luos Library Needed function///////////////////////////
/******************************************************************************
 * @brief Luos HAL general initialisation
 * @param None
 * @return None
 ******************************************************************************/
void LuosHAL_Init(void)
{
    //IO Initialization
    LuosHAL_GPIOInit();

    // Flash Initialization
    LuosHAL_FlashInit();

    // CRC Initialization
    LuosHAL_CRCInit();

    //Com Initialization
    LuosHAL_ComInit(DEFAULTBAUDRATE);

    //Timeout Initialization
    LuosHAL_TimeoutInit();
}
/******************************************************************************
 * @brief Luos HAL general disable IRQ
 * @param None
 * @return None
 ******************************************************************************/
void LuosHAL_SetIrqState(uint8_t Enable)
{
    if (Enable == true)
    {
        __enable_irq();
    }
    else
    {
        __disable_irq();
    }
}
/******************************************************************************
 * @brief Luos HAL general systick tick at 1ms
 * @param None
 * @return tick Counter
 ******************************************************************************/
uint32_t LuosHAL_GetSystick(void)
{
    return HAL_GetTick();
}
/******************************************************************************
 * @brief Luos HAL Initialize Generale communication inter node
 * @param Select a baudrate for the Com
 * @return none
 ******************************************************************************/
void LuosHAL_ComInit(uint32_t Baudrate)
{
    LUOS_COM_CLOCK_ENABLE();

    LL_USART_InitTypeDef USART_InitStruct;

    // Initialise USART1
    LL_USART_Disable(LUOS_COM);
    USART_InitStruct.BaudRate = Baudrate;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    while (LL_USART_Init(LUOS_COM, &USART_InitStruct) != SUCCESS)
        ;
    LL_USART_Enable(LUOS_COM);

    // Enable Reception interrupt
    LL_USART_EnableIT_RXNE(LUOS_COM);

    // Enable Reception timeout interrupt
    // the timeout expressed in nb of bits duration
    LL_USART_SetRxTimeout(LUOS_COM, TIMEOUT_VAL * (8 + 1 + 1));
    LL_USART_EnableRxTimeout(LUOS_COM);
    LL_USART_EnableIT_RTO(LUOS_COM);

    HAL_NVIC_EnableIRQ(LUOS_COM_IRQ);
    HAL_NVIC_SetPriority(LUOS_COM_IRQ, 0, 1);
}
/******************************************************************************
 * @brief Tx enable/disable relative to com
 * @param None
 * @return None
 ******************************************************************************/
void LuosHAL_SetTxState(uint8_t Enable)
{
    if (Enable == true)
    {
        HAL_GPIO_WritePin(TX_EN_PORT, TX_EN_PIN, GPIO_PIN_SET);
        // Sometime the TX set is too slow and the driver switch in sleep mode...
        HAL_GPIO_WritePin(TX_EN_PORT, TX_EN_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(TX_EN_PORT, TX_EN_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(TX_EN_PORT, TX_EN_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(TX_EN_PORT, TX_EN_PIN, GPIO_PIN_RESET);
    }
}
/******************************************************************************
 * @brief Rx enable/disable relative to com
 * @param
 * @return
 ******************************************************************************/
void LuosHAL_SetRxState(uint8_t Enable)
{
    if (Enable == true)
    {
        HAL_GPIO_WritePin(RX_EN_PORT, RX_EN_PIN, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(RX_EN_PORT, RX_EN_PIN, GPIO_PIN_SET);
    }
}
/******************************************************************************
 * @brief Luos Timeout initialisation
 * @param None
 * @return None
 ******************************************************************************/
static void LuosHAL_TimeoutInit(void)
{
}
/******************************************************************************
 * @brief Luos Timeout for Rx communication
 * @param None
 * @return None
 ******************************************************************************/
void LuosHAL_ComRxTimeout(void)
{
}
/******************************************************************************
 * @brief Luos Timeout for Tx communication
 * @param None
 * @return None
 ******************************************************************************/
void LuosHAL_ComTxTimeout(void)
{
    while (!LL_USART_IsActiveFlag_TC(LUOS_COM));
}
/******************************************************************************
 * @brief Process data receive
 * @param None
 * @return None
 ******************************************************************************/
static inline void LuosHAL_ComReceive(void)
{
    // check if we receive a data
    if ((LL_USART_IsActiveFlag_RXNE(LUOS_COM) != RESET) && (LL_USART_IsEnabledIT_RXNE(LUOS_COM) != RESET))
    {
        uint8_t data = LL_USART_ReceiveData8(LUOS_COM);
        ctx.data_cb(&data); // send reception byte to state machine
    }
    else if ((LL_USART_IsActiveFlag_RTO(LUOS_COM) != RESET) && (LL_USART_IsEnabledIT_RTO(LUOS_COM) != RESET))
    {
        // Check if a timeout on reception occure
        if (ctx.tx_lock)
        {
            Recep_Timeout();
        }
        else
        {
            //ERROR
        }
        LL_USART_ClearFlag_RTO(LUOS_COM);
        LL_USART_SetRxTimeout(LUOS_COM, TIMEOUT_VAL * (8 + 1 + 1));
    }
    else
    {
        //clear error flag
        LL_USART_ClearFlag_PE(LUOS_COM);
        LL_USART_ClearFlag_FE(LUOS_COM);
        LL_USART_ClearFlag_NE(LUOS_COM);
        LL_USART_ClearFlag_ORE(LUOS_COM); // error generate when disable IT receive when transmit
    }
}
/******************************************************************************
 * @brief Process data transmit
 * @param None
 * @return None
 ******************************************************************************/
uint8_t LuosHAL_ComTransmit(uint8_t *data, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++)
    {
        while (!LL_USART_IsActiveFlag_TXE(LUOS_COM))
        {
        }
        if (ctx.collision)
        {
            // There is a collision
            ctx.collision = FALSE;
            return 1;
        }
        LL_USART_TransmitData8(LUOS_COM, *(data + i));
    }
    return 0;
}
/******************************************************************************
 * @brief set state of Txlock detection pin
 * @param None
 * @return Lock status
 ******************************************************************************/
void LuosHAL_SetTxLockDetecState(uint8_t Enable)
{
    if((TX_LOCK_DETECT_PIN != DISABLE)||(TX_LOCK_DETECT_PORT != DISABLE))
    {
        // Set Pin Tx Detec
        // Input / Pull up
        GPIO_InitStruct.Pin = TX_LOCK_DETECT_PIN;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        if(Enable == true)
        {
            __HAL_GPIO_EXTI_CLEAR_IT(TX_LOCK_DETECT_PIN);
            GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
            HAL_GPIO_Init(TX_LOCK_DETECT_PORT, &GPIO_InitStruct);
        }
        else
        {
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            HAL_GPIO_Init(TX_LOCK_DETECT_PORT, &GPIO_InitStruct);
        }
    }
}
/******************************************************************************
 * @brief get Lock Com transmit status
 * @param None
 * @return Lock status
 ******************************************************************************/
uint8_t LuosHAL_GetTxLockState(void)
{
    uint8_t result = false;
    if (READ_BIT(LUOS_COM->ISR, USART_ISR_BUSY) == (USART_ISR_BUSY))
    {
        result = true;
    }
    return result;
}
/******************************************************************************
 * @brief Initialisation GPIO
 * @param None
 * @return None
 ******************************************************************************/
static void LuosHAL_GPIOInit(void)
{
    //Activate Clock for PIN choosen in luosHAL
    PORT_CLOCK_ENABLE();

    /*Configure GPIO pin : COM_LVL_DOWN_PIN */
    GPIO_InitStruct.Pin = COM_LVL_DOWN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    if((COM_LVL_DOWN_PIN != DISABLE)||(COM_LVL_DOWN_PORT != DISABLE))
    {
        HAL_GPIO_WritePin(COM_LVL_DOWN_PORT, COM_LVL_DOWN_PIN, GPIO_PIN_RESET); // Setup pull down pins
        HAL_GPIO_Init(COM_LVL_DOWN_PORT, &GPIO_InitStruct);
    }

    /*Configure GPIO pin : COM_LVL_UP_PIN */
    GPIO_InitStruct.Pin = COM_LVL_UP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    if((COM_LVL_UP_PIN != DISABLE)||(COM_LVL_UP_PORT != DISABLE))
    {
        HAL_GPIO_Init(COM_LVL_UP_PORT, &GPIO_InitStruct);
        HAL_GPIO_WritePin(COM_LVL_UP_PORT, COM_LVL_UP_PIN, GPIO_PIN_SET);       // Setup pull up pins
    }

    /*Configure GPIO pins : RxEN_Pin */
    GPIO_InitStruct.Pin = RX_EN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RX_EN_PORT, &GPIO_InitStruct);

    /*Configure GPIO pins : RxEN_Pin */
    GPIO_InitStruct.Pin = TX_EN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TX_EN_PORT, &GPIO_InitStruct);

    /*Configure GPIO pins : PTPA_Pin */
    GPIO_InitStruct.Pin = PTPA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PTPA_PORT, &GPIO_InitStruct);

    /*Configure GPIO pins : PTPB_Pin */
    GPIO_InitStruct.Pin = PTPB_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PTPB_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : TxPin */
    GPIO_InitStruct.Pin = COM_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = COM_TX_AF;
    HAL_GPIO_Init(COM_TX_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : RxPin */
    GPIO_InitStruct.Pin = COM_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = COM_RX_AF;
    HAL_GPIO_Init(COM_RX_PORT, &GPIO_InitStruct);

    // Setup PTP lines
    LuosHAL_SetPTPDefaultState(BRANCH_A);
    LuosHAL_SetPTPDefaultState(BRANCH_B);
    //set Lock TX detection
    LuosHAL_SetTxLockDetecState(true);

    //activate IT for PTP and TX
    HAL_NVIC_SetPriority(PTPA_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(PTPA_IRQ);

    HAL_NVIC_SetPriority(PTPB_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(PTPB_IRQ);

    HAL_NVIC_SetPriority(TX_LOCK_DETECT_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(TX_LOCK_DETECT_IRQ);
}
/******************************************************************************
 * @brief callback for GPIO IT
 * @param GPIO IT line
 * @return None
 ******************************************************************************/
void LuosHAL_GPIOProcess(uint16_t GPIO)
{
    //Process for PTP Detetion
    if (GPIO == PTPA_PIN)
    {
        Detec_PtpHandler(BRANCH_A);
        return;
    }
    else if (GPIO == PTPB_PIN)
    {
        Detec_PtpHandler(BRANCH_B);
        return;
    }
    //Process For Com Transmit Detection
    else if (GPIO == TX_LOCK_DETECT_PIN)
    {
        if(ctx.tx_lock == false)
        {
            ctx.tx_lock = true;
            LuosHAL_SetTxLockDetecState(false);
        }
        return;
    }  
}
/******************************************************************************
 * @brief Set PTP for Detection on branch
 * @param PTP branch
 * @return None
 ******************************************************************************/
void LuosHAL_SetPTPDefaultState(branch_t branch)
{
    // Pull Down / IT mode / Rising Edge
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    if (branch == BRANCH_A)
    {
        // set the PTPA pin as input pull-down IRQ triggered on rising edge event
        __HAL_GPIO_EXTI_CLEAR_IT(PTPA_PIN);
        GPIO_InitStruct.Pin = PTPA_PIN;
        HAL_GPIO_Init(PTPA_PORT, &GPIO_InitStruct);
    }
    else if (branch == BRANCH_B)
    {
        // set the PTPB pin as input pull-down IRQ triggered on rising edge event
        __HAL_GPIO_EXTI_CLEAR_IT(PTPB_PIN);
        GPIO_InitStruct.Pin = PTPB_PIN;
        HAL_GPIO_Init(PTPB_PORT, &GPIO_InitStruct);
    }
}
/******************************************************************************
 * @brief Set PTP for reverse detection on branch
 * @param PTP branch
 * @return None
 ******************************************************************************/
void LuosHAL_SetPTPReverseState(branch_t branch)
{
    // Pull Down / IT mode / Falling Edge
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // reverse the detection edge
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    if (branch == BRANCH_A)
    {
        GPIO_InitStruct.Pin = PTPA_PIN;
        HAL_GPIO_Init(PTPA_PORT, &GPIO_InitStruct);
    }
    else if (branch == BRANCH_B)
    {
        GPIO_InitStruct.Pin = PTPB_PIN;
        HAL_GPIO_Init(PTPB_PORT, &GPIO_InitStruct);
    }
}
/******************************************************************************
 * @brief Set PTP line
 * @param PTP branch
 * @return None
 ******************************************************************************/
void LuosHAL_PushPTP(branch_t branch)
{
    // Pull Down / Output mode
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Clean edge/state detection and set the PTP pin as output
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    if (branch == BRANCH_A)
    {
        GPIO_InitStruct.Pin = PTPA_PIN;
        HAL_GPIO_Init(PTPA_PORT, &GPIO_InitStruct);
        HAL_GPIO_WritePin(PTPA_PORT, PTPA_PIN, GPIO_PIN_SET); // Set the PTPA pin
    }
    else if (branch == BRANCH_B)
    {
        GPIO_InitStruct.Pin = PTPB_PIN;
        HAL_GPIO_Init(PTPB_PORT, &GPIO_InitStruct);
        HAL_GPIO_WritePin(PTPB_PORT, PTPB_PIN, GPIO_PIN_SET); // Set the PTPB pin
    }
}
/******************************************************************************
 * @brief Get PTP line
 * @param PTP branch
 * @return Line state
 ******************************************************************************/
uint8_t LuosHAL_GetPTPState(branch_t branch)
{
    // Pull Down / Input mode
    if (branch == BRANCH_A)
    {
        return (HAL_GPIO_ReadPin(PTPA_PORT, PTPA_PIN));
    }
    else if (branch == BRANCH_B)
    {
        return (HAL_GPIO_ReadPin(PTPB_PORT, PTPB_PIN));
    }
    return 0;
}
/******************************************************************************
 * @brief Initialize CRC Process
 * @param None
 * @return None
 ******************************************************************************/
static void LuosHAL_CRCInit(void)
{
    __HAL_RCC_CRC_CLK_ENABLE();
    hcrc.Instance = CRC;
    hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
    hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
    hcrc.Init.GeneratingPolynomial = 7;
    hcrc.Init.CRCLength = CRC_POLYLENGTH_16B;
    hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
    hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
    if (HAL_CRC_Init(&hcrc) != HAL_OK)
    {
        while (1);
    }
}
/******************************************************************************
 * @brief Compute CRC
 * @param None
 * @return None
 ******************************************************************************/
void LuosHAL_ComputeCRC(uint8_t *data, uint16_t size, uint8_t *crc)
{
    uint16_t calc;
    if (size > 1)
    {
        calc = (unsigned short)HAL_CRC_Calculate(&hcrc, (uint32_t *)data, size);
    }
    else
    {
        calc = (unsigned short)HAL_CRC_Accumulate(&hcrc, (uint32_t *)data, 1);
    }
    crc[0] = (unsigned char)calc;
    crc[1] = (unsigned char)(calc >> 8);
}
/******************************************************************************
 * @brief Flash Initialisation
 * @param None
 * @return None
 ******************************************************************************/
static void LuosHAL_FlashInit(void)
{
}
/******************************************************************************
 * @brief Erase flash page where Luos keep permanente information
 * @param None
 * @return None
 ******************************************************************************/
static void LuosHAL_FlashEraseLuosMemoryInfo(void)
{
    uint32_t page_error = 0;
    FLASH_EraseInitTypeDef s_eraseinit;

    s_eraseinit.TypeErase = FLASH_TYPEERASE_PAGES;
    s_eraseinit.PageAddress = ADDRESS_LAST_PAGE_FLASH;
    s_eraseinit.NbPages = 1;

    // Erase Page
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&s_eraseinit, &page_error);
    HAL_FLASH_Lock();
}
/******************************************************************************
 * @brief Write flash page where Luos keep permanente information
 * @param Address page / size to write / pointer to data to write
 * @return
 ******************************************************************************/
void LuosHAL_FlashWriteLuosMemoryInfo(uint32_t addr, uint16_t size, uint8_t *data)
{
    // Before writing we have to erase the entire page
    // to do that we have to backup current falues by copying it into RAM
    uint8_t page_backup[PAGE_SIZE];
    memcpy(page_backup, (void *)ADDRESS_ALIASES_FLASH, PAGE_SIZE);

    // Now we can erase the page
    LuosHAL_FlashEraseLuosMemoryInfo();

    // Then add input data into backuped value on RAM
    uint32_t RAMaddr = (addr - ADDRESS_ALIASES_FLASH);
    memcpy(&page_backup[RAMaddr], data, size);

    // and copy it into flash
    HAL_FLASH_Unlock();

    // ST hal flash program function write data by uint64_t raw data
    for (uint32_t i = 0; i < PAGE_SIZE; i += sizeof(uint64_t))
    {
        while (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, i + ADDRESS_ALIASES_FLASH, *(uint64_t *)(&page_backup[i])) != HAL_OK)
            ;
    }
    HAL_FLASH_Lock();
}
/******************************************************************************
 * @brief read information from page where Luos keep permanente information
 * @param Address info / size to read / pointer callback data to read
 * @return
 ******************************************************************************/
void LuosHAL_FlashReadLuosMemoryInfo(uint32_t addr, uint16_t size, uint8_t *data)
{
    memcpy(data, (void *)(addr), size);
}

/////////////////////////Special LuosHAL function///////////////////////////
void PINOUT_IRQHANDLER(uint16_t GPIO_Pin)
{
    LuosHAL_GPIOProcess(GPIO_Pin);
}
void LUOS_COM_IRQHANDLER()
{
    LuosHAL_ComReceive();
}
