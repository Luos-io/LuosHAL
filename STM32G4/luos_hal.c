/******************************************************************************
 * @file luosHAL
 * @brief Luos Hardware Abstration Layer. Describe Low layer fonction
 * @MCU Family STM32G4
 * @author Luos
 * @version 0.0.0
 ******************************************************************************/
#include "luos_hal.h"

#include <stdbool.h>
#include <string.h>
#include "reception.h"
#include "context.h"

//MCU dependencies this HAL is for family STM32G4 you can find
//the HAL stm32cubeg4 on ST web site
#include "stm32g4xx_ll_usart.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
#ifdef STM32G4xx_HAL_CRC_H
CRC_HandleTypeDef hcrc;
#endif
GPIO_InitTypeDef GPIO_InitStruct = {0};

typedef struct
{
    uint16_t Pin;
    GPIO_TypeDef *Port;
    uint8_t IRQ;
} Port_t;

Port_t PTP[NBR_PORT];
/*******************************************************************************
 * Function
 ******************************************************************************/
static void LuosHAL_SystickInit(void);
static void LuosHAL_FlashInit(void);
static void LuosHAL_CRCInit(void);
static void LuosHAL_TimeoutInit(void);
static void LuosHAL_ResetTimeout(void);
static inline void LuosHAL_ComTimeout(void);
static void LuosHAL_GPIOInit(void);
static void LuosHAL_FlashEraseLuosMemoryInfo(void);
static inline void LuosHAL_ComReceive(void);
static inline void LuosHAL_GPIOProcess(uint16_t GPIO);
static void LuosHAL_RegisterPTP(void);

/////////////////////////Luos Library Needed function///////////////////////////

/******************************************************************************
 * @brief Luos HAL general initialisation
 * @param None
 * @return None
 ******************************************************************************/
void LuosHAL_Init(void)
{
    //Systick Initialization
    LuosHAL_SystickInit();

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
 * @brief Luos HAL general systick tick at 1ms initialize
 * @param None
 * @return tick Counter
 ******************************************************************************/
static void LuosHAL_SystickInit(void)
{
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

    // Initialise USART3
    LL_USART_Disable(LUOS_COM);
    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
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
 * @brief Process data receive
 * @param None
 * @return None
 ******************************************************************************/
static inline void LuosHAL_ComReceive(void)
{

    if (LL_USART_IsActiveFlag_FE(LUOS_COM) != RESET)
    {
        LL_USART_ClearFlag_FE(LUOS_COM);
        ctx.rx.status.rx_framing_error = true;
    }

    if ((LL_USART_IsActiveFlag_RXNE(LUOS_COM) != RESET) && (LL_USART_IsEnabledIT_RXNE(LUOS_COM) != RESET))
    {
        uint8_t data = LL_USART_ReceiveData8(LUOS_COM);
        ctx.rx.callback(&data); // send reception byte to state machine
    }
    else if ((LL_USART_IsActiveFlag_RTO(LUOS_COM) != RESET) && (LL_USART_IsEnabledIT_RTO(LUOS_COM) != RESET))
    {
        // Check if a timeout on reception occure
        if (ctx.tx.lock)
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
        LUOS_COM->ICR = 0xFFFFFFFF;
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
        if (ctx.tx.collision)
        {
            // There is a collision
            ctx.tx.collision = FALSE;
            return 1;
        }
        LL_USART_TransmitData8(LUOS_COM, *(data + i));
    }
    return 0;
}
/******************************************************************************
 * @brief Luos Tx communication complete
 * @param None
 * @return None
 ******************************************************************************/
void LuosHAL_ComTxComplete(void)
{
    while (!LL_USART_IsActiveFlag_TC(LUOS_COM))
        ;
}
/******************************************************************************
 * @brief set state of Txlock detection pin
 * @param None
 * @return Lock status
 ******************************************************************************/
void LuosHAL_SetTxLockDetecState(uint8_t Enable)
{
}
/******************************************************************************
 * @brief get Lock Com transmit status this is the HW that can generate lock TX
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
    else if ((HAL_GPIO_ReadPin(TX_LOCK_DETECT_PORT, TX_LOCK_DETECT_PIN) == 0) &&
             ((TX_LOCK_DETECT_PIN != DISABLE) && (TX_LOCK_DETECT_PORT != DISABLE)))
    {
        result = true;
    }
    return result;
}
/******************************************************************************
 * @brief Luos Timeout initialisation
 * @param None
 * @return None
 ******************************************************************************/
static void LuosHAL_TimeoutInit(void)
{
    // Enable Reception timeout interrupt
    // the timeout expressed in nb of bits duration
    LL_USART_SetRxTimeout(LUOS_COM, TIMEOUT_VAL * (8 + 1 + 1));
    LL_USART_EnableRxTimeout(LUOS_COM);
    LL_USART_EnableIT_RTO(LUOS_COM);
}
/******************************************************************************
 * @brief Luos Timeout for Rx communication
 * @param None
 * @return None
 ******************************************************************************/
static void LuosHAL_ResetTimeout(void)
{
}
/******************************************************************************
 * @brief Luos Timeout for Rx communication
 * @param None
 * @return None
 ******************************************************************************/
static inline void LuosHAL_ComTimeout(void)
{
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
    if ((COM_LVL_DOWN_PIN != DISABLE) || (COM_LVL_DOWN_PORT != DISABLE))
    {
        HAL_GPIO_Init(COM_LVL_DOWN_PORT, &GPIO_InitStruct);
        HAL_GPIO_WritePin(COM_LVL_DOWN_PORT, COM_LVL_DOWN_PIN, GPIO_PIN_RESET); // Setup pull down pins
    }

    /*Configure GPIO pin : COM_LVL_UP_PIN */
    GPIO_InitStruct.Pin = COM_LVL_UP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    if ((COM_LVL_UP_PIN != DISABLE) || (COM_LVL_UP_PORT != DISABLE))
    {
        HAL_GPIO_Init(COM_LVL_UP_PORT, &GPIO_InitStruct);
        HAL_GPIO_WritePin(COM_LVL_UP_PORT, COM_LVL_UP_PIN, GPIO_PIN_SET); // Setup pull up pins
    }

    /*Configure GPIO pins : RxEN_Pin */
    GPIO_InitStruct.Pin = RX_EN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RX_EN_PORT, &GPIO_InitStruct);

    /*Configure GPIO pins : TxEN_Pin */
    GPIO_InitStruct.Pin = TX_EN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TX_EN_PORT, &GPIO_InitStruct);

    /*Configure GPIO pins : TX_LOCK_DETECT_Pin */
    GPIO_InitStruct.Pin = TX_LOCK_DETECT_PIN;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    if (TX_LOCK_DETECT_IRQ != DISABLE)
    {
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    }
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    if ((TX_LOCK_DETECT_PIN != DISABLE) || (TX_LOCK_DETECT_PORT != DISABLE))
    {
        HAL_GPIO_Init(TX_LOCK_DETECT_PORT, &GPIO_InitStruct);
    }

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

    //configure PTP
    LuosHAL_RegisterPTP();
    for (uint8_t i = 0; i < NBR_PORT; i++) /*Configure GPIO pins : PTP_Pin */
    {
        GPIO_InitStruct.Pin = PTP[i].Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(PTP[i].Port, &GPIO_InitStruct);
        // Setup PTP lines
        LuosHAL_SetPTPDefaultState(i);
        //activate IT for PTP
        HAL_NVIC_SetPriority(PTP[i].IRQ, 0, 0);
        HAL_NVIC_EnableIRQ(PTP[i].IRQ);
    }

    if (TX_LOCK_DETECT_IRQ != DISABLE)
    {
        //set Lock TX detection
        LuosHAL_SetTxLockDetecState(true);
        HAL_NVIC_SetPriority(TX_LOCK_DETECT_IRQ, 0, 0);
        HAL_NVIC_EnableIRQ(TX_LOCK_DETECT_IRQ);
    }
}
/******************************************************************************
 * @brief Register PTP
 * @param void
 * @return None
 ******************************************************************************/
static void LuosHAL_RegisterPTP(void)
{
#if (NBR_PORT >= 1)
    PTP[0].Pin = PTPA_PIN;
    PTP[0].Port = PTPA_PORT;
    PTP[0].IRQ = PTPA_IRQ;
#endif

#if (NBR_PORT >= 2)
    PTP[1].Pin = PTPB_PIN;
    PTP[1].Port = PTPB_PORT;
    PTP[1].IRQ = PTPB_IRQ;
#endif

#if (NBR_PORT >= 3)
    PTP[2].Pin = PTPC_PIN;
    PTP[2].Port = PTPC_PORT;
    PTP[2].IRQ = PTPC_IRQ;
#endif

#if (NBR_PORT >= 4)
    PTP[3].Pin = PTPD_PIN;
    PTP[3].Port = PTPD_PORT;
    PTP[3].IRQ = PTPD_IRQ;
#endif
}
/******************************************************************************
 * @brief callback for GPIO IT
 * @param GPIO IT line
 * @return None
 ******************************************************************************/
static inline void LuosHAL_GPIOProcess(uint16_t GPIO)
{
    ////Process for Tx Lock Detec
    if (GPIO == TX_LOCK_DETECT_PIN)
    {
    }
    else
    {
        for (uint8_t i = 0; i < NBR_PORT; i++)
        {
            if (GPIO == PTP[i].Pin)
            {
                PortMng_PtpHandler(i);
                break;
            }
        }
    }
}
/******************************************************************************
 * @brief Set PTP for Detection on branch
 * @param PTP branch
 * @return None
 ******************************************************************************/
void LuosHAL_SetPTPDefaultState(uint8_t PTPNbr)
{
    __HAL_GPIO_EXTI_CLEAR_IT(PTP[PTPNbr].Pin);
    // Pull Down / IT mode / Rising Edge
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Pin = PTP[PTPNbr].Pin;
    HAL_GPIO_Init(PTP[PTPNbr].Port, &GPIO_InitStruct);
}
/******************************************************************************
 * @brief Set PTP for reverse detection on branch
 * @param PTP branch
 * @return None
 ******************************************************************************/
void LuosHAL_SetPTPReverseState(uint8_t PTPNbr)
{
    __HAL_GPIO_EXTI_CLEAR_IT(PTP[PTPNbr].Pin);
    // Pull Down / IT mode / Falling Edge
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // reverse the detection edge
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Pin = PTP[PTPNbr].Pin;
    HAL_GPIO_Init(PTP[PTPNbr].Port, &GPIO_InitStruct);
}
/******************************************************************************
 * @brief Set PTP line
 * @param PTP branch
 * @return None
 ******************************************************************************/
void LuosHAL_PushPTP(uint8_t PTPNbr)
{
    // Pull Down / Output mode
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Clean edge/state detection and set the PTP pin as output
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Pin = PTP[PTPNbr].Pin;
    HAL_GPIO_Init(PTP[PTPNbr].Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(PTP[PTPNbr].Port, PTP[PTPNbr].Pin, GPIO_PIN_SET);
}
/******************************************************************************
 * @brief Get PTP line
 * @param PTP branch
 * @return Line state
 ******************************************************************************/
uint8_t LuosHAL_GetPTPState(uint8_t PTPNbr)
{
    // Pull Down / Input mode
    return (HAL_GPIO_ReadPin(PTP[PTPNbr].Port, PTP[PTPNbr].Pin));
}
/******************************************************************************
 * @brief Initialize CRC Process
 * @param None
 * @return None
 ******************************************************************************/
static void LuosHAL_CRCInit(void)
{
#ifdef STM32G4xx_HAL_CRC_H
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
        while (1)
            ;
    }
#endif
}
/******************************************************************************
 * @brief Compute CRC
 * @param None
 * @return None
 ******************************************************************************/
void LuosHAL_ComputeCRC(uint8_t *data, uint8_t *crc)
{
#ifdef STM32G4xx_HAL_CRC_H
    hcrc.Instance->INIT = *(uint16_t *)crc;
    __HAL_CRC_DR_RESET(&hcrc);
    *(uint16_t *)crc = (uint16_t)HAL_CRC_Accumulate(&hcrc, (uint32_t *)data, 1);
#else
    for (uint8_t i = 0; i < 1; ++i)
    {
        uint16_t dbyte = data[i];
        *(uint16_t *)crc ^= dbyte << 8;
        for (uint8_t j = 0; j < 8; ++j)
        {
            uint16_t mix = *(uint16_t *)crc & 0x8000;
            *(uint16_t *)crc = (*(uint16_t *)crc << 1);
            if (mix)
                *(uint16_t *)crc = *(uint16_t *)crc ^ 0x0007;
        }
    }
#endif
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
    s_eraseinit.Banks = FLASH_BANK_2;
    s_eraseinit.Page = FLASH_PAGE_NB - 1;
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
    // to do that we have to backup current values by copying it into RAM
    uint32_t mask = 0xFFFFFFFFU ^ (PAGE_SIZE-1);
    uint32_t page_addr = addr & mask; // find the first address of the page

    uint8_t page_backup[PAGE_SIZE];
    memcpy(page_backup, (void *)page_addr, PAGE_SIZE);

    // Now we can erase the page
    LuosHAL_FlashEraseLuosMemoryInfo();

    // Then add input data into backuped value on RAM
    uint32_t RAMaddr = (addr - page_addr);
    memcpy(&page_backup[RAMaddr], data, size);

    // and copy it into flash
    HAL_FLASH_Unlock();

    // ST hal flash program function write data by uint64_t raw data
    for (uint32_t i = 0; i < PAGE_SIZE; i += sizeof(uint64_t))
    {
        while (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, i + page_addr, *(uint64_t *)(&page_backup[i])) != HAL_OK)
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
