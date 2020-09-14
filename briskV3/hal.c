#include "hal.h"
#include "reception.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include "detection.h"

volatile unsigned char *crc_ptr;

/**
 * \fn void USART1_IRQHandler(void)
 * \brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
 *
 */
void USART3_IRQHandler(void)
{
    // check if we receive a data
    if ((LL_USART_IsActiveFlag_RXNE(USART3) != RESET) && (LL_USART_IsEnabledIT_RXNE(USART3) != RESET))
    {
        uint8_t data = LL_USART_ReceiveData8(USART3);
        ctx.data_cb(&data); // send reception byte to state machine
                            //return;
    }
    // Check if a timeout on reception occure
    if ((LL_USART_IsActiveFlag_RTO(USART3) != RESET) && (LL_USART_IsEnabledIT_RTO(USART3) != RESET))
    {
        if (ctx.tx_lock)
        {
            timeout();
        }
        else
        {
            //ERROR
        }
        LL_USART_ClearFlag_RTO(USART3);
        LL_USART_SetRxTimeout(USART3, TIMEOUT_VAL * (8 + 1 + 1));
        //return;
    }
    USART3->ICR = 0xFFFF;
}

char HAL_is_tx_lock(void)
{
    if (ctx.tx_lock)
    {
        return 1;
    }
    else
    {
        return (READ_BIT(USART1->ISR, USART_ISR_BUSY) == (USART_ISR_BUSY));
    }
}

/**
 * \fn HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 * \brief PTP interrupt management
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ROBUS_PTPA_Pin)
    {
        ptp_handler(BRANCH_A);
        return;
    }
    if (GPIO_Pin == ROBUS_PTPB_Pin)
    {
        ptp_handler(BRANCH_B);
        return;
    }
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
    CRC_HandleTypeDef hcrc;
    hcrc.Instance = CRC;
    hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
    hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
    hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
    hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
    unsigned short calc;
    if (size > 1)
    {
        calc = (unsigned short)HAL_CRC_Calculate(&hcrc, data, size);
    }
    else
    {
        calc = (unsigned short)HAL_CRC_Accumulate(&hcrc, data, 1);
    }
    crc[0] = (unsigned char)calc;
    crc[1] = (unsigned char)(calc >> 8);
}

void reverse_detection(branch_t branch)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // reverse the detection edge
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    if (branch == BRANCH_A)
    {
        GPIO_InitStruct.Pin = ROBUS_PTPA_Pin;
        HAL_GPIO_Init(ROBUS_PTPA_GPIO_Port, &GPIO_InitStruct);
    }
    else if (branch == BRANCH_B)
    {
        GPIO_InitStruct.Pin = ROBUS_PTPB_Pin;
        HAL_GPIO_Init(ROBUS_PTPB_GPIO_Port, &GPIO_InitStruct);
    }
}

void set_PTP(branch_t branch)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Clean edge/state detection and set the PTP pin as output
    if (branch == BRANCH_A)
    {
        GPIO_InitStruct.Pin = ROBUS_PTPA_Pin;
        HAL_GPIO_Init(ROBUS_PTPA_GPIO_Port, &GPIO_InitStruct);
        HAL_GPIO_WritePin(ROBUS_PTPA_GPIO_Port, ROBUS_PTPA_Pin, GPIO_PIN_SET); // Reset the PTPA pin
    }
    else if (branch == BRANCH_B)
    {
        GPIO_InitStruct.Pin = ROBUS_PTPB_Pin;
        HAL_GPIO_Init(ROBUS_PTPB_GPIO_Port, &GPIO_InitStruct);
        HAL_GPIO_WritePin(ROBUS_PTPB_GPIO_Port, ROBUS_PTPB_Pin, GPIO_PIN_SET); // Reset the PTPB pin
    }
}

void reset_PTP(branch_t branch)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    if (branch == BRANCH_A)
    {
        // set the PTPA pin as input pull-up IRQ triggered on falling edge event
        __HAL_GPIO_EXTI_CLEAR_IT(ROBUS_PTPA_Pin);
        GPIO_InitStruct.Pin = ROBUS_PTPA_Pin;
        HAL_GPIO_Init(ROBUS_PTPA_GPIO_Port, &GPIO_InitStruct);
    }
    else if (branch == BRANCH_B)
    {
        // set the PTPB pin as input pull-up IRQ triggered on falling edge event
        __HAL_GPIO_EXTI_CLEAR_IT(ROBUS_PTPB_Pin);
        GPIO_InitStruct.Pin = ROBUS_PTPB_Pin;
        HAL_GPIO_Init(ROBUS_PTPB_GPIO_Port, &GPIO_InitStruct);
    }
}

void set_baudrate(unsigned int baudrate)
{
    LL_USART_Disable(USART3);
    LL_USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = baudrate;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;

    LL_USART_Init(USART3, &USART_InitStruct);
    LL_USART_Enable(USART3);
}

/**
 * \fn void hal_init(void)
 * \brief hardware configuration (clock, communication, DMA...)
 */
void hal_init(void)
{
    // Serial init
    // Enable Reception interrupt
    LL_USART_EnableIT_RXNE(USART3);
    // Enable Reception timeout interrupt
    // the timeout expressed in nb of bits duration
    LL_USART_EnableRxTimeout(USART3);
    LL_USART_EnableIT_RTO(USART3);
    LL_USART_SetRxTimeout(USART3, TIMEOUT_VAL * (8 + 1 + 1));
    // Setup Robus baudrate
    set_baudrate(DEFAULTBAUDRATE);
    // Setup data direction
    HAL_GPIO_WritePin(ROBUS_DE_GPIO_Port, ROBUS_DE_Pin, GPIO_PIN_RESET); // Disable emitter | Enable Receiver only - Hardware DE impossible
    // Setup pull ups pins
    HAL_GPIO_WritePin(RS485_LVL_UP_GPIO_Port, RS485_LVL_UP_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RS485_LVL_DOWN_GPIO_Port, RS485_LVL_DOWN_Pin, GPIO_PIN_RESET);

    // setup crc
    CRC_HandleTypeDef hcrc;
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
        Error_Handler();
    }

    // Setup PTP lines
    reset_PTP(BRANCH_A);
    reset_PTP(BRANCH_B);
    reset_detection();
    HAL_NVIC_EnableIRQ(USART3_IRQn);
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
        while (!LL_USART_IsActiveFlag_TXE(USART3))
        {
        }
        if (ctx.collision)
        {
            // There is a collision
            ctx.collision = FALSE;
            return 1;
        }
        LL_USART_TransmitData8(USART3, *(data + i));
    }
    return 0;
}

void hal_wait_transmit_end(void)
{
    while (!LL_USART_IsActiveFlag_TC(USART3))
        ;
}

unsigned char get_PTP(branch_t branch)
{

    if (branch == BRANCH_A)
    {
        return (HAL_GPIO_ReadPin(ROBUS_PTPA_GPIO_Port, ROBUS_PTPA_Pin));
    }
    else if (branch == BRANCH_B)
    {
        return (HAL_GPIO_ReadPin(ROBUS_PTPB_GPIO_Port, ROBUS_PTPB_Pin));
    }
    return 0;
}

/**
 * \fn void hal_disable_irq(void)
 * \brief disable IRQ
 *
 * \return error
 */
void hal_disable_irq(void)
{
    __disable_irq();
}

/**
 * \fn void hal_enable_irq(void)
 * \brief enable IRQ
 *
 * \return error
 */
void hal_enable_irq(void)
{
    __enable_irq();
}

/**
 * \fn void hal_enable_rx(void)
 * \brief enable RX hard channel
 *
 * \return error
 */
void hal_enable_rx(void)
{
    HAL_GPIO_WritePin(ROBUS_RE_GPIO_Port, ROBUS_RE_Pin, GPIO_PIN_RESET);
}

/**
 * \fn void hal_disable_rx(void)
 * \brief disable RX hard channel
 *
 * \return error
 */
void hal_disable_rx(void)
{
    HAL_GPIO_WritePin(ROBUS_RE_GPIO_Port, ROBUS_RE_Pin, GPIO_PIN_SET);
}

/**
 * \fn void hal_enable_tx(void)
 * \brief enable TX hard channel
 *
 * \return error
 */
void hal_enable_tx(void)
{
    HAL_GPIO_WritePin(ROBUS_DE_GPIO_Port, ROBUS_DE_Pin, GPIO_PIN_SET);
}

/**
 * \fn void hal_disable_tx(void)
 * \brief disable TX hard channel
 *
 * \return error
 */
void hal_disable_tx(void)
{
    HAL_GPIO_WritePin(ROBUS_DE_GPIO_Port, ROBUS_DE_Pin, GPIO_PIN_RESET);
}
