#ifndef _UART_HEADR_
#define _UART_HEADR_
#include "stm32f4xx_driver.h"

#define USART_ENABLE                    (1U)
#define USART_DISABLE                   (0U)
#define UART_WORD_LENGTH_8_BITS         (0U)
#define UART_WORD_LENGTH_9_BITS         (1U)
#define UART_PARITY_ENABLE              (0U)
#define UART_PARITY_DISABLE             (1U)
#define UART_EVEN_PARITY                (0U)
#define UART_ODD_PARITY                 (1U)
#define UART_INTERUPT_ENABLE            (1U)
#define UART_INTERUPT_DISABLE           (0U)


#define STOP_BIT_1                      (0U)
#define STOP_BIT_0_5                    (1U)
#define STOP_BIT_2                      (2U)
#define STOP_BIT_1_5                    (3U)


/* bit position */
#define UART_TC_BIT_POS                                 (6U)
#define UART_TE_BIT_POS                                 (3U)
#define UART_UE_BIT_POS                                 (13U)
#define UART_PARITY_ERROR_BIT_POS                       (0U)
#define UART_FRAMING_ERROR_BIT_POS                      (1U)
#define UART_RXNE_BIT_POS                               (5U)
#define UART_TRANSMISSION_CMPLT_BIT_POS                 (6U)
#define UART_TXE_BIT_POS                                (7U)
#define UART_TRANSMITTER_ENABLE_BIT_POS                 (3U)
#define UART_RECEIVER_ENABLE_BIT_POS                    (2U)
#define UART_STOP_BITS_BIT_POS                          (12U)
#define UART_USART_ENABLE_BIT_POS                       (13U)
#define UART_M_BIT_POS                                  (12U)
#define UART_PCE_BIT_POS                                (10U)
#define UART_PARITY_SELECTION_BIT_POS                   (9U)
#define UART_MANTISSA_POS                               (4U)
#define UART_OVER8_BIT_POS                              (15U)
#define UART_FRACTION_POS                               (0U)

#define UART_BUSY_IN_TX                                 (0U)
#define UART_BUSY_IN_RX                                 (1U)
#define UART_READY                                      (2U)

#define UART4_IRQ_NUMBER                                (52U)
#define UART5_IRQ_NUMBER                                (53U)
#define UART4_DEFAULT_PRIORITY                          (59U)
#define UART5_DEFAULT_PRIORITY                          (60U)

typedef struct 
{
    /* data */
    __vo uint32_t SR;
    __vo uint32_t DR;
    __vo uint32_t BRR;
    __vo uint32_t CR1;
    __vo uint32_t CR2;
    __vo uint32_t CR3;
    __vo uint32_t GTPR;
}UART_RegDef_t;
typedef struct 
{
    /* data */
    uint8_t USARTEnable;
    uint8_t WordLength;
    uint8_t Parity;
    uint8_t ParitySelection;
    uint8_t NumberOfStopBit;
    uint32_t BaudRate;
}USART_Config_t;

typedef struct 
{
    /* data */
    UART_RegDef_t *USARTx;
    USART_Config_t UART_Config;
    uint8_t *TxBuffer;
    uint8_t TxLength;
    uint8_t TxState;
}USART_Handle_t;
void Control_USART(UART_RegDef_t *USARTx,uint8_t EnOrDi);
void USART_Init(USART_Handle_t *USART_Handle);
void USART_PeriClockControl(UART_RegDef_t *USARTx,uint8_t EnorDi);

void USART_TransmitData(USART_Handle_t *USART_Handle_t,uint8_t *buffer,uint8_t Length);
uint8_t USART_TransmitDataIT(USART_Handle_t *USART_Handle_t,uint8_t *buffer,uint8_t Length);
void USART_Handling(USART_Handle_t *USART_Handle_t);
void USART_ReceiveData(USART_Handle_t *USART_Handle_t,uint8_t *buffer,uint8_t Length);
void USART_ReceiveDataIT(USART_Handle_t *USART_Handle_t,uint8_t *buffer,uint8_t Length);

void USART_InteruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
/*  */
void USART_IrqHandling(uint8_t PinNum);

void USART_PriorityConfig(uint8_t, uint8_t);




#endif
