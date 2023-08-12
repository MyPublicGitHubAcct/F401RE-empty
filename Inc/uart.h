/*
 * uart.h
 * */

#ifndef UART_H_
#define UART_H_

#include <stm32f401xe.h>
#include <stdint.h>

#define GPIOAEN               (1U<<0)
#define UART2EN               (1U<<17)
#define CR1_TE                (1U<<3)
#define CR1_RE                (1U<<2)
#define CR1_UE                (1U<<13)
#define SR_TXE                (1U<<7)

#define SR_RXNE               (1U<<5)
#define CR1_RXNEIE            (1U<<5)

#define DMA1EN                (1U<<21)
#define CHSEL4                (1U<<27)
#define DMA_MEM_INC           (1U<<10)
#define DMA_DIR_MEM_TO_PERIPH (1U<<6)
#define DMA_CR_TCIE           (1U<<4)
#define DMA_CR_EN             (1U<<0)
#define UART_CR3_DMAT         (1U<<7)

#define SYS_FREQ              16000000
#define APB1_CLK              SYS_FREQ
#define UART_BAUDREATE        115200

void dma1_stream6_init(uint32_t src, uint32_t dst, uint32_t len);
void uart2_rxtx_init(void);
void uart2_rx_interrupt_init(void);
void uart2_tx_init(void);
char uart2_read(void);

#endif /** UART_H_ */
