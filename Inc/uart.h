/*
 * uart.h
 * */

#ifndef UART_H_
#define UART_H_

#include <stm32f401xe.h>

#define GPIOAEN         (1U<<0)
#define UART2EN         (1U<<17)
#define CR1_TE          (1U<<3)
#define CR1_RE          (1U<<2)
#define CR1_UE          (1U<<13)
#define SR_TXE          (1U<<7)
#define SR_RXNE         (1U<<5)
#define CR1_RXNEIE      (1U<<5)
#define SYS_FREQ        16000000
#define APB1_CLK        SYS_FREQ
#define UART_BAUDREATE  115200

void uart2_rxtx_init(void);
void uart2_rx_interrupt_init(void);
void uart2_tx_init(void);
char uart2_read(void);

#endif /** UART_H_ */
