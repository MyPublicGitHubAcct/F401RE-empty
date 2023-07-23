/*
 * uart.h
 * */

#ifndef UART_H_
#define UART_H_

#include <stm32f401xe.h>

void uart2_rxtx_init(void);
void uart2_tx_init(void);
char uart2_read(void);

#endif /** UART_H_ */
