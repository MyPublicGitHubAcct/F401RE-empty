/*
 * uart.c
 * */

#include "uart.h"
#include <stdint.h>

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t periphClk, uint32_t baudRate);
static uint16_t compute_uart_bd(uint32_t periphClk, uint32_t baudRate);

void uart2_write(int ch);

int __io_putchar(int ch)
{
	uart2_write(ch);
	return ch;
}

void dma1_stream6_init(uint32_t src, uint32_t dst, uint32_t len)
{
	/** enable clock access to DMA1 */
	RCC->AHB1ENR |= DMA1EN;

	/** disable the DMA1, stream 6 */
	DMA1_Stream6->CR &=~DMA_CR_EN;

	/** wait until DMA1 stream 6 is disabled */
	while(DMA1_Stream6->CR & DMA_CR_EN){}

	/** declare interrupt flags for DMA1 stream 6 */
	DMA1->HIFCR |= (1U<<16);
	DMA1->HIFCR |= (1U<<18);
	DMA1->HIFCR |= (1U<<19);
	DMA1->HIFCR |= (1U<<20);
	DMA1->HIFCR |= (1U<<21);

	/** set the destination buffer */
	DMA1_Stream6->PAR = dst;

	/** set the source buffer */
	DMA1_Stream6->M0AR = src;

	/** set the length */
	DMA1_Stream6->NDTR = len;

	/** select the stream 6, channel 4 */
	DMA1_Stream6->CR = CHSEL4; // clear whole register and write this bit

	/** enable memory increment */
	DMA1_Stream6->CR |= DMA_MEM_INC;

	/** configure the transfer direction (memory to peripheral) */
	DMA1_Stream6->CR |= DMA_DIR_MEM_TO_PERIPH;

	/** enable DMA transfer complete interrupt */
	DMA1_Stream6->CR |= DMA_CR_TCIE;

	/** enable direct mode / disable the FIFO mode, enable direct mode */
	DMA1_Stream6->FCR = 0;  // write zero to whole register

	/** enable DMA1, stream 6 */
	DMA1_Stream6->CR |= DMA_CR_EN;

	/** enable UART2 transmitter DMA */
	USART2->CR3 |= UART_CR3_DMAT;

	/** enable DMA interrupt in NVIC */
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

void uart2_rxtx_init(void)
{
	/**************** configure the UART GIO pin **********/
    RCC->AHB1ENR |= GPIOAEN;

    /** Set PA2 mode to alternate function mode */
    GPIOA->MODER &=~(1U<<4);
    GPIOA->MODER |= (1U<<5);

    /** Set PA2 alternate function type to UART_TX (AF07) = 0111 */
    GPIOA->AFR[0] |= (1U<<8);
    GPIOA->AFR[0] |= (1U<<9);
    GPIOA->AFR[0] |= (1U<<10);
    GPIOA->AFR[0] &=~(1U<<11);

    /** Set PA3 mode to alternate function mode */
    GPIOA->MODER &=~(1U<<6);
    GPIOA->MODER |= (1U<<7);

    /** Set PA3 alternate function type to UART_RX (AF07) = 0111 */
    GPIOA->AFR[0] |= (1U<<12);
    GPIOA->AFR[0] |= (1U<<13);
    GPIOA->AFR[0] |= (1U<<14);
    GPIOA->AFR[0] &=~(1U<<15);

    /**************** Configure uart module ***************/
    RCC->APB1ENR |= UART2EN;

    /** Configure baudrate */
    uart_set_baudrate(USART2, APB1_CLK, UART_BAUDREATE);

    /** Configure the transfer direction */
    USART2->CR1 = (CR1_TE | CR1_RE); // use or operation to allow both;

    /** Enable uart module */
    USART2->CR1 |= CR1_UE;
}

void uart2_rx_interrupt_init(void)
{
	/**************** configure the UART GIO pin **********/
    RCC->AHB1ENR |= GPIOAEN;

    /** Set PA2 mode to alternate function mode */
    GPIOA->MODER &=~(1U<<4);
    GPIOA->MODER |= (1U<<5);

    /** Set PA2 alternate function type to UART_TX (AF07) = 0111 */
    GPIOA->AFR[0] |= (1U<<8);
    GPIOA->AFR[0] |= (1U<<9);
    GPIOA->AFR[0] |= (1U<<10);
    GPIOA->AFR[0] &=~(1U<<11);

    /** Set PA3 mode to alternate function mode */
    GPIOA->MODER &=~(1U<<6);
    GPIOA->MODER |= (1U<<7);

    /** Set PA3 alternate function type to UART_RX (AF07) = 0111 */
    GPIOA->AFR[0] |= (1U<<12);
    GPIOA->AFR[0] |= (1U<<13);
    GPIOA->AFR[0] |= (1U<<14);
    GPIOA->AFR[0] &=~(1U<<15);

    /**************** Configure uart module ***************/
    RCC->APB1ENR |= UART2EN;

    /** Configure baudrate */
    uart_set_baudrate(USART2, APB1_CLK, UART_BAUDREATE);

    /** Configure the transfer direction */
    USART2->CR1 = (CR1_TE | CR1_RE); // use 'or' operation to allow both
    USART2->CR1 |= CR1_RXNEIE;       // enable the RXNE interrupt

    /*Enable UART2 interrupt in NVIC*/
    NVIC_EnableIRQ(USART2_IRQn);

    /** Enable uart module */
    USART2->CR1 |= CR1_UE;
}

void uart2_tx_init(void)
{
	/**************** Configure uart gpio pin ***************/
	/** Enable clock access to gpioa */
	RCC->AHB1ENR |= GPIOAEN;

	/** Set PA2 mode to alternate function mode */
	GPIOA->MODER &=~(1U<<4);
	GPIOA->MODER |= (1U<<5);

	/** Set PA2 alternate function type to UART_TX (AF07) */
	GPIOA->AFR[0] |= (1U<<8);
	GPIOA->AFR[0] |= (1U<<9);
	GPIOA->AFR[0] |= (1U<<10);
	GPIOA->AFR[0] &= ~(1U<<11);

	/**************** Configure uart module ***************/
	/** Enable clock access to uart2 */
	RCC->APB1ENR |= UART2EN;

	/*Configure baudrate*/
	uart_set_baudrate(USART2, APB1_CLK, UART_BAUDREATE);

	/** Configure the transfer direction */
	 USART2->CR1 =  CR1_TE;

	/** Enable uart module */
	 USART2->CR1 |= CR1_UE;
}

char uart2_read(void)
{
    /** Make sure the receive data register is not empty */
    while(!(USART2->SR & SR_RXNE)){}

    /** Read the data */
    return USART2->DR;
}

void uart2_write(int ch)
{
    /** Make sure the transmit data register is empty */
    while(!(USART2->SR & SR_TXE)){}

    /** Write to the transmit data register */
    USART2->DR = (ch & 0xFF);
}

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t periphClk, uint32_t baudRate)
{
	USARTx->BRR = compute_uart_bd(periphClk, baudRate);
}

static uint16_t compute_uart_bd(uint32_t periphClk, uint32_t baudRate)
{
	return ((periphClk + (baudRate / 2U)) / baudRate);
}

