/**
 * Where is the LED?
 * User guide for Nucleo board (um1724-stm32-nucleo64-boards-mb1136.pdf)
 * states 'PA5', or port (A) and pin (5).
 *
 * To identify the memory location needed to get to PortA, look in the
 * microcontroller's data sheet (stm32f401re.pdf).
 *
 * Use the reference manual (rm0368-stm32f401x.pdf) to identify specific
 * registers & pins e.g., the RCC enable register.
 *
 * Videos 5, 6, 7, 10 are important to understanding the use of pins.
 *
 * GPIO modules have at least 2 registers:
 * - Direction register (set pin as input or output)
 * - Data register (write to or read from pin)
 *
 * Two types of buses:
 * - Advanced High-Performance Bus (AHB) - 1 clock cycle to access peripherals
 * - Advanced Peripheral Bus (APB) - minimum of 2 clock cycles to access peripherals
 *
 * Clock Sources
 * - On-chip RC Oscillator - least precise
 * - Externally Connected Crystal - Most precise
 * - Phase Locked Loop (PLL) - Programmable
 *
 * PA5 or PB13
 *
 * Offsets are from the data sheet.
 *
 * Need to type-cast the defines to volatile unsigned integer to avoid
 * the compiler optimizing them away.
 *
 * AHB1  0x4002 0000 - 0x4FFF FFFF
 * GPIOA 0x4002 0000 - 0x4002 03FF
 * GPIOB 0x4002 0400 - 0x4002 07FF
 * RCC   0x4002 3800 - 0x4002 3BFF
 *
 * */

/** VERSION 1 */

//#define PERIPH_BASE				(0x40000000UL)
//
//#define AHB1PERIPH_OFFSET		(0x00020000UL)
//#define AHB1PERIPH_BASE	        (PERIPH_BASE + AHB1PERIPH_OFFSET)
//
//#define GPIOA_OFFSET			(0x0000UL)
//#define GPIOA_BASE				(AHB1PERIPH_BASE + GPIOA_OFFSET)
//
//#define RCC_OFFSET				(0x3800UL)
//#define RCC_BASE				(AHB1PERIPH_BASE + RCC_OFFSET)
//
//#define AHB1EN_R_OFFSET			(0x30UL)
//#define RCC_AHB1EN_R            (*(volatile unsigned int *)(RCC_BASE +  AHB1EN_R_OFFSET))
//
//#define MODE_R_OFFSET			(0x00UL)
//#define GPIOA_MODE_R			(*(volatile unsigned int *)(GPIOA_BASE + MODE_R_OFFSET))
//
//#define OD_R_OFFSET				(0x14UL)
//#define GPIOA_OD_R				(*(volatile unsigned int *)(GPIOA_BASE +  OD_R_OFFSET))
//
//#define  GPIOAEN				(1U<<0) //   0b 0000 0000 0000 0000 0000 0000 0000 0001
//
//#define PIN5					(1U<<5)
//#define LED_PIN					 PIN5
//
//
//int main(void)
//{
//	// enable clock access to GPIOA
//	RCC_AHB1EN_R  |=  GPIOAEN;
//
//	// set PA5 as an output pin
//	GPIOA_MODE_R  |= (1U<<10);  // Set bit 10 to 1
//	GPIOA_MODE_R  &=~(1U<<11); //  Set bit 11 to 0
//
//    while(1)
//    {
//    	// set PA5 high
//    	//GPIOA_OD_R |= LED_PIN;
//
//    	GPIOA_OD_R ^= LED_PIN;
//    	for(int i=0; i<1000000; i++){}
//    }
//}


/** Version 3
 *
 * Search on STM32F4 at st.com and download the STM32CubeF4 firmware
 * package on the Tools and Software tab. Alternatively this can be
 * found on GitHub here -> https://github.com/STMicroelectronics/STM32CubeF4
 * _But_, the repo does not include some of the middleware due to licensing.
 * Either way, this is about 2 Gb so in this project, put only the essential
 * files in the 'Inc' folder.
 *
 * In this project, includes can be either option 1 or 2 (used) as below.
 * Option 2 allows for better intellisense in CubeIDE.
 * */

//#define STM32F401xE			1
//#include <stm32f4xx.h>

//#include <stm32f401xe.h>
//
//#define GPIOAEN 			(1U<<0)
//#define PIN5				(1U<<5)
//#define LED_PIN				PIN5
//
//int main(void)
//{
//	RCC->AHB1ENR |= GPIOAEN;  // enable clock access to GPIOA
//	GPIOA->MODER |= (1U<<10); // Set bit 10 to 1
//	GPIOA->MODER &=~(1U<<11); // Set bit 11 to 0
//
//	while (1)
//	{
//		GPIOA->ODR ^=LED_PIN;
//		for(int i=0; i<1000000; i++){}
//	}
//}

/** Version 4 - Using the bit set reset register */

//#include <stm32f401xe.h>
//
//#define GPIOAEN 			(1U<<0)
//#define PIN5_ON				(1U<<5)
//#define PIN5_OFF			(1U<<21)
//
//const int TIME = 1000000;
//
//int main(void)
//{
//	RCC->AHB1ENR |= GPIOAEN;  // enable clock access to GPIOA
//	// set PA5 as an output pin - mode for pin 5 is General Purpose Output (01)
//	GPIOA->MODER |= (1U<<10); // Set bit 10 to 1
//	GPIOA->MODER &=~(1U<<11); // Set bit 11 to 0
//
//	while (1)
//	{
//		GPIOA->BSRR = PIN5_ON;
//		for(int i=0; i<TIME; i++){}
//
//		GPIOA->BSRR = PIN5_OFF;
//		for(int i=0; i<TIME; i++){}
//	}
//}

/** Version 5 - Write the GPIO input driver */
// Want to use the user push button as an input. When it is pressed, the
// LED turns off.
//
// The user push button is connected to I/O PC13. First, we need to give
// it clock access. To do that, use the bus information from the data sheet.
// We find we can use AHB1 (AHB1ENR), which is bit 2.

//#include <stm32f401xe.h>
//
//#define GPIOAEN        (1U<<0) // shift 1 to position 0
//#define GPIOCEN        (1U<<2) // shift 1 to position 2
//
//#define PIN5_ON        (1U<<5)  // LED on
//#define PIN5_OFF       (1U<<21) // LED off (reset)
//#define PIN13          (1U<<13) // button on
//
//int main(void)
//{
//	/** Enable clock access to GPIOA and GPIOC */
//	RCC->AHB1ENR |= GPIOAEN;
//	RCC->AHB1ENR |= GPIOCEN;
//
//	/** Set PA5 as output pin (MODER registers 10 and 11) */
//	GPIOA->MODER |= (1U<<10); // Set bit 10 to 1
//	GPIOA->MODER &=~(1U<<11); // Set bit 11 to 0
//
//	/** Set PA13 as input pin (MODER registers 26 and 27) */
//	GPIOC->MODER &=~(1U<<26); // Set bit 26 to 0
//	GPIOC->MODER &=~(1U<<27); // Set bit 27 to 0
//
//
//	while (1)
//	{
//		/** Check if the button (PC13) is pressed (low) */
//		if(GPIOC->IDR & PIN13)
//		{
//			GPIOA->BSRR = PIN5_ON;
//		}
//		else
//		{
//			GPIOA->BSRR = PIN5_OFF;
//		}
//	}
//}

/** Version 6 - USART */

/**
 * Universal Asynchronous Receiver/Transmitter (UART) is a serial
 * (i.e., one bit at a time) communication method. Whereas synchronous
 * methods transmit clock with data, UART and other asynchronous methods
 * only transmit data. To accomplish this, the transmitter and receiver
 * must agree on the clock speed (baudrate).
 *
 * The STM32 microcontroller's UART module can act as an UART or as an
 * USART (Universal Synchronous Asynchronous Receiver/Transmitter).
 * It can be used either way.
 *
 * __Transmission modes__
 *
 * - _Duplex_ - data can be transmitted and received.
 * - _Simplex_ - data can be transmitted or received, not both.
 * - _Half Duplex_ - data can be transmitted one way at a time.
 * - _Full Duplex_ - data can be transmitted both ways at the same time.
 *
 * In asynchronous transmission, each byte (character) is packed between
 * start and stop bits. The _start bit_ is always 1 bit whose value is
 * always 0. The _stop bit_ can be 1 or 2 bits and its value is always 1.
 *
 * Example: One frame (read from the right) with data "0100 0001"
 * ```11 0100 0001 0```.
 *
 * __UART Configuration Parameters__
 * _Baudrate_ - connection speed expressed in bits per second.
 * _Stop Bit_ - number of stop bits transmitted, can be one or two.
 * _Parity_ - indicates the parity mode, odd or even. Used for error checking.
 * _Mode_ - specifies whether RX or TX mode is enabled (or disabled).
 * _Word Length_ - the number of bits transmitted/received. Can be 8 or 9.
 * _Hardware Flow Control_ - whether this is enabled or disabled.
 * */

// USART2 is connected to USB, which is connected to the computer.

//#include <stdint.h>
//#include <stm32f401xe.h>
//
//#define GPIOAEN         (1U<<0)  // shift 1 to position 0
//#define UART2EN         (1U<<17) // enable UART
//#define CR1_TE          (1U<<3)
//#define CR1_UE          (1U<<13)
//#define SR_TXE          (1U<<7)
//#define SYS_FREQ        16000000
//#define APB1_CLK        SYS_FREQ
//#define UART_BAUDREATE  115200
//
//static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t periphClk, uint32_t baudRate);
//static uint16_t compute_uart_bd(uint32_t periphClk, uint32_t baudRate);
//
//void uar2_tx_init(void);
//void uart2_write(int ch);
//
//int main(void)
//{
//	uar2_tx_init();
//
//	while (1)
//	{
//		uart2_write('Y');
//	}
//}
//
//void uar2_tx_init(void)
//{
//    /**************** configure the UART GIO pin **********/
//    RCC->AHB1ENR |= GPIOAEN; // Enable clock access to GPIOA
//    GPIOA->MODER &=~(1U<<4); // Set PA2 mode to alternate function mode
//    GPIOA->MODER |= (1U<<5); // Set PA2 mode to alternate function mode
//
//    // Set PA2 alternate function type to UART_TX (AF07) = 0111
//    GPIOA->AFR[0] |= (1U<<8);
//    GPIOA->AFR[0] |= (1U<<9);
//    GPIOA->AFR[0] |= (1U<<10);
//    GPIOA->AFR[0] &=~(1U<<11);
//
//    /**************** configure UART module ***************/
//    RCC->APB1ENR |= UART2EN; // enable clock access to UART2
//    uart_set_baudrate(USART2, APB1_CLK, UART_BAUDREATE); // configure baudrate
//    USART2->CR1 = CR1_TE; // configure transfer direction (set all intentionally)
//    USART2->CR1 |= CR1_UE; // enable the UART module (bit 13)
//}
//
//void uart2_write(int ch)
//{
//    /** Make sure transmit data register is empty (TXE). ```USART2->SR & SR_TXE```
//     *  will return true if the bit is set. Otherwise, it will loop, effectively
//     *  waiting for something to write.
//     *  */
//    while(!(USART2->SR & SR_TXE)){}
//    /** Write to transmit data register
//     *  Note: the DR register is 9-bit long and we are interested in the first
//     *  8 bits so we mask ch with 0xFF (255 in decimal, or 16*16).
//     * */
//    USART2->DR = (ch & 0xFF);
//}
//
//static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t periphClk, uint32_t baudRate)
//{
//	USARTx->BRR = compute_uart_bd(periphClk, baudRate);
//}
//
//static uint16_t compute_uart_bd(uint32_t periphClk, uint32_t baudRate)
//{
//	return ((periphClk + (baudRate / 2U)) / baudRate);
//}

//Notes from Version 6...
//
//
//The answer from another course:
//
//1. I now understand WHAT the USART_BRR is, what it's for, and how/why to program it. It took TONS of reading on different sites and numerous passes thru the debugger until it all FINALLY became clear!!!
//
//2. I now FULLY understand that the formula Israel used in the compute_uart_bd(..) function IS NOT BASED, AT ALL, ON THE formula presented in the Reference Manual on PG:518 Section 19.3.4. However he did derive/create that formula -->what it is doing is calculating the decimal number required to set ALL 16 bits of the USART_BRR. But it's a "MAGIC" formula since it is not actually doing the calculation(s) required to derive the USART:Mantissa and the USART:Fraction which are the 2 values programmed into the USART_BRR register.
//2a. Isreal's formula, of course, works...but we (the class audience) have NO IDEA how he derived that formula in order to come up with a value that sets ALL 16 BITS [15:0] of the USART_BRR ALL AT ONCE!!!
//3. The following is a replacement to the uart_set_baudrate() function that Isreal presented in class. It does away with the call to the "MAGIC" function compute_uart_bd(..)
//3a. It is fully commented so ANYBODY can understand how/why the USART_BRR is being programmed as it is.
//3b. I'm sure the steps presented can be consolidated into a more efficient function but I programmed it the way I did so it's a "show your work" kind of function --- allowing anybody reading it to both understand the algorithm presented from start to finish...with all intermediate steps.
//3c. For clarity all actual code lines are in bold and all comments are non-bold but start with //

////=============================================================================
//// functions to calculate and then set the USART_BRR (baud rate register)
//// RefMan Pg:550 Sec:19.6.3 USART_BRR (Baud Rate Register)
//static void uart_set_baudrate(uint32_t periph_clk, uint32_t baudrate)
//{
//double mantissa;
//double fraction;
//uint16_t USARTDIV_MANT;
//uint16_t USARTDIV_FRAC;
//
//// RefMan Pg:518 Sec:19.3.4 (Fractional Baud rate generation)
//// Using a value of OVER8 = 0 the formula posed in Equation #1 reduces to:
//// USARTDIV = Fck / 16 * baud
//// This reduced formula results in the following (for our project):
//// USARTDIV = 16000000 / 16 * 115200 (16Mhz and baudrate of 115200)
//// USARTDIV = 8.6806
//// This makes the mantissa (whole portion) portion of USARTDIV = 8
//// This makes the fractional portion of USARTDIV .6806
//// Turning .6806 into a 16 bit number: .6806 * 16 = 10.8896 which
//// rounded to the next whole number is = 11
//// The mantissa binary value (8) is: 0000 0000 1000 (bits 11:0 of the USART_BRR)
//// The fractional binary value (11) is: 1011 (bits 3:0 of the USART_BRR
//// Bottom Line: We need to set the USART_BRR to:
//// 0b 0000 0000 0000 0000 0000 0000 1000 1011 or 0x8b
//// See: RefMan Pg 550 Sec:19.6.3 (Baud Rate Register USART_BRR)
//// Doing this programatically as follows:
//// Step #1: Calculate the mantissa using the above formula and store in a temp double
//mantissa = (double)periph_clk / (double)(baudrate * 16);
//// Step #2: Calculate the fraction using the above formula and store in a temp double|
//fraction = ((mantissa - ((long)mantissa))) * 16;
//// Step #3: Round the fraction double up if fractional portion of fraction is > .50
//if (fraction - ((long)fraction) >= .5)
//fraction++;
//// Step #4: store the mantissa into a uint16_t value (USARTDIV_MANT)
//// which stores only the whole portion of the mantissa double
//USARTDIV_MANT = mantissa;
//// Step #5: store the fraction into a unit16_t value (USARTDIV_FRACT
//// which stores only the whole portion of the fraction double
//USARTDIV_FRAC = fraction;
//// Step #6: set the 1st 4 bits [3:0] of the USART_BRR register to USARTDIV's fraction
//USART2->BRR = (USARTDIV_FRAC << 0);
//// Step #7: set the next 12 bits [11:0] of the USART_BRR to USARTDIV's mantissa
//USART2->BRR |= (USARTDIV_MANT << 4);
//}
//============================================================================
//I have fully tested the above function in the 1-uart_driver project in the class and it works perfectly...setting the USART2 peripheral up for communications with a serial terminal!!!
//WRAPUP:
//I would highly recommend that the above be added to EVERY SINGLY CLASS where Israel uses his two functions: uart_set_baudrate(..) which calls the mysterious compute_uart_bd(..) function. Since he NEVER, EVER, explains what the USART_BRR does or how/why it's being programmed.....the above function, with the detailed comments, will provide clarity to all the students that are "mystified" by the formula presented in compute_uart_bd(..).
//I would still like to know/understand how Israel came up with the formula in compute_uart_bd(..) so that it derives the full 16 bit number, in one pass, required to set the USART_BRR. It's pretty cool how it works but IT IS NOT DERIVED FROM THE FORMULA IN THE REFMANUAL!!!



/** Version 7 - using fprint */

//#include <stdio.h>
//#include <stdint.h>
//#include <stm32f401xe.h>
//
//#define GPIOAEN         (1U<<0)
//#define UART2EN         (1U<<17)
//#define CR1_TE          (1U<<3)
//#define CR1_UE          (1U<<13)
//#define SR_TXE          (1U<<7)
//#define SYS_FREQ        16000000
//#define APB1_CLK        SYS_FREQ
//#define UART_BAUDREATE  115200
//
//static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t periphClk, uint32_t baudRate);
//static uint16_t compute_uart_bd(uint32_t periphClk, uint32_t baudRate);
//
//void uar2_tx_init(void);
//void uart2_write(int ch);
//
//int __io_putchar(int ch)
//{
//	uart2_write(ch);
//	return ch;
//}
//
//int main(void)
//{
//	uar2_tx_init();
//
//	while (1)
//	{
//		printf("hello!\n\r");
//	}
//}
//
//void uar2_tx_init(void)
//{
//    RCC->AHB1ENR |= GPIOAEN;
//    GPIOA->MODER &=~(1U<<4);
//    GPIOA->MODER |= (1U<<5);
//
//    GPIOA->AFR[0] |= (1U<<8);
//    GPIOA->AFR[0] |= (1U<<9);
//    GPIOA->AFR[0] |= (1U<<10);
//    GPIOA->AFR[0] &=~(1U<<11);
//
//    RCC->APB1ENR |= UART2EN;
//    uart_set_baudrate(USART2, APB1_CLK, UART_BAUDREATE);
//    USART2->CR1 = CR1_TE;
//    USART2->CR1 |= CR1_UE;
//}
//
//void uart2_write(int ch)
//{
//    while(!(USART2->SR & SR_TXE)){}
//    USART2->DR = (ch & 0xFF);
//}
//
//static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t periphClk, uint32_t baudRate)
//{
//	USARTx->BRR = compute_uart_bd(periphClk, baudRate);
//}
//
//static uint16_t compute_uart_bd(uint32_t periphClk, uint32_t baudRate)
//{
//	return ((periphClk + (baudRate / 2U)) / baudRate);
//}


/** Version 8 - making it modular */

//#include <stdio.h>
//#include <stm32f401xe.h>
//#include "uart.h"
//
//int main(void)
//{
//	uar2_tx_init();
//
//	while (1)
//	{
//		printf("hello!\n\r");
//	}
//}


/** Version 9 - UART Receiver Drive */

//#include <stdio.h>
//#include <stm32f401xe.h>
//#include "uart.h"
//
//#define GPIOAEN               (1U<<0)
//#define GPIOA_5               (1U<<5)
//#define LED_PIN               GPIOA_5
//
//char key;
//
//int main(void)
//{
//    RCC->AHB1ENR |= GPIOAEN;  // enable clock access to GPIOA
//    GPIOA->MODER |= (1U<<10); // set PA5 as output pin
//    GPIOA->MODER &=~(1U<<11); // set PA5 as output pin
//
//    uart2_rxtx_init();
//
//    while (1)
//    {
//        //printf("hello!\n\r");
//        key = uart2_read();
//        if(key=='1')
//        {
//            GPIOA->ODR |= LED_PIN;
//        }
//        else
//        {
//        	GPIOA->ODR &=~LED_PIN;
//        }
//    }
//}


/** Version 10 - Develop an ADC driver */

//#include <stdio.h>
//#include <stdint.h>
//#include <stm32f401xe.h>
//#include "uart.h"
//#include "adc.h"
//
//uint32_t sensor_value;
//
//int main(void)
//{
//    uart2_rxtx_init();
//    pa1_adc_init();
//
//    while (1)
//    {
//    	start_conversion();
//
//    	sensor_value = adc_read();
//    	printf("Sensor value = %d \n\r", (int)sensor_value);
//    }
//}

/** Version 11 - Develop a continuous ADC driver */

//#include <stdio.h>
//#include <stdint.h>
//#include <stm32f401xe.h>
//#include "uart.h"
//#include "adc.h"
//
//uint32_t sensor_value;
//
//int main(void)
//{
//    uart2_rxtx_init();
//    pa1_adc_init();
//
//    // to enable continuous conversion, remove otherwise
//    start_conversion();
//
//    while (1)
//    {
////    	start_conversion();  // un-comment if not continuous conversion
//
//    	sensor_value = adc_read();
//    	printf("Sensor value = %d \n\r", (int)sensor_value);
//    }
//}

/** Version 12 - Systick delay */

//#include <stdio.h>
//#include <stdint.h>
//#include <stm32f401xe.h>
//#include "uart.h"
//#include "adc.h"
//#include "systick.h"
//
//#define GPIOAEN                (1U<<0)
//#define PIN5                   (1U<<5)
//#define LED                    PIN5
//
//
//int main(void)
//{
//	RCC->AHB1ENR |= GPIOAEN;
//	GPIOA->MODER |= (1U<<10);
//	GPIOA->MODER &=~(1U<<11);
//
//    uart2_rxtx_init();
//
//    while (1)
//    {
//        printf("A second passed\n\r");
//        GPIOA->ODR ^= LED;     // toggle the led pin
//        systickDelayMs(1000);  // wait 1 second
//    }
//}

/** Version 13 - Timers */

//#include <stdio.h>
//#include <stdint.h>
//#include <stm32f401xe.h>
//#include "uart.h"
//#include "tim.h"
//
//#define GPIOAEN                (1U<<0)
//#define PIN5                   (1U<<5)
//#define LED                    PIN5
//
//int main(void)
//{
//	RCC->AHB1ENR |= GPIOAEN;
//	GPIOA->MODER |= (1U<<10);
//	GPIOA->MODER &=~(1U<<11);
//
//	uart2_tx_init();
//	tim2_1hz_init();
//
//    while (1)
//    {
//    	while(!(TIM2->SR & SR_UIF)){}
//    	TIM2->SR &=~SR_UIF;
//
//        printf("A second passed !! \n\r");
//        GPIOA->ODR ^= LED;     // toggle the led pin
//    }
//}

/** Version 14 - Timer output compare driver */

//#include <stm32f401xe.h>
//#include "tim.h"
//
//int main(void)
//{
//
//	tim2_pa5_output_compare();
//
//    while (1){}
//}


/* Version 15 - Timer input capture driver
 * This time, the timing of the LED blinking is determined by value of another pin.
 *
 * To test : connect a jumper wire from PA5 to PA6. on F401RE, this is
 * D13 and D12 respectively.
 * */

//#include <stm32f401xe.h>
//#include "tim.h"
//
//int timestamp = 0;
//
//int main(void)
//{
//
//	tim2_pa5_output_compare();
//	tim3_pa6_input_capture();
//
//    while (1)
//    {
//    	/** wait until edge is captured */
//    	while(!(TIM3->SR & SR_CC1IF)){}
//
//    	/** read captured value */
//    	timestamp =  TIM3->CCR1;
//    }
//}

/** Version 16 - use interrupt to print and toggle LED */

//#include <stm32f401xe.h>
//#include <stdio.h>
//#include "exti.h"
//#include "uart.h"
//
//#define GPIOAEN                (1U<<0)
//#define PIN5                   (1U<<5)
//#define LED                    PIN5
//
//static void exti_callback(void);
//
//int main(void)
//{
//	RCC->AHB1ENR |= GPIOAEN;
//	GPIOA->MODER |= (1U<<10);
//	GPIOA->MODER &=~(1U<<11);
//
//	uart2_tx_init();
//	pc13_exti_init();
//
//    while (1){}
//}
//
///* Need to implement the interrupt. We determine that this is the
// * correct one to implement by looking in the vector table in the startup
// * file (startup_stm32f401retx.s). This one handles EXTI interrupt lines 10
// * through 15. We are using interrupt EXTI 13.
// * EXTI->PR is the pending register. */
//void EXTI15_10_IRQHandler(void)
//{
//	if((EXTI->PR & LINE13) != 0)
//	{
//		/** Clear the PR flag */
//		EXTI->PR |= LINE13;
//
//		/** Do something */
//		exti_callback();
//	}
//}
//
//static void exti_callback(void)
//{
//	printf("BTN pressed...\n\r");
//	GPIOA->ODR ^=LED;
//}


/* Version 17 - Developing the UART Interrupt driver
 * To test, run this in debug, click on the play button,
 * connect CoolTerm, click in its text area, and press a
 *  key on the keyboard. That value should be displayed
 * in the Live Expressions area of STMCubeIDE.
 * */

//#include <stm32f401xe.h>
//#include <stdio.h>
//#include "uart.h"
//
//#define GPIOAEN                (1U<<0)
//#define GPIOA_5                (1U<<5)
//#define LED_PIN                GPIOA_5
//
//char key;
//
//static void uart_callback(void);
//
//int main(void)
//{
//	RCC->AHB1ENR |= GPIOAEN;
//	GPIOA->MODER |= (1U<<10);
//	GPIOA->MODER &=~(1U<<11);
//
//	uart2_rx_interrupt_init();
//
//    while (1){}
//}
//
//static void uart_callback(void)
//{
//	key = USART2->DR;
//
//	if(key == '1')
//	{
//		GPIOA->ODR |= LED_PIN;
//	}
//	else
//	{
//		GPIOA->ODR &=~LED_PIN;
//	}
//}
//
///** Because we are using an interrupt, we need to override
// *  the handler to meet our needs */
//void USART2_IRQHandler(void)
//{
//	/** Check if RXNE is set (read data register) */
//	if(USART2->SR & SR_RXNE)
//	{
//		uart_callback();
//	}
//}


/** Version 18 - Developing the ADC Interrupt driver
 * To test, run this in debug, click on the play button, connect CoolTerm.
 * The value of sensor_value (random numbers because nothing is connected
 * to ADC1) will be displayed in both the Live Expressions area of STMCubeIDE
 * as well as in CoolTerm.
 * */

//#include <stdio.h>
//#include <stdint.h>
//#include <stm32f401xe.h>
//#include "uart.h"
//#include "adc.h"
//
//static void adc_callback(void);
//uint32_t sensor_value;
//
//int main(void)
//{
//    uart2_tx_init();
//    pa1_adc_interrupt_init();
//    start_conversion();
//
//    while (1){}
//}
//
///** Using an interrupt allows us to avoid using blocking code
// * like while(!(ADC1->SR & SR_EOC)){} in adc_read() */
//static void adc_callback(void)
//{
//    //start_conversion();
//
//	sensor_value = ADC1->DR;  // don't need to call adc_read()
//	printf("Sensor value = %d \n\r", (int)sensor_value);
//}
//
///** The name of the handler can be found in the vector table */
//void ADC_IRQHandler(void)
//{
//	/** Check for End Of Conversion in SR */
//    if((ADC1->SR & SR_EOC) != 0)
//    {
//    	/** Clear EOC */
//    	ADC1->SR &=~SR_EOC;
//    	/** read the ADC */
//    	adc_callback();
//    }
//}


/** Version 19 Developing the Systick Interrupt driver */

//#include <stdio.h>
//#include <stm32f401xe.h>
//#include "uart.h"
//#include "systick.h"
//
//#define GPIOAEN                (1U<<0)
//#define PIN5                   (1U<<5)
//#define LED                    PIN5
//
//int main(void)
//{
//	RCC->AHB1ENR |= GPIOAEN;
//	GPIOA->MODER |= (1U<<10);
//	GPIOA->MODER &=~(1U<<11);
//
//    uart2_rxtx_init();
//    systick_1hz_interrupt();
//
//    while (1){}
//}
//
//static void systick_callback(void)
//{
//	printf("A second passed\n\r");
//	GPIOA->ODR ^= LED;     // toggle the led pin
//}
//
///** override handler */
//void SysTick_Handler(void)
//{
//	systick_callback();
//}

/** Version 20 Developing the Timer Interrupt driver */

//#include <stdio.h>
//#include <stdint.h>
//#include <stm32f401xe.h>
//#include "uart.h"
//#include "tim.h"
//
//#define GPIOAEN                (1U<<0)
//#define PIN5                   (1U<<5)
//#define LED                    PIN5
//
//void tim2_callback(void);
//
//int main(void)
//{
//	RCC->AHB1ENR |= GPIOAEN;
//	GPIOA->MODER |= (1U<<10);
//	GPIOA->MODER &=~(1U<<11);
//
//	uart2_tx_init();
//	tim2_1hz_interrupt_init();
//
//    while (1){}
//}
//
//void tim2_callback(void)
//{
//	printf("Another second passed !! \n\r");
//	GPIOA->ODR ^= LED;     // toggle the led pin
//}
//
///** implement interrupt request handler */
//void TIM2_IRQHandler(void)
//{
//	/** clear update interrupt flag */
//	TIM2->SR &=~SR_UIF;
//
//	/** do something */
//	tim2_callback();
//}


/** Version 21 - Developing the UART Transmitter DMA driver
 * To test, run this in debug, click on the play button,
 * connect CoolTerm, click in its text area, and press a
 *  key on the keyboard. That value should be displayed
 * in the Live Expressions area of STMCubeIDE.
 * */

#include <stm32f401xe.h>
#include <stdio.h>
#include "uart.h"

#define GPIOAEN               (1U<<0)
#define GPIOA_5               (1U<<5)
#define LED_PIN               GPIOA_5

#define HISR_TCIF6            (1U<<21)
#define HIFCR_CTCIF6          (1U<<21)

static void dma_callback(void);

int main(void)
{
	char message[31] = "Hello from Stm32 DMA transfer\n\r";

	RCC->AHB1ENR |= GPIOAEN;
	GPIOA->MODER |= (1U<<10);
	GPIOA->MODER &=~(1U<<11);

	uart2_tx_init();
	dma1_stream6_init((uint32_t)message, (uint32_t)&USART2->DR, 31);

    while (1){}
}

static void dma_callback(void)
{
	GPIOA->ODR |= LED_PIN;
}

void DMA1_Stream6_IRQHandler(void)
{
	/** check for transfer complete interrupt */
	if(DMA1->HISR & HISR_TCIF6)
	{
		/** clear flag */
		DMA1->HIFCR |= HIFCR_CTCIF6;

		/** do something */
		dma_callback();
	}
}





