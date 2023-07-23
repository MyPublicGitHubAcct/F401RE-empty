/*
 * adc.c
 */
#include <stm32f401xe.h>
#include "adc.h"

#define GPIOAEN           (1U<<0)
#define ADC1EN            (1U<<8)
#define ADC_CH1           (1U<<0)
#define ADC_SEQ_LEN_1     0x00
#define CR2_ADON          (1U<<0)
#define CR2_SWSTART       (1U<<30)
#define SR_EOC            (1U<<1)

#define CR2_CONT          (1U<<1) // continuous conversion bit

void pa1_adc_init(void)
{
	/******* Configure the ADC GPIO pin *******/
	/** Enable clock access to GPIOA */
	RCC->AHB1ENR |= GPIOAEN;
	/** Set the mode of PA1 to analog input */
	GPIOA->MODER |= (1U<<2);
	GPIOA->MODER |= (1U<<3);

	/******* Configure the ADC module *********/
	/** Enable clock access to ADC */
	RCC->APB2ENR |= ADC1EN;
	/** Conversion sequence start */
	ADC1->SQR3 = ADC_CH1;
	/** Conversion sequence length */
	/**
	 * Example: ADC to be configured with three channels...ch2, ch3, ch5
	 * Order we want to sample the channels:
	 * first  = ch5 (write binary 5 in SQ1)
	 * second = ch2 (write binary 2 in SQ2)
	 * third  = ch3 (write binary 3 in SQ3)
	 * The length of the sequence (L) is the number of channels to convert.
	 *
	 * Here, we just fill SQR1 with zeros
	 * */
	ADC1->SQR1 = ADC_SEQ_LEN_1;
	/** Enable ADC module */
	ADC1->CR2 |= CR2_ADON;
}

void start_conversion(void)
{
	// to enable continuous conversion, remove otherwise
	ADC1->CR2 |= CR2_CONT;

	/** Start ADC conversion
	 * Need to use software start (SWSTART), which is pin 30 on ADC control
	 * register 2 (in this case). This can be done directly or with a timer.
	 * */
	ADC1->CR2 |= CR2_SWSTART;
}

uint32_t adc_read(void)
{
	/** Wait for conversion to be complete - check EOC in status register */
	while(!(ADC1->SR & SR_EOC)){}
	/** Read converted result from the data register */
	return (ADC1->DR);
}
