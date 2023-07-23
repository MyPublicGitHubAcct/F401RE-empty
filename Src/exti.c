/*
 * exti.c
 * PC13 - pushbutton
 */

#include "exti.h"

#define GPIOCEN          (1U<<2)
#define SYSCFGEN         (1U<<14)

void pc13_exti_init()
{
	/** Disable the global interrupt */
	__disable_irq();  // function in cmsis_gcc.h

	/** Enable clock access for GPIOC */
	RCC->AHB1ENR |= GPIOCEN;

	/** Set PC13 as input */
	GPIOC->MODER &=~(1U<<26);
	GPIOC->MODER &=~(1U<<27);

	/** Enable clock access to SYSCFG */
	RCC->APB2ENR |= SYSCFGEN;

	/** Select PORTC for EXTI13
	 * - in EXT config register 4, set bit 5 to 1
	 * (all other bits are already 0) */
	SYSCFG->EXTICR[3] |= (1U<<5);

	/** Unmask EXTI13 - this enables the interrupt */
	EXTI->IMR |= (1U<<13);

	/** Select falling edge trigger */
	EXTI->FTSR |= (1U<<13);

	/** Enable EXTI13 line in NVIC
	 * NVIC_EnableIRQ() is defined in core_cm4.h
	 * EXTI15_10_IRQn is defined in stm32f401ex.h */
	NVIC_EnableIRQ(EXTI15_10_IRQn);  // set to 40

	/** Enable global interrupts */
	__enable_irq();  // function in cmsis_gcc.h
}
