/*
 * tim.c
 */

#include <stm32f401xe.h>

/** TIM2 will be used here, it is enabled through the APB1 bus. */
#define TIM2EN           (1U<<0)
#define CR1_CEN          (1U<<0)

#define OC_TOGGLE        ((1U<<4) | (1U<<5))
#define CCER_CC1E        (1U<<0) // enable compare mode
#define GPIOAEN          (1U<<0)
#define AFR5_TIM         (1U<<20)

#define TIM3EN           (1U<<1)
#define AFR6_TIM         (1U<<25)
#define CCER_CC1S        (1U<<0)

void tim2_1hz_init(void)
{
	/** enable clock access to timer 2 */
	RCC->APB1ENR |= TIM2EN;

	/** set the prescaler value to 1Hz */
	TIM2->PSC = 1600-1;  // 16,000,000 / 10,000

	/** set auto-reload value */
	TIM2->ARR = 10000-1;

	/** clear counter */
	TIM2->CNT = 0;

	/** enable the timer */
	TIM2->CR1 = CR1_CEN;
}

void tim2_pa5_output_compare(void)
{
	/** enable clock access to GPIOA */
	RCC->AHB1ENR |= GPIOAEN;

	/** set PA5 mode to alternate function */
	GPIOA->MODER &=~(1U<<10);
	GPIOA->MODER |= (1U<<11);

	/** set PA5 alternate function type to TIM2_CH1 (AF01) */
	GPIOA->AFR[0] |= AFR5_TIM;

	/** enable clock access to timer 2 */
	RCC->APB1ENR |= TIM2EN;

	/** set the prescaler value to 1Hz */
	TIM2->PSC = 1600-1;  // 16,000,000 / 10,000

	/** set auto-reload value */
	TIM2->ARR = 10000-1;

	/** set output compare toggle mode */
	TIM2->CCMR1 = OC_TOGGLE;

	/** enable timer 2, channel 1 in compare mode */
    TIM2->CCER |= CCER_CC1E;

	/** clear counter */
	TIM2->CNT = 0;

	/** enable the timer */
	TIM2->CR1 = CR1_CEN;
}

void tim3_pa6_input_capture(void)
{
	/** enable clock access to GPIOA */
	RCC->AHB1ENR |= GPIOAEN;

	/** set PA6 mode to alternate function */
	GPIOA->MODER &=~(1U<<12);
	GPIOA->MODER |= (1U<<13);

	/** set PA6 alternate function type to TIM3_CH1 (AF02) */
	GPIOA->AFR[0] |= AFR6_TIM;

	/** enable clock access to timer 3 */
	RCC->APB1ENR |= TIM3EN;

	/** set the prescaler */
	TIM3->PSC = 16000-1;  // 16,000,000 / 16,000

	/** set CH1 to input capture mode */
	TIM3->CCMR1 = CCER_CC1S;

	/** set CH1 to capture at rising edge (which is the default) */
	TIM3->CCER = CCER_CC1E;

	/** enable timer 3 */
	TIM3->CR1 = CR1_CEN;
}
