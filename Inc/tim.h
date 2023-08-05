/*
 * tim.h
 */

#ifndef TIM_H_
#define TIM_H_

#define SR_UIF            (1U<<0)  // timer status register
#define SR_CC1IF          (1U<<0)  // capture/compare 3 interrupt flag

/** TIM2 and TIM3 are enabled through the APB1 bus. */
#define TIM2EN           (1U<<0)
#define CR1_CEN          (1U<<0)
#define DIER_UIE         (1U<<0)

#define OC_TOGGLE        ((1U<<4) | (1U<<5))
#define CCER_CC1E        (1U<<0) // enable compare mode
#define GPIOAEN          (1U<<0)
#define AFR5_TIM         (1U<<20)

#define TIM3EN           (1U<<1)
#define AFR6_TIM         (1U<<25)
#define CCER_CC1S        (1U<<0)

void tim2_1hz_init(void);
void tim2_1hz_interrupt_init(void);
void tim2_pa5_output_compare(void);
void tim3_pa6_input_capture(void);

#endif /* TIM_H_ */
