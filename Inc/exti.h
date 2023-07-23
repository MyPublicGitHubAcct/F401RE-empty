/*
 * exti.h
 */

#ifndef EXTI_H_
#define EXTI_H_

#include <stm32f401xe.h>

void pc13_exti_init();

#define LINE13       (1U<<13)

#endif /* EXTI_H_ */
