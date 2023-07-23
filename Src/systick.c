/*
 * systick.c
 */

#include <stm32f401xe.h>
#include "systick.h"

/** Need to look in the generic guide from ARM to get all information
 * for SysTick not included in ST guides. There are separate guides for M4
 * and M7 */

#define SYSTICK_LOAD_VAL    16000    //(cycles per millisecond)
#define CTRL_ENABLE         (1U<<0)  // enable bit (0 disabled, 1 enabled)
#define CTRL_CLKSRC         (1U<<2)  // system clock bit (0 external, 1 system)
#define CTRL_COUNTFLAG      (1U<<16) // (returns 1 if timer counted to 0 since last time this was read)

void systickDelayMs(int delay)
{
	/** reload with number of clocks per millisecond */
	/* Note: LOAD is referred to as RELOAD in the ARM guide. */
	SysTick->LOAD = SYSTICK_LOAD_VAL;

	/** clear SysTick current value register */
	/* Note: VAL is SysTick to as SYST_CVR in the ARM guide. */
	SysTick->VAL = 0;

	/** enable SysTick and select internal clock source */
	/* Note: Binary OR Operator (|) copies a bit if it exists in either operand. Both
	 * are set to 1 above, so SysTick->CTRL is set to 1. */
	SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC;

	for(int i=0; i<delay; i++)
	{
		/** wait until the COUNTFLAG is set */
		/* Note: Binary AND Operator (&) copies a bit to the result if it exists in
		 * both operands. So, this translates to if */
		while((SysTick->CTRL & CTRL_COUNTFLAG) == 0){}
	}
	SysTick->CTRL = 0;
}
