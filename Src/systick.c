/*
 * systick.c
 */

#include <stm32f401xe.h>
#include "systick.h"

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

void systick_1hz_interrupt(void)
{
	/** reload with number of clocks per second */
	/* Note: LOAD is referred to as RELOAD in the ARM guide. */
	SysTick->LOAD = ONE_SEC_LOAD - 1;  // speed of the LED blink

	/** clear SysTick current value register */
	/* Note: VAL is SysTick to as SYST_CVR in the ARM guide. */
	SysTick->VAL = 0;

	/** enable SysTick and select internal clock source */
	/* Note: Binary OR Operator (|) copies a bit if it exists in either operand. Both
	 * are set to 1 above, so SysTick->CTRL is set to 1. */
	SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC;

	/** enable SysTick interrupt */
    SysTick->CTRL |= CTRL_TICKINT;
}


