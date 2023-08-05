/*
 * systick.h
 */

#ifndef SYSTICK_H_
#define SYSTICK_H_

/** Need to look in the generic guide from ARM to get all information for SysTick
 *  not included in ST guides. There are separate guides for M4 and M7 */

#define SYSTICK_LOAD_VAL    16000    // cycles per millisecond (clock source is 16MHz)
#define CTRL_ENABLE         (1U<<0)  // enable bit (0 disabled, 1 enabled)
#define CTRL_CLKSRC         (1U<<2)  // system clock bit (0 external, 1 system)
#define CTRL_COUNTFLAG      (1U<<16) // (returns 1 if timer counted to 0 since last time this was read)
#define CTRL_TICKINT        (1U<<1)  // counting down to 0 asserts the SysTick exception request

#define ONE_SEC_LOAD        16000000 // cycles per second (clock source is 16MHz)

void systickDelayMs(int delay);
void systick_1hz_interrupt(void);

#endif /* SYSTICK_H_ */
