#include "systick.h"
#include "stm32f407xx.h"

#define CTRL_ENABLE (1U<<0)
#define CTRL_CLKSRC (1U<<2)

void systick_counter_init(void)
{

	SysTick->CTRL =0;
	SysTick->LOAD = 0x00FFFFFF;
	SysTick->VAL =0;

	SysTick->CTRL |= (1U<<2);

	SysTick->CTRL |= (1U<<0);

//	SysTick->CALIB =0;
//
//	/*Configuring CTRL register*/
//
//	SysTick->CTRL |= (0U<<1);

}
