/*
 * System_Clock.h
 *
 *  Created on: Dec 6, 2025
 *      Author: wiki
 */

#ifndef SYSTEM_CLOCK_H_
#define SYSTEM_CLOCK_H_

#include "stm32c0xx.h"

void System_ClockConfig(void);// Configure STM32C031 to run at 48 MHz using HSI
void SysTick_Init(uint32_t);//Configure SysTick to generate 1 ms interrupts
void Delay_ms(uint32_t);//Non-blocking delay using SysTick counter
uint32_t GetTick(void);//Returns elapsed milliseconds since SysTick_Init()


#endif /* SYSTEM_CLOCK_H_ */
