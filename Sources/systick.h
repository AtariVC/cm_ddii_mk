#ifndef _SYSTICK_H_
#define _SYSTICK_H_

#include "1986ve8_lib/cm4ikmcu.h"

/**
 * @brief структура для удобства урпавления регистром управления SysTick-таймера
 * 
 */
typedef union
{
	uint32_t whole;
	struct
	{
		uint32_t enable : 1;
		uint32_t tickint : 1;
		uint32_t rsrv : 13;
		uint32_t countflag : 1;
	} field;
}typeSysTickControlRegister;

/**
 * @brief структура для удобства урпавления регистром контроля SysTick-таймера
 * 
 */
typedef union
{
	uint32_t whole;
	struct
	{
		uint32_t value : 24;
	} field;
}typeSysTickLoadRegister;

/**
 * @brief структура для удобства урпавления регистром значения SysTick-таймера
 * 
 */
typedef union
{
	uint32_t whole;
	struct
	{
		uint32_t value : 24;
	} field;
}typeSysTickValRegister;

void systick_init(uint32_t period_ms);
__weak void Systick_Callback(void);

#endif
