/**
  ******************************************************************************
  * @file           : systick.c
  * @version        : v1.0
  * @brief          : функции управления таймером SysTick
  * @author					: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
	* @date						: 2021.09.16
  ******************************************************************************
  */

#include "systick.h"

/**
 * @brief Инициализация SysTick таймера
 * 
 * @param period_ms период счета SysTick-таймера
 */
void systick_init(uint32_t period_ms)
{
  SysTick->CTRL = (1 << SYSTICK_TICKINT) | (1 << SYSTICK_TICKINT) | (1 << SYSTICK_ENABLE);

  // SysTick->LOAD
  
  // SysTick->VAL
  
  // SysTick->CALIB

  NVIC_EnableIRQ(SysTick_IRQn);
}

/**
 * @brief слабая функция 
 * обработка в месте использования
 * @return __weak 
 */
__weak void Systick_Callback(void);
{
  
}

/**
 * @brief обработка прерываний через CallBack
 * 
 */
void Systick_Handler(void){
  Systick_Callback();
}
