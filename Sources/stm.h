#ifndef _STM_H_
#define _STM_H_

#include "1986ve8_lib/cm4ikmcu.h"
#include "gpio.h"

#pragma pack(2)
/** 
  * @brief  структура управления STM-сигналами
  */
typedef struct
{
  type_SINGLE_GPIO gpio;       //! вывод МК для каждого сигнала СТМ
  uint8_t const_state;         //! состояние СТМ сигнала по умолчанию, при отсутсвии временных состояний
  uint8_t temporary_state;     //! значение временного состояния СТМ сигнала
  uint32_t temporary_timeout_ms;  //! время до перехода в const-значение
} type_STM_Model;

void stm_init(type_STM_Model *stm_ptr, uint8_t value, PortControl* port, uint8_t num);
void stm_process(type_STM_Model *stm_ptr, uint32_t period_ms);
void stm_const_set(type_STM_Model *stm_ptr, uint8_t val);
void stm_temporary_set(type_STM_Model *stm_ptr, uint8_t val, uint32_t timeout_ms);
uint8_t stm_get(type_STM_Model *stm_ptr);

#endif

