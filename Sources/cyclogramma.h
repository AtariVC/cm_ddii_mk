#ifndef _CYCLOGRAMMA_H_
#define _CYCLOGRAMMA_H_

#include <string.h>
#include "1986ve8_lib/cm4ikmcu.h"

// дефайны для переменных
#define CYCLO_MAX_STEP 16

#define CYCLO_MODE_OFF 0
#define CYCLO_MODE_WORK 1
#define CYCLO_MODE_READY 2
#define CYCLO_MODE_PAUSE 3

#define CYCLO_DO_NOTHING (cyclo_do_nothing)

// структуры данных
#pragma pack(push, 2)

/** 
  * @brief  структура с отдельным шагом циклограммы
  * @param func фунция обработчик шага циклограммы
*/
typedef  struct
{
  void (*func) (void* ctrl_struct);
  void* ctrl_struct;  // возможно получится через отдельные шаги сделать совместные циклограммы через различные управляющие структуры
  uint32_t delay_to_next_step_ms;
}typeCyclogrammaStep;

/** 
  * @brief  структура управления отдельной циклограммой
  */
typedef  struct
{
  typeCyclogrammaStep step[CYCLO_MAX_STEP];
  uint32_t step_number;
  uint8_t mode;
  uint8_t current_step;
  uint32_t last_step_time, last_step_duration, pause_time;
  //
  uint32_t last_call_time_ms;
}typeCyclogramma;

#pragma pack(pop)
//
void cyclo_init(typeCyclogramma* cyclo_ptr);
int8_t cyclo_add_step(typeCyclogramma* cyclo_ptr, void (*func) (void*), void* ctrl_struct, uint32_t delay_ms);
uint8_t cyclo_handler(typeCyclogramma* cyclo_ptr, uint32_t time_ms);
void cyclo_start(typeCyclogramma* cyclo_ptr);
void cyclo_stop(typeCyclogramma* cyclo_ptr);
void cyclo_pause(typeCyclogramma* cyclo_ptr);
void cyclo_do_nothing(void* ctrl_struct);
//
#endif
