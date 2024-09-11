/**
  ******************************************************************************
  * @file           : task_planner.c
  * @version        : v1.0
  * @brief          : планировщик работы процессов
  * @note           : работа планировщика идет через SysTick-таймер (при необходимости заменить на другой таймер)
  * @author					: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
	* @date						: 2021.09.16
  ******************************************************************************
  */

#include "task_planner.h"

/**
  * @brief  инициализзация модуля работы с процессами
	* @param  tp_ptr указатель на структуру управления
  */
void tp_init(typeTPStruct* tp_ptr)
{
  //
  __tp_set_default_parameters(tp_ptr);
  //

}

/**
  * @brief  установка всех параметров в значение по умолчанию
	* @param  tp_ptr указатель на структуру управления
  */
void __tp_set_default_parameters(typeTPStruct* tp_ptr)
{
  //изначальное зануление всех полей
  memset((uint8_t*)tp_ptr, 0x00, sizeof(typeTPStruct));
  //
}

/**
  * @brief  регистрация процесса для исполнения в общем потоке
	* @param  tp_ptr указатель на структуру управления
	* @param  action указатель на стандартизованную функцию управления процессом
	* @param  control_struct_ptr указатель на структуру управления процессом
	* @param  sh_mem_offset начало участка памяти доступного для данного процесса
	* @param  sh_mem_len длина памяти доступная данному процессу
  * @retval статус выполнения функции:  -1 - нет места для процесса, другое - номер процесса
  */
int8_t tp_process_registration(typeTPStruct* tp_ptr, int8_t (*action) (void*, uint64_t, typeProcessInterfaceStruct*), void* control_struct_ptr, uint32_t sh_mem_offset, uint32_t sh_mem_len)
{
  uint8_t i=0;
  for (i=0; i<TP_PROCESS_MAX_NUM; i++)
  {
    if (tp_ptr->process[i].action == 0){
      tp_ptr->process_num = i+1;
      tp_ptr->process[i].num = i;
      tp_ptr->process[i].action = action;
      tp_ptr->process[i].control_struct_ptr = control_struct_ptr;
      tp_ptr->process[i].interface.self_num = i;
      tp_ptr->process[i].interface.event = tp_ptr->event;
      tp_ptr->process[i].interface.shared_mem = &tp_ptr->shared_mem[sh_mem_offset];
      tp_ptr->process[i].interface.shared_mem_len = sh_mem_len;
      if(sh_mem_offset + sh_mem_len >= TP_SHARED_MEM_VOL_B) {
        tp_ptr->error_counter++;
        return -1;
      }
      return i;
    }
  }
  tp_ptr->error_counter++;
  return -1;
}

/**
  * @brief  обработчик процессов
	* @param  tp_ptr указатель на структуру управления
  */
void tp_handler(typeTPStruct* tp_ptr)
{
  uint32_t hanler_duration = 0, process_duration=0;
  volatile uint64_t handler_start_time = 0;
  //
  if ((tp_ptr->time.full_us - tp_ptr->last_call_time_us) >= TP_PROCESS_PERIOD_MS*1000){
		//
    tp_ptr->last_call_time_us = tp_ptr->time.full_us;
    // запуск обработки процесса
    handler_start_time = tp_ptr->time.full_us;
    while(hanler_duration < (0.5*TP_PROCESS_PERIOD_MS*1000))
    {
      //
      dbg_gpio(DBG_GPIO_ON);
      //
      if (tp_task_run(tp_ptr, tp_ptr->active_process_num, &process_duration) == TP_PROCESS_ABSENT){
        tp_ptr->active_process_num = 0;
        dbg_gpio(DBG_GPIO_OFF);
        break;
      }
      else{
        tp_ptr->active_process_num += 1;
        dbg_gpio(DBG_GPIO_OFF);
      }
      //
      hanler_duration = tp_ptr->time.full_us - handler_start_time;
      //
      if (hanler_duration > TP_PROCESS_WARNING_MS*1000){
        tp_ptr->status |= TP_STATUS_WARNING;
      }
      else if (hanler_duration > TP_PROCESS_ERROR_MS*1000){
        tp_ptr->error_counter++;
        tp_ptr->status |= TP_STATUS_ERROR;
      }
    }
    //
    tp_ptr->work_time.full_us += hanler_duration;
    tp_ptr->work_percantage = (100.*tp_ptr->work_time.full_us/tp_ptr->time.full_us);
    //
  }
	// пересчет времени в удобный формат
  __time_recalculate(&tp_ptr->time);
	__time_recalculate(&tp_ptr->work_time);
  //
}

/**
  * @brief  запуск отдельного action
	* @param  tp_ptr указатель на структуру управления
	* @param  process_num указатель на структуру управления
	* @param  action_time_ms_ptr указатель переменную с временем выполнения задачи
  * @retval статус выполнения задачи
  */
int8_t tp_task_run(typeTPStruct* tp_ptr, uint8_t process_num, uint32_t* task_time_ms)
{
  uint32_t task_start = 0;
  uint8_t status;
  //
  *task_time_ms = 0;
  //
  task_start = tp_ptr->time.full_us;
  if (tp_ptr->process[process_num].action == 0){
    return TP_PROCESS_ABSENT;
  }
  else{
    status = tp_ptr->process[tp_ptr->active_process_num].action(tp_ptr->process[tp_ptr->active_process_num].control_struct_ptr, tp_ptr->time.full_us, &tp_ptr->process[tp_ptr->active_process_num].interface);
    *task_time_ms = tp_ptr->time.full_us - task_start;
    tp_ptr->process[tp_ptr->active_process_num].process_time.full_us += *task_time_ms;
    tp_ptr->process[tp_ptr->active_process_num].work_percantage = (100.*tp_ptr->process[tp_ptr->active_process_num].process_time.full_us/tp_ptr->time.full_us);
    __time_recalculate(&tp_ptr->process[tp_ptr->active_process_num].process_time);
    return status;
  }
}

/**
  * @brief  обработка вызова в прерввании в 1мс таймере
	* @param  tp_ptr указатель на структуру управления
  */
void tp_timer_handler(typeTPStruct* tp_ptr)
{
	tp_ptr->time.full_us += 100;
}

/**
  * @brief  преобразования общего количество ms в читаемую форму
	* @param  time_ptr указатель на структуру времени

  */
void __time_recalculate(typeTimeStruct* time_ptr)
{
  uint32_t second = time_ptr->full_us/1000000;
	uint32_t tmp = time_ptr->full_us & 0xFFFFFFFF;
  //
  time_ptr->day = (second / (86400));
  second -= time_ptr->day*86400;
  //
  time_ptr->hour = (second / (3600));
  second -= time_ptr->hour*3600;
  //
  time_ptr->minute = (second / (60));
  second -= time_ptr->minute*60;
  //
  time_ptr->second = second;
  //
  time_ptr->ms = (tmp / 1000) % 1000;
}


/**
  * @brief  репрезентация времени в строковой форме
	* @param  time_ptr указатель на структуру времени

  */
uint8_t __time_str_repr(typeTimeStruct* time_ptr, char* time_string)
{
  //
  scanf(time_string, "d:%03d %02d:%02d:%02d.%03d", time_ptr->day, time_ptr->hour, time_ptr->minute, time_ptr->second, time_ptr->ms);
  return strlen(time_string);
}
