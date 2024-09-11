  /**
  ******************************************************************************
  * @file           : cyclogramma.c
  * @version        : v1.0
  * @brief          : библиотека для организации циклограмм опроса переферии
	* @note						: используется на базе планировщика задач, следить, что бы каждый шаг был не более 25 мс
  * @author			    : Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  * @date						: 2021.09.25
  ******************************************************************************
  */

#include "cyclogramma.h"

/**
  * @brief  инициализация структуры управления циклограммой
	* @param  cyclo_ptr указатель на структуру управления
  */
void cyclo_init(typeCyclogramma* cyclo_ptr)
{
	memset((uint8_t*)cyclo_ptr, 0x00, sizeof(typeCyclogramma));
}

/**
  * @brief  добавление шага циклограммы 
	* @param  cyclo_ptr указатель на структуру управления
	* @param  func функция, выполняемая в данном шаге
	* @param  ctrl_struct указатель на структуру управления для передачи в функцию обработки шага
	* @param  delay_ms задержка до следующего шага
	* @retval  количество добавленных шагов всего либо ошибка добавления (-1)
  */
int8_t cyclo_add_step(typeCyclogramma* cyclo_ptr, void (*func) (void*), void* ctrl_struct, uint32_t delay_ms)
{
	if (cyclo_ptr->step_number == CYCLO_MAX_STEP) return -1;
	else {
		cyclo_ptr->step[cyclo_ptr->step_number].ctrl_struct = ctrl_struct;
		cyclo_ptr->step[cyclo_ptr->step_number].func = func;
		cyclo_ptr->step[cyclo_ptr->step_number].delay_to_next_step_ms = delay_ms;
		//
		cyclo_ptr->step_number += 1;
		return cyclo_ptr->step_number;
	}
}

/**
  * @brief  обработка состояния циклограммы
	* @note  небходимо вызывать чаще, чем расстановленные задержки: времянное разрешение работы равно периоду вызова
	* @param  cyclo_ptr указатель на структуру управления
	* @param  time_ms текущее время работы МК в мс
	* @retval  количество добавленных шагов всего либо ошибка добавления (-1)
  */
uint8_t cyclo_handler(typeCyclogramma* cyclo_ptr, uint32_t time_ms)
{
	uint32_t call_period = time_ms - cyclo_ptr->last_call_time_ms;
	switch(cyclo_ptr->mode){
	case (CYCLO_MODE_OFF):
		//
		break;
	case (CYCLO_MODE_PAUSE):
		cyclo_ptr->pause_time += call_period;
		break;
	case (CYCLO_MODE_READY):
		cyclo_ptr->last_step_time = time_ms;
		cyclo_ptr->mode = CYCLO_MODE_WORK;
		break;
	case (CYCLO_MODE_WORK):
		// запускаем функцию-обработчик шага
		if((cyclo_ptr->last_step_duration == 0) || (cyclo_ptr->current_step >= cyclo_ptr->step_number)){
			if(cyclo_ptr->step[cyclo_ptr->current_step].func != 0) {
				cyclo_ptr->step[cyclo_ptr->current_step].func(cyclo_ptr->step[cyclo_ptr->current_step].ctrl_struct);
				cyclo_ptr->last_step_duration += 1;
			}
			else{
				cyclo_stop(cyclo_ptr);
				break;
			}
		}
		// обрабатываем задержки между шагами
		if(cyclo_ptr->last_step_duration >= cyclo_ptr->step[cyclo_ptr->current_step].delay_to_next_step_ms) {
			cyclo_ptr->current_step += 1;
			cyclo_ptr->last_step_duration = 0;
		}
		else {
			cyclo_ptr->last_step_duration += (time_ms - cyclo_ptr->last_step_time);
		}
		cyclo_ptr->last_step_time = time_ms;
		break;
	}
	//
	cyclo_ptr->last_call_time_ms = time_ms;
	//
	return 0;
}

/**
  * @brief  запуск циклограммы или возвращение из паузы
	* @param  cyclo_ptr указатель на структуру управления
	* @param  cyclo_ptr указатель на структуру управления
  */
void cyclo_start(typeCyclogramma* cyclo_ptr)
{
	switch(cyclo_ptr->mode){
		case (CYCLO_MODE_OFF):
			cyclo_ptr->mode = CYCLO_MODE_READY;
			cyclo_ptr->current_step = 0;
			break;
		case (CYCLO_MODE_PAUSE):
			cyclo_ptr->mode = CYCLO_MODE_WORK;
			cyclo_ptr->last_step_time += cyclo_ptr->pause_time;
			break;
	}
}

/**
  * @brief  остановка циклограммы
	* @param  cyclo_ptr указатель на структуру управления
  */
void cyclo_stop(typeCyclogramma* cyclo_ptr)
{
	cyclo_ptr->mode = CYCLO_MODE_OFF;
	cyclo_ptr->current_step = 0;
	cyclo_ptr->last_step_time = 0;
	cyclo_ptr->last_step_duration = 0;
	cyclo_ptr->pause_time = 0;
}

/**
  * @brief  пауза циклограммы
	* @param  cyclo_ptr указатель на структуру управления
  */
void cyclo_pause(typeCyclogramma* cyclo_ptr)
{
	cyclo_ptr->mode = CYCLO_MODE_PAUSE;
	cyclo_ptr->pause_time = 0;
}

/**
  * @brief  функция заглушка, которая делает ничего
	* @param  cyclo_ptr указатель на структуру управления
  */
void cyclo_do_nothing(void* ctrl_struct)
{
	//
}
