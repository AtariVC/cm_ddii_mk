/**
 * @file stm.c
 * @author Алексей Стюф (a-styuf@yandex.ru)
 * @brief модуль управления сигналами СТМ:
 *    - поддержка установки постоянных значений
 *    - поддержка устновки временных значений с возвратам к постоянным
 * @version 0.1
 * @date 2022-03-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "stm.h"

/**
 * @brief инициализация модуля управления СТМ
 * 
 * @param stm_ptr укаазтель на структуру управления модулем СТМ
 * @param value начальное значение СТМ
 * @param port указатель на порт GPIO 
 * @param num номер линии в порте, используемой для СТМ сигнала
 */
void stm_init(type_STM_Model *stm_ptr, uint8_t value, PortControl* port, uint8_t num)
{
  gpio_init(&stm_ptr->gpio, port, num);
  stm_ptr->const_state = value;
  stm_ptr->temporary_state = 0;
  stm_ptr->temporary_timeout_ms = 0;
}

/**
 * @brief функция обработки состояния СТМ
 * 
 * @param stm_ptr укаазтель на структуру управления модулем СТМ
 * @param period_ms время после прошлого вызова данной функции (должно быть больше 0 для корректной обработки состояния)
 */
void stm_process(type_STM_Model *stm_ptr, uint32_t period_ms){
  if (stm_ptr->temporary_timeout_ms > 0){
    if (stm_ptr->temporary_timeout_ms >= period_ms){
      stm_ptr->temporary_timeout_ms -= period_ms;
    }
    else{
      stm_ptr->temporary_timeout_ms = 0;
    }
    //
    gpio_set(&stm_ptr->gpio, stm_ptr->temporary_state);
  }
  else{
    gpio_set(&stm_ptr->gpio, stm_ptr->const_state);
  }
}

/**
 * @brief установка постоянного значения СТМ
 * 
 * @param stm_ptr укаазтель на структуру управления модулем СТМ
 * @param val значение стм: 0-0, не 0 - 1
 */
void stm_const_set(type_STM_Model *stm_ptr, uint8_t val){
  stm_ptr->const_state = val;
}

/**
 * @brief установка временного значения СТМ
 * 
 * @param stm_ptr укаазтель на структуру управления модулем СТМ
 * @param val значение стм: 0-0, не 0 - 1
 * @param timeout_ms время выставление временного значения СТМ
 */
void stm_temporary_set(type_STM_Model *stm_ptr, uint8_t val, uint32_t timeout_ms){
  stm_ptr->temporary_state = val;
  stm_ptr->temporary_timeout_ms = timeout_ms;
}


/**
 * @brief запрос текущего значения СТМ
 * 
 * @param stm_ptr укаазтель на структуру управления модулем СТМ
 * @return uint8_t 1 или 0
 */
uint8_t stm_get(type_STM_Model *stm_ptr)
{
  return gpio_get(&stm_ptr->gpio);
}

