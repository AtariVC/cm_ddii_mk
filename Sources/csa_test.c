/**
  ******************************************************************************
  * @file           : csa_test.c
  * @note			: содержит функции инициализации и запуска генератора импульсов
  *                   для проверки работы платы ddii_csa.
  * @version        : v0.0
  * @brief          : общие системные фунции
  * @author			: Владислав Комаров/Vladislav Komarov <Vladislav150899@yandex.ru>
  * @date			: 2024.26.01
  ******************************************************************************
  */

#include "csa_test.h"
#include "gpio.h"
#include "timers.h"

#include "task_planner.h"

/** 
  * @brief  структура инициализации теста
*/
// глобальные переменные
uint8_t flag_gpio = 0;

// структуры
// type_SINGLE_GPIO* gpio_csa_test[AMNT_INIT_PORTS];
// type_SINGLE_GPIO* gpio_csa_test;




/**
  * @brief  инициализация тестового воздействия
  * @param  gpio_ptr указатель на програмную модель устройства
  * @param  port указатель на порт устройства типа PortControl
  * @param  num номер GPIO внутри port
  */

// void csa_test_init(void){
// 		uint8_t i = 0;
//     for (i = 0; i < NUM_INIT_PORTS; i++){
//         gpio_csa_test[i]->port = PORTE;
//     }
//     gpio_csa_test[0]->num = 13;
//     gpio_csa_test[1]->num = 15;
//     gpio_csa_test[2]->num = 17;
//     gpio_csa_test[3]->num = 19;
//     gpio_csa_test[4]->num = 20;
//     gpio_csa_test[5]->num = 21;
//     gpio_csa_test[6]->num = 22;
//     gpio_csa_test[7]->num = 23;
// }

/**
  * @brief  инициализация процесса тестового воздействия
  * @param  ctrl_struct  указатель на програмную модель устройства
  * @param  time_us глобальное время
  */
int8_t csa_test_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface){
  uint8_t i;
  type_CSA_TEST_Struct* csa_test_ptr = (type_CSA_TEST_Struct*)ctrl_struct;
  //
  if ((time_us - csa_test_ptr->last_call_time_us) > (PROCESS_PERIOD*1000)) {
    csa_test_ptr->last_call_time_us = time_us;
    if (csa_test_ptr->enable_test == 1){
      INT_TMR1_CallBack(); // Вызываем обработчик прерываний TIMER1 из main (костыль) 
      if (csa_test_ptr->counter <= csa_test_ptr->count_process){
        csa_test_ptr->counter ++;
        // csa_all_together_test(csa_test_ptr); // включаем все пины вместе
        // for (i = 0; i < NUM_INIT_PORTS; i++){
        //   gpio_togle(&csa_test_ptr->gpio_csa_test[i]);
        // }
      }
      else{
        csa_test_ptr->counter = 0;
        if (csa_test_ptr->count_flag == 1){
          csa_test_ptr->enable_test = 0;
        }
        for (i = 0; i < NUM_INIT_PORTS; i++){
          gpio_set(&csa_test_ptr->gpio_csa_test[i], 0);
        }
      }
    }
    return 1;
  }
  else{
    return 0;
  }
}

void csa_all_together_test(type_CSA_TEST_Struct* csa_test_ptr){
  if(csa_test_ptr->on_off_flag == 1){
    csa_test_ptr->on_off_flag = 0;
    csa_test_ptr->port->SRXTX = 0x00FAA000;
  }
  else{
    csa_test_ptr->on_off_flag = 1;
    csa_test_ptr->port->CRXTX = 0x00FAA000;
  }
}
void csa_all_together_off(type_CSA_TEST_Struct* csa_test_ptr){
  csa_test_ptr->port->CRXTX = 0x00FAA000;
}


void csa_ladder_on_test(type_CSA_TEST_Struct* csa_test_ptr){
  if(csa_test_ptr->on_off_flag == 1){
    csa_test_ptr->on_off_flag = 0;
    switch (csa_test_ptr->count_ladder_on){
      case 0:
        csa_test_ptr->port->SRXTX = 0x00002000;
        csa_test_ptr->count_ladder_on++;
        break;
      case 1:
        csa_test_ptr->port->SRXTX = 0x0000A000;
        csa_test_ptr->count_ladder_on++;
        break;
      case 2:
        csa_test_ptr->port->SRXTX = 0x0002A000;
        csa_test_ptr->count_ladder_on++;
        break;
      case 3:
        csa_test_ptr->port->SRXTX = 0x000AA000;
        csa_test_ptr->count_ladder_on++;
        break;
      case 4:
        csa_test_ptr->port->SRXTX = 0x001AA000;
        csa_test_ptr->count_ladder_on++;
        break;
      case 5:
        csa_test_ptr->port->SRXTX = 0x003AA000;
        csa_test_ptr->count_ladder_on++;
        break;
      case 6:
        csa_test_ptr->port->SRXTX = 0x007AA000;
        csa_test_ptr->count_ladder_on++;
        break;
      case 7:
        csa_test_ptr->port->SRXTX = 0x00FAA000;
        csa_test_ptr->count_ladder_on = 0;
        break;
    }
  }
  else{
    csa_test_ptr->on_off_flag = 1;
    csa_test_ptr->port->CRXTX = 0x00FAA000;
  }
}

void csa_test_init(type_CSA_TEST_Struct* csa_test_ptr){
  csa_test_ptr->enable_test = 0; //выключатель теста 1 - on
  csa_test_ptr->counter = 0; // счетчик импульсов
  csa_test_ptr->count_flag = 0; // режим счета импульсов
  csa_test_ptr->count_process = 200;
  csa_test_ptr->count_ladder_on = 0; // выводим сигнал поочереди лесенкой от 0 до 7
  csa_test_ptr->on_off_flag = 1;
  csa_test_ptr->timer = 1; // период маргания
  csa_test_ptr->port = PORTE;
  gpio_init(&csa_test_ptr->gpio_csa_test[0], PORTE, 13);
  gpio_init(&csa_test_ptr->gpio_csa_test[1], PORTE, 15);
  gpio_init(&csa_test_ptr->gpio_csa_test[2], PORTE, 17);
  gpio_init(&csa_test_ptr->gpio_csa_test[3], PORTE, 19);
  gpio_init(&csa_test_ptr->gpio_csa_test[4], PORTE, 20);
  gpio_init(&csa_test_ptr->gpio_csa_test[5], PORTE, 21);
  gpio_init(&csa_test_ptr->gpio_csa_test[6], PORTE, 22);
  gpio_init(&csa_test_ptr->gpio_csa_test[7], PORTE, 23);
  // gpio_set(&csa_test_struct->gpio_csa_test, 1);
  // gpio_csa_test->port = PORTE;
  // gpio_csa_test->num = 19;
}

