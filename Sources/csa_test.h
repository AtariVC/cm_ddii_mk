#ifndef _CSA_TEST_H_
#define _CSA_TEST_H_

#include "1986ve8_lib/cm4ikmcu.h"
#include "gpio.h"
#include "task_planner.h"

#define NUM_INIT_PORTS 8 // колличество портов для инициализации
#define PROCESS_PERIOD 1  // период запуска процесса в мсек

/**
  * @brief  структура управления АЦП
  */
typedef struct
{
  type_SINGLE_GPIO gpio_csa_test[NUM_INIT_PORTS];
  uint32_t counter; // общий счетчик
  uint8_t count_flag;
  uint8_t enable_test;
  uint8_t count_ladder_on;
  uint32_t count_process;
  PortControl* port;
  uint8_t on_off_flag;
  uint32_t timer;
  //
  uint64_t last_call_time_us;
}type_CSA_TEST_Struct;


void csa_test_init(type_CSA_TEST_Struct* csa_test_ptr);
int8_t csa_test_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface);
void csa_all_together_test(type_CSA_TEST_Struct* csa_test_ptr);
void csa_ladder_on_test(type_CSA_TEST_Struct* csa_test_ptr);
void csa_all_together_off(type_CSA_TEST_Struct* csa_test_ptr);

#endif
