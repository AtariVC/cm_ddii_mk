#ifndef _ADC_H_
#define _ADC_H_

#include <string.h>
#include <math.h>
#include "1986ve8_lib/cm4ikmcu.h"

#include "debug.h"
#include "task_planner.h"


#define IRQn_ADC0 (IRQn_Type)119

#define ADC0_CHAN_NUM 24
#define ADC0_TEMP_CHANNEL 23
#define ADC0_MEAN_VAL_PWR_OF_2 (4)
#define ADC0_MEAN_VAL_CNT (1<<ADC0_MEAN_VAL_PWR_OF_2)


// из примера от Миландр
#define FACTORY_ADC_TEMP25      1700.      // ADC value = 1700 @ 25C = 1.36996V
#define FACTORY_ADC_AVG_SLOPE      6.      // ADC delta value @ 1C, from milandr demo project
#define FACTORY_TEMP25            25.

#define ADC_VAL_0V5 (612.)
#define ADC_VAL_3V0 (3688.)
#define ADC_A ((3.0-0.5) / (ADC_VAL_3V0 - ADC_VAL_0V5))
#define ADC_B (3.0-(ADC_A*ADC_VAL_3V0))

#define ADC_PROCESS_PERIOD 250

/**
  * @brief  структура управления АЦП
  */
typedef struct
{
  ADCxControl* regs;
  uint16_t data[ADC0_CHAN_NUM][ADC0_MEAN_VAL_CNT];
  uint32_t data_mean[ADC0_CHAN_NUM];
  uint8_t data_cnt[ADC0_CHAN_NUM];
  float a, b;
  uint8_t error_cnt;
  float voltage_arr[ADC0_CHAN_NUM];
  float mcu_temp;
  //
  uint64_t last_call_time_us;
}typeADCStruct;

void adc_init(typeADCStruct* adc_ptr, ADCxControl* regs);
int8_t adc_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface);
//
float adc_ch_voltage(typeADCStruct* adc_ptr, uint8_t ch_num);
float adc_get_mcu_temp(typeADCStruct* adc_ptr);
void adc_new_val_process(typeADCStruct* adc_ptr, uint16_t channel, uint16_t value);
//
void  __adc_calibration_init(float* arr, float val, uint16_t num);
//

void INT_ADC0_Handler(void);
__weak void INT_ADC0_CallBack(void);

#endif
