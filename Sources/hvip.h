#ifndef _HVIP_H_
#define _HVIP_H_

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "1986ve8_lib/cm4ikmcu.h"
#include "timers.h"
#include "adc.h"
#include "gpio.h"
#include "debug.h"
#include "pid.h"

//defines

//! периода обработки состояния прогармной модели
#define HVIP_PROCESS_PERIOD_MS (1000)

//! PID Current-stabilisation
#define PID_K 1.0
#define PID_P 0.005
#define PID_D 0.0
#define PID_I 0.0
#define HVIP_PID_MAX_REACTION 1.0

//! Параметры отслеживания состояни регуляции
#define HVIP_MAX_VOLTAGE_COEFF (1.1)
#define HVIP_VOLTAGE_MAX_ERROR (0.1)

//! Состояние работы модуля
#define HVIP_STATE_HV             (1<<0)
#define HVIP_STATE_OVEVRCURRENT   (1<<1)
#define HVIP_STATE_FEEDBACKERROR  (1<<2)
#define HVIP_STATE_OVERVOLTAGE    (1<<3)

//! Режимы работы модуля
#define HVIP_MODE_OFF       0
#define HVIP_MODE_ON        1


// data structures
#pragma pack(2)
/**
 * @brief программная отчета о работе модуля высоковольтного ВИП
 * 
 */
typedef struct
{
  uint16_t h_voltage;
  uint16_t current;
  uint8_t state;
  uint8_t reserve;
}type_HVIP_frame_report;

/**
 * @brief программная модель управления высоковольтного ВИП
 * 
 */
typedef struct 
{
	uint8_t mode;  //! режим работы
  uint8_t state;  //! состояние работы модуля
  uint16_t pwm_val;  //! значение регуляции 0-999 (используется для PWM с количеством шаогов - 1000)
  float pwm_val_float;  //! 0-999 значение регуляции в типе с плавающей точкой (необходимо дляточной подстройки)
  float v_fb, v_hv;  //! измеренные параметры работы высоковольтного ВИП
  float v_hv_desired;  //! желаемое значение на выходе высоковольтного ВИП
  float current, max_current;  //! потребляемый ток и его верхняя границы
  float a_u, b_u, a_i, b_i;  //! коэффициенты пересчета сигналов обратной свзяи в реальные значения величин I и U
  // a_i = 1E6/(r_sh*r_fb);
  uint8_t pwm_ch_num; //! номер канала используемого PWM **аппаратнозависимый**
  //
  typeADCStruct* adc_ptr;  //! указатель на структуру управления АЦП, встроенным в МК
  uint8_t adc_ch_hv, adc_ch_i24;  //! номера каналов АЦП, используемых для получения обратной связи
  //
  // float hvip_desired_voltage[3];
  type_SINGLE_GPIO hv_inh; //! GPIO для отключения ВИП
  type_PID_model pid;   //! Структура управления ПИД-регулятором
  type_HVIP_frame_report report; //! Отчет о работе модуля
  //! поддрежка task_planner
  uint64_t last_call_time_us;
  float max_pwm_val; // ограничение значения pwm сверху
  uint8_t type_regulation; // 0 - pid регуляция, 1 - линейная
  uint8_t flag_overvolt;
}type_HVIP;

int8_t hvip_init(type_HVIP* hvip_ptr, uint8_t mode,
                  typeADCStruct* adc_ptr, uint8_t adc_ch_hv, uint8_t adc_ch_i24, 
                  PortControl* inh_port, uint8_t inh_num,
                  uint8_t pwm_ch_num,
                  float pwm_val, float desired_voltage_V, float max_current_A,
                  float A_U, float B_U,
                  float A_I, float B_I,
                  float max_pwm, uint8_t type_regulation);

int8_t hvip_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface);
void hvip_process(type_HVIP* hvip_ptr, uint16_t period_ms);
void hvip_set_mode(type_HVIP* hvip_ptr, uint8_t mode);
void hvip_set_voltage(type_HVIP* hvip_ptr, float voltage);
uint8_t hvip_get_str_report(type_HVIP* hvip_ptr, char* report);
void hvip_form_report(type_HVIP* hvip_ptr);
void hvip_set_pwm(type_HVIP* hvip_ptr, float pwm);

#endif
