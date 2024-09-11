#ifndef _PID_H_
#define _PID_H_

#include "1986ve8_lib/cm4ikmcu.h"
#include <math.h>
#include "digital_filter.h"
#include "debug.h"

#pragma pack(2)
/** 
  * @brief  структура управления ПИД-моделью
  */
typedef struct
{
  float K_coeff; //! общий коэффициент для всех типов регуляции (по умолчанию 1.0)
  float P_coeff, I_coeff, D_coeff; //! коэффициенты формулы регуляции
  float P_reaction, I_reaction, D_reaction; //! значение отделных слогаемых реакций
  float integral; //! интеграл, накапливаемый для интегрального члена
  float value_prev, derivate; //! предыдущее полученное значение и производная для диффиренциального члена
  float value;  //! текущее значение, полученное из обратной связи
  float value_desired; //! желаемое значение на вызоде регулироемой системы
  float error;  //! ошибка регулирования как разница между желаемым значением и полученным в обратной связи
  float reaction_max;  //! максимально допустимое значение реакции
  float reaction;   //! значение реакции ПИД-регулятора
} type_PID_model;
//
void pid_init(type_PID_model* pid_ptr, float K, float P, float I, float D, float reaction_max);
void pid_reset(type_PID_model* pid_ptr);
void pid_refresh(type_PID_model* pid_ptr);
void pid_set_desired_value(type_PID_model* pid_ptr, float desired_value);
void pid_set_coeff(type_PID_model* pid_ptr, float K, float P, float I, float D);
float pid_step_calc(type_PID_model* pid_ptr, float value, uint16_t period_ms);
uint8_t pid_get_str_report(type_PID_model* pid_ptr, char* report);

#endif
