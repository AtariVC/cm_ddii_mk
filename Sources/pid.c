#include "pid.h"
  /**
  ******************************************************************************
  * @file           pid.c
  * @version        v1.0
  * @brief          Реализация прогармной модели отдельного ПИД регулятора. 
  * @note           Примечание: Использует логику с плавающей точкой.      
  * @note           Подсчет по формуле: Reaction = K[P*v + D*(dv/dt) + I*integral_error], где integral_v - интеграл ошибки. **todo: заменить описание на latex формулу**
  * @author			    Стюф Алексей [Alexey Styuf] <a-styuf@yandex.ru>
  ******************************************************************************
  */


/**
  * @brief  подсчет шага для ПИД
  * @param  pid_ptr указатель на програмную модель устройства
  * @param  K нормализующий коэффициент
  * @param  P пропорциональный коэффициент 
  * @param  I интегральный коэффициент
  * @param  D диффиренциальный коэффициент
  * @param  reaction_max максимальное воздействие
  */
void pid_init(type_PID_model* pid_ptr, float K, float P, float I, float D, float reaction_max)
{
  pid_reset(pid_ptr);
  pid_ptr->K_coeff = K;
  pid_ptr->P_coeff = P;
  pid_ptr->I_coeff = I;
  pid_ptr->D_coeff = D;
  pid_ptr->reaction_max = reaction_max;
}

/**
  * @brief  подсчет шага для ПИД
  * @param  pid_ptr указатель на програмную модель устройства
  */
void pid_reset(type_PID_model* pid_ptr)
{
  pid_ptr->value = 0.0;
  pid_ptr->value_desired = 0.0;
  pid_ptr->value_prev = 0.0;
  pid_ptr->derivate = 0.0;
  pid_ptr->error = 0.0;
  pid_ptr->integral = 0.0;
  pid_ptr->K_coeff = 0.0;
  pid_ptr->P_coeff = 0.0;
  pid_ptr->I_coeff = 0.0;
  pid_ptr->D_coeff = 0.0;
  pid_ptr->P_reaction = 0.0;
  pid_ptr->I_reaction = 0.0;
  pid_ptr->D_reaction = 0.0;
  pid_ptr->reaction = 0.0;
  pid_ptr->reaction_max = 0.0;
}

/**
  * @brief  сброс ппамяти ПИД для инициализации нового подсчета
  * @detailed 
  * @param  pid_ptr указатель на програмную модель устройства
  */
void pid_refresh(type_PID_model* pid_ptr)
{
  pid_ptr->integral = 0.0;
  pid_ptr->P_reaction = 0.0;
  pid_ptr->I_reaction = 0.0;
  pid_ptr->D_reaction = 0.0;
  pid_ptr->reaction = 0.0;
}

/**
  * @brief  установка желаемого значения выходного сигнала
  * @param  pid_ptr указатель на програмную модель устройства
  * @param  desired_value значение регулируемой величины, к которой происходит подстройка
  */
void pid_set_desired_value(type_PID_model* pid_ptr, float desired_value)
{
  pid_refresh(pid_ptr);
  pid_ptr->value_desired = desired_value;
}

/**
  * @brief  установка коэффициентов ПИД рекулятора
  * @param  pid_ptr указатель на програмную модель устройства
  * @param  P пропорциональный коэффициент 
  * @param  I интегральный коэффициент
  * @param  D диффиренциальный коэффициент
  */
void pid_set_coeff(type_PID_model* pid_ptr, float K, float P, float I, float D)
{
  pid_ptr->K_coeff = K;
  pid_ptr->P_coeff = P;
  pid_ptr->I_coeff = I;
  pid_ptr->D_coeff = D;
}

/**
  * @brief  подсчет шага для ПИД
  * @param  pid_ptr указатель на програмную модель устройства
  * @param  value значение регулируемой переменной
  * @param  period_ms период вызова данной функции
  * @retval необходимая реакция для поддержания значения
  */
float pid_step_calc(type_PID_model* pid_ptr, float value, uint16_t period_ms)
{
  pid_ptr->value = value;
  //
  pid_ptr->derivate = (pid_ptr->value - pid_ptr->value_prev)/(period_ms/1000.);
  pid_ptr->error = pid_ptr->value_desired - pid_ptr->value;
  pid_ptr->integral += pid_ptr->error*(period_ms/1000.);
  //
  pid_ptr->P_reaction = pid_ptr->K_coeff*(pid_ptr->P_coeff*pid_ptr->error);
  pid_ptr->I_reaction = pid_ptr->K_coeff*(pid_ptr->I_coeff*pid_ptr->integral);
  pid_ptr->D_reaction = pid_ptr->K_coeff*(pid_ptr->D_coeff*pid_ptr->derivate);
  pid_ptr->reaction = pid_ptr->P_reaction + pid_ptr->D_reaction + pid_ptr->I_reaction;
  //
  if (pid_ptr->reaction > pid_ptr->reaction_max) pid_ptr->reaction = pid_ptr->reaction_max;
  else if (pid_ptr->reaction < -(pid_ptr->reaction_max)) pid_ptr->reaction = -pid_ptr->reaction_max;
  //
  pid_ptr->value_prev = pid_ptr->value;
  //
  return pid_ptr->reaction;
}

/**
  * @brief  подсчет шага для линейной регуляции
  * @param  current_value текущее значение управляемой переменной
  * @param  target_value целевое значение управляемой переменной
  * @param  inc_accur точность регулировки
  * @param  target_err допустимое отклонение от целевого значения
  * @param  gap если разница между текущем значение больше gap то можно inc сделать больше
  * @retval необходимая реакция для поддержания значения
  */
float linear_increment(float current_value, float target_value, float inc_accur, float target_err, float gap){
  float overgap_mult = 100;

  if (fabs(current_value - target_value) >= target_err){
    if (current_value > target_value){
      if (fabs(current_value - target_value) > gap){
        return -inc_accur * overgap_mult;
      }
      else{
        return -inc_accur;
      }
    }
    else{
      if (fabs(current_value - target_value) > gap){
        return inc_accur * overgap_mult;
      }
      else{
        return inc_accur;
      }
    }
  }
}


/**
  * @brief  вывод отладочной информации для ПИД
  * @param  pid_ptr указатель на програмную модель устройства
  * @param  report указатель на массив для отчета
  * @retval длина массива для отчета
  */
uint8_t pid_get_str_report(type_PID_model* pid_ptr, char* report)
{
  char report_str[128] = {0};
  sprintf(report_str, "PID: P=%.3f I=%.3f D=%.3f React=%.3f", 
                      pid_ptr->P_reaction,
                      pid_ptr->I_reaction,
                      pid_ptr->D_reaction,
                      pid_ptr->reaction);
	memcpy(report, report_str, 127);
	report[127] = 0;
  return strlen(report_str);
}
