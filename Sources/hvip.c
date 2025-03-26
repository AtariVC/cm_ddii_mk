/**
  ******************************************************************************
  * @file           : hvip.c
  * @version        : v1.0
  * @brief          : библиотека для работы с програмной моделью ВИП
  * @author			    : Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "hvip.h"

/**
 * @brief функция иниицализации модуля управления высоковольтным ВИП-ом через PWM
 * 
 * @param hvip_ptr указатель на структуру управления
 * @param mode начальный режим работы канала
 * @param adc_ptr указатель на модель АЦП для получения данных обратной связи
 * @param adc_ch_hv номер канала АЦП для обратной связи по напряжению
 * @param adc_ch_i24 номер канала АЦП для обратной связи по току
 * @param inh_port указатель на структуру управления портом GPIO для сигнала INH
 * @param inh_num номер IO  в порте для сигнала INH
 * @param pwm_ch_num номер используемого канала PWM (максимум 4 (0..3), собирается с &0x03)
 * @param pwm_val стартовое значение PWM
 * @param desired_voltage_V целевое значение напряжение
 * @param max_current_A максимальное значение потребляемого тока
 * @param A_U коэффициент A для формулы U[V] = A_U * ADC + B_U
 * @param B_U коэффициент B для формулы U[V] = A_U * ADC + B_U
 * @param A_I коэффициент A для формулы I[A] = A_I * ADC + B_I
 * @param B_I коэффициент B для формулы I[A] = A_I * ADC + B_I
 * @return int8_t статус инициализации (NU, всегда 1)
 */
int8_t hvip_init(type_HVIP* hvip_ptr, uint8_t mode,
                  typeADCStruct* adc_ptr, uint8_t adc_ch_hv, uint8_t adc_ch_i24, 
                  PortControl* inh_port, uint8_t inh_num,
                  uint8_t pwm_ch_num,
                  float pwm_val, float desired_voltage_V, float max_current_A,
                  float A_U, float B_U,
                  float A_I, float B_I,
                  float max_pwm, uint8_t type_regulation
                  )
{
  //
  hvip_ptr->mode = mode;
  hvip_ptr->state = 0;
  hvip_ptr->pwm_ch_num = pwm_ch_num & 0x03;
  hvip_ptr->pwm_val = pwm_val;
  hvip_ptr->pwm_val_float = pwm_val;
  //
  hvip_ptr->v_hv = 0;
  hvip_ptr->v_hv_desired = desired_voltage_V;
  hvip_ptr->current = 0;
  hvip_ptr->max_current = max_current_A;
  hvip_ptr->v_fb = 0;
  hvip_ptr->flag_overvolt = 0;
  //
  hvip_ptr->a_u = A_U;
  hvip_ptr->b_u = B_U;
  hvip_ptr->a_i = A_I;
  hvip_ptr->b_i = B_I;
  //
  gpio_init(&hvip_ptr->hv_inh, inh_port, inh_num);
  gpio_set(&hvip_ptr->hv_inh, 1);
  //
  hvip_ptr->adc_ptr = adc_ptr;
  hvip_ptr->adc_ch_hv = adc_ch_hv;
  hvip_ptr->adc_ch_i24 = adc_ch_i24;
  //
  pid_init(&hvip_ptr->pid, PID_K, PID_P, PID_D, PID_I, HVIP_PID_MAX_REACTION);
  hvip_set_voltage(hvip_ptr, hvip_ptr->v_hv_desired);
  //
  hvip_ptr->last_call_time_us = 0;

  hvip_ptr->max_pwm_val = max_pwm;
  hvip_ptr->type_regulation = type_regulation;
  return 1;
}

/**
  * @brief  обработчик pwm для планировщика задач
	* @param  hvip_ptr указатель на структуру управления	
	* @param  time_us время мк us
	* @param  interface интерфейс для общения данного модуля с другими
  */
int8_t hvip_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface)
{
  type_HVIP* hvip_ptr = (type_HVIP*)ctrl_struct;
  uint64_t interval_us = time_us - hvip_ptr->last_call_time_us;
  //
	if (interval_us > (HVIP_PROCESS_PERIOD_MS*1000)) {
		hvip_ptr->last_call_time_us = time_us;
		// user code
    hvip_process(hvip_ptr, interval_us/1000);
    //
		return 1;
	}
	else {
		return 0;
	}
}

/**
 * @brief обработка состояния hvip
 * 
 * @param hvip_ptr указатель на программную модель устройства
 * @param period_ms период вызова данной функции
 */
void hvip_process(type_HVIP* hvip_ptr, uint16_t period_ms)
{
  float pwm_step;
  //
  hvip_ptr->v_fb = adc_ch_voltage(hvip_ptr->adc_ptr, hvip_ptr->adc_ch_hv);
  hvip_ptr->v_hv = hvip_ptr->a_u*hvip_ptr->v_fb + hvip_ptr->b_u;
  hvip_ptr->current = hvip_ptr->a_i*adc_ch_voltage(hvip_ptr->adc_ptr, hvip_ptr->adc_ch_i24) + hvip_ptr->b_i; 
  //
  pid_set_desired_value(&hvip_ptr->pid, hvip_ptr->v_hv_desired);
  if (hvip_ptr->mode == HVIP_MODE_ON){
    if (hvip_ptr->v_hv <= (hvip_ptr->v_hv_desired*HVIP_MAX_VOLTAGE_COEFF)){ // проверка на перенапряжение
      if (hvip_ptr->flag_overvolt == 0){
        hvip_ptr->flag_overvolt = 1;
        hvip_ptr->pwm_val_float = hvip_ptr->max_pwm_val;
      }
      else{
        hvip_ptr->pwm_val_float = 0;
        hvip_ptr->flag_overvolt = 0;
      }
    }
    if (hvip_ptr->pwm_val_float < 0){ // из-за ошибки преобразования иногда pwm может быть меньше 0, и gpio застывает в 1
      hvip_ptr->pwm_val_float = hvip_ptr->max_pwm_val;
    }
    else{
      if (hvip_ptr->type_regulation == 0){
        pwm_step = pid_step_calc(&hvip_ptr->pid, hvip_ptr->v_hv, period_ms);
      }
      else{
        pwm_step = linear_increment(hvip_ptr->v_hv, hvip_ptr->v_hv_desired, 0.0005, 0.05, 1);
      }
      hvip_ptr->pwm_val_float += pwm_step;  // верная строчка, реакцию добавляем к значению PWM 
      if (hvip_ptr->pwm_val_float >= hvip_ptr->max_pwm_val){
        hvip_ptr->pwm_val_float = hvip_ptr->max_pwm_val;
      }
    }
  }
  else{
    hvip_ptr->pwm_val_float = 0;
    pid_refresh(&hvip_ptr->pid);
  }
  hvip_ptr->pwm_val = (uint16_t)hvip_ptr->pwm_val_float;  //todo: проблема скачков связана с переходом целой части от 12 к 13, соответственно поблема где-то в обработке дробной части
  // state calculate
  if ((HVIP_VOLTAGE_MAX_ERROR*(hvip_ptr->v_hv_desired)) >= fabs(hvip_ptr->pid.error)){
    hvip_ptr->state |= HVIP_STATE_HV;
  }
  else{
    hvip_ptr->state &= ~HVIP_STATE_HV;
  }
  if (hvip_ptr->v_hv >= (hvip_ptr->v_hv_desired*HVIP_MAX_VOLTAGE_COEFF)) {
    hvip_ptr->state |= HVIP_STATE_OVERVOLTAGE;
  }
  else{
    hvip_ptr->state &= ~HVIP_STATE_OVERVOLTAGE;
  }
  if (hvip_ptr->max_current < hvip_ptr->current){
    hvip_ptr->state |= HVIP_STATE_OVEVRCURRENT;
  }
  else{
    hvip_ptr->state &= ~HVIP_STATE_OVEVRCURRENT;
  }
  // 
  if (hvip_ptr->mode & HVIP_MODE_ON){
    gpio_set(&hvip_ptr->hv_inh, 0);
  }
  else{
    gpio_set(&hvip_ptr->hv_inh, 1);
  }
  //
  Timer_PWM_Set_Fp(hvip_ptr->pwm_ch_num, hvip_ptr->pwm_val_float);
  //
  hvip_form_report(hvip_ptr);
}

/**
  * @brief  установка режима работы
  * @param  hvip_ptr указатель на програмную модель устройства
  */
void hvip_set_mode(type_HVIP* hvip_ptr, uint8_t mode)
{
  hvip_ptr->mode = mode;
}

/**
  * @brief  установка выходного напряжения
  * @param  hvip_ptr указатель на програмную модель устройства
  */
void hvip_set_voltage(type_HVIP* hvip_ptr, float voltage)
{
  hvip_ptr->v_hv_desired = voltage;
	//pid_refresh(&hvip_ptr->pid);
}

void hvip_set_pwm(type_HVIP* hvip_ptr, float pwm){
  hvip_ptr->pwm_val_float = pwm;
}


/**
  * @brief  вывод отладочной информации в строку
  * @param  hvip_ptr указатель на програмную модель устройства
  * @param  report указатель на массив для отчета
  * @retval длина строки для отчета
  */
uint8_t hvip_get_str_report(type_HVIP* hvip_ptr, char* report)
{
  char report_str[128] = {0};
  sprintf(report_str, "MVIP: v_fb=%.3f v_hv=%.1f i_24v=%.1f pwm=%.3f md=0x%02X st=0x%02X", 
                      hvip_ptr->v_fb,
                      hvip_ptr->v_hv,
                      hvip_ptr->current*1000,
                      hvip_ptr->pwm_val_float,
                      hvip_ptr->mode,
                      hvip_ptr->state); 
	memcpy(report, report_str, 127);
	report[127] = 0;
  return strlen(report_str);
}

/**
  * @brief  формирование отладочной для использования в кадрe
  * @param  hvip_ptr указатель на програмную модель устройства
  */
void hvip_form_report(type_HVIP* hvip_ptr)
{
  hvip_ptr->report.h_voltage = (int16_t)floor(hvip_ptr->v_hv);
  hvip_ptr->report.current = (int16_t)floor(hvip_ptr->current*256);
  hvip_ptr->report.state = hvip_ptr->state;
  hvip_ptr->report.reserve = 0xFE;
}

