/**
  ******************************************************************************
  * @file           : adc.c
  * @version        : v1.0
  * @brief          : библиотека для работы с АЦП0
  * @note           : библиотека построена псевдопотоке на прерывании //todo: поменять на работу с DMA
  * @author					: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
	* @date						: 2021.09.02
  ******************************************************************************
  */
#include "adc.h"

/**
  * @brief  инициализзация структуры ADC
	* @param  adc_ptr указатель на структуру управления
  */
void adc_init(typeADCStruct* adc_ptr, ADCxControl* regs)
{
  //
  float cal_a[ADC0_CHAN_NUM];
  float cal_b[ADC0_CHAN_NUM];
  int i, sm;
  //
  __adc_calibration_init(cal_a, ADC_A, ADC0_CHAN_NUM);
  __adc_calibration_init(cal_b, ADC_B, ADC0_CHAN_NUM);
  adc_ptr->error_cnt = 0;
  memset((uint8_t*)adc_ptr->data_cnt, 0x00, sizeof(adc_ptr->data_cnt));
  adc_ptr->a = ADC_A;
  adc_ptr->b = ADC_B;
  //
  adc_ptr->regs = regs;
  //
  CLK_CNTR->KEY = _KEY_;
  CLK_CNTR->PER1_CLK |= (1<<23);  //clock ADC0
  ADC0->KEY = _KEY_;
  ADC0->CONFIG1 = (255<<12)|(511<<3);  //19-12: пауза перед началом измерения; 11-3: время зарядки АЦП
  /* Калибровка АЦП средствами МК */
  ADC0->FIFOEN1 = 0xF0000000;
  ADC0->CONFIG0 = 1;  //enable ADC0
  sm = 0;
  for(i=0; i<4; i++) {
    ADC0->CONTROL = (60 + i)<<8 | 1;
    while((ADC0->STATUS & 1)==0);
    sm = sm + (ADC0->RESULT & 0xFFF);
    }
  sm = sm >> 2;
  ADC0->FIFOEN1 = 0;
  ADC0->CONFIG1 = (ADC0->CONFIG1 & 0xFFFFF) | (sm << 20);
  /**/
  ADC0->CONFIG2 = 0x01009004;  // enable interrupt, temp.sensor on
  ADC0->CHSEL0 =  0x00FFFFFF;  // 0x0080007F
  ADC0->FIFOEN0 = 0x00FFFFFF;  // 0x0080007F
  ADC0->CONFIG0 = 0x00000025;  // enable ADC0, SELMODE=1, continues mode
  NVIC_EnableIRQ(IRQn_ADC0);
  //
}

/**
  * @brief  функция для запуска в планировщике задач
  * @note  не обязательна к запуску, создана для удобства контроля напряжения на каналах
	* @param  ctrl_struct указатель на програмную модель устройства
	* @param  time_us глобальное время
  */
int8_t adc_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface)
{
  uint8_t i=0;
  typeADCStruct* adc_ptr = (typeADCStruct*)ctrl_struct;
  //
  if ((time_us - adc_ptr->last_call_time_us) > (ADC_PROCESS_PERIOD*1000)) {
    adc_ptr->last_call_time_us = time_us;
    
    for (i=0;i<ADC0_CHAN_NUM;i++){
      adc_ptr->voltage_arr[i] = adc_ch_voltage(adc_ptr, i);
    }
    adc_ptr->mcu_temp = adc_get_mcu_temp(adc_ptr);
    return 1;
  }
  else{
    return 0;
  }
}

/**
  * @brief  запрос данных канала АЦП в В
	* @param  adc_ptr указатель на програмную модель устройства
	* @param  ch_num номер канала
	* @retval значение канала АЦП в В
  */
float adc_ch_voltage(typeADCStruct* adc_ptr, uint8_t ch_num)
{
	float adc_ch_voltage;
  NVIC_DisableIRQ(IRQn_ADC0);
  adc_ch_voltage = (adc_ptr->a)*adc_ptr->data_mean[ch_num] + adc_ptr->b;
  NVIC_EnableIRQ(IRQn_ADC0);
  return adc_ch_voltage;
}


/**
  * @brief  инициализзация структуры ADC
	* @param  adc_ptr указатель на структуру управления
  * @retval uint16_t MCU temp
  */
float adc_get_mcu_temp(typeADCStruct* adc_ptr)
{
	float temp_fp;
	temp_fp = (((FACTORY_ADC_TEMP25-adc_ptr->data_mean[ADC0_TEMP_CHANNEL])/FACTORY_ADC_AVG_SLOPE) + FACTORY_TEMP25);
	return temp_fp;
}

/**
  * @brief  обработка нового значения канала АЦП (для использование в обработчике прерываний)
  * @param  adc_ptr указатель на програмную модель устройства 
  * @param  channel номер канала
  * @param  value новое значение, полученное в канале АЦП
  */
void adc_new_val_process(typeADCStruct* adc_ptr, uint16_t channel, uint16_t value)
{
  uint8_t i=0;
  volatile uint32_t adc_val_summ = 0;
  //
  if (channel >= ADC0_CHAN_NUM) {
    adc_ptr->error_cnt += 1;
  }
  else{
    //
    adc_ptr->data_cnt[channel] =  (adc_ptr->data_cnt[channel]<(ADC0_MEAN_VAL_CNT-1)) ? 
                                  (adc_ptr->data_cnt[channel] + 1) : 
                                  0;
    //
    adc_ptr->data[channel][adc_ptr->data_cnt[channel]] = value;
    //
    for (i=0; i<ADC0_MEAN_VAL_CNT; i++){
      adc_val_summ += adc_ptr->data[channel][i];
    }
    adc_ptr->data_mean[channel] = (uint16_t)(adc_val_summ >> (ADC0_MEAN_VAL_PWR_OF_2));
  }
}

/**
  * @brief  _static_ заполнение массива одинаковыми значениями по параметру длины
  * @param arr указатель на массив для заполнения
  * @param val значение для заполнения
  * @param num количество элементов для заполнения
  */
void  __adc_calibration_init(float* arr, float val, uint16_t num)
{
  uint16_t i=0;
  for(i=0; i<num; i++){
    arr[i] = val;
  }
}

/**
  * @brief  обработчик прерывания АЦП
  */
void INT_ADC0_Handler(void) {
  INT_ADC0_CallBack();
}

/**
  * @brief  CallBack от обработчика прерывания АЦП, для описания в main с использованием структуры управления АЦП
  */
__weak void INT_ADC0_CallBack(void)
{
  //
}
