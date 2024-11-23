#ifndef _TERMO_RES_H_
#define _TERMO_RES_H_

#include <string.h>
#include <stdint.h>
#include "1986ve8_lib/cm4ikmcu.h"
#include "adc.h"

#define TRES_V_REF    (4.77)
#define TRES_R1_REF   (3E3) // 3k Ohm

#define TRES_TYPE     (1E3) // Pt1000

#define RES_CONDUCTOR  (7) // Сопротивление проводника

#define TRES_CAL_TEMP {-100, -50, -40, -30. -20, -10, -00, +10, +20, +30, +40, +50, +100, +200, +200, +200}
#define TRES_CAL_RES  {602.6, 803.1, 842.7, 882.2, 921.6, 960.9, 1000.0, 1039.0, 1077.9, 1116.7, 1155.4, 1194.0, 1385, 1758.4, 1758.4, 1758.4}

#define TRES_APPROX(R) (6519.0 / 2.0 - (6519.0 / 2.0) * (1.0 + (-4.0 * ((R) - 1000.0056) / (6519.0 * 6519.0 * 0.0006)) / 2.0 - ((-4.0 * ((R) - 1000.0056) / (6519.0 * 6519.0 * 0.0006)) * (-4.0 * ((R) - 1000.0056) / (6519.0 * 6519.0 * 0.0006))) / 8.0))

#define TRES_CHANNELS_NUM 4
#define TRES_ADC_CHANNELS {8, 9, 10, 11}

#pragma pack(2)
/** 
  * @brief  структура управления отдельным термо-резистором, включенным по схеме резистивного делителя (R1 - верхнее плечо, термосопротивление - нижнее плечо)
  */
typedef struct
{
  typeADCStruct* adc_ptr;
  uint8_t adc_ch_num;
  float v_ref, v_out;
  float r1_val, tres_val;
  float temp;
  int16_t temp_i16;
} type_TRES_model;

//
void tres_init(type_TRES_model* t_res_ptr, typeADCStruct* adc_ptr, uint8_t adc_ch_num);
void tres_set_parameters(type_TRES_model* t_res_ptr, typeADCStruct* adc_ch, uint8_t adc_ch_num, float vref, float r1_val, float tres_val);
void tres_update(type_TRES_model* t_res_ptr);
float tres_get_temp(type_TRES_model* t_res_ptr);
int16_t tres_get_temp_i16(type_TRES_model* t_res_ptr);

float _linear_interpolation(float x, float* array_y, float* array_x, uint16_t length);
float _calc_tr_res(float u_ref, float u_sign, float r_1);

#endif
