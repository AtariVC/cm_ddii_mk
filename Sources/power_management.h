#ifndef _POWER_MANAGER_H_
#define _POWER_MANAGER_H_


#include "1986ve8_lib/cm4ikmcu.h"
#include "adc.h"
#include "gpio.h"
#include "timers.h"

/**
 * @brief типы каналов по способу включения/отключения
 */
#define NU 0  //! функция отключения/включение не предусмотрена
#define FLAG 1  //! функция отключения/включение реализована установкой сигнала в постоянное значение
#define PULSE 2  //! функция отключения/включение реализована подачей импульса переключения состояния

#define PWR_PULSE_TIME_MS 20  //! длительность импульса включения/отключения для типа канала 2
#define PWR_ON_DELAY 5000     //! задержка на включение каждого следующего устройства при включении питания

/**
  * @brief  определяем номера каналов, последнее поле - количество каналов
*/
typedef enum PWR_CH
{
  PWR_HV1, PWR_HV2, PWR_HV3, PWR_HV4,
  PWR_CH_NUMBER
} type_PWR_CH;

/**
  * @brief  распределение каналов в соответствии с enum PWR_CH
  * @note   длина совпадает с PWR_CH_NUMBER
  */
#define PWR_ADC_CH_NUM {1, 5, 7, 9}
#define PWR_CAL_RES_SHUNT_OHM {0.5, 4.7, 4.7, 4.7}
#define PWR_CAL_FB_SHUNT_OHM {51E3, 10E3, 10E3, 10E3}

#define PWR_CURRENT_BOUND {3*150, 3*55, 3*55, 3*36}

#define PWR_GPIO_PORT_ON {NULL, NULL, NULL, NULL}
#define PWR_GPIO_NUM_ON {NULL, NULL, NULL, NULL}

#define PWR_GPIO_PORT_OFF {NULL, PORTC, PORTC, NULL}
#define PWR_GPIO_NUM_OFF {NULL, 15, 16, NULL}

#define PWR_CHANNELS_TYPES {NU, FLAG, FLAG, NU}
#define PWR_AUTO_CTRL {0, 0, 0, 0} //указываем те каналы, которые мы можем отключать или включать автоматически


#define PWR_PROCESS_PERIOD 100

/**
  * @brief  структура управления каналом питания
  */
typedef struct
{
  uint8_t type, auto_ctrl;
  type_SINGLE_GPIO gpio_on, gpio_off;
  typeADCStruct* adc_ptr;
  uint8_t adc_ch_num;
  float adc_a, adc_b, current_fp;
  uint16_t current, current_bound;
}typePowerCh;

/**
  * @brief  структура управления каналом питания
  */
typedef struct
{
  typePowerCh ch[PWR_CH_NUMBER];
  uint32_t status;
  uint32_t state;
  float curr_report_fp[PWR_CH_NUMBER];
  //
  uint64_t last_call_time_us;
}typePower;
//
void pwr_ch_init(typePowerCh* ch_ptr, uint8_t type, uint8_t auto_ctrl, PortControl* port_on, uint8_t num_on, PortControl* port_off, uint8_t num_off, typeADCStruct* adc_ptr, uint8_t adc_ch_num, float adc_a, float adc_b, uint16_t bound);
void pwr_ch_on_off(typePowerCh* pwr_ch_ptr, uint8_t state);
uint8_t pwr_ch_process(typePowerCh* pwr_ch_ptr);
//
void pwr_init(typePower* pwr_ptr, typeADCStruct* adc_ptr);
void pwr_process(typePower* pwr_ptr);
void pwr_on_off_by_num(typePower* pwr_ptr, uint8_t num, uint8_t state);
void pwr_on_off_by_num_auto(typePower* pwr_ptr, uint8_t num, uint8_t state);
void pwr_all_on_off(typePower* pwr_ptr, uint8_t state);
void pwr_status_reset_by_num(typePower* pwr_ptr, uint8_t num);
void pwr_set_state(typePower* pwr_ptr, uint32_t state);
void pwr_set_bound(typePower* pwr_ptr, uint8_t num, uint16_t bound);
void pwr_create_report(typePower* pwr_ptr);
// static
void __pwr_calc_current_coefficients(float r_sh, float r_fb, float* adc_a_ptr, float* adc_b_ptr);
// task planer function
int8_t pwr_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface);

#endif
