/**
  ******************************************************************************
  * @file           : power_management.c
  * @version        : v1.0
  * @brief          : библиотека для работы с питанием ЦМ
  * @author					: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
	* @date						: 2021.09.05
  ******************************************************************************
  */

#include "power_management.h"

// channels
/**
  * @brief  инициализация отдельного канала управления питаниием
	* @param  ch_ptr указатель на структуру управления отдельным каналом питания
	* @param  type типы каналов по способу включения/отключения: NU, FLAG, PULSE
	* @param  auto_ctrl 1 - канал может отключаться автоматически, 0 - канал отклчюается только вручную
	* @param  port_on порт gpio включения питания
	* @param  num_on канал gpio включения питания
	* @param  port_off порт gpio отключения питания
	* @param  num_off канал gpio откключения питания
	* @param  adc_ptr указатель на структур АЦП
	* @param  adc_ch_num канал АЦП соответствующий измерителю тока
	* @param  adc_a коэффициент линейного перевода I(mA)=adc_a*ADC_U(V)C+adc_b
	* @param  adc_b коэффициент линейного перевода I(mA)=adc_a*ADC_U(V)+adc_b
	* @param  bound граница токовой защиты
  */
void pwr_ch_init(typePowerCh* ch_ptr, uint8_t type, uint8_t auto_ctrl, PortControl* port_on, uint8_t num_on, PortControl* port_off, uint8_t num_off, typeADCStruct* adc_ptr, uint8_t adc_ch_num, float adc_a, float adc_b, uint16_t bound)
{
	ch_ptr->type = type;
	ch_ptr->auto_ctrl = auto_ctrl;
	gpio_init(&ch_ptr->gpio_on, port_on, num_on);
	gpio_init(&ch_ptr->gpio_off, port_off, num_off);
	ch_ptr->adc_ptr = adc_ptr;
	ch_ptr->adc_ch_num = adc_ch_num;
	ch_ptr->adc_a = adc_a;
	ch_ptr->adc_b = adc_b;
	ch_ptr->current_bound = bound;
}

/**
  * @brief  включение/откюлчение канала питания
	* @param  pwr_ch_ptr указатель на структуру управления
	* @param  state  1: on, 0: off
  */
void pwr_ch_on_off(typePowerCh* pwr_ch_ptr, uint8_t state)
{
	switch(pwr_ch_ptr->type){
		case(FLAG):
				if (state & 0x01){
					gpio_set(&pwr_ch_ptr->gpio_on, GPIO_ON);
					gpio_set(&pwr_ch_ptr->gpio_off, GPIO_OFF);
				}
				else{
					gpio_set(&pwr_ch_ptr->gpio_on, GPIO_OFF);
					gpio_set(&pwr_ch_ptr->gpio_off, GPIO_ON);
				}
			break;
		case(PULSE):
				if (state & 0x01){
					gpio_set(&pwr_ch_ptr->gpio_on, GPIO_ON);
					gpio_set(&pwr_ch_ptr->gpio_off, GPIO_OFF);
				}
				else{
					gpio_set(&pwr_ch_ptr->gpio_on, GPIO_OFF);
					gpio_set(&pwr_ch_ptr->gpio_off, GPIO_ON);
				}
				Timer_Delay(PWR_PULSE_TIME_MS);
				gpio_set(&pwr_ch_ptr->gpio_on, GPIO_OFF);
				gpio_set(&pwr_ch_ptr->gpio_off, GPIO_OFF);
			break;
		case(NU):
		default:
			break;
	}
}

/**
  * @brief  обработка состояния отдельного канала
	* @param  pwr_ch_ptr указатель на структуру управления
	* @retval статус выхода тока из нормы: 1-не норма, 0-норма
  */
uint8_t pwr_ch_process(typePowerCh* pwr_ch_ptr)
{
	//todo: необходимо переделать работу с ацп в данном модуле: либо встроить модуль АЦП в модуль управления питанием, либо организовать передачу через расшаренную память
	pwr_ch_ptr->current_fp = pwr_ch_ptr->adc_a * adc_ch_voltage(pwr_ch_ptr->adc_ptr, pwr_ch_ptr->adc_ch_num) + pwr_ch_ptr->adc_b;
	if (pwr_ch_ptr->current_fp > 0) {
		pwr_ch_ptr->current = (uint16_t)floor(pwr_ch_ptr->current_fp);
	}
	else {
		pwr_ch_ptr->current = 0;
	}
	//
	if (pwr_ch_ptr->current_bound == 0){
		return 0;
	}
	else if(pwr_ch_ptr->current >= pwr_ch_ptr->current_bound){
		return 1;
	}
	return 0;
}

// power system

/**
  * @brief  инициализация модуля управления питаниием
  * @param  pwr_ptr указатель на структуру управления модулем питания
  */
void pwr_init(typePower* pwr_ptr, typeADCStruct* adc_ptr)
{
	uint8_t i = 0;
	//
	uint8_t type_arr[PWR_CH_NUMBER] = PWR_CHANNELS_TYPES;
	uint8_t auto_ctrl_arr[PWR_CH_NUMBER] = PWR_AUTO_CTRL;
	PortControl* port_on_arr[PWR_CH_NUMBER] = PWR_GPIO_PORT_ON;
	PortControl* port_off_arr[PWR_CH_NUMBER] = PWR_GPIO_PORT_OFF;
	uint8_t num_on_arr[PWR_CH_NUMBER] = PWR_GPIO_NUM_ON;
	uint8_t num_off_arr[PWR_CH_NUMBER] = PWR_GPIO_NUM_OFF;
	uint8_t adc_ch_num[PWR_CH_NUMBER] = PWR_ADC_CH_NUM;
	float r_sh_arr[PWR_CH_NUMBER] = PWR_CAL_RES_SHUNT_OHM;
	float r_fb_arr[PWR_CH_NUMBER] = PWR_CAL_FB_SHUNT_OHM;
	float adc_a = 0, adc_b = 0;
	uint16_t bound_arr[PWR_CH_NUMBER] = PWR_CURRENT_BOUND;
	//
	pwr_ptr->state = 0x0000;
	pwr_ptr->status = 0x0000;
	// for (i=0; i<PWR_CH_NUMBER; i++){
	// 	pwr_ptr->state |= (1<<i);
	// }
	//
	for (i=0; i<PWR_CH_NUMBER; i++){
		__pwr_calc_current_coefficients(r_sh_arr[i], r_fb_arr[i], &adc_a, &adc_b);
		pwr_ch_init(&pwr_ptr->ch[i], type_arr[i], auto_ctrl_arr[i], port_on_arr[i], num_on_arr[i], port_off_arr[i], num_off_arr[i], adc_ptr, adc_ch_num[i], adc_a, adc_b, bound_arr[i]);
	}
	//
}

/**
  * @brief  обработка всех каналов измерения
	* @param  pwr_ptr указатель на структуру управления
  */
void pwr_process(typePower* pwr_ptr)
{
	uint8_t i=0;
	// подсчет токов и состояний
	for(i=0; i<PWR_CH_NUMBER; i++){
		if (pwr_ch_process(&pwr_ptr->ch[i])) {
			pwr_ptr->status |= (1 << i);
			pwr_on_off_by_num_auto(pwr_ptr, i, 0);
		}
	}
	//
	pwr_create_report(pwr_ptr);
}

/**
  * @brief  включение/откюлчение канала питания по номеру программно (с учетом флага автоматического контроля)
	* @param  pwr_ch_ptr указатель на структуру управления
	* @param  num номер канала
	* @param  state  1: on, 0: off
  */
void pwr_on_off_by_num(typePower* pwr_ptr, uint8_t num, uint8_t state)
{
	pwr_ch_on_off(&pwr_ptr->ch[num], state);
	if (state){
		pwr_ptr->state |= (1 << num);
	}
	else{
		pwr_ptr->state &= ~(1 << num);
	}
}

/**
  * @brief  включение/откюлчение канала питания по номеру программно (с учетом флага автоматического контроля)
	* @param  pwr_ch_ptr указатель на структуру управления
	* @param  num номер канала
	* @param  state  1: on, 0: off
  */
void pwr_on_off_by_num_auto(typePower* pwr_ptr, uint8_t num, uint8_t state)
{
	if(pwr_ptr->ch[num].auto_ctrl){
		pwr_on_off_by_num(pwr_ptr, num, state);
	}
}

/**
  * @brief  включение/отключение всех каналов питания c учетом флага автоматического контроля
	* @param  pwr_ptr указатель на структуру управления
	* @param  state  1: on, 0: off
  */
void pwr_all_on_off(typePower* pwr_ptr, uint8_t state)
{
	uint8_t i=0;
	for(i=0; i<PWR_CH_NUMBER; i++){
		Timer_Delay(PWR_ON_DELAY);
		pwr_ch_on_off(&pwr_ptr->ch[i], state);
		pwr_ptr->state |= (1 << i);
	}
}

/**
  * @brief  сброс статуса модуля питания
	* @param  pwr_ptr указатель на структуру управления
  */
void pwr_status_reset_by_num(typePower* pwr_ptr, uint8_t num)
{
	pwr_ptr->status &= ~(1 << num);
}

/**
  * @brief  установка состояния модулей с одновременным вклчением, отключением
	* @param  pwr_ptr указатель на структуру управления
  */
void pwr_set_state(typePower* pwr_ptr, uint32_t state)
{
	uint8_t i=0;
	pwr_ptr->state = state;
	for (i=0; i<PWR_CH_NUMBER; i++){
		pwr_on_off_by_num(pwr_ptr, i, (pwr_ptr->state >> i) & 0x01);
	}
}

/**
  * @brief  установка уровня токовой защиты
	* @param  pwr_ptr указатель на структуру управления
	* @param  bound указатель на структуру управления
  */
void pwr_set_bound(typePower* pwr_ptr, uint8_t num, uint16_t bound)
{
	pwr_ptr->ch[num].current_bound = bound;
	pwr_ptr->status &= ~(1<<num);
}

/**
  * @brief  создание отчета для удобства отображения
	* @param  pwr_ptr указатель на структуру управления
  */
void pwr_create_report(typePower* pwr_ptr)
{
	uint8_t i;
	for (i=0; i<PWR_CH_NUMBER; i++){
		pwr_ptr->curr_report_fp[i] = pwr_ptr->ch[i].current_fp;
	}

}


// static
/**
  * @brief  подсчет коэффициенто перевод и сопротивлений в калибровочные коэффициенты  I(mA)=adc_a*ADC_U(V)+adc_b
	* @param  r_sh сопротивление для измерения тока
	* @param  r_fb сопротивление обратной связи
	* @param  adc_a_ptr указатель на переменную adc_a
	* @param  adc_b_ptr указатель на переменную adc_b
  */
void __pwr_calc_current_coefficients(float r_sh, float r_fb, float* adc_a_ptr, float* adc_b_ptr)
{
	*adc_a_ptr = 1E6/(r_sh*r_fb);
	*adc_b_ptr = 0;
}

// Task planner process handler

/**
  * @brief  функция для запуска в планировщике задач обработки каналов питания
	* @param  ctrl_struct указатель на програмную модель устройства
	* @param  time_ms глобальное время
  */
int8_t pwr_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface)
{
  typePower* pwr_ptr = (typePower*)ctrl_struct;
	if ((time_us - pwr_ptr->last_call_time_us) > (PWR_PROCESS_PERIOD*1000)) {
		pwr_ptr->last_call_time_us = time_us;
		pwr_process(pwr_ptr);
		return 1;
	}
	else {
		return 0;
	}
}
