#include "1986ve8_lib/cm4ikmcu.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "sysinit.h"
#include "cm.h"
#include "power_management.h"
#include "adc.h"
#include "mko.h"
#include "debug.h"
#include "internal_bus.h"
#include "frames.h"
#include "mpp.h"
#include "task_planner.h"
#include "hvip.h"



// создание объектов отдельных программных моделей
typeTPStruct tp;	//! обязательный объект планировщика задач
typeCMModel cm;		//! объект управления переферией ЦМ
typeMPPStruct mpp[MPP_DEV_NUM]; //! програмная модель переферийных модулей МПП

// прототипы функций для использования внутри main.c (описание в фунциях)
void __main_process_registration_box(void);
void __main_init_perepherial_modules(void);
void __main_base_init(void);

/**
 * @brief суперцикл
 * правка только с согласованием с автором проекта
 * остальные правки на свой страх и риск
 * @return int никогда не возращает, так как работает бесконечно
 */
int main() {
	// базовая инициализация
	System_Init();  // инициализация переферии микроконтроллера: платформозависимая
	Timers_Init();	// инициализация таймеров и управления временем
	//
	cm_init(&cm, CM, CM_SELF_MB_ID, FRAME_DEV_ID, CM+1);  // инициализация объекта програмной модели ЦМ (включает в себя все переферийные модели ЦМ: ВШ, МКО, GPIO, ADC и т.п.)
	cm_load_cfg(&cm);	// загрузка конфигурации из энергонезависимой памяти
	//
	pwr_all_on_off(&cm.pwr, 1);  // включение переферии согласно настройкам каналов управления питанием
	//
	__main_init_perepherial_modules();  // инициализация объектов переферии, подключаемых к ЦМ
	//
	tp_init(&tp);	// иниицализация планировщика задач
	// Системные процессы
	__main_process_registration_box();  //регистрация процессов обработки ЦМ и переферии
	//
	while(1) {
		tp_handler(&tp); // обработка планировщика задач
	}
}

/**
  * @brief  обертка для инициализации всей переферии
*/
void __main_init_perepherial_modules(void)
{
	// настройки каналов МПП
	uint32_t mpp_offsets_array[MPP_DEV_NUM] = MPP_DEFAULT_OFFSET;
	uint32_t mpp_ib_id[MPP_DEV_NUM] = MPP_ID;
	uint32_t mpp_ch_num[MPP_DEV_NUM] = MPP_CHANNENUM_ID;
	uint8_t uint8_var = 0;
	/**
	 * @brief  цикл используется для последовательной нумерации схожих устройств. Использование необязяательно
	 */
	for (uint8_var = 0; uint8_var<MPP_DEV_NUM; uint8_var++){
		mpp_init(	&mpp[uint8_var],
					MPP1 + uint8_var,
					mpp_ib_id[uint8_var],
					FRAME_DEV_ID, MPP1+uint8_var+1,
					mpp_ch_num[uint8_var],
					mpp_offsets_array[uint8_var],
					&cm.ib,
					&cm.global_frame_num);
	}
}

// Командный интерфейс
/**
  * @brief  обертка для регистрации всех необходимых процессов
*/
void __main_process_registration_box(void)
{
	uint8_t uint8_var;
	// процессы для поддрежания работы ЦМ и его систем: питания, АЦП, внутренней шины
	tp_process_registration(&tp, &cm_process_tp, &cm, CM*64, TP_SHARED_MEM_VOL_B);
	tp_process_registration(&tp, &adc_process_tp, &cm.adc, 0, 0);
	tp_process_registration(&tp, &pwr_process_tp, &cm.pwr, 0, 0);
	tp_process_registration(&tp, &ib_process_tp, &cm.ib, 0, 0);
	for(uint8_var=0; uint8_var<HVIP_NUM; uint8_var++){
		tp_process_registration(&tp, &hvip_process_tp, &cm.hvip[uint8_var], 0, 0);
	}
	// Процессы переферийных устройств
	for (uint8_var = 0; uint8_var < MPP_DEV_NUM; uint8_var++){
		tp_process_registration(&tp, &mpp_process_tp, &mpp[uint8_var], 64*(MPP1+uint8_var), 64);
	}
}

/**
  * @brief  обертка для базовой инициализации БЭ через команду МКО (или ВШ)
*/
void __main_base_init(void)
{
	uint8_t uint8_var = 0;
	// отключение переферии
	pwr_all_on_off(&cm.pwr, 0);  //todo: возможно необходимо поместить данную функциб после инициализации ЦМ
	// отключение планировщика задач
	tp_init(&tp);
	// архивация памяти
	fr_mem_format(&cm.mem);
	// инициализация структур
	cm_init(&cm, CM, 1, FRAME_DEV_ID, CM+1);
	// включение переферии
	pwr_all_on_off(&cm.pwr, 1);
	__main_init_perepherial_modules();
	Timer_Delay(1000);
	// дополнительная инициализация переферии
	for (uint8_var = 0; uint8_var < MPP_DEV_NUM; uint8_var++){
		mpp_arch_mem_init(&mpp[uint8_var]);
	}
	// сброс времени
	Time_Set(0, &cm.ctrl.diff_time, &cm.ctrl.diff_time_fractional);
	// регистрация процессов
	__main_process_registration_box();
}

void cm_mko_command_interface_handler(typeCMModel *cm_ptr)
{
	typeFrameStruct frame;
	//
	if (mko_need_to_process(&cm_ptr->mko_rt)){
		switch(cm_ptr->mko_rt.cw.field.sub_addr){
			case CM_MKO_SA_CMD:
				switch(cm_ptr->mko_rt.data[0]){
					case (CMD_TEST):
						//
						break;
					case (CMD_SYNCH_TIME):
						cm_mko_cmd_synch_time(cm_ptr);
						break;
					case (CMD_INIT):
						__main_base_init();
						break;
					case (CMD_SET_INTERVAL):
						cm_set_interval_value(cm_ptr, cm_ptr->mko_rt.data[1], cm_ptr->mko_rt.data[2]);
						break;
					case (CMD_SET_MPP_OFFSET):
						if ((cm_ptr->mko_rt.data[1] >= 1) && (cm_ptr->mko_rt.data[1] <= MPP_DEV_NUM)) {
							mpp_set_offset(&mpp[cm_ptr->mko_rt.data[1] - 1], cm_ptr->mko_rt.data[2]);
						}
						break;
					case (CMD_CONST_MODE):
						if (cm_ptr->mko_rt.data[1] == 1) mpp_constant_mode(&mpp[0], 1); // команда шировковещательная
						else if (cm_ptr->mko_rt.data[1] == 0) mpp_constant_mode(&mpp[0], 0);
						break;
					case (CMD_HVIP_SET_MODE):
						if (cm_ptr->mko_rt.data[1] < HVIP_NUM) {
							hvip_set_mode(&cm_ptr->hvip[cm_ptr->mko_rt.data[1]], (cm_ptr->mko_rt.data[2]) & 0x01);
							pid_refresh(&cm_ptr->hvip[cm_ptr->mko_rt.data[1]].pid);
						}
						break;
					case (CMD_HVIP_SET_VAL):
						if (cm_ptr->mko_rt.data[1] < HVIP_NUM) {
							hvip_set_voltage(&cm_ptr->hvip[cm_ptr->mko_rt.data[1]], (float)cm_ptr->mko_rt.data[2]);
						}
						break;
					case (CMD_HVIP_SET_PID):
						if (cm_ptr->mko_rt.data[1] < HVIP_NUM) {
							pid_set_coeff(	&cm_ptr->hvip[cm_ptr->mko_rt.data[1]].pid,
											((cm_ptr->mko_rt.data[2] >> 8) & 0xFF) / 16.,
											((cm_ptr->mko_rt.data[2] >> 0) & 0xFF) / 16.,
											((cm_ptr->mko_rt.data[3] >> 8) & 0xFF) / 16.,
											((cm_ptr->mko_rt.data[3] >> 0) & 0xFF) / 16.
											);
							pid_refresh(&cm_ptr->hvip[cm_ptr->mko_rt.data[1]].pid);
						}
						break;
				}
				break;
			case CM_MKO_SA_ARCH_REQUEST_CM:
				if (cm_ptr->mko_rt.data[0] == 0){
					fr_mem_read_data_frame(&cm_ptr->mem, (uint8_t*)&frame);
					mko_rt_write_to_subaddr(&cm_ptr->mko_rt, CM_MKO_SA_ARCH_READ_CM, (uint16_t*)&frame);
				}
				break;
			case CM_MKO_SA_TECH:
				switch(cm_ptr->mko_rt.data[0]){
					case (TCMD_CHECK_MEM):
						fr_mem_check(&cm_ptr->mem);
						break;
					default:
						break;
				}
				break;
		}
	}
}

/**
 * @brief обработчик отладочных команд через ВШ
 *
 * @param cm_ptr указатель на объект управления ЦМ
 */
void cm_dbg_ib_command_handler(typeCMModel* cm_ptr)
{
	typeFrameStruct frame;
	//
	if (cm_ptr->ib.command_frame_flag){
		cm_ptr->ib.command_frame_flag = 0;
		if (cm_ptr->ib.command_frame.dev_id == CM_SELF_MB_ID){
			if (cm_ptr->ib.command_frame.f_code == MB_F_CODE_16){
				switch(cm_ptr->ib.command_frame.reg_addr){
					case CM_DBG_CMD_SWITCH_ON_OFF:
						cm_ptr->ib.global_dbg_flag = __REV16(cm_ptr->ib.command_frame.data[0]) & 0x01;
						break;
					case CM_DBG_CMD_CM_RESET:
						// todo: need to add cm_software or pwr reset
						break;
					case CM_DBG_CMD_CM_CHECK_MEM:
						fr_mem_check(&cm_ptr->mem);
						break;
					case CM_DBG_CMD_CM_INIT:
						if (__REV16(cm_ptr->ib.command_frame.data[0]) == 0xAA55) __main_base_init();
						break;
					case CM_DBG_CMD_ARCH_REQUEST:
						if (__REV16(cm_ptr->ib.command_frame.data[0]) == 0x0000){
							fr_mem_read_data_frame(&cm_ptr->mem, (uint8_t*)&frame);
							mko_rt_write_to_subaddr(&cm_ptr->mko_rt, CM_MKO_SA_ARCH_READ_CM, (uint16_t*)&frame);
						}
						break;
				}
			}
		}
	}
}

// Обработка callback-функций от прерываний

/**
  * @brief  обработчик прерывания АЦП
  */
void INT_ADC0_CallBack(void) {
	volatile uint32_t rslt;
	uint16_t channel, value;
	//
	NVIC_DisableIRQ(IRQn_ADC0);
	//
	while(cm.adc.regs->STATUS & 1) {
		rslt = cm.adc.regs->RESULT;
		channel = *((uint16_t*)&rslt + 1);
		value = *((uint16_t*)&rslt);
		if (channel < ADC0_CHAN_NUM){
			adc_new_val_process(&cm.adc, channel, value);
		}
	}
	//
	NVIC_EnableIRQ(IRQn_ADC0);
}

/**
  * @brief  обработчик прерывания МКО в режиме ОУ
  */
void INT_MIL0_Callback(void)
{

	mko_bc_transaction_handler(&cm.mko_bc);
}

/**
  * @brief  обработчик прерывания МКО в режиме КШ
  */
void INT_MIL1_Callback(void)
{
	mko_rt_transaction_handler(&cm.mko_rt);
}

/**
  * @brief  обработчик прерывания от SysTick-таймера
  */
void Systick_Handler(void){
	tp_timer_handler(&tp);
}
