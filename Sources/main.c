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
#include "gpio.h"
#include "timers.h"
#include "termo_res.h"
#include "ddii.h"

// создание объектов отдельных программных моделей
typeTPStruct tp;	//! обязательный объект планировщика задач
typeCMModel cm;		//! объект управления переферией ЦМ
typeMPPStruct mpp[MPP_DEV_NUM]; //! програмная модель переферийных модулей МПП
typeFrameStruct ddii_frame; //! кадр ДДИИ
uint8_t ctrl_data[52];
uint16_t ctrl_frame[32];
type_SINGLE_GPIO ddii_gpio_ptr;
typeDDIIStruct ddii;
// type_CSA_TEST_Struct csa_test;
// прототипы функций для использования внутри main.c (описание в фунциях)
void __main_process_registration_box(void);
void __main_init_perepherial_modules(void);
void __main_base_init(void);
// void ddii_constant_data_forming(uint8_t* data);
void ddii_struc_to_data16_t(typeFrameStruct* ptr_frame, uint16_t* data);
void ddii_constant_frame_forming(typeFrameStruct* ptr_frame, uint8_t* data);
// void ddii_power_detector_on(type_SINGLE_GPIO* ddii_gpio_ptr);

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
	//cm_load_cfg(&cm);	// загрузка конфигурации из энергонезависимой памяти
	pwr_all_on_off(&cm.pwr, 1);  // включение переферии согласно настройкам каналов управления питанием
	// ddii_power_detector_on(&ddii_gpio_ptr);
	__main_init_perepherial_modules();  // инициализация объектов переферии, подключаемых к ЦМ
	//
	tp_init(&tp);  // иниицализация планировщика задач
	// Системные процессы
	csa_test_init(&ddii.csa_test);
	__main_process_registration_box();  // регистрация процессов обработки ЦМ и переферии
	//ddii_constant_data_forming(ctrl_data);
	//ddii_constant_frame_forming(&ddii_frame, ctrl_data);
	//ddii_struc_to_data16_t(&ddii_frame, ctrl_frame);
	//mko_rt_write_to_subaddr(&cm.mko_rt, 1, ctrl_frame);
	while(1) {
		tp_handler(&tp); // обработка планировщика задач
	}
}

/*
  * @brief  обертка для инициализации всей переферии
*/
// Бесполезная функция. TODO: Убрать
// void ddii_power_detector_on(type_SINGLE_GPIO* ddii_gpio_ptr){
	
// 	gpio_init(ddii_gpio_ptr, PORTC, 15);
// 	Timer_Delay(6000);
// 	gpio_set(ddii_gpio_ptr, 0);
// }
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
		mpp_init(&mpp[uint8_var],
				MPP1 + uint8_var,
				mpp_ib_id[uint8_var],
				FRAME_DEV_ID, 
				MPP1+uint8_var+1,
				mpp_ch_num[uint8_var],
				mpp_offsets_array[uint8_var],
				&cm.ib,
				&cm.global_frame_num);
	}
	// инициализация ДДИИ
	ddii_init(&ddii,
			&cm,
			&mpp[0],
			DDII, 
			MKO_ADDRESS_DEFAULT, 
			FABRICATION_NUMBER, 
			DDII+1,
			&cm.mko_rt,
			MKO_BUS_A,
			&cm.global_frame_num);

	// опрделитель кадра ддии - 0C8E 
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
	// процесс тест ЗЧУ
	tp_process_registration(&tp, &csa_test_process_tp, &ddii.csa_test, 0, 0);
	tp_process_registration(&tp, &ddii_process_tp, &ddii, 0, 0);
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
	pwr_all_on_off(&cm.pwr, 0);  //todo: возможно необходимо поместить данную функцию после инициализации ЦМ
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
						// __main_base_init();
						break;
					case (CMD_REC_FRAME):
						ddii_send_mko_frame(&ddii);
						// формирование команды
						// ctrl_data[1] = 0x0FF1;
						// ctrl_data[2] = DDII_MK_COMMAND_CONSTANT_MODE;
						// ctrl_data[3] = 1;
						//diii_constant_frame_forming(&ddii);
						//memcpy((uint16_t*)ctrl_data, (uint16_t*)&ddii.frame.row, sizeof(typeFrameStruct));
						// memcopy((uint16_t*)&ddii.frame.row, (uint16_t*)&ddii.frame.row, sizeof(typeFrameStruct));
						
						//ddii_constant_data_forming(ctrl_data);
						//ddii_constant_frame_forming(&ddii_frame, ctrl_data);
						//ddii_struc_to_data16_t(&ddii_frame, ctrl_frame);
						//mko_rt_write_to_subaddr(&cm.mko_rt, 1, ctrl_frame);
						// mko_bc_transaction_start(&cm_ptr->mko_rt,
						// 						MKO_MODE_WRITE,
						// 						13,
						// 						DDII_MKO_SA_CTRL,
						// 						1,
						// 						(uint16_t*)&ctrl_data,
						// 						32);
						break;
					case (CMD_SET_INTERVAL):
						cm_set_interval_value(cm_ptr, cm_ptr->mko_rt.data[1], cm_ptr->mko_rt.data[2]);
						break;
					case (CMD_SET_MPP_OFFSET):
						if ((cm_ptr->mko_rt.data[1] >= 1) && (cm_ptr->mko_rt.data[1] <= MPP_DEV_NUM)) {
							mpp_set_offset(&mpp[cm_ptr->mko_rt.data[1] - 1], cm_ptr->mko_rt.data[2]);
						}
						break;
					case (CMD_CONST_MODE): // 7?
						if (cm_ptr->mko_rt.data[1] == 1){
							ddii.mode = CONSTATNT_MODE;
							// mpp_constant_mode(&mpp[0], 1); // команда шировковещательная
							// ddii_constant_mode(&ddii, 1);
						} 
						else if (cm_ptr->mko_rt.data[1] == 0){
							ddii.mode = COMBAT_MODE;
							// mpp_constant_mode(&mpp[0], 0);
							// ddii_constant_mode(&ddii, 0);
						}
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
					case (CM_MKO_SET_HH):
						memcpy(&ddii.cfg.mpp_HH, &cm_ptr->mko_rt.data[1], 2 * 8);
						ddii_mpp_set_level_hh(&ddii);
						ddii_download_cfg_inmem(&ddii);
						break;
				}
				break;
			// case CM_MKO_SA_ARCH_REQUEST_CM:
			// 	if (cm_ptr->mko_rt.data[0] == 0){
			// 		fr_mem_read_data_frame(&cm_ptr->mem, (uint8_t*)&frame);
			// 		mko_rt_write_to_subaddr(&cm_ptr->mko_rt, CM_MKO_SA_ARCH_READ_CM, (uint16_t*)&frame);
			// 	}
			// 	break;
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
	// ddii_constant_mode();

	// if (mko_need_to_process(&cm_ptr->mko_bc)){
	// 	switch(cm_ptr->mko_bc.cw.field.sub_addr){
	// 		case():
				
	// 	}
	// }
}

/**
 * @brief обработчик отладочных команд через ВШ
 *
 * @param cm_ptr указатель на объект управления ЦМ
 */
void cm_dbg_ib_command_handler(typeCMModel* cm_ptr)
{
	uint16_t tmp_buf[200] = {0xFEFE};
	//
	if (cm_ptr->ib.command_frame_flag){
		cm_ptr->ib.command_frame_flag = 0;
		if (cm_ptr->ib.command_frame.dev_id == CM_SELF_MB_ID){
			if (cm_ptr->ib.command_frame.f_code == MB_F_CODE_16){
				switch(cm_ptr->ib.command_frame.reg_addr){
					case CM_DBG_CMD_SWITCH_MODE:
						if (cm_ptr->ib.command_frame.data[1] != COMBAT_MODE){
							mpp->ib->global_dbg_flag = 0x00;
							if (cm_ptr->ib.command_frame.data[1] == CONSTATNT_MODE){
								ddii.mode = CONSTATNT_MODE;
								ib_run_transaction(&cm_ptr->ib, CM_SELF_MB_ID, MB_F_CODE_16, CM_DBG_CMD_SWITCH_MODE,1, (uint16_t*)&ddii.mode);
							}
							if (cm_ptr->ib.command_frame.data[1] == SILENT_MODE){
								ddii.mode = SILENT_MODE;
								mpp->ib->global_dbg_flag = 0x01;
								ib_run_transaction(&cm_ptr->ib, CM_SELF_MB_ID, MB_F_CODE_16, CM_DBG_CMD_SWITCH_MODE,1, (uint16_t*)&ddii.mode);
							}
							if (cm_ptr->ib.command_frame.data[1] == DEBUG_MODE){
								ddii.mode = DEBUG_MODE;
								mpp->ib->global_dbg_flag = 0x01;
								ib_run_transaction(&cm_ptr->ib, CM_SELF_MB_ID, MB_F_CODE_16, CM_DBG_CMD_SWITCH_MODE,1, (uint16_t*)&ddii.mode);
							}
						} 
						else{
							mpp->ib->global_dbg_flag = 0x00;
							ddii.mode = COMBAT_MODE;
						}
						break;
					case CMD_HVIP_ON_OFF:
						mpp->ib->global_dbg_flag = 0x01;
						ib_run_transaction(&cm_ptr->ib, CM_SELF_MB_ID, MB_F_CODE_16, cm_ptr->ib.command_frame.reg_addr,cm_ptr->ib.command_frame.reg_cnt, NULL);
						mpp->ib->global_dbg_flag = 0x00;
						if (cm_ptr->ib.command_frame.data[1] == PIPS_CH_VOLTAGE){
							ddii.cm->hvip[1].mode = cm_ptr->ib.command_frame.data[3];
						}
						else if(cm_ptr->ib.command_frame.data[1] == SIPM_CH_VOLTAGE){
							ddii.cm->hvip[2].mode = cm_ptr->ib.command_frame.data[3];
						}
						else if(cm_ptr->ib.command_frame.data[1] == CHERENKOV_CH_VOLTAGE){
							ddii.cm->hvip[0].mode = cm_ptr->ib.command_frame.data[3];
						}
						break;
					case CMD_DBG_UPDATE_DATA:
						mpp->ib->global_dbg_flag = 0x01;
						ib_run_transaction(&cm_ptr->ib, CM_SELF_MB_ID, MB_F_CODE_16, cm_ptr->ib.command_frame.reg_addr,cm_ptr->ib.command_frame.reg_cnt, NULL);
						Timer_Delay(5);
						ddii_mpp_get_data(&ddii);
						mpp->ib->global_dbg_flag = 0x00;
						break;
					case CM_DBG_CMD_CSA_TEST_ENABLE:
						if (cm_ptr->ib.command_frame.data[1] == 1){
							ddii.csa_test.enable_test = 1;
						}
						else{
							ddii.csa_test.enable_test = 0;
						}
						break;
					case CM_DBG_UPDATE_CFG:
						mpp->ib->global_dbg_flag= 0x01;
						ib_run_transaction(&cm_ptr->ib, CM_SELF_MB_ID, MB_F_CODE_16, cm_ptr->ib.command_frame.reg_addr,cm_ptr->ib.command_frame.reg_cnt, NULL);
						Timer_Delay(5);
						ddii_update_cfg(&ddii, cm_ptr->ib.command_frame.data);
						mpp->ib->global_dbg_flag = 0x00;
						break;
					case CM_DBG_CMD_CM_RESET:
						// todo: need to add cm_software or pwr reset
						break;
					case CM_DBG_CMD_CM_CHECK_MEM:
						fr_mem_check(&cm_ptr->mem);
						break;
					// case CM_DBG_CMD_CM_INIT:
					// 	if (__REV16(cm_ptr->ib.command_frame.data[0]) == 0xAA55) __main_base_init();
					// 	break;
					// case CM_DBG_CMD_ARCH_REQUEST:
					// 	if (__REV16(cm_ptr->ib.command_frame.data[0]) == 0x0000){
					// 		fr_mem_read_data_frame(&cm_ptr->mem, (uint8_t*)&frame);
					// 		mko_rt_write_to_subaddr(&cm_ptr->mko_rt, CM_MKO_SA_ARCH_READ_CM, (uint16_t*)&frame);
					// 		// ib_run_transaction(&cm_ptr->ib, 0xFF, 106, 0, 2, (uint16_t*)&frame);
					// 	}
					// 	break;
					case CM_DBG_SET_VOLTAGE:
					// TODO: Проверить
						mpp->ib->global_dbg_flag = 0x01;
						ib_run_transaction(&cm_ptr->ib, CM_SELF_MB_ID, MB_F_CODE_16, cm_ptr->ib.command_frame.reg_addr,cm_ptr->ib.command_frame.reg_cnt, NULL);
						ddii_cmd_set_voltage_pwm(&ddii, cm_ptr->ib.command_frame.data);
						mpp->ib->global_dbg_flag = 0x00;
						break;
					case CM_DBG_SET_HVIP_AB:
					// TODO: Проверить
						mpp->ib->global_dbg_flag = 0x01;
						ib_run_transaction(&cm_ptr->ib, CM_SELF_MB_ID, MB_F_CODE_16, cm_ptr->ib.command_frame.reg_addr,cm_ptr->ib.command_frame.reg_cnt, NULL);
						ddii_hvip_set_coef_a_b(&ddii, cm_ptr->ib.command_frame.data);
						mpp->ib->global_dbg_flag = 0x00;
						break;
					case SET_DEFAULT_CFG:
						ib_run_transaction(&cm_ptr->ib, CM_SELF_MB_ID, MB_F_CODE_16, cm_ptr->ib.command_frame.reg_addr,cm_ptr->ib.command_frame.reg_cnt, NULL);
						fr_mem_format(&ddii.mem);
						ddii_set_default_cfg(&ddii);
						break;
					case SET_VOLTAGE_CORRECTION_MODE:
						ib_run_transaction(&cm_ptr->ib, CM_SELF_MB_ID, MB_F_CODE_16, cm_ptr->ib.command_frame.reg_addr,cm_ptr->ib.command_frame.reg_cnt, NULL);
						ddii.voltage_correction_mode = cm_ptr->ib.command_frame.data[0];
				}
			}
			if (cm_ptr->ib.command_frame.f_code == MB_F_CODE_3){
				switch(cm_ptr->ib.command_frame.reg_addr){
					case CMD_DBG_GET_CFG:
						mpp->ib->global_dbg_flag = 0x01;
						memcpy(tmp_buf, (uint16_t*)&ddii.cfg, sizeof(ddii.cfg));
						ib_run_transaction(&cm_ptr->ib, CM_SELF_MB_ID, MB_F_CODE_3, 0, cm_ptr->ib.command_frame.byte_cnt/2, tmp_buf);
						mpp->ib->global_dbg_flag = 0x00;
						break;
					case CMD_DBG_GET_TELEMETRIA:
						mpp->ib->global_dbg_flag = 0x01;
						ddii_struct_telemetria_forming(&ddii);
						memcpy(tmp_buf, (uint16_t*)&ddii.telmtr_struct, sizeof(ddii.telmtr_struct));
						ib_run_transaction(&cm_ptr->ib, CM_SELF_MB_ID, MB_F_CODE_3, 0, cm_ptr->ib.command_frame.byte_cnt/2, tmp_buf);
						mpp->ib->global_dbg_flag = 0x00;
						break;
					// TODO: Проверить
					case CM_DBG_GET_CFG_VOLTAGE:
						mpp->ib->global_dbg_flag = 0x01;
						memcpy(tmp_buf, (uint16_t*)&ddii.cfg.hvip_voltage, sizeof(ddii.cfg.hvip_voltage));
						ib_run_transaction(&cm_ptr->ib, CM_SELF_MB_ID, MB_F_CODE_3, cm_ptr->ib.command_frame.reg_addr, cm_ptr->ib.command_frame.byte_cnt/2, tmp_buf);
						mpp->ib->global_dbg_flag = 0x00;
						break;
					case CM_DBG_GET_CFG_PWM:
						mpp->ib->global_dbg_flag = 0x01;
						memcpy(tmp_buf, (uint16_t*)&ddii.cfg.hvip_pwm_val, sizeof(ddii.cfg.hvip_pwm_val));
						ib_run_transaction(&cm_ptr->ib, CM_SELF_MB_ID, MB_F_CODE_3, cm_ptr->ib.command_frame.reg_addr, cm_ptr->ib.command_frame.byte_cnt/2, tmp_buf);
						mpp->ib->global_dbg_flag = 0x00;
						break;
					case CM_DBG_GET_VOLTAGE:
						mpp->ib->global_dbg_flag = 0x01;
						ddii_update_voltage(&ddii);
						memcpy(tmp_buf, (uint16_t*)&ddii.telmtr_struct.hvip_data, sizeof(typeDDII_HVIP_Data)*HVIP_NUM);
						ib_run_transaction(&cm_ptr->ib, CM_SELF_MB_ID, MB_F_CODE_3, 0, cm_ptr->ib.command_frame.byte_cnt/2, tmp_buf);
						mpp->ib->global_dbg_flag = 0x00;
						break;
					case CM_DBG_GET_HVIP_AB:
					// TODO: Проверить
						mpp->ib->global_dbg_flag = 0x01;
						ddii_hvip_get_coef_a_b(&ddii, cm_ptr->ib.command_frame.data);
						memcpy(tmp_buf, (uint16_t*)&ddii.hvip_AB, sizeof(typeDDIIhvip_AB)*HVIP_NUM);
						ib_run_transaction(&cm_ptr->ib, CM_SELF_MB_ID, MB_F_CODE_3, 0, cm_ptr->ib.command_frame.byte_cnt/2, tmp_buf);
						mpp->ib->global_dbg_flag = 0x00;
						break;
					case CM_GET_TERM:
						memcpy(tmp_buf, (uint16_t*)&ddii.term_struct, sizeof(ddii.term_struct));
						ib_run_transaction(&cm_ptr->ib, CM_SELF_MB_ID, MB_F_CODE_3, 0, cm_ptr->ib.command_frame.byte_cnt/2, tmp_buf);
						break;
				}
			}
		}
	}
}

void ddii_struc_to_data16_t(typeFrameStruct* ptr_frame, uint16_t* data){
	uint16_t *ptr = (uint16_t*)ptr_frame;
	uint16_t i;
	for (i = 0; i < 32; ++i) {
    	data[i] = *ptr++;
	}
}

void ddii_constant_frame_forming(typeFrameStruct* ptr_frame, uint8_t* data){
	ptr_frame->label = 0x0FF1;
	ptr_frame->definer = frame_definer(1, FRAME_DEV_ID, 1, 2); // 0x0C8E
	ptr_frame->num = (cm.global_frame_num++)&0xFFFF;
	ptr_frame->time = Get_Time_s();
	memcpy((uint8_t*)ptr_frame->data, (uint8_t*)data, 52);
	ptr_frame->crc16 = frame_crc16((uint8_t*)ptr_frame, sizeof(typeFrameStruct) - 2);
}



// Обработка callback-функций от прерываний

/**
  * @brief  обработчик прерывания АЦП
  */
void INT_ADC0_CallBack(void){
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

void INT_TMR1_CallBack(void) {
	if (ddii.csa_test.enable_test == 1){
		csa_all_together_test(&ddii.csa_test);
		Timer_Start(ddii.csa_test.timer, 1);
	}
	else
		csa_all_together_off(&ddii.csa_test);
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
