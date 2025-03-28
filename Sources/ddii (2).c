/**
  ******************************************************************************
  * @file           : ddii.c
  * @version        : v1.0
  * @brief          : библиотека для управления модулем ДДИИ, использует объект MKO в режиме контроллера канала.
					Занимается опросом МПП по циклограмме.
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>, 
						Комаров Владислав <vlad.komarov2@gmail.com>
  * @date			: 2022.02.23
  ******************************************************************************
  */

#include "ddii.h"
#include "frame_mem.h"
#include <cstdint>
#include <stdint.h>


/**
	* @brief  инициализация структуры управления
	* @param  ddii_ptr указатель на структуру управления
	* @param self_num  номер устройства с точки зрения ЦМ
	* @param  mko_addr адресс устройства ДДИИ
	* @param  device_number номер устройства в котором работает ДДИИ
	* @param  frame_type тип кадра для ДДИИ
	* @param  mko_bc_ptr указатель на структуру управления МКО для ДДИИ
	* @param  gl_fr_num указатель на сковозной глобальный номер кадра
  */

void ddii_init(typeDDIIStruct* ddii_ptr,
            typeCMModel* cm,
            typeMPPStruct* mpp_ptr,
            uint8_t self_num,
            uint8_t mko_addr,
            uint16_t device_number,
            uint16_t frame_type,
            typeMKOStruct* mko_bc_ptr,
            uint8_t mko_bus,
            uint32_t* gl_fr_num){
	//                     
	
	ddii_ptr->mko_addr = mko_addr;
	ddii_ptr->mko_bus = mko_bus;
	ddii_ptr->device_number = device_number;
	ddii_ptr->frame_type = frame_type;
	ddii_ptr->mko_bc_ptr = mko_bc_ptr;
	ddii_ptr->self_num = self_num;
	ddii_ptr->global_frame_num_ptr = gl_fr_num;
	ddii_ptr->mem = cm->mem;
	ddii_ptr->mpp = mpp_ptr;
	ddii_ptr->cm = cm;
	ddii_ptr->cm->ib.global_dbg_flag = 0;
	ddii_ptr->interval_ms = DDII_DEFAULT_INTERVAL_MS;
	ddii_ptr->mode = COMBAT_MODE;
	ddii_reset_parameters(ddii_ptr);
	
	ddii_download_cfg(ddii_ptr);
	// включение ДДИИ (реализовано в main)
	// ...
	// работы с циклограммами
	ddii_meas_cycl_init(ddii_ptr);
}

void ddii_reset_parameters(typeDDIIStruct* ddii_ptr){
	// обнуляем кадр
	memset((uint8_t*)&ddii_ptr->frame, 0xFE, sizeof(ddii_ptr->frame));
	//
	ddii_ptr->frame_data_ready = 0x00;
	//
	memset((uint8_t*)ddii_ptr->rec_fifo, 0xFE, sizeof(ddii_ptr->rec_fifo));
	ddii_ptr->rec_num = 0;
	ddii_ptr->rec_max = 0;
	//
	ddii_ptr->last_call_time_us = 0;
	ddii_ptr->meas_event_num = 0;
	ddii_ptr->interval_ms = DDII_DEFAULT_INTERVAL_MS;
	ddii_ptr->mode = COMBAT_MODE;
	ddii_ptr->mko_bc_ptr->mko_rcv_flag = 0x00;
}

/////////////// Планировщик задач /////////////////
/**
	* @brief  процесс обработки состояния
	* @param  ddii_ptr указатель на структуру управления
	* @param  time_us время МК us
  */
int8_t ddii_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface){
	uint8_t retval = 0;
    typeDDIIStruct* ddii_ptr = (typeDDIIStruct*)ctrl_struct;
	uint16_t data[sizeof(ddii_ptr->ddii_mpp_data)];
	ddii_ptr->curent_time = time_us;
	/// запуск обработчика по интервалу
	if ((time_us - ddii_ptr->last_call_time_us) > (ddii_ptr->interval_ms*1000)) {
		ddii_ptr->last_call_time_us = time_us;
		// user code begin
		
		// user code end
		retval = 1;
	}
	// проверка режима
	if(ddii_ptr->mode != SILENT_MODE){
		// обработка event-ов  //todo: сделать специальные фукнции обработки event-ов
		if (interface->event[ddii_ptr->self_num] & DDII_EVENT_MEAS_INTERVAL_START) 
		{
			interface->event[ddii_ptr->self_num] &= ~DDII_EVENT_MEAS_INTERVAL_START;
			cyclo_start(&ddii_ptr->meas_cyclo);
			retval = 1;
		}
		else {
			//
		}
		// обработка циклограммы
		cyclo_handler(&ddii_ptr->meas_cyclo, time_us/1000);
		// обработка передачи данных
		if(ddii_ptr->frame_data_ready)
		{
			ddii_ptr->frame_data_ready = 0;
			memcpy(interface->shared_mem, (uint8_t*)&ddii_ptr->frame, sizeof(typeDDIIFrameUnion));
			interface->event[ddii_ptr->self_num] |= DDII_EVENT_MEAS_INTERVAL_DATA_READY;
			ddii_send_mko_frame(ddii_ptr);
			// дублирование команды мко во внутреннюю шину
			// ib_run_transaction(ddii_ptr->mpp->ib, ddii_ptr->mko_addr, MB_F_CODE_16, COMAND_SEND_FRAME, sizeof(ddii_ptr->frame)/2, (uint16_t*)&ddii_ptr->frame);
		}
		if (ddii_ptr->meas_cyclo.mode != CYCLO_MODE_WORK){
			if (ddii_ptr->mode == DEBUG_MODE){
				interface->event[ddii_ptr->self_num] |= DDII_EVENT_MEAS_INTERVAL_START;
			}
			if (ddii_ptr->mode == COMBAT_MODE){
				//if (ddii_ptr->mko_bc_ptr->mko_rcv_flag){ // Проверка статуса МКО
					interface->event[ddii_ptr->self_num] |= DDII_EVENT_MEAS_INTERVAL_START;
					ddii_ptr->mko_bc_ptr->mko_rcv_flag &= 0;
				//}
			}
		}
	}
	return retval;
}

void ddii_send_mko_frame(typeDDIIStruct* ddii_ptr){
	uint16_t ctrl_data[32];
	memcpy(ctrl_data, (uint8_t*)&ddii_ptr->frame, sizeof(ddii_ptr->frame));
	mko_bc_transaction_start(ddii_ptr->mko_bc_ptr, 
							MKO_MODE_WRITE, 
							ddii_ptr->mko_addr, 
							DDII_MKO_SA_CTRL, 
							ddii_ptr->mko_bus, 
							ctrl_data, 32);
}

/////////////// Циклограммы /////////////////
/**
  * @brief  инициализация циклограмм работы с ДДИИ
	* @param  ddii_ptr указатель на структуру управления
  */
void ddii_meas_cycl_init(typeDDIIStruct* ddii_ptr)
{
	// циклограмма инициализации МПП
	cyclo_init(&ddii_ptr->meas_cyclo);
	//
	ddii_ptr->meas_cyclo.mode = CYCLO_MODE_WORK;
	cyclo_add_step(&ddii_ptr->meas_cyclo, ddii_meas_cycl_start, (void*)ddii_ptr, ddii_ptr->interval_ms);
	cyclo_add_step(&ddii_ptr->meas_cyclo, ddii_meas_cycl_read, (void*)ddii_ptr, 0);
	cyclo_add_step(&ddii_ptr->meas_cyclo, ddii_meas_cycl_frame_forming, (void*)ddii_ptr, 0);
}

/**
  * @brief  обертка функция для согласования типов
  * @param  ctrl_struct указатель на структуру управления
  */
void ddii_meas_cycl_start(void* ctrl_struct)
{
	typeDDIIStruct* ddii_ptr = (typeDDIIStruct*) ctrl_struct;
	ddii_start_mpp_measure(ddii_ptr, 1);
	ddii_ptr->curent_time_measure = ddii_ptr->curent_time;
	// ddii_read_data_frame(ddii_ptr);
}

/**
    * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
void ddii_meas_cycl_read(void* ctrl_struct)
{
	typeDDIIStruct* ddii_ptr = (typeDDIIStruct*) ctrl_struct;
	// ddii_read_data_frame(ddii_ptr);
	ddii_start_mpp_measure(ddii_ptr, 0);
	ddii_mpp_get_data(ddii_ptr);
	ddii_struct_telemetria_forming(ddii_ptr);
}

/**
  * @brief  формирование кадра ДДИИ
	* @param  ctrl_struct указатель на структуру управления
  */
void ddii_meas_cycl_frame_forming(void* ctrl_struct)
{
	typeDDIIStruct* ddii_ptr = (typeDDIIStruct*) ctrl_struct;
	ddii_frame_forming(ddii_ptr);
}

void ddii_frame_struct_forming(typeDDIIStruct* ddii_ptr){
	uint16_t duration_measure = 0, tmp[2], tmp2[2];
	uint8_t i = 0;
	uint32_t count_1_6 = 0;
	uint16_t tmp_buf[sizeof(ddii_ptr->dataframe.constant_frame_data)];
	if (ddii_ptr->mode != CONSTATNT_MODE){
		// Длительность измерения
		duration_measure = (uint16_t)((ddii_ptr->curent_time - ddii_ptr->curent_time_measure)/1000);
		duration_measure = __REV16(duration_measure);
		memcpy(&ddii_ptr->dataframe.frame.duration_meas, (uint8_t*)&duration_measure, 2);
		memcpy(&ddii_ptr->dataframe.frame.particle, (uint8_t*)&ddii_ptr->ddii_mpp_data.particle, sizeof(ddii_ptr->ddii_mpp_data.particle));
		for(i = 0; i<sizeof(ddii_ptr->ddii_mpp_data.particle.hist.Hist16)/2; i++){
				count_1_6 += ddii_ptr->ddii_mpp_data.particle.hist.Hist16[i];
			}
		for(i = 3; i<6; i++){
				count_1_6 += ddii_ptr->ddii_mpp_data.particle.hist.Hist32[i];
		}
		memcpy(tmp, (uint16_t*)&count_1_6, 4);
		count_1_6 = __REV16(count_1_6);
		for(i=0; i<2; i++){
			tmp2[i] = tmp[1-i];
		}
		memcpy(&ddii_ptr->dataframe.frame.common_count_1_6, (uint16_t*)&tmp2, sizeof(count_1_6));	
	}
	else{
		ddii_constant_data_forming(ddii_ptr);
		memcpy(tmp_buf, ddii_ptr->dataframe.constant_frame_data, sizeof(tmp_buf));
		for (i = 0; i < sizeof(tmp_buf)/2; i++){
			tmp_buf[i] = __REV16(tmp_buf[i]);
		}
		memcpy(ddii_ptr->dataframe.constant_frame_data, (uint16_t*)&tmp_buf, sizeof(ddii_ptr->dataframe.constant_frame_data));
	}
}

///////////////Функции взаимодействия с конфигурацией /////////////////
/**
	* @brief  Загрузка конфигурации ддии из памяти
	* @param  ddii_ptr указатель на структуру управления
  */
void ddii_download_cfg(typeDDIIStruct* ddii_ptr){
	uint8_t ddii_cfg[64], i;
	fr_mem_format(&ddii_ptr->mem);///// 
	fr_mem_param_load(&ddii_ptr->mem, ddii_cfg);
	memcpy((uint8_t*)&ddii_ptr->cfg, (uint8_t*)&ddii_cfg, sizeof(typeDDII_cfg));
	if (ddii_ptr->cfg.head == HEAD){
		ddii_set_cfg(ddii_ptr);
	}
	else{
		fr_mem_format(&ddii_ptr->mem);
		ddii_set_default_cfg(ddii_ptr);
	}
}

void ddii_download_cfg_inmem(typeDDIIStruct* ddii_ptr){
	fr_mem_param_save(&ddii_ptr->mem, (uint8_t*)&ddii_ptr->cfg);
}

/**
	* @brief  Устанавка конфигурфции ДДИИ
	* @param  ddii_ptr указатель на структуру управления
  */
void ddii_set_cfg(typeDDIIStruct* ddii_ptr){
	ddii_ptr->mpp->id = ddii_ptr->cfg.mpp_id;
	ddii_mpp_set_level_triger(ddii_ptr);
	ddii_mpp_set_level_hh(ddii_ptr);

}

void ddii_set_cfg_parser(typeDDIIStruct* ddii_ptr, uint8_t data){
	memset(&ddii_ptr->cfg, data, sizeof(ddii_ptr->cfg));
}

/**
	* @brief  Устанавка дефолтной конфигурфции
	* @param  ddii_ptr указатель на структуру управления
	*
  */
void ddii_set_default_cfg(typeDDIIStruct* ddii_ptr){
	uint16_t level_hh[8] = {0, 64, 128, 256, 512, 1024, 1100, 1200};
	uint16_t level_trig[2] = {REG_MPP_LEVEL_TRIG, 10}, i;
	float pwm_buf[3] = {20.43, 17.4, 17.806}, voltage_buf[3] = {24, 24, 24};
	uint8_t buf_tmp[4];
	ddii_ptr->cfg.mpp_id = 15;
	memcpy(ddii_ptr->cfg.hvip_pwm_val, pwm_buf, sizeof(pwm_buf));
	memcpy(ddii_ptr->cfg.hvip_voltage, voltage_buf, sizeof(voltage_buf));
	memcpy((uint16_t*)&ddii_ptr->cfg.mpp_HH, (uint16_t*)&level_hh, sizeof(level_hh));
	memcpy((uint8_t*)&ddii_ptr->cfg.mpp_level_trig, (uint8_t*)&level_trig[1], sizeof(level_trig[1]));
	ddii_ptr->cfg.head = 0x0FF1;
	for (i = 0; i < sizeof(level_hh)/2; i++){
		level_hh[i] = __REV16(level_hh[i]);
	}
	for (i = 0; i < sizeof(level_trig)/2; i++){
		level_trig[i] = __REV16(level_trig[i]);
	}
	// trig
	ib_run_transaction(ddii_ptr->mpp->ib, ddii_ptr->mpp->id, MB_F_CODE_16, REG_MPP_COMAND, sizeof(level_trig)/2, level_trig);
	// level hh
	ib_run_transaction(ddii_ptr->mpp->ib, ddii_ptr->mpp->id, MB_F_CODE_16, REG_MPP_LEVEL_HH, sizeof(level_hh)/2, level_hh);
	ddii_ptr->interval_ms = DDII_DEFAULT_INTERVAL_MS;
	ddii_ptr->cfg.interval_measure = DDII_DEFAULT_INTERVAL_MS;
	ddii_download_cfg_inmem(ddii_ptr);
}

///////////////Функции для взаимодействия с mpp /////////////////

/**
	* @brief  Устанавливает тригер срабатывания МПП
	* @param  ddii_ptr указатель на структуру управления
  */
void ddii_mpp_set_level_triger(typeDDIIStruct* ddii_ptr){
	uint16_t data = ddii_ptr->cfg.mpp_level_trig;
	ib_run_transaction(ddii_ptr->mpp->ib, ddii_ptr->mpp->id, MB_F_CODE_3, REG_MPP_LEVEL_TRIG, sizeof(data), &data);
}

/**
	* @brief  Устанавливает уровни гистограмм МПП
	* @param  ddii_ptr указатель на структуру управления
  */
void ddii_mpp_set_level_hh(typeDDIIStruct* ddii_ptr){
	uint16_t* data = ddii_ptr->cfg.mpp_HH;
	ib_run_transaction(ddii_ptr->mpp->ib, ddii_ptr->mpp->id, MB_F_CODE_16, REG_MPP_LEVEL_HH, sizeof(data), data);
}

/**
  * @brief  Начать или остановить процесс регисрации
  * @param  ctrl_struct Указатель на структуру управления
  * @param  enable  включение/отключение реигтсрации мпп
  */
void ddii_start_mpp_measure(typeDDIIStruct* ddii_ptr, uint16_t enable){
	uint16_t data[2];
	data[0] = __REV16((COMAND_MPP_START_STOP_MEAS));
	data[1] = __REV16(enable); // 1 - start 0 - stop
	ib_run_transaction(ddii_ptr->mpp->ib, ddii_ptr->mpp->id, MB_F_CODE_16, REG_MPP_COMAND, 2, data);
}

/**
  * @brief  Запрос данных от мпп
  * @param  ctrl_struct Указатель на структуру управления
  * @param  enable  включение/отключение реигтсрации мпп
  */
void ddii_mpp_get_data(typeDDIIStruct* ddii_ptr){
	uint16_t buf[sizeof(ddii_ptr->ddii_mpp_data)];
	uint8_t reg_cnt = sizeof(ddii_ptr->ddii_mpp_data)/2 - 1, i;
	// data
	memset(buf, 0, sizeof(ddii_ptr->ddii_mpp_data)/2);
	ib_run_transaction(ddii_ptr->mpp->ib, ddii_ptr->mpp->id, MB_F_CODE_3, REG_MPP_STRUCT_DATA, reg_cnt, 0);
	ib_get_answer_data(ddii_ptr->mpp->ib, (uint8_t*)buf, reg_cnt*2+2);
	for (i = 0; i < reg_cnt; i++){
		buf[i] = __REV16(buf[i]);
	}
	memcpy((uint8_t*)&ddii_ptr->ddii_mpp_data, (uint8_t*)buf, sizeof(ddii_ptr->ddii_mpp_data) - 2);
	// level
	ib_run_transaction(ddii_ptr->mpp->ib, ddii_ptr->mpp->id, MB_F_CODE_3, REG_MPP_LEVEL, 1, 0);
	ib_get_answer_data(ddii_ptr->mpp->ib, (uint8_t*)buf, 2);
	for (i = 0; i < 2; i++){
		buf[i] = __REV16(buf[i]);
	}
	memcpy((uint8_t*)&ddii_ptr->ddii_mpp_data.Level, (uint8_t*)buf, 2);
}

void ddii_reset_mpp_struct_measure(typeDDIIStruct* ddii_ptr){
	uint8_t len = 36;
	uint16_t ctrl_data[36];
	memset(ctrl_data, 0x00, sizeof(ddii_ptr->ddii_mpp_data));
	ddii_start_mpp_measure(ddii_ptr, 0);
	ib_run_transaction(ddii_ptr->mpp->ib, ddii_ptr->mpp->id, MB_F_CODE_16, REG_MPP_HIST, len, ctrl_data);
	ddii_download_cfg(ddii_ptr);
}

/////////////// Кадры МКО /////////////////
/**
	* @brief  формирование данных для кадра ДДИИ
	* @param  ddii_ptr указатель на структуру управления
  */
// void diii_constant_frame_forming(typeDDIIStruct* ddii_ptr){
// 	uint16_t tmp_buf[sizeof(ddii_ptr->dataframe.constant_frame_data)], i;
// 	ddii_constant_data_forming(ddii_ptr);
// 	for (i = 0; i < sizeof(ddii_ptr->dataframe.constant_frame_data)/2; i++){
// 		ddii_ptr->dataframe.constant_frame_data[i] = __REV16(ddii_ptr->dataframe.constant_frame_data[i]);
// 	}
// 	ddii_frame_forming(ddii_ptr);
// 	// memcpy((uint8_t*)&ddii_ptr->frame.row.data, (uint8_t*)&ddii_ptr->frame.ddii.meas.data, sizeof(typeDDIIRec));
// 	ddii_ptr->frame.row.crc16 = frame_crc16((uint8_t*)&ddii_ptr->frame.row, sizeof(typeFrameStruct) - 2);
// }

/**
  * @brief  формирование кадра ДДИИ c выставлением флага
	* @param  ddii_ptr указатель на структуру управления
	* @retval  статус: 0 - кадр не сформирован, 1 - кадр сформирован
  */
int8_t ddii_frame_forming(typeDDIIStruct* ddii_ptr)
{
    uint8_t i = 0;
		typeDDIIRec rec_tmp;
		//
		if (ddii_ptr->rec_num >= DDII_MEAS_NUMBER){
			ddii_ptr->frame.row.label = 0x0FF1;
			ddii_ptr->frame.row.definer = frame_definer(0, ddii_ptr->device_number, NULL, ddii_ptr->frame_type);
			ddii_ptr->frame.row.num = (*ddii_ptr->global_frame_num_ptr++)&0xFFFF;
			ddii_ptr->frame.row.time = Get_Time_s();
			//
			// if (ddii_ptr->mode == CONSTATNT_MODE){
			// 	ddii_constant_data_forming(ddii_ptr);
			// }
			// if (ddii_ptr->mode == COMBAT_MODE){
			ddii_frame_struct_forming(ddii_ptr);
			memcpy(ddii_ptr->frame.ddii.meas.data, &ddii_ptr->dataframe, sizeof(ddii_ptr->dataframe));
				// ddii_put_data_in_row_frame_mko(ddii_ptr);
			// 	// memcpy((uint8_t*)&ddii_ptr->frame.row.data, (uint8_t*)&rec_tmp, sizeof(typeDDIIRec));
			// }
			
			//
			ddii_ptr->frame.row.crc16 = frame_crc16((uint8_t*)&ddii_ptr->frame.row, sizeof(typeFrameStruct) - 2);
			//
			ddii_ptr->frame_data_ready = 1;
			return 1;
		}
		else{
			return 0;
		}

}

/**
  * @brief  включение режима констант (не используется для ДДИИ)
	* @param  ddii_ptr указатель на структуру управления
	* @param  on_off 1 - включение, 0 - отключение
  */
void ddii_constant_mode(typeDDIIStruct* ddii_ptr, uint32_t on_off)
{
	uint16_t ctrl_data[32] = {0};
	// формирование команды
	ctrl_data[0] = 0x0FF1;
	ctrl_data[5] = DDII_MK_COMMAND_CONSTANT_MODE;
	ctrl_data[6] = (on_off & 0x01) ? 0x0001 : 0x0000;
	
	//....
	mko_bc_transaction_start(ddii_ptr->mko_bc_ptr, 
							MKO_MODE_WRITE, 
							ddii_ptr->mko_addr, 
							DDII_MKO_SA_CTRL, 
							ddii_ptr->mko_bus, 
							ctrl_data, 32);
}

/**
  * @brief  запуск ДДИИ
	* @param  ddii_ptr указатель на структуру управления
	* @param  ddii_rd_ptr установка указателя чтения архивной памяти
  */
// void ddii_start_mpp_mesure(typeDDIIStruct* ddii_ptr, uint32_t ddii_rd_ptr)
// {
// 	uint16_t ctrl_data[32] = {0};
// 	// формирование команды
// 	ctrl_data[0] = 0x0FF1;
// 	ctrl_data[5] = DDII_MK_COMMAND_START;
// 	ctrl_data[6] = ddii_rd_ptr & 0xFFFF;
// 	//....
// 	mko_bc_transaction_start(ddii_ptr->mko_bc_ptr, 
// 							MKO_MODE_WRITE, 
// 							ddii_ptr->mko_addr, 
// 							DDII_MKO_SA_CTRL, 
// 							ddii_ptr->mko_bus, 
// 							ctrl_data, 32);
// }



/**
  * @brief  чтение измерения ДДИИ
	* @param  ddii_ptr указатель на структуру управления
	* @param  meas_num W - нужное число измерений, R - оставшееся число измерений (+0)
  */
void ddii_read_data_frame(typeDDIIStruct *ddii_ptr)
{
	// mko_bc_transaction_start(ddii_ptr->mko_bc_ptr, 
	// 						MKO_MODE_READ, 
	// 						ddii_ptr->mko_addr, 
	// 						DDII_MKO_SA_DATA_FRAME, 
	// 						ddii_ptr->mko_bus, 
	// 						(uint16_t*)&ddii_ptr->data_frame, 32);
}

/**
  * @brief  запись измерения в fifo
	* @param  ddii_ptr указатель на структуру управления
	* @param  data указатель на массив с кадром данных
	* @retval  статус записи: 1 - ок, <0 - ошибка
  */
int8_t ddii_write_fifo(typeDDIIStruct *ddii_ptr, typeDDIIRec* data)
{
	if (ddii_ptr->rec_num >= DDII_REC_FIFO_DEPTH){
		return -1;
	}
	else{
		memcpy((uint8_t*)&ddii_ptr->rec_fifo[ddii_ptr->rec_num], (uint8_t*)data, sizeof(typeDDIIRec));
		ddii_ptr->rec_num += 1;
		if (ddii_ptr->rec_num > ddii_ptr->rec_max) ddii_ptr->rec_max = ddii_ptr->rec_num;  //сбор информации о заполненности fifo
		return 1;
	}
}

/**
  * @brief  взятие измерения из fifo
	* @param  ddii_ptr указатель на структуру управления
	* @param  data указатель на массив с кадром данных
	* @retval  статус: 0 - fifo-пуст, 1 - ок
  */
int8_t ddii_read_fifo(typeDDIIStruct* ddii_ptr, typeDDIIRec* data)
{
	if (ddii_ptr->rec_num == 0){
		return 0;
	}
	else{
		ddii_ptr->rec_num -= 1;
		memcpy((uint8_t*)data, (uint8_t*)&ddii_ptr->rec_fifo[0], sizeof(typeDDIIRec));
		memmove((uint8_t*)&ddii_ptr->rec_fifo[0], (uint8_t*)&ddii_ptr->rec_fifo[1], sizeof(typeDDIIRec)*ddii_ptr->rec_num);
		memset((uint8_t*)&ddii_ptr->rec_fifo[ddii_ptr->rec_num], 0x00, sizeof(typeDDIIRec)*(DDII_REC_FIFO_DEPTH - ddii_ptr->rec_num));
		return 1;
	}
}

/**
 * @brief переварачивание байт внутр 16-ти битных переменных
 * 
 * 
 * @param ddii_rec структура, принимаемая из модуля ДЭП по ВШ
 */
void _ddii_rec_rev(typeDDIIRec* ddii_rec)
{
  //
}

/**
 * @brief Устанавливает требуемое напряжение и pwm
 * 
 * @param ddii_rec структура, принимаемая из модуля ДЭП по ВШ
 */
void ddii_cmd_set_voltage_pwm(typeDDIIStruct* ddii_ptr, uint8_t *data){
	uint8_t tmp_buf[12], i;
  //
	memset(tmp_buf, 0x00, sizeof(tmp_buf));
	memcpy(tmp_buf, data, sizeof(tmp_buf));
	for(i =0; i < HVIP_NUM; i++){
		memcpy(&ddii_ptr->cm->hvip[i].v_hv_desired, &tmp_buf[i*4], sizeof(ddii_ptr->cm->hvip->v_hv_desired));
	}
	memset(tmp_buf, 0x00, sizeof(tmp_buf));
	memcpy(tmp_buf, &data[12], sizeof(tmp_buf));
	for(i = 0; i < HVIP_NUM; i++){
		memcpy(&ddii_ptr->cm->hvip[i].pwm_val_float, &tmp_buf[i*4], sizeof(ddii_ptr->cm->hvip->pwm_val_float));
	}
}

void ddii_struct_telemetria_forming(typeDDIIStruct* ddii_ptr){
	ddii_ptr->telmtr_struct.ddii_mode = 				ddii_ptr->mode;
	ddii_ptr->telmtr_struct.Level = 					ddii_ptr->ddii_mpp_data.Level;
	ddii_ptr->telmtr_struct.csa_test_on = 				ddii_ptr->csa_test.enable_test;
	ddii_ptr->telmtr_struct.ddii_interval_request = 	ddii_ptr->interval_ms;
	ddii_ptr->telmtr_struct.ACQ1_Peack = 				ddii_ptr->ddii_mpp_data.ACQ1_Peak;
	ddii_ptr->telmtr_struct.ACQ2_Peack = 				ddii_ptr->ddii_mpp_data.ACQ2_Peak;
	ddii_update_voltage(ddii_ptr);
	memcpy(ddii_ptr->telmtr_struct.particle_telmtr.hist.Hist32, ddii_ptr->dataframe.frame.particle.hist.Hist32, sizeof(ddii_ptr->ddii_mpp_data.particle.hist.Hist32));
	memcpy(ddii_ptr->telmtr_struct.particle_telmtr.hist.Hist16, ddii_ptr->dataframe.frame.particle.hist.Hist16, sizeof(ddii_ptr->ddii_mpp_data.particle.hist.Hist16));
	memcpy(ddii_ptr->telmtr_struct.particle_telmtr.hist.HCP, ddii_ptr->dataframe.frame.particle.hist.HCP, sizeof(ddii_ptr->ddii_mpp_data.particle.hist.HCP));
	memcpy(ddii_ptr->telmtr_struct.HH, ddii_ptr->ddii_mpp_data.HH, sizeof(ddii_ptr->ddii_mpp_data.HH));
}

void ddii_update_voltage(typeDDIIStruct* ddii_ptr){
	uint8_t i;
	for(i = 0; i < 3; i++){
		ddii_ptr->telmtr_struct.hvip_data[i].v_hv = ddii_ptr->cm->hvip[i].v_hv;
		ddii_ptr->telmtr_struct.hvip_data[i].hv_pwm = ddii_ptr->cm->hvip[i].pwm_val_float;
		ddii_ptr->telmtr_struct.hvip_data[i].hv_curent = ddii_ptr->cm->hvip[i].current;
		ddii_ptr->telmtr_struct.hvip_data[i].hv_mode = ddii_ptr->cm->hvip[i].mode;
	}
}

/**
	* @brief  формирование данных для кадра ДДИИ
	* @param  ddii_ptr указатель на структуру управления
  */
void ddii_constant_data_forming(typeDDIIStruct* ddii_ptr){
	ddii_ptr->dataframe.constant_frame_data[0] 	= 0xE0;
	ddii_ptr->dataframe.constant_frame_data[1] 	= 0xE1;
	ddii_ptr->dataframe.constant_frame_data[2] 	= 0xE0;
	ddii_ptr->dataframe.constant_frame_data[3] 	= 0xE1;
	ddii_ptr->dataframe.constant_frame_data[4] 	= 0xE2;
	ddii_ptr->dataframe.constant_frame_data[5] 	= 0xE3;
	ddii_ptr->dataframe.constant_frame_data[6] 	= 0xE4;
	ddii_ptr->dataframe.constant_frame_data[7] 	= 0xE5;
	ddii_ptr->dataframe.constant_frame_data[8] 	= 0xE6;
	ddii_ptr->dataframe.constant_frame_data[9] 	= 0xE7;
	ddii_ptr->dataframe.constant_frame_data[10] = 0xE8;
	ddii_ptr->dataframe.constant_frame_data[11] = 0xE9;
	ddii_ptr->dataframe.constant_frame_data[12] = 0xE0;
	ddii_ptr->dataframe.constant_frame_data[13] = 0xE1;
	ddii_ptr->dataframe.constant_frame_data[14] = 0xE2;
	ddii_ptr->dataframe.constant_frame_data[15] = 0xE3;
	ddii_ptr->dataframe.constant_frame_data[16] = 0xE4;
	ddii_ptr->dataframe.constant_frame_data[17] = 0xE5;
	ddii_ptr->dataframe.constant_frame_data[18] = 0xE6;
	ddii_ptr->dataframe.constant_frame_data[19] = 0xE7;
	ddii_ptr->dataframe.constant_frame_data[20] = 0xE8;
	ddii_ptr->dataframe.constant_frame_data[21] = 0xE9;
	ddii_ptr->dataframe.constant_frame_data[22] = 0xEA;
	ddii_ptr->dataframe.constant_frame_data[23] = 0xEB;
	ddii_ptr->dataframe.constant_frame_data[24] = 0xEC;
	ddii_ptr->dataframe.constant_frame_data[25] = 0xED;
	ddii_ptr->dataframe.constant_frame_data[26] = 0xEE;
	ddii_ptr->dataframe.constant_frame_data[27] = 0xEF;
	ddii_ptr->dataframe.constant_frame_data[28] = 0xE0;
	ddii_ptr->dataframe.constant_frame_data[29] = 0xE1;
	ddii_ptr->dataframe.constant_frame_data[30] = 0xE2;
	ddii_ptr->dataframe.constant_frame_data[31] = 0xE3;
	ddii_ptr->dataframe.constant_frame_data[32] = 0xE4;
	ddii_ptr->dataframe.constant_frame_data[33] = 0xE5;
	ddii_ptr->dataframe.constant_frame_data[34] = 0xE6;
	ddii_ptr->dataframe.constant_frame_data[35] = 0xE7;
	ddii_ptr->dataframe.constant_frame_data[36] = 0xE8;
	ddii_ptr->dataframe.constant_frame_data[37] = 0xE9;
	ddii_ptr->dataframe.constant_frame_data[38] = 0xEA;
	ddii_ptr->dataframe.constant_frame_data[39] = 0xE0;
	ddii_ptr->dataframe.constant_frame_data[40] = 0xE1;
	ddii_ptr->dataframe.constant_frame_data[41] = 0xE2;
	ddii_ptr->dataframe.constant_frame_data[42] = 0xE3;
	ddii_ptr->dataframe.constant_frame_data[43] = 0xE4;
	ddii_ptr->dataframe.constant_frame_data[44] = 0xE5;
	ddii_ptr->dataframe.constant_frame_data[45] = 0xE6;
	ddii_ptr->dataframe.constant_frame_data[46] = 0xE7;
	ddii_ptr->dataframe.constant_frame_data[47] = 0xE8;
	ddii_ptr->dataframe.constant_frame_data[48] = 0xE0;
	ddii_ptr->dataframe.constant_frame_data[49] = 0xE1;
	ddii_ptr->dataframe.constant_frame_data[50] = 0xE2;
	ddii_ptr->dataframe.constant_frame_data[51] = 0xE3;
}


