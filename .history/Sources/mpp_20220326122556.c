  /**
  ******************************************************************************
  * @file           : mpp.c
  * @version        : v1.0
  * @brief          : библиотека для управления отдельным каналом МПП, использует объект ВШ (internal_bus)
	* @note						: возможно использовать два отдельных канала в одном МПП как два независимых устройства (такой подход добавляет небольшой оверхэд, но объеденяет описание МПП с одним или несколькими каналами)
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  * @date			: 2021.09.22
  ******************************************************************************
  */

#include "mpp.h"

/**
  * @brief  инициализация структуры управления
	* @param  mpp_ptr указатель на структуру управления
	* @param  id номер устройства на внутренней шине
	* @param  device_number номер устройства в котором работает МПП
	* @param  frame_type тип кадра для МПП
	* @param  channel номер используемого канала МПП
	* @param  offset значение уставки определение помехи в кв АЦП
	* @param  ib_ptr указатель на внутреннюю шину
	* @param  gl_fr_num указатель на сковозной глобальный номер кадра
  */
void mpp_init(typeMPPStruct* mpp_ptr, uint8_t self_num, uint8_t id, uint16_t device_number, uint16_t frame_type, uint8_t channel, uint32_t offset, typeIBStruct* ib_ptr, uint32_t* gl_fr_num)
{
  mpp_reset_parameters(mpp_ptr);
	mpp_ptr->id = id;
	mpp_ptr->device_number = device_number;
	mpp_ptr->frame_type = frame_type;
	mpp_ptr->channel = channel;
	mpp_ptr->ib = ib_ptr;
	mpp_ptr->self_num = self_num;
	mpp_ptr->global_frame_num_ptr = gl_fr_num;
	// учтановка параметров по умолчанию
	// todo: возможно в инициализацию необходимо добавить прописывание offset и pwr_bound
	mpp_ptr->offset = offset;
	mpp_set_offset(mpp_ptr, offset);
	// работы с циклограммамив  МПП
	mpp_meas_cycl_init(mpp_ptr);
}

/**
  * @brief  инициализация структуры управления
	* @param  mpp_ptr указатель на структуру управления
  */
void mpp_reset_parameters(typeMPPStruct* mpp_ptr)
{
	//зануление данных на передачу в МКО и память
	memset((uint8_t*)mpp_ptr->rec_buff, 0xFE, sizeof(mpp_ptr->rec_buff));
	memset((uint8_t*)&mpp_ptr->frame, 0xFE, sizeof(mpp_ptr->frame));
	mpp_ptr->frame_data_ready = 0x00;
	mpp_ptr->rec_ptr = 0;
	//
	mpp_ptr->offset = 0; 
	mpp_ptr->mode = 0; 
	mpp_ptr->pwr_off_bound = 0; 
	mpp_ptr->forced_start_flag = 0;
	mpp_ptr->forced_start_timeout = 0;
	mpp_ptr->frame_pulse_cnt = 0;
	mpp_ptr->last_call_time_us = 0;
	mpp_ptr->const_mode = 0;
	mpp_ptr->interval_ms = MPP_DEFAULT_INTERVAL_MS;
	mpp_ptr->meas_event_num = 0;
}

/**
  * @brief  процесс обработки состояния МПП
	* @param  mpp_ptr указатель на структуру управления
	* @param  time_us время МК us
  */
int8_t mpp_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface)
{
	uint8_t retval = 0;
  typeMPPStruct* mpp_ptr = (typeMPPStruct*)ctrl_struct;
	/// запус обработчика по интервалу
	if ((time_us - mpp_ptr->last_call_time_us) > (mpp_ptr->interval_ms*1000)) {
		mpp_ptr->last_call_time_us = time_us;
		// user code begin
		
		// user code end
		retval = 1;
	}
	// обработка event-ов  //todo: сделать специальные фукнции обработки event-ов
	if (interface->event[mpp_ptr->self_num] & MPP_EVENT_MEAS_INTERVAL_START) 
	{
		interface->event[mpp_ptr->self_num] &= ~MPP_EVENT_MEAS_INTERVAL_START;
		cyclo_start(&mpp_ptr->meas_cyclo);
		retval = 1;
	}
	else {
		//
	}
	//обработка циклограммы
	cyclo_handler(&mpp_ptr->meas_cyclo, time_us/1000);
	//обработка передачи данных
	if(mpp_ptr->frame_data_ready)
	{
		mpp_ptr->frame_data_ready = 0;
		memcpy(interface->shared_mem, (uint8_t*)&mpp_ptr->frame, sizeof(typeMPPFrameUnion));
		interface->event[mpp_ptr->self_num] |= MPP_EVENT_MEAS_INTERVAL_DATA_READY;
	}
	return retval;
}


/**
  * @brief  функция получения данных для складывания в кадр
	* @param  mpp_ptr указатель на структуру управления
	* @param  struct_num указатель на кадр
	* @retval  1 - кадр сформирован, 0 - недостаточно данных для кадра
  */
int8_t mpp_frame_forming(typeMPPStruct* mpp_ptr)
{
	if(mpp_ptr->rec_ptr >= 2){
		mpp_ptr->frame.row.label = 0x0FF1;
		mpp_ptr->frame.row.definer = frame_definer(0, mpp_ptr->device_number, NULL, mpp_ptr->frame_type);
		mpp_ptr->frame.row.num = (*mpp_ptr->global_frame_num_ptr++)&0xFFFF;
		mpp_ptr->frame.row.time = Get_Time_s();
		mpp_ptr->rec_ptr -= 1;
		mpp_ptr->frame.mpp.rec[0] = mpp_ptr->rec_buff[mpp_ptr->rec_ptr];
		mpp_ptr->rec_ptr -= 1;
		mpp_ptr->frame.mpp.rec[1] = mpp_ptr->rec_buff[mpp_ptr->rec_ptr];
		mpp_ptr->frame.row.crc16 = frame_crc16((uint8_t*)&mpp_ptr->frame.row, sizeof(typeFrameStruct) - 2);
		//
		mpp_ptr->frame_data_ready = 1;
		return 1;
	}
	else{
		return 0;
	}
}

/**
  * @brief  установка времени МПП (широковещательная)
	* @param  mpp_ptr указатель на структуру управления
	* @param  time_s время МК s
  */
void mpp_time_set(typeMPPStruct* mpp_ptr, uint32_t time_s)
{
	uint16_t data[2];
	data[0] = (time_s >> 16) & 0xFFFF;
	data[1] = (time_s >> 0) & 0xFFFF; 
	ib_run_transaction(mpp_ptr->ib, 0xFF, 111, 0, 2, data);
}

/**
  * @brief  включение отключение работы отдельного канала
	* @param  mpp_ptr указатель на структуру управления
	* @param  on_off 1 - включение, 0 - отключение
  */
void mpp_on_off(typeMPPStruct* mpp_ptr, uint32_t on_off)
{
	uint16_t data[2];
	mpp_ptr->mode = (on_off) ? 0x01 : 0x00;
	data[0] = __REV16((mpp_ptr->channel << 8) | (2));
	data[1] = __REV16(mpp_ptr->mode); 
	ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 16, 0, 2, data);
}

/**
  * @brief  включение отключение режима константа (широковещательная)
	* @param  mpp_ptr указатель на структуру управления
	* @param  on_off 1 - включение, 0 - отключение
  */
void mpp_constant_mode(typeMPPStruct* mpp_ptr, uint32_t on_off)
{
	uint16_t data[2];
	mpp_ptr->const_mode = (on_off) ? 0x55AA : 0x5500;
	data[0] = __REV16(0x00);
	data[1] = __REV16(mpp_ptr->const_mode); 
	ib_run_transaction(mpp_ptr->ib, 0xFF, 106, 0, 2, data);
}

/**
  * @brief  инициализация памяти
	* @param  mpp_ptr указатель на структуру управления
  */
void mpp_arch_mem_init(typeMPPStruct* mpp_ptr)
{
	uint16_t data[2];
	data[0] = __REV16((mpp_ptr->channel << 8) | (84));
	data[1] = __REV16(0x0000); 
	ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 16, 0, 2, data);
}
/**
  * @brief  установка уровня срабатывания для МПП
	* @param  mpp_ptr указатель на структуру управления
	* @param  offset уровень срабатывания в квантах АЦП
  */
void mpp_set_offset(typeMPPStruct* mpp_ptr, uint16_t offset)
{
	uint16_t data[2];
	mpp_ptr->offset = offset;
	data[0] = __REV16((mpp_ptr->channel << 8) | (1));
	data[1] = __REV16(mpp_ptr->offset);
	ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 16, 0, 2, data);
}

/**
  * @brief  установка уровня отключения питания при превышении напряжения для МПП
	* @param  mpp_ptr указатель на структуру управления
	* @param  bound уровень срабатывания в квантах АЦП
  */
void mpp_pwr_off_bound_offset(typeMPPStruct* mpp_ptr, uint16_t bound)
{
	uint16_t data[2];
	mpp_ptr->pwr_off_bound = bound;
	data[0] = __REV16((mpp_ptr->channel << 8) | (90));
	data[1] = __REV16(mpp_ptr->pwr_off_bound);
	ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 16, 0, 2, data);
}

/**
  * @brief  запрос значений установки уровня срабатывания и количества помех в архивной памяти МПП
	* @param  mpp_ptr указатель на структуру управления
	* @param  bound уровень срабатывания в квантах АЦП
  */
void mpp_arch_count_offset_get(typeMPPStruct* mpp_ptr)
{
	uint8_t in_data[8], data_offset = 0;
	if (ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 3, 119, 4, NULL) > 0){
		data_offset = (mpp_ptr->channel == 0) ? 0 : 2;
		ib_get_answer_data(mpp_ptr->ib, in_data, 8);
		mpp_ptr->frame.mpp.arch_count = __REV16(*(uint16_t*)&in_data[0 + data_offset]);
		mpp_ptr->frame.mpp.offset = __REV16(*(uint16_t*)&in_data[4 + data_offset]);
	}
	else{
		//
	}
}

/**
  * @brief  инициализация памяти МПП
	* @param  mpp_ptr указатель на структуру управления
  */
void mpp_arch_memory_init(typeMPPStruct* mpp_ptr)
{
	uint16_t data[2];
	data[0] = __REV16((mpp_ptr->channel << 8) | (84));
	data[1] = __REV16(0x0000);
	ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 16, 0, 2, data);
}

/**
  * @brief  принудительный запуск регистрации МПП
	* @param  mpp_ptr указатель на структуру управления
  */
void mpp_forced_start(typeMPPStruct* mpp_ptr)
{
	uint16_t data[2];
	data[0] = __REV16((mpp_ptr->channel << 8) | (81));
	data[1] = __REV16(0x0001);
	ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 16, 0, 2, data);
}

/**
  * @brief  принудительный запуск регистрации МПП при наличии флага запуска
	* @param  mpp_ptr указатель на структуру управления
  */
void mpp_relative_forced_start(typeMPPStruct* mpp_ptr)
{
	if(mpp_ptr->forced_start_flag){
		mpp_ptr->forced_start_flag = 0;
		if (mpp_ptr->forced_start_timeout >= MPP_FORCE_START_PERIOD_TIMEOUT){
			mpp_ptr->forced_start_timeout = 0;
			mpp_forced_start(mpp_ptr);
		}
		else{
			mpp_ptr->forced_start_timeout += 1;
			mpp_on_off(mpp_ptr, 1);
		}
	} 
}

/**
  * @brief  запрос на подготовку 2х структур помех из архивной памяти МПП
	* @param  mpp_ptr указатель на структуру управления
  */
void mpp_struct_request(typeMPPStruct* mpp_ptr)
{
	uint16_t data[2];
	data[0] = __REV16((mpp_ptr->channel << 8) | (4));
	data[1] = __REV16(0x0001);
	ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 16, 0, 2, data);
}

/**
  * @brief  запрос к МПП на выдачу 2-х структур из архивной памяти в оперативную
	* @param  mpp_ptr указатель на структуру управления
  */
void mpp_struct_get(typeMPPStruct* mpp_ptr)
{
	uint8_t i, in_data[64];
	typeMPPRec rec[2];
	// todo: сделать запрос двух помех. Одна помеха из-за ошибки в модуле МПП, которая не дает читать две помехи за раз (подробности у А.А.Дорошкина)
	if (ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 3, 7, 13, NULL) > 0){
		ib_get_answer_data(mpp_ptr->ib, in_data, 26*1);
		// сохраняем полученные структуры в свои переменные, но отбрасываем номер канала
		memcpy((uint8_t*)&rec[0], in_data+2, sizeof(typeMPPRec));
		//memcpy((uint8_t*)&rec[1], in_data+2+24+2, sizeof(typeMPPRec));
		// приводим порядок байт к используемому в МК
		__mpp_struct_rev(&rec[0]);
		//__mpp_struct_rev(&rec[1]);
		//
		mpp_ptr->forced_start_flag = 1;
		//
		for (i=0; i<1; i++){
			if((rec[i].AcqTime_s != 0) || (rec[i].AcqTime_us != 0)){ // проверяем есть ли измерение в прочитанных данных
				mpp_ptr->forced_start_flag = 0; // если кадры были получены, то принудительный запуск МПП не нужен
				// заполняем буфер с помехами: если буфер переполнился пишем всегда в полседюю помеху пока буфер не будет вычитан
				// предполагаем, что такая ситуация маловероятна
				mpp_ptr->rec_buff[mpp_ptr->rec_ptr] = rec[i];
				mpp_ptr->rec_ptr++;
				if (mpp_ptr->rec_ptr >= MPP_REC_FIFO_DEPTH) mpp_ptr->rec_ptr = MPP_REC_FIFO_DEPTH-1;
				if (mpp_ptr->rec_ptr >= 2) {
					mpp_frame_forming(mpp_ptr);
				}
			}
		}
	}
}

/**
  * @brief  перестановка байт в словах попарно в структуре МПП
	* @param  mpp_struct_ptr указатель на структуру
  */
void __mpp_struct_rev(typeMPPRec* mpp_struct_ptr)
{
		uint16_t data[32];
    uint8_t i = 0;
    memcpy((uint8_t *)data, (uint8_t *)mpp_struct_ptr, sizeof(typeMPPRec));
    for (i=0; i<(sizeof(typeMPPRec)/2); i++) {
        data[i] = __REV16(data[i]);
    }
    memcpy((uint8_t *)mpp_struct_ptr, (uint8_t *)data, sizeof(typeMPPRec));
}

/**
  * @brief  инициализация циклограмм работы с МПП
	* @param  mpp_ptr указатель на структуру управления
  */
void mpp_meas_cycl_init(typeMPPStruct* mpp_ptr)
{
	uint32_t channel_delay;
	// циклограмма инициализации МПП
	cyclo_init(&mpp_ptr->meas_cyclo);
	// todo: продумать систему расшивки работы с каналами МПП. Возможно надо вернуться к идее работы с МПП в одном процессе для двух каналов
	channel_delay = (mpp_ptr->channel == 0) ? 0 : MPP_CHANNEL_REQUEST_SHIFT_MS;  // костыль из-за того, что во время подготовки данных первого канала, второй канал не может запрашивать данные из архива
	//
	cyclo_add_step(&mpp_ptr->meas_cyclo, CYCLO_DO_NOTHING, (void*)mpp_ptr, 0 + channel_delay);
	cyclo_add_step(&mpp_ptr->meas_cyclo, mpp_meas_cycl_struct_request, (void*)mpp_ptr, 150);
	cyclo_add_step(&mpp_ptr->meas_cyclo, mpp_meas_cycl_struct_get, (void*)mpp_ptr, 0);
	cyclo_add_step(&mpp_ptr->meas_cyclo, mpp_meas_cycl_arch_offset_get, (void*)mpp_ptr, 0);
	cyclo_add_step(&mpp_ptr->meas_cyclo, mpp_meas_cycl_forced_start, (void*)mpp_ptr, 100);
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
void mpp_meas_cycl_on(void* ctrl_struct)
{
	typeMPPStruct* mpp_ptr = (typeMPPStruct*) ctrl_struct;
	mpp_on_off(mpp_ptr, 1);
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
void mpp_meas_cycl_arch_offset_get(void* ctrl_struct)
{
	typeMPPStruct* mpp_ptr = (typeMPPStruct*) ctrl_struct;
	mpp_arch_count_offset_get(mpp_ptr);
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
void mpp_meas_cycl_struct_request(void* ctrl_struct)
{
	typeMPPStruct* mpp_ptr = (typeMPPStruct*) ctrl_struct;
	mpp_struct_request(mpp_ptr);
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
void mpp_meas_cycl_struct_get(void* ctrl_struct)
{
	typeMPPStruct* mpp_ptr = (typeMPPStruct*) ctrl_struct;
	mpp_struct_get(mpp_ptr);
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
void mpp_meas_cycl_forced_start(void* ctrl_struct)
{
	typeMPPStruct* mpp_ptr = (typeMPPStruct*) ctrl_struct;
	mpp_relative_forced_start(mpp_ptr);
}
