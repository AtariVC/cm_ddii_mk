#ifndef _MPP_H_
#define _MPP_H_

#include <string.h>
#include "1986ve8_lib/cm4ikmcu.h"
#include "internal_bus.h"
#include "task_planner.h"
#include "cyclogramma.h"
#include "frames.h"
#include "debug.h"
#include "gpio.h"

// дефайны для переменных
#define MPP_REC_FIFO_DEPTH 4

#define MPP_DEFAULT_INTERVAL_MS (10000)

#define MPP_FORCE_START_PERIOD_TIMEOUT 1

#define MPP_EVENT_MEAS_INTERVAL_START       (1<<0)
#define MPP_EVENT_MEAS_INTERVAL_DATA_READY  (1<<1)

/// костыль для работы с отдельными каналами МПП
#define MPP_CHANNEL_REQUEST_SHIFT_MS        (300)

//
#pragma pack(push, 2)

// команды mpp
#define SET_LEVEL_ADC_MPP (1)
#define START_STOP_REG_MPP (2)
#define SET_LEVEL_BIN_MPP (8)
#define FILTR (10)
#define CALIBRATE_ADC_MPP (80)
#define SET_TRIGER_ACTUATION (81) // принужительный запуск МПП. Тригер срабатывания.

/** 
  * @brief  структура помехи МПП
  */
typedef struct
{
    uint32_t AcqTime_s;         //+0
    uint32_t AcqTime_us;        //+4
    uint32_t WidhtTime;         //+8
    uint16_t ZeroCount;         //+12
    uint16_t Peak;              //+14
    uint32_t Power;             //+16
    uint16_t Mean;              //+20
    uint16_t Noise;             //+22
}typeMPPRec; //24


/** 
  * @brief  структура кадар МПП
  */
typedef union{
  typeFrameStruct row;
  struct{
    uint16_t header[5];
    uint16_t arch_count;
    uint16_t offset;
    typeMPPRec rec[2];
    uint16_t crc16;
  } mpp;
}typeMPPFrameUnion;

/** 
  * @brief  структура управления МПП
  */
typedef struct
{
  // interfaces
  typeIBStruct* ib;
  // сfg
	uint16_t id;			          // id на внутренней шине
	uint16_t self_num;          // номер устройства с точки зрения ЦМ
	uint16_t device_number, frame_type;  //параметры прибора МПП, в котором он используется
	uint8_t channel;			      // канал МПП: 0 или 1
  uint16_t offset, pwr_off_bound, mode, const_mode;  // данные установки
  uint16_t interval_ms;
  uint32_t *global_frame_num_ptr;
  // to task_planner
  uint8_t meas_event_num;
  uint64_t last_call_time_us;
  // data
  typeMPPFrameUnion frame;
  uint8_t frame_data_ready;  // флаг готовности данных в памяти на отправку в другой процесс
  typeMPPRec rec_buff[MPP_REC_FIFO_DEPTH];
  uint8_t rec_ptr;
  // general
	uint8_t forced_start_flag;  // флаг необходимости принудительного запуска
	uint8_t forced_start_timeout;  // таймаут на запуск принудительного старта
	uint8_t frame_pulse_cnt;    // количество считанных помех с последнего формирования кадра
  // cyclogram_ctrl
  typeCyclogramma meas_cyclo;
  type_SINGLE_GPIO mpp_on_off;
} typeMPPStruct;

#pragma pack(pop)

//
void mpp_init(typeMPPStruct* mpp_ptr, uint8_t self_num, uint8_t id, uint16_t device_number, uint16_t frame_type, uint8_t channel, uint32_t offset, typeIBStruct* ib_ptr, uint32_t* gl_fr_num);
void mpp_reset_parameters(typeMPPStruct* mpp_ptr);
//
int8_t mpp_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface);
int8_t mpp_frame_forming(typeMPPStruct* mpp_ptr);
//
void mpp_time_set(typeMPPStruct* mpp_ptr, uint32_t time_s);
void mpp_on_off(typeMPPStruct* mpp_ptr, uint32_t on_off);
void mpp_constant_mode(typeMPPStruct* mpp_ptr, uint32_t on_off);
void mpp_arch_mem_init(typeMPPStruct* mpp_ptr);
void mpp_set_offset(typeMPPStruct* mpp_ptr, uint16_t offset);
void mpp_pwr_off_bound_offset(typeMPPStruct* mpp_ptr, uint16_t bound);
//
void mpp_arch_count_offset_get(typeMPPStruct* mpp_ptr);
void mpp_arch_memory_init(typeMPPStruct* mpp_ptr);
void mpp_forced_start(typeMPPStruct* mpp_ptr);
void mpp_relative_forced_start(typeMPPStruct* mpp_ptr);
//
void mpp_struct_request(typeMPPStruct* mpp_ptr);
void mpp_struct_get(typeMPPStruct* mpp_ptr);
int8_t mpp_frame_data_get(typeMPPStruct* mpp_ptr, uint8_t* buff);
void mpp_struct_ready_event(typeMPPStruct* mpp_ptr);
// функции для работы циклограмы измерительного интервала
void mpp_meas_cycl_init(typeMPPStruct* mpp_ptr);
void mpp_meas_cycl_on(void* ctrl_struct);
void mpp_meas_cycl_arch_offset_get(void* ctrl_struct);
void mpp_meas_cycl_struct_request(void* ctrl_struct);
void mpp_meas_cycl_struct_get(void* ctrl_struct);
void mpp_meas_cycl_forced_start(void* ctrl_struct);
//
void __mpp_struct_rev(typeMPPRec* mpp_struct_ptr);
//
#endif
