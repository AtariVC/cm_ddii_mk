#ifndef _DDII_H_
#define _DDII_H_

#include <stdint.h>
#include "1986ve8_lib/cm4ikmcu.h"
#include "uarts.h"
#include "task_planner.h"
#include "cyclogramma.h"
#include "frames.h"
#include "debug.h"
#include "mko.h"
#include "mpp.h"
#include "frame_mem.h"
#include "csa_test.h"
#include "internal_bus.h"
#include "timers.h"
#include "cm.h"
#include "termo_res.h"

// дефайны для переменных
#define DDII_DEFAULT_INTERVAL_S (10)

#define DDII_EVENT_MEAS_INTERVAL_START       (1<<0)
#define DDII_EVENT_MEAS_INTERVAL_DATA_READY  (1<<1)

#define DDII_MEAS_NUMBER              0 // было 1
#define DDII_REC_FIFO_DEPTH           64

#define DDII_MKO_SA_DATA_FRAME        0x0F
#define DDII_MKO_SA_CTRL              1

#define DDII_MK_COMMAND_LINK_CHECK    0
#define DDII_MK_COMMAND_SET_MODE      1
#define DDII_MK_COMMAND_START         2
#define DDII_MK_COMMAND_CONSTANT_MODE 17

#define HEAD                          0x0FF1

#define REG_MPP_COMAND                0x0000
#define REG_MPP_LEVEL_TRIG            0x0001
#define REG_MPP_LEVEL_HH              0x000B
#define REG_MPP_WAVEFORM              0x0009
#define REG_MPP_HIST                  0x0014
#define REG_MPP_CALIBRATE_ADC         0x0050
#define REG_MPP_TRIGER                0x0051
#define REG_MPP_FILTR                 0x000A
#define REG_MPP_STRUCT_DATA           0x0006
#define REG_MPP_LEVEL                 0x0079




#define DDII_EVENT_MEAS_INTERVAL_START       (1<<0)
#define DDII_EVENT_MEAS_INTERVAL_DATA_READY  (1<<1)

#define DDII_MEAS_NUMBER              0 // было 1
#define DDII_REC_FIFO_DEPTH           64

#define DDII_MKO_SA_DATA_FRAME        0x0F
#define DDII_MKO_SA_CTRL              1

#define DDII_MK_COMMAND_LINK_CHECK    0
#define DDII_MK_COMMAND_SET_MODE      1
#define DDII_MK_COMMAND_START         2
#define DDII_MK_COMMAND_CONSTANT_MODE 17

#define HEAD                          0x0FF1

#define REG_MPP_COMAND                0x0000
#define REG_MPP_LEVEL_TRIG            0x0001
#define REG_MPP_LEVEL_HH              0x000B
#define REG_MPP_WAVEFORM              0x0009
#define REG_MPP_HIST                  0x0014
#define REG_MPP_CALIBRATE_ADC         0x0050
#define REG_MPP_TRIGER                0x0051
#define REG_MPP_FILTR                 0x000A
#define REG_MPP_STRUCT_DATA           0x0006
#define REG_MPP_LEVEL                 0x0079


#define COMAND_MPP_START_STOP_MEAS    0x0002
#define COMAND_SEND_FRAME             0x00FE

#define DDII_MPP_ID                   0x000E

#define DELAY_START_MEASURE           1000

/////////////// EVENTS /////////////
#define EVENT_START_MEASURE           0x01
#define EVENT_STOP_MEASURE            0x00

/////////////// MODE /////////////////
#define DEBUG_MODE                    0x000C
#define SILENT_MODE                   0x000D
#define COMBAT_MODE                   0x000E
#define CONSTATNT_MODE                0x000F

#define PIPS_CH_VOLTAGE               1
#define SIPM_CH_VOLTAGE               2
#define CHERENKOV_CH_VOLTAGE          3

#define DEFAULT_CFG                   0
#define LOADED_CFG                    1

#define TERM_NUM 2
#define TERM_ADC_CHANNELS {12, 11}
//
#pragma pack(push, 2)

/**
 * @brief структура с данными ДДИИ для общения по ВШ
 * 
 */
typedef struct
{
  uint8_t data[52];
}typeDDIIRec;

/** 
  * @brief  структура кадар ДДИИ
  */
typedef union{
  typeFrameStruct row;
  struct{
    uint16_t header[5];
    typeDDIIRec meas;
    uint16_t crc16;
  } ddii;
}typeDDIIFrameUnion;


/** 
  * @brief  структура данных температуры
  */
typedef union{
  struct{
    float term_sipm_01;
    float term_cherenkov_02;
  } ddii_term;
  float   data_term[TERM_NUM];
}typeDDIITerm;

/** 
  * @brief  структура конфигурфции ддии
  */
typedef struct {
	uint16_t head;                 // + 0
	uint16_t mpp_level_trig;       // + 2
	uint16_t mpp_HH[8];            // + 20
	float    hvip_pwm_val[HVIP_NUM];      // + 32
	float    hvip_voltage[HVIP_NUM];      // + 44
  uint16_t mpp_id;               // + 46
  uint32_t interval_measure;     // + 50
  uint16_t volt_corr_mode;       // + 52
}typeDDII_cfg;

typedef union{
  struct{
    uint16_t  HCP[5];
    uint32_t  Hist32[6];
    uint16_t  Hist16[6];
  }hist;
  struct{
    uint16_t   HCP_1;             // +0 ТЗЧ E > 1 МэВ
    uint16_t   HCP_5;             // +2 ТЗЧ E > 5 МэВ
    uint16_t   HCP_10;            // +4 ТЗЧ E > 10 МэВ
    uint16_t   HCP_20;            // +6 ТЗЧ E > 20 МэВ
    uint16_t   HCP_45;            // +8 ТЗЧ E > 45 МэВ
    uint32_t   electron_0_1;      // +10 Электроны E > 0.1 МэВ
    uint32_t   electron_0_5;      // +14 Электроны E > 0.5 МэВ
    uint32_t   electron_0_8;      // +16 Электроны E > 0.8 МэВ
    uint32_t   electron_1_6;      // +20 Электроны E > 1.6 МэВ
    uint32_t   electron_3;        // +24 Электроны E > 3 МэВ
    uint32_t   electron_5;        // +28 Электроны E > 5 МэВ
    uint16_t   proton_10;         // +32 Протоны E > 10 МэВ
    uint16_t   proton_30;         // +34 Протоны E > 30 МэВ
    uint16_t   proton_60;         // +36 Протоны E > 60 МэВ
    uint16_t   proton_100;        // +38 Протоны E > 100 МэВ
    uint16_t   proton_200;        // +40 Протоны E > 200 МэВ
    uint16_t   proton_500;        // +42 Протоны E > 500 МэВ
  }particle;                      // +44
}typeDDII_Particle_Union;

typedef struct{
  float     v_hv;                  // + 0
  float     hv_pwm;                // + 4
  float     hv_current;             // + 8
  uint8_t   hv_mode;               // + 12
}typeDDII_HVIP_Data;               // + 13

typedef struct{
  float a_u;
  float b_u;
  float a_i;
  float b_i;
}typeDDIIhvip_AB;

typedef struct{
  uint16_t                ddii_mode;              // + 0
  uint16_t                HH[8];                  // + 2
  uint16_t                Level;                  // + 18
  typeDDII_HVIP_Data      hvip_data[HVIP_NUM];    // + 20
  uint8_t                 csa_test_on;            // + 59
  uint16_t                ddii_interval_request;  // + 60
  uint16_t                ACQ1_Peack;             // + 62
  uint16_t                ACQ2_Peack;             // + 64
  typeDDII_Particle_Union particle_telmtr;        // + 66
  uint16_t                rd_ptr;                 // +110
  uint16_t                wr_ptr;                 // +112
}typeDDII_DB_Telemetry;                          // + 114

/** 
  * @brief  структура данных МПП
  */
typedef struct{
  uint16_t TmpCount;    // Колличество импульсов /+0
  uint16_t ACQ1_Peak;   // Пик ADC_A /+2
  uint16_t ACQ2_Peak;   // Пик ADC_B /+4
  uint16_t DDIN_Peak;   // Маска состояни цифровых каналов (0-4) 11111 - все каналы находятся в 1 (DIO 1, 3, 5, 7, 9)
  uint16_t BinNum;      // Номер последнего инкрементированного бина /+8
  uint16_t HH[8];       // Уровни 8 карманов 5 электроны и 3 протона /+10
  uint16_t res2;
  typeDDII_Particle_Union particle; //+50
  uint16_t Level; //+60 // Уровень триггера
}typeDDII_Data_MPP;

typedef union{
	uint8_t constant_frame_data[sizeof(typeDDIIRec)];
  struct{
    uint16_t   duration_meas;     // Длительность измерения
    typeDDII_Particle_Union particle;
    uint32_t   common_count_1_6;  // Счетчик общего числа  E > 1.6 МэВ
  }frame;
}typeDDII_Frame_Union;


typedef struct
{
  // interfaces
  typeMKOStruct* mko_bc_ptr;
  // сfg
	uint16_t mko_addr;			             // id на внутренней шине
	uint16_t mko_bus;			               // id на внутренней шине
	uint16_t self_num;                   // номер устройства с точки зрения ЦМ
	uint16_t device_number, frame_type;  //параметры прибора МПП, в котором он используется
  uint16_t interval_ms;
  uint32_t *global_frame_num_ptr;
  // to task_planner
  uint8_t meas_event_num;
  uint8_t status_load_cfg;
  uint64_t last_call_time_us;
  // data
  typeDDIIFrameUnion frame;
  uint8_t frame_data_ready;  // флаг готовности данных в памяти на отправку в другой процесс
  // fifo для обработки данных ДДИИ
  typeDDIIRec rec_fifo[DDII_REC_FIFO_DEPTH];
  int8_t rec_num, rec_max;
  // general
  // cyclogram_ctrl
  typeCyclogramma meas_cyclo;
  typeMPPStruct* mpp;
  typeCMModel* cm;
  typeFRAME_MEM mem;
  typeDDII_cfg cfg;
  uint8_t mpp_on_off;
  typeDDII_Data_MPP ddii_mpp_data;
  uint16_t mode;
  uint8_t event;
  uint8_t frame_ptr;
  uint64_t curent_time_measure;
  uint64_t curent_time;
  type_CSA_TEST_Struct csa_test;
  //typeDDIICounterParticle counter_particle;
  typeDDII_DB_Telemetry telmtr_struct; 
  typeDDII_Frame_Union dataframe;
  uint8_t voltage_correction_mode;
  typeDDIIhvip_AB hvip_AB[HVIP_NUM];
  typeDDIITerm term_struct;
  type_TRES_model term_model[TERM_NUM];
  typeCyclogramma term_cyclo;
  float desired_hv[HVIP_NUM];
  uint16_t write_inmem_flag;
} typeDDIIStruct;



#pragma pack(pop)

//
void ddii_init(typeDDIIStruct* ddii_ptr,
              typeCMModel* cm,
              typeMPPStruct* mpp_ptr,
              uint8_t self_num,
              uint8_t mko_addr,
              uint16_t device_number,
              uint16_t frame_type,
              typeMKOStruct* mko_bc_ptr,
              uint8_t mko_bus,
              uint32_t* gl_fr_num);
void ddii_reset_parameters(typeDDIIStruct* ddii_ptr);
//
int8_t ddii_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface);
int8_t ddii_frame_forming(typeDDIIStruct* ddii_ptr);
//
void ddii_constant_mode(typeDDIIStruct* ddii_ptr, uint32_t on_off);
void ddii_start(typeDDIIStruct* ddii_ptr, uint32_t ddii_rd_ptr);
//
void ddii_read_data_frame(typeDDIIStruct *ddii_ptr);
//
int8_t ddii_write_fifo(typeDDIIStruct *ddii_ptr, typeDDIIRec* data);
int8_t ddii_read_fifo(typeDDIIStruct* ddii_ptr, typeDDIIRec* data);
// функции для работы циклограмы измерительного интервала
void ddii_meas_cycl_init(typeDDIIStruct* ddii_ptr);
void ddii_meas_cycl_start(void* ctrl_struct);
void ddii_meas_cycl_read(void* ctrl_struct);
void ddii_meas_cycl_frame_forming(void* ctrl_struct);
void ddii_constant_data_forming(typeDDIIStruct* ddii_ptr);
//
void __ddii_struct_rev(typeDDIIRec* ddii_struct_ptr);
void ddii_constant_data_forming(typeDDIIStruct* ddii_ptr);
void ddii_get_particle_struct(typeDDIIStruct* ddii_ptr);
void ddii_init_level_mpp(typeDDIIStruct* ddii_pt);
void ddii_download_cfg(typeDDIIStruct* ddii_ptr);
void ddii_set_cfg_mpp(typeDDIIStruct* ddii_ptr);
void ddii_mpp_set_level_hh(typeDDIIStruct* ddii_ptr);
void ddii_set_default_cfg(typeDDIIStruct* ddii_ptr);
void ddii_mpp_set_level_triger(typeDDIIStruct* ddii_ptr);
void ddii_mpp_get_data(typeDDIIStruct* ddii_ptr);
void ddii_reset_mpp_struct_measure(typeDDIIStruct* ddii_ptr);
void ddii_start_mpp_measure(typeDDIIStruct* ddii_ptr, uint16_t enable);
void ddii_reset_mpp_struct_measure(typeDDIIStruct* ddii_ptr);
void ddii_send_mko_frame(typeDDIIStruct* ddii_ptr);
void ddii_struct_telemetria_forming(typeDDIIStruct* ddii_ptr);
void ddii_frame_struct_forming(typeDDIIStruct* ddii_ptr);
void ddii_download_cfg_inmem(typeDDIIStruct* ddii_ptr);
void ddii_update_cfg(typeDDIIStruct* ddii_ptr, uint8_t* data);
void ddii_update_voltage(typeDDIIStruct* ddii_ptr);
void ddii_cmd_set_voltage_pwm(typeDDIIStruct* ddii_ptr, uint8_t *data);
void ddii_set_cfg(typeDDIIStruct* ddii_ptr);
void reverse_data(uint8_t* data, uint16_t* out_data);
void ddii_hvip_set_coef_a_b(typeDDIIStruct* ddii_ptr, uint8_t* data);
void ddii_hvip_get_coef_a_b(typeDDIIStruct* ddii_ptr, uint8_t* data);
void ddii_term(typeDDIIStruct* ddii_ptr);
void ddii_term_cycl_start(void* ctrl_struct);
void ddii_get_mko_mpp_config_data(typeDDIIStruct* ddii_ptr);
void ddii_get_mko_hvip_temp(typeDDIIStruct* ddii_ptr);
void ddii_get_mko_temp(typeDDIIStruct* ddii_ptr);
void ddii_get_mko_hvip(typeDDIIStruct* ddii_ptr, uint16_t ch);
void ddii_write_frame_inmem(typeDDIIStruct* ddii_ptr);
void ddii_read_frame_inmem(typeDDIIStruct* ddii_ptr);
void ddii_mko_read_mem(typeDDIIStruct* ddii_ptr);
void ddii_mko_write_mem(typeDDIIStruct* ddii_ptr);

//
// void ddii_power_detector_on(type_SINGLE_GPIO* ddii_gpio_ptr);
#endif
