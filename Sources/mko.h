#ifndef _MKO_H_
#define _MKO_H_

#include "1986ve8_lib/cm4ikmcu.h"
#include "debug.h"
#include "string.h"
#include "gpio.h"

#define IRQn_MIL_STD_15531 (IRQn_Type)117
#define IRQn_MIL_STD_15532 (IRQn_Type)118

#define MKO_BC 0  //bus controller
#define MKO_RT 1  //range terminal

#define MKO_BUS_A 0
#define MKO_BUS_B 1

#define MKO_MODE_WRITE 0
#define MKO_MODE_READ 1

/**
  * @brief  именование линий адреса МКО
*/
typedef enum MKO_ADDR
{
	ADDR0, ADDR1, ADDR2, ADDR3, ADDR4, ADDR5, PARITY,
	ADDR_CH_NUMBER
} typeMKOADDR;
#define MKO_ADDR_GPIO_PORT {PORTB, PORTB, PORTB, PORTB, PORTB, PORTB}
// #define MKO_ADDR_GPIO_LINE {12, 13, 14, 15, 16, 17}
#define MKO_ADDR_GPIO_LINE {9, 10, 11, 12, 13, 14}

typedef union
{
	uint16_t whole;
	struct
	{
		uint16_t leng : 5;
		uint16_t sub_addr : 5;
		uint16_t rd_wr : 1;
		uint16_t addr : 5;
	} field;
}typeCommandWord;

typedef union
{
	uint16_t whole;
	struct
	{
		uint16_t malfunction : 1;
		uint16_t ctrl_ackn : 1;
		uint16_t abonent_malfunction : 1;
		uint16_t busy : 1;
		uint16_t broad_cmd : 1;
		uint16_t not_used : 2;
		uint16_t mem_empty : 1;
		uint16_t service_ack : 1;
		uint16_t must_be_zero : 1;
		uint16_t error : 1;
		uint16_t addr : 5;
	} field;
}typeAnswerWord;

typedef struct  //  max 62 - параетры ЦМ для сохоранения
{
	MIL1553Control *regs;
	uint8_t type;
	IRQn_Type irq;
	//
	type_SINGLE_GPIO gpio[ADDR_CH_NUMBER];
	uint8_t addr;
	//
	typeCommandWord cw; //+0
	typeAnswerWord aw; //+2
  uint16_t data[32]; //+13
	//
	uint16_t msg; //+8
	uint8_t num; //+10
	uint8_t rcv_a;  //+10
	uint8_t rcv_b;  //+10
	uint8_t error; //+11
	uint8_t error_cnt; //+12
	//
	int need_to_process_flag;
	uint8_t mko_rcv_flag;
}typeMKOStruct;


void mko_init(typeMKOStruct *mko_ptr, MIL1553Control *regs, uint8_t type, uint8_t mko_addr);
void mko_set_busy(typeMKOStruct *mko_ptr);
void mko_release_busy(typeMKOStruct *mko_ptr);
void mko_set_aw_bit_7(typeMKOStruct *mko_ptr, uint8_t val);
void mko_clear_data(typeMKOStruct *mko_ptr);
//Работа в режиме ОУ (RT)
void mko_rt_transaction_handler(typeMKOStruct *mko_ptr);
void mko_rt_write_to_subaddr(typeMKOStruct *mko_ptr, uint8_t subaddr, uint16_t* data);
void mko_rt_read_from_subaddr(typeMKOStruct *mko_ptr, uint8_t subaddr, uint16_t* data);
//Работа в режиме КШ (BC)
void mko_bc_transaction_handler(typeMKOStruct *mko_ptr);
void mko_bc_transaction_start(typeMKOStruct *mko_ptr, uint8_t mode, uint8_t addr, uint8_t subaddr, uint8_t bus, uint16_t* data, uint8_t leng);
//
void mko_block_transmitter(typeMKOStruct *mko_ptr);
void mko_get_error(typeMKOStruct *mko_ptr, uint8_t* error, uint8_t* error_cnt);
uint8_t mko_get_addr_from_gpio(typeMKOStruct *mko_ptr);
void __mko_cmd_msg(typeMKOStruct *mko_ptr);
uint8_t mko_need_to_process(typeMKOStruct *mko_ptr);

//непараметризуемые функции

__weak void INT_MIL0_Callback(void);
__weak void INT_MIL1_Callback(void);

void INT_MIL0_Handler(void);
void INT_MIL1_Handler(void);

#endif
