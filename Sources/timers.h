#ifndef _timer_H_
#define _timer_H_

#include <string.h>
#include <stdio.h>
#include <math.h>

#define IRQn_TMR0 (IRQn_Type)93
#define IRQn_TMR1 (IRQn_Type)94
#define IRQn_TMR2 (IRQn_Type)95
#define IRQn_TMR3 (IRQn_Type)96

#pragma pack(1)

/**
  * @brief  структура для удобства работы с временем в 1/256 секунд
*/
typedef struct
{
    uint8_t low_part;
    uint32_t mid_part;
    uint8_t high_part;
    uint16_t zero_part;
}typeCMTime;


void Timers_Init(void);
void Timer_Start(uint32_t time_ms, uint8_t irq_flag);
void Timer_Stop(void);
uint8_t Timer_Status(void);
void Timer_Delay(uint32_t delay_ms);
void Time_Set(uint64_t time, int16_t* diff_time_s, int8_t* diff_time_low); // time в 1/(2^16) секундах
uint32_t Get_Time_s(void);
void Get_Time_sec_parts(uint32_t* sec, uint8_t* parts);
void Timer_PWM_Set(uint8_t ch_num, uint16_t pwm_val);
void Timer_PWM_Set_Fp(uint8_t ch_num, float pwm_val_fp);
__weak void INT_TMR1_CallBack(void);
#endif
