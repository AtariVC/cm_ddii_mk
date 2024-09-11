/**
 * @file timers.c
 * @author a-styuf (a-styuf@yandex.ru)
 * @brief описание аппаратной реализации таймеров для 1986ВЕ8, минимальное количество таймеров - 3
 *< TMR0 - используются для работы внутренней шины (здесь не представлен)
 *< TMR1 - для общего использования
 *< TMR2 - счетчик глобального времени
 *< TMR3 - PWM
 * Таймеры используют прерывания для выставления статусов.
 * @version 0.1
 * @date 2022-03-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "1986ve8_lib/cm4ikmcu.h"
#include "timers.h"
#include "csa_test.h"

uint8_t timer_status = 0;  //! глобальная переменная для хранения статусов Таймеров
uint8_t irq_timer_flag = 0;
uint8_t high_byte_time_s = 0;  //! глобальная переменная для хранения старшей части глобального времени
uint8_t pwm_adder[4] = {0}, pwm_adder_cnter[4] = {0};
uint16_t CCR1_base[4] = {0};
uint32_t *TMR_CCR[4] = {    (uint32_t *)(MDR_TMR3_BASE + 0x0010), 
                            (uint32_t *)(MDR_TMR3_BASE + 0x0014),
                            (uint32_t *)(MDR_TMR3_BASE + 0x0018),
                            (uint32_t *)(MDR_TMR3_BASE + 0x001C)
                        };
uint8_t flag = 0;

/**
 * @brief инициализация таймеров для испльзования в программе
 * TMR0 - используются для работы внутренней шины (здесь не представлен)
 * TMR1 - для общего использования
 * TMR2 - для общего использования
 * TMR3 - PWM
 */
void Timers_Init(void)
{
    //
    CLK_CNTR->KEY = _KEY_;
    CLK_CNTR->PER0_CLK |= (1<<24)|(1<<25)|(1<<26);  //включение Timer1, Timer2 bи Timer3
    CLK_CNTR->TIM1_CLK = (1<<16)| 39;  //timer clock freq = 1 MHz
    CLK_CNTR->TIM2_CLK = (1<<16)| 249; //timer clock freq = 160kHz
    CLK_CNTR->TIM3_CLK = (1<<16)| 3;   //timer clock freq = 10 MHz
    //
    MDR_TMR1->CNTRL = 0x00000000;
    MDR_TMR2->CNTRL = 0x00000000;
    MDR_TMR3->CNTRL = 0x00000000;
    //Настраиваем работу основных таймеров
    MDR_TMR1->CNT = 0x00000000;  //Начальное значение счетчика
    MDR_TMR1->PSG = 25;  // 999 Предделитель частоты //из 1 МГц получаем 1 кГц
    MDR_TMR1->IE = 0x00000002;  //Разрешение генерировать прерывание при CNT = ARR
    //счетчик глобального времени
    MDR_TMR2->CNT = 0x00000000;  //Начальное значение счетчика
    MDR_TMR2->PSG = 624;  //Предделитель частоты //из 10 кГц получаем 256 Гц (время БКУ 6-ти байтовое, но используем только 5)
    MDR_TMR2->ARR = 0xFFFFFFFF;//считаем постоянно
    MDR_TMR2->CNTRL = 0x00000001;  //Счет вверх по TIM_CLK. Разрешение работы таймера
    MDR_TMR2->IE = 0x00000002;  //Разрешение генерировать прерывание при CNT = ARR
    MDR_TMR2->STATUS = 0x00;
    //PWM
    MDR_TMR3->CNT = 0x00000000;  //Начальное значение счетчика
    MDR_TMR3->PSG = 0;  //Предделитель частоты 1
    MDR_TMR3->IE = 0x00000002;  //Разрешение генерировать прерывание при CNT = ARR, передний фронту на входе ETR
    MDR_TMR3->CH1_CNTRL = 0x00000C00;  // OCCM = 6: 1, если DIR = 0 (счет вверх), CNT < CCR, иначе 0;
    MDR_TMR3->CH2_CNTRL = 0x00000C00;  // OCCM = 6: 1, если DIR = 0 (счет вверх), CNT < CCR, иначе 0;
    MDR_TMR3->CH3_CNTRL = 0x00000C00;  // OCCM = 6: 1, если DIR = 0 (счет вверх), CNT < CCR, иначе 0;
    MDR_TMR3->CH4_CNTRL = 0x00000C00;  // OCCM = 6: 1, если DIR = 0 (счет вверх), CNT < CCR, иначе 0;
    MDR_TMR3->ARR = (100 - 1); //период PWM - 100kHz
    MDR_TMR3->CH1_CNTRL1 = 0x00000909;
    MDR_TMR3->CH2_CNTRL1 = 0x00000909;
    MDR_TMR3->CH3_CNTRL1 = 0x00000909;
    MDR_TMR3->CH4_CNTRL1 = 0x00000909;
    MDR_TMR3->CNTRL = 0x00000001;  //Счет вверх по TIM_CLK. Разрешение работы таймера
    //
    NVIC_EnableIRQ(IRQn_TMR1);
    NVIC_EnableIRQ(IRQn_TMR2);
    NVIC_EnableIRQ(IRQn_TMR3);
}

/**
 * @brief обработчки прерывания от таймерs
 *  - взводит статус о том, что таймер досчитал
 * 
 */
void INT_TMR1_Handler(void) 
{
    timer_status |= (1 << 1);
    MDR_TMR1->STATUS = 0x0000;
    MDR_TMR1->CNT = 0x00000000;
    MDR_TMR1->CNTRL = 0x00000000;
    if (irq_timer_flag == 1){
        INT_TMR1_CallBack();
    }
}




void INT_TMR2_Handler(void) 
{
    high_byte_time_s += 1;   
}

void INT_TMR3_Handler(void) 
{
    uint8_t i = 0;
    //
    MDR_TMR3->STATUS = 0x0000;
    for (i=0; i<4; i++){
        pwm_adder_cnter[i] += 1;
        pwm_adder_cnter[i]&= 0x01F;
        if (pwm_adder_cnter[i] == pwm_adder[i]) *TMR_CCR[i] = CCR1_base[i];
        else if (pwm_adder_cnter[i] == 0) *TMR_CCR[i] = CCR1_base[i] + 1;
    }
}

/**
 * @brief Работа с заданием задержек.
 * 
 * @param time_ms задержка в мс
 * @param irq_flag если 1, то включаем обработчик INT_TMR1_CallBack
 */
void Timer_Start(uint32_t time_ms, uint8_t irq_flag)
{  
    NVIC_DisableIRQ(IRQn_TMR1);
    MDR_TMR1->ARR = time_ms;  //задание времени в мс
    MDR_TMR1->CNT = 0x00000000;  //Начальное значение счетчика
    MDR_TMR1->CNTRL = 0x00000001;  //Счет вверх по TIM_CLK. Разрешение работы таймера
    irq_timer_flag = irq_flag;
    NVIC_EnableIRQ(IRQn_TMR1);
}

/**
 * @brief Остановка работы таймера по номеру.
 * 
 */
void Timer_Stop(void)
{  
    NVIC_DisableIRQ(IRQn_TMR1);
    MDR_TMR1->CNTRL = 0x00000000;  //Счет вверх по TIM_CLK. Разрешение работы таймера
    MDR_TMR1->ARR = 0x00;  //задание времени в мс
    MDR_TMR1->CNT = 0x00000000;  //Начальное значение счетчика
    NVIC_EnableIRQ(IRQn_TMR1);
}

/**
 * @brief проверка работы таймера
 * 
 * @return uint8_t 0 - таймер еще считает; 1-закончил счет
 */
uint8_t Timer_Status(void)
{
    uint8_t status = 0x0000;
    NVIC_DisableIRQ(IRQn_TMR1);
    status = timer_status & (1 << 1);
    if (status != 0)
    {
        timer_status &= ~(1 << 1);
    }
    NVIC_EnableIRQ(IRQn_TMR1);
    return status;
}


/**
  * @brief  запуск блокирующей задержки
  * @param  delay_ms  величина задержки в мс на
  */
void Timer_Delay(uint32_t delay_ms)
{   
    Timer_Start(delay_ms*40, 0);
    while (Timer_Status() == 0);

}

/**
 * @brief Установка таймера для реального времени
 * 
 * @param time  устанавливаемое значение в в 1/(2^16) секундах
 * @param diff_time_s указатель на значение, в которе будет записана разница времени в секундах
 * @param diff_time_low указатель на значение, в которе будет записана разница времени в (1/2^8)
 */
void Time_Set(uint64_t time, int16_t* diff_time_s, int8_t* diff_time_low)
{
    volatile int64_t diff_time = 0;
    uint64_t current_time = 0;
	typeCMTime cm_time = {0, 0, 0, 0};
	//
    NVIC_DisableIRQ(IRQn_TMR2);
    //
    cm_time.low_part = 0x00;
    cm_time.mid_part = MDR_TMR2->CNT;
    cm_time.high_part = high_byte_time_s;
    cm_time.zero_part = 0x0000;
    //
    memcpy((uint8_t*)&current_time, (uint8_t*)&cm_time, 8);
    diff_time = time - current_time;
    //сохраняем системные данные связанные с синхронизацией
    *diff_time_s = (diff_time  >> 16) & 0xFFFF;
    *diff_time_low = (diff_time >> 8) & 0xFF;
    //сохраняем системные данные связанные с синхронизацией
    MDR_TMR2->CNT = (time >> 8) & 0xFFFFFFFF;
    high_byte_time_s = (time >> 40) & 0xFF;
    //
    NVIC_EnableIRQ(IRQn_TMR2);
}

/**
 * @brief Возвразающает значение текущего времени в секундаъ
 * @note получаем время от таймера, а потом обрезаем до секунд 
 * @return uint32_t текущее время в секундах
 */
uint32_t Get_Time_s(void) // получаем время от таймера, а потом обрезаем до секунд 
{
    uint32_t time_s = 0, time_s_old = 0;
    time_s_old = MDR_TMR2->CNT;
    time_s = ((uint64_t)high_byte_time_s << 24) + ((time_s_old >> 8) & 0xFFFFFF);
    return time_s; // переставляем по 16 бит для заполнения поля времени кадров
}

/**
 * @brief Функция для получения времени в секундах (sec), и дробной части времени в 1/256 секнды (parts)
 * 
 * @param sec указатель на переменную куда будет записано значение текущего времени в секундах
 * @param parts указатель на переменную куда будет записано дробная часть текущего времени в 1/256 секунды
 */
void Get_Time_sec_parts(uint32_t* sec, uint8_t* parts) // получение полного времени
{
    uint32_t time_s_old = 0, time_s = 0;
	
    time_s_old = MDR_TMR2->CNT;
	time_s = ((uint32_t)high_byte_time_s << 24) + ((time_s_old >> 8) & 0xFFFFFF); 
    *sec = time_s;
    *parts = time_s_old & 0xFF;
}


/**
  * @brief  установка значения PWM для таймера 2
  * @param  ch_num номер канала PWM
  * @param  pwm_val знчение заполеннности PWM; 0-99 в 1%
  */
void Timer_PWM_Set(uint8_t ch_num, uint16_t pwm_val)
{   
    if (pwm_val >= 100){
        *TMR_CCR[ch_num&0x3] = 99;
    }
    else{
        *TMR_CCR[ch_num&0x3] = pwm_val;
    }
}

/**
  * @brief  установка значения PWM для таймера 3 в float
  * @param  ch_num номер канала PWM
  * @param  pwm_val_fp знчение заполеннности PWM; 0-99 в 1%, дробная часть также работает до точности 1/(1<<16)
  */
void Timer_PWM_Set_Fp(uint8_t ch_num, float pwm_val_fp)
{   
    NVIC_DisableIRQ(IRQn_TMR3);
    if (pwm_val_fp >= 100){
        CCR1_base[ch_num&0x3] = 99;
        pwm_adder[ch_num&0x3] = 0.9*(1<<5);
    }
    else{
        CCR1_base[ch_num&0x3] = (uint16_t)pwm_val_fp;
        pwm_adder[ch_num&0x3] = (uint32_t)(pwm_val_fp*(1<<5))&0x1F;
    }
    NVIC_EnableIRQ(IRQn_TMR3);
}

/**
  * @brief  CallBack от обработчика прерывания TMR1, для описания в main
  */
__weak void INT_TMR1_CallBack(void)
{
  //
}



