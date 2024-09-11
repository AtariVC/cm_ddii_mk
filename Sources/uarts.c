/**
  ******************************************************************************
  * @file           : uarts.c
  * @version        : v1.0
  * @brief          : низкоуровневые драйверы для раобты с UART0, UART1/
  *                   UART0 - пакетная передача с использованием таймера, заготовка под ModBus (и в итоге для ВШ)
  *                   UART1 - упрощенный вариант
  *                 !! **Внимание** !! данынй модуль испольузет таймер. Данный таймер не должен быть использован в других местах
  * @author			    : Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  * @date						: 2022.02.07
  ******************************************************************************
  */

#include "1986ve8_lib/cm4ikmcu.h"
#include <string.h>
#include "wdt.h"


#define IRQn_UART0 (IRQn_Type)108
#define IRQn_UART1 (IRQn_Type)109


uint8_t Rx0Buff[256];
uint8_t Rx0BuffPtr;
uint16_t CRC16;

uint8_t Rx1Buff[256];
uint8_t Rx1BuffPtr;

void GetCRC(uint8_t *data, int len);

//*** UART 0 ***//

void UART0_Init() {
  Rx0BuffPtr = 0;
  CLK_CNTR->KEY = _KEY_;
  CLK_CNTR->PER1_CLK |= (1<<6);  //clock for UART0
  CLK_CNTR->UART0_CLK = ((uint32_t)13<<28)|(1<<16)|(1<<0);  //clock for UART0 = MAXCLK/2 = 20MHz
  MDR_UART0->IBRD = 10;  //baudrate = 125000
  //MDR_UART0->IBRD = 11;  //baudrate ~115200
  MDR_UART0->FBRD = 0;
  MDR_UART0->LCR_H = (3<<5);  //8bit
  MDR_UART0->IMSC = (1<<4);  //Rx interrupt enable
  MDR_UART0->CR = (1<<9)|(1<<8)|(1<<0);  //enable RX, TX
  /*Timer TMR_0*/
  CLK_CNTR->PER0_CLK |= (1<<23);  //timer0 clock enable
  CLK_CNTR->TIM0_CLK = (1<<16)| 39;  //timer clock freq = 1 MHz
  MDR_TMR0->ARR = 1000;  //packet gap = 1 ms
  MDR_TMR0->CNTRL = 1;  //timer enable
  /*enable interrupt*/
  NVIC_EnableIRQ(IRQn_UART0);
}

void UART0_SendPacket(uint8_t *buff, uint8_t leng) {
  uint8_t i;
  MDR_UART0->CR &= ~(1<<9);  //çàïðåò ïðèåìà
  for(i=0; i<leng; i++) {
    while((MDR_UART0->FR & (1<<7))==0);
    MDR_UART0->DR = buff[i];
    WDRST;
    }
  while(MDR_UART0->FR & (1<<3));  //wait for Busy
  MDR_UART0->CR |= (1<<9);  //âíîâü ðàçðåøåíèå ïðèåìà
}

int8_t UART0_PacketInWaiting(void)
{
	if ((MDR_TMR0->STATUS & (1<<1)) == 0){
		return 1;
	}
	else {
		return 0;
	}
}

int8_t UART0_PacketReady(void)
{
	if((Rx0BuffPtr) && (MDR_TMR0->STATUS & (1<<1))){
		return 1;
	}
	else {
		return 0;
	}
}

int8_t UART0_GetPacket(uint8_t *buff, uint8_t *leng) {
  NVIC_DisableIRQ(IRQn_UART0); //
  if((Rx0BuffPtr)&&(MDR_TMR0->STATUS & (1<<1))) {  //åñòü ÷òî-òî â áóôåðå è ïðåâûøåí ìåæñèìâîëüíûé èíòåðâàë
    *leng = Rx0BuffPtr;
    Rx0BuffPtr = 0;
    memcpy(buff, Rx0Buff, *leng);
    NVIC_EnableIRQ(IRQn_UART0);
    return 1;
  }
  else {
      NVIC_EnableIRQ(IRQn_UART0);
      return 0;
  }
}

void INT_UART0_Handler(void) {
  uint8_t rxb;
  if(MDR_UART0->RIS & (1<<4)) {  //ïðåðûâàíèå ïî Rx
    rxb = MDR_UART0->DR;
    if(MDR_TMR0->STATUS & (1<<1)) {  //timeout
      Rx0BuffPtr = 0;
    }
    Rx0Buff[Rx0BuffPtr++] = rxb;
    MDR_TMR0->CNT = 0;
    MDR_TMR0->STATUS = 0;
    }
}

//*** UART 1 ***//

/**
 * @brief инициализация ядра UART на работу
 * 
 */
void UART1_Init(void)
{
  Rx1BuffPtr = 0;
  CLK_CNTR->KEY = _KEY_;
  CLK_CNTR->PER1_CLK |= (1<<7);  //clock for UART1
  CLK_CNTR->UART1_CLK = ((uint32_t)13<<28)|(1<<16)|(1<<0);  //clock for UART1 = MAXCLK/2 = 20MHz
  MDR_UART1->IBRD = 65;  // baudrate = 19200  BRD = 65.104
  MDR_UART1->FBRD = 7;   // 0.104*64
  MDR_UART1->LCR_H = (3<<5);  //8bit
  MDR_UART1->IMSC = (1<<4);  //Rx interrupt enable
  MDR_UART1->CR = (1<<9)|(1<<8)|(1<<0);  //enable RX, TX
  /*enable interrupt*/
  NVIC_EnableIRQ(IRQn_UART1);
}

/**
 * @brief отправка данных через ядро UART, блокирующий
 * 
 * @param buff указатель на буфер для передачи данных
 * @param leng дляна данных для передачи (в байтах)
 */
void UART1_SendPacket(uint8_t *buff, uint8_t leng) 
{
  uint8_t i;
  MDR_UART1->CR &= ~(1<<9);  //запрет приема
  for(i=0; i<leng; i++) {
    while((MDR_UART1->FR & (1<<7))==0);
    MDR_UART1->DR = buff[i];
    WDRST;
    }
  while(MDR_UART1->FR & (1<<3));  //wait for Busy
  MDR_UART1->CR |= (1<<9);  //разрешение приема
}

/**
 * @brief прием данныхчерез ядро UART, копирование из буфера, который собирается прерываниями
 * 
 * @param buff указатель на массив для сохранения данных
 * @param *leng указатель на переменную с количество
 * @return uint8_t количество прочитанных данных
 */
uint8_t UART1_GetPacket(uint8_t *buff, uint8_t *leng)
{
  NVIC_DisableIRQ(IRQn_UART1);
  //
  memcpy(buff, Rx1Buff, Rx1BuffPtr);
  NVIC_EnableIRQ(IRQn_UART1);
  //
  Rx1BuffPtr = 0;
  return 0;
}

/**
 * @brief обработка прерывания от UART
 * 
 */
void INT_UART1_Handler(void)
{
  if(MDR_UART1->RIS & (1<<4)) {  //обработка прерывания от приемника
    if(Rx1BuffPtr <= 255){
      Rx1Buff[Rx1BuffPtr++] = MDR_UART1->DR;
    }
  }
}
