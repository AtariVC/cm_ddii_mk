/**
  ******************************************************************************
  * @file           : spi.c
  * @version        : v1.0
  * @brief          : низкоуровневая библиотека для работы с SPI на основе библиотеки Дорошкина А.А.
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "spi.h"

void SPI_Init()
{
  //
	CLK_CNTR->KEY = _KEY_;
  CLK_CNTR->PER0_CLK |= (1<<17);  //enable PortE clock
  CLK_CNTR->PER1_CLK |= (1<<2);  //enable clock for SSP0
  //
  MDR_SSP0->CR0 = 0x407;  // rate=1MHz,  SPH=0, SPO=0, Motorola, 8bit
  MDR_SSP0->CPSR = 2;  // rate=1MHz
  //
  PORTE->KEY = _KEY_;
  PORTE->CFUNC[0] = 0xF0000000;
  PORTE->CFUNC[1] = 0x00000F0F;
  PORTE->SFUNC[0] = 0x60000000;
  PORTE->SFUNC[1] = 0x00000606;
  PORTE->SANALOG =  0x00000780;
  PORTE->SRXTX =    0x00000780;  // установка значений порта
  //PORTE->SOE =      0x00000000;
  PORTE->SPWR[0] =  0x002A8000;  // 7-10 - b10
  //
  MDR_SSP0->CR1 = 2;  //enable SSP0
}

uint8_t SPI_Exchange(uint8_t data)
{
  MDR_SSP0->DR = data;
  while((MDR_SSP0->SR & 0x10) != 0);  //while busy
  return (uint8_t)MDR_SSP0->DR;
}

void SPI_MExchange(uint8_t *data_in, uint8_t *data_out, int len)
{
  int i;
  if(data_in == SPI_NULL) {
    for(i=0; i<len; i++) SPI_Exchange(data_out[i]);
    }
  else if(data_out == SPI_NULL) {
    for(i=0; i<len; i++) data_in[i] = SPI_Exchange(0);
    }
  else {
    for(i=0; i<len; i++) data_in[i] = SPI_Exchange(data_out[i]);
    }
}
