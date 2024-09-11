#include "1986ve8_lib/cm4ikmcu.h"
#include "sysinit.h"


#ifdef _EXT_MEM_RLS_
const uint64_t ExtBusCfgCode[] __attribute__ ((at(0x10000400))) = 
{0x7171717171717171, 0x7171717171717171, 0,0,0,0,0,0,0,0};
#endif

extern uint32_t __Vectors;

void System_Init() {
  *((uint32_t*)0xE000ED08) = (uint32_t)&__Vectors;

  BKP->KEY = _KEY_;
  BKP->REG_60_TMR0 |= 1<<28; // diasable POR
  BKP->REG_60_TMR1 |= 1<<28; //
  BKP->REG_60_TMR2 |= 1<<28; //
  
  FT_CNTR->KEY = _KEY_;
  FT_CNTR->RESET_EVENT0 = 0x0; // hide all errors
  FT_CNTR->RESET_EVENT1 = 0x0; //
  FT_CNTR->RESET_EVENT2 = 0x0; //
  FT_CNTR->RESET_EVENT3 = 0x0; //
  FT_CNTR->RESET_EVENT4 = 0x0; //

  /*-------------------- Ports Init --------------------------*/
  CLK_CNTR->KEY = _KEY_;
  CLK_CNTR->PER0_CLK |= (1<<13)|(1<<14)|(1<<15)|(1<<16)|(1<<17);   //port A,B,C,D,E clock enable

  /* PA[29,28,26,25,24,23,21,20]-MIL0 */
	PORTA->KEY = _KEY_;
	PORTA->CFUNC[2]  = 0xF0FF0000;
	PORTA->SFUNC[2] =  0xC0CC0000;
	PORTA->CFUNC[3]  = 0x00FF0FFF;
	PORTA->SFUNC[3] =  0x00CC0CCC;
	PORTA->SPWR[1] =   0x003C0F00;
	PORTA->SANALOG =   0x37B00000;
	PORTA->SPULLDOWN = 0x31800000;

  /* PB[31,29,28]-MIL1; PB[26,25]-UART1; PB[19,18]-out1 TM; PB[17]-out1 MKADR; PB[16,15]-UART0; PB[14:9]-inpu MKADR; PB[0]-inpu ID */
  PORTB->KEY = _KEY_;
  PORTB->SANALOG =   0xFFFFFFFF;
  PORTB->SFUNC[3] =  0xD0DD0550;
  PORTB->SFUNC[2] =  0x00000005;
  PORTB->SFUNC[1] =  0x50000000;
  PORTB->SRXTX =     0x000E0000;
  PORTB->SOE =       0x000E0000;
  PORTB->SPWR[1] =   0x8A140055;
  PORTB->SPWR[0] =   0x40000000;
  PORTB->SPULLUP =   0x00007F01;
  PORTB->SPULLDOWN = 0x49F000FE;

  /* PC[31:30]-A[1:0]; PC[20:8]-ADC; PC[0]-MIL1; PC[16] - PWR MPP ON/OFF */
  PORTC->KEY = _KEY_;
  PORTC->SANALOG =   0xFFE180FF; // 0xFFE000FF
  PORTC->SFUNC[3] =  0x22000000; // 0x22000000
  PORTC->SFUNC[2] =  0x00000000; // 
  PORTC->SFUNC[1] =  0x00000000; //
  PORTC->SFUNC[0] =  0x0000000D; // 0x0000000D
  PORTC->SPWR[1] =   0xF0000001; // 0xF0000000
  PORTC->SPWR[0] =   0x40000002; // 0x00000002
  PORTC->SRXTX = 0x00018000;
  PORTC->SPD = 0x00000000;
  PORTC->SOE = 0x00018000;
  PORTC->SPULLDOWN = 0xFFE180FE; // 0xFFE000FE

  /* PD[31:30]-D[1:0]; PD[24:23]-~WE,~OE; PD[19]-~CS;  PD[18:15]-out1 ~CS[4:1]; PD[12:0]-A[14:2] */  
  PORTD->KEY = _KEY_;
  PORTD->SANALOG =   0xFFFFFFFF;
  PORTD->SFUNC[3] =  0x22000002;
  PORTD->SFUNC[2] =  0x20002000;
  PORTD->SFUNC[1] =  0x00022222;
  PORTD->SFUNC[0] =  0x22222222;
  PORTD->SRXTX =     0x00078000;
  PORTD->SOE =       0x00078000;
  PORTD->SPWR[1] =   0xF003C0EA;
  PORTD->SPWR[0] =   0xC3FFFFFF;
  PORTD->SPULLDOWN = 0x3E706000;

  /*PE[13, 15, 17, 19:23]-test csa PE[23:12]-discret out except 14, 16, 18, 20 - PWM; PE[10,8,7]-SPI; PE[5:0]-D[7:2]*/
  PORTE->KEY = _KEY_;
  PORTE->SANALOG   =  0xFFFFFFFF;
  PORTE->SFUNC[0]  =  0x60222222;
  PORTE->SFUNC[1]  =  0x08080606;
  PORTE->SFUNC[2]  =  0x00000808;
  PORTE->SFUNC[3]  =  0x00000000;
	PORTE->SRXTX     =  0x00015000;
  PORTE->SPD       =  0x00000000;
  PORTE->SOE       =  0x00FFF000;
  PORTE->SPWR[0]   =  0x55228FFF;
  PORTE->SPWR[1]   =  0x00005555;
  PORTE->SPULLDOWN =  0xFFFFFE40;

  /*------------ enable regions -----------*/
  EXT_BUS_CNTR->KEY = _KEY_;
  EXT_BUS_CNTR->RGN0_CNTRL = 0x4000361;
  EXT_BUS_CNTR->RGN2_CNTRL = 0x321;

  /*---------- System clock, PLL0 ----------*/
  CLK_CNTR->KEY = _KEY_;
  CLK_CNTR->HSE0_CLK = (1<<27)|(1<<28); // Enable HSE0 gen
  while((CLK_CNTR->HSE0_STAT & (1 << 20)) == 0); //wait HSE0
  CLK_CNTR->PLL0_CLK = (2<<29)|(1<<28)|(1<<27)|((SYSINIT_PLL0_N)<<8)|((SYSINIT_PLL0_Q-1)<<0);  //PLL0 On, Fout = 10*8/2 = 40MHz
  while((CLK_CNTR->PLL0_CLK & (1<<28)) == 0); //wait PLL ready
  CLK_CNTR->MAX_CLK = 8;  //MAX_CLOCK = PLL0 = 40 MHz
  CLK_CNTR->CPU_CLK = 0;  //Core clock: MAX_CLOCK

  /*---------- SysTick ----------*/
  SysTick_Config(SYSINIT_SYS_TICK_100US_VALUE);  //set 1ms period

  /*---------- Clock for ADC0 -------------*/
  CLK_CNTR->ADC0_CLK = ((uint32_t)1<<28)|(1<<16)| 7;  //HSE0 clock for ADC, freq=0.5 MHz
  CLK_CNTR->SSP0_CLK = ((uint32_t)1<<28)|(1<<16)| 0;  //HSE0 clock for SSP, freq=10MHz

#ifndef _INT_RAM_
  ICACHE->KEY = _KEY_;
  ICACHE->CNTL = 0x0B;
#endif
}



