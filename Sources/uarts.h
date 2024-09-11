#ifndef _UARTS_H_
#define _UARTS_H_

#include "1986ve8_lib/cm4ikmcu.h"

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

void UART0_Init(void);
void UART0_SendPacket(uint8_t *buff, uint8_t leng);
int8_t UART0_PacketInWaiting(void);
int8_t UART0_PacketReady(void);
int8_t UART0_GetPacket(uint8_t *buff, uint8_t *leng);

void UART1_Init(void);
void UART1_SendPacket(uint8_t *buff, uint8_t *leng);
uint8_t UART1_GetPacket(uint8_t *buff, uint8_t *leng);

#endif

