#ifndef _GPIO_H_
#define _GPIO_H_

#include "1986ve8_lib/cm4ikmcu.h"

#define GPIO_ON 1
#define GPIO_OFF 0

#pragma pack(2)
/** 
  * @brief  структура управления одним GPIO
  */
typedef struct
{
  PortControl* port;
  uint8_t num;
} type_SINGLE_GPIO;

//
void gpio_init(type_SINGLE_GPIO* gpio_ptr, PortControl* port, uint8_t num);
void gpio_set(type_SINGLE_GPIO* gpio_ptr, uint8_t val);
uint8_t gpio_get(type_SINGLE_GPIO* gpio_ptr);
void gpio_togle(type_SINGLE_GPIO* gpio_ptr);

#endif
