  /**
  ******************************************************************************
  * @file           : debug.c
  * @version        : v1.0
  * @brief          : функции для отладки, отключаемы define DEBUG
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "debug.h"


/**
  * @brief  управление GPIO для отладки
  * @param  state состояние включения
  */
void dbg_gpio(uint8_t state)
{
#ifdef DEBUG
  PortControl* port = DBG_GPIO_PORT;
  if (state == DBG_GPIO_ON){
			port->SRXTX = (1<<(DBG_GPIO_NUM));
		}
		else{
			port->CRXTX = (1<<(DBG_GPIO_NUM));
		}
#endif
}
