/**
  ******************************************************************************
  * @file           : spi_fram.c
  * @version        : v1.0
  * @brief          : низкоуровневая библиотека для работы с SPI FRAM
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "spi_fram.h"

/**
  * @brief  инициализация програмной модели работы с fram
  * @param  fram_ptr указатель на програмную модель устройства
  * @retval статус инициализации
  */
int8_t fram_init(type_FRAM* fram_ptr)
{
  uint8_t i;
  PortControl* port_arr[FRAM_MEM_NUMBER] = MEM_CS_PORT;
  uint8_t line_arr[FRAM_MEM_NUMBER] = MEM_CS_LINE;
  //
  SPI_Init();
  for(i=0; i<FRAM_MEM_NUMBER; i++){
    gpio_init(&fram_ptr->cs[i], port_arr[i], line_arr[i]);
  }
  //
  return 1;
}

/**
  * @brief  запись в отдельный чип памяти fram
  * @param  fram_ptr указатель на програмную модель устройства
  * @param  mem_num  номер чипа в массиве fram-памятей
  * @param  fram_addr  байтовый адрес внутри fram
  * @param  data_ptr  указатель на массив данных
  * @param  leng  длина данных
  */
void fram_write(type_FRAM* fram_ptr, uint8_t mem_num, uint32_t fram_addr, uint8_t *data_ptr, uint16_t leng) {
  gpio_set(&fram_ptr->cs[mem_num], 0);
  SPI_Exchange(0x06);  //WREN
  gpio_set(&fram_ptr->cs[mem_num], 1);
  gpio_set(&fram_ptr->cs[mem_num], 0);
  SPI_Exchange(0x02);  //WRITE
  SPI_Exchange(*((uint8_t*)&fram_addr+2));
  SPI_Exchange(*((uint8_t*)&fram_addr+1));
  SPI_Exchange(*((uint8_t*)&fram_addr));  
  SPI_MExchange(SPI_NULL, data_ptr, leng);
  gpio_set(&fram_ptr->cs[mem_num], 1);
}

/**
  * @brief  чтение из отдельный чип памяти fram
  * @param  fram_ptr указатель на програмную модель устройства
  * @param  mem_num  номер чипа в массиве fram-памятей
  * @param  fram_addr  байтовый адрес внутри fram
  * @param  data_ptr  указатель на массив данных
  * @param  leng  длина данных
  */
void fram_read(type_FRAM* fram_ptr, uint8_t mem_num, uint32_t fram_addr, uint8_t *data_ptr, uint16_t leng)
{
  gpio_set(&fram_ptr->cs[mem_num], 0);
  SPI_Exchange(0x03);  //READ
  SPI_Exchange(*((uint8_t*)&fram_addr+2));
  SPI_Exchange(*((uint8_t*)&fram_addr+1));
  SPI_Exchange(*((uint8_t*)&fram_addr));  
  SPI_MExchange(data_ptr, SPI_NULL, leng);
  gpio_set(&fram_ptr->cs[mem_num], 1);
}

/**
  * @brief  запись кадра в общую память
  * @param  fram_ptr указатель на програмную модель устройства
  * @param  addr_fr адрес в кадрах
  * @param  data_ptr указатель на массив данных (длиной в количество кадров)
  */
int8_t fram_write_frame(type_FRAM* fram_ptr, uint32_t addr_fr, uint8_t *data_ptr)
{
  uint8_t mem_num;
  uint32_t fram_addr_b, global_addr_b;
  // определение параметров записи
  global_addr_b = addr_fr * FRAM_FRAME_VOLUME_B;
  mem_num = global_addr_b / FRAM_VOLUME_B;
  fram_addr_b = global_addr_b % FRAM_VOLUME_B;
  // проверка параметров записи
  if(mem_num >= FRAM_MEM_NUMBER) {
    return -1;
  }
  if(fram_addr_b >= FRAM_VOLUME_B) {
    return -2;
  }
  // запись в нужную микросхему
  fram_write(fram_ptr, mem_num, fram_addr_b, data_ptr, FRAM_FRAME_VOLUME_B);
  return 0;
}

/**
  * @brief  чтение кадра из общей памяти
  * @param  fram_ptr указатель на програмную модель устройства
  * @param  addr_fr адрес в кадрах
  * @param  data_ptr указатель на массив данных (длиной в количество кадров)
  */
int8_t fram_read_frame(type_FRAM* fram_ptr, uint32_t addr_fr, uint8_t *data_ptr)
{
  uint8_t mem_num;
  uint32_t fram_addr_b, global_addr_b;
  // определение параметров записи
  global_addr_b = addr_fr * FRAM_FRAME_VOLUME_B;
  mem_num = global_addr_b / FRAM_VOLUME_B;
  fram_addr_b = global_addr_b % FRAM_VOLUME_B;
  // проверка параметров записи
  if(mem_num >= FRAM_MEM_NUMBER){
    return -1;
  }
  if(fram_addr_b >= FRAM_VOLUME_B){
    return -2;
  }
  // запись в нужную микросхему
  fram_read(fram_ptr, mem_num, fram_addr_b, data_ptr, FRAM_FRAME_VOLUME_B);
  return 0;
}
