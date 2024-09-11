/**
  ******************************************************************************
  * @file           : mko_rt.c
  * @version        : v1.0
  * @brief          : библиотека для работы с МКО в режиме оконечного устройства ОУ и КШ (RT and BC - range terminal, BC - bus controller)
  * @note           : ноги для управления настраиваются отдельно в sysinit.c
  * @note           : платформозависимый вариант для 1986ve8t
  * @author					: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
	* @date						: 2021.09.14
  ******************************************************************************
  */
#include "mko.h"

uint16_t MKOIVect;

/**
  * @brief  инициализация программной модели МКО
  * @param  mko_ptr указатель на програмную модель устройства 
  * @param  type тип устройства (MKO_BC или MKO_RT)
  * @param  regs указатель на регистры управления МКО
  * @param  mko_addr 0 - значение берется с перемычек, не 0 - используется указанное значение (только для MKO_RT)
  */
void mko_init(typeMKOStruct *mko_ptr, MIL1553Control *regs, uint8_t type, uint8_t mko_addr) 
{
  PortControl* mko_gpio_port[ADDR_CH_NUMBER] = MKO_ADDR_GPIO_PORT;
  uint8_t mko_gpio_line[ADDR_CH_NUMBER] = MKO_ADDR_GPIO_LINE;
  uint8_t i;
  //
	mko_ptr->regs = regs;
  mko_ptr->type = type;
  // привязка gpio адреса к используемым портам
  for (i=0; i<ADDR_CH_NUMBER; i++){
    gpio_init(&mko_ptr->gpio[i], mko_gpio_port[i], mko_gpio_line[i]);
  }
  //
	if (mko_addr==0){
		mko_ptr->addr = mko_get_addr_from_gpio(mko_ptr);
	}
  else{
    mko_ptr->addr = mko_addr;
  }
	//
	if (regs == MIL_STD_15531){
    mko_ptr->irq = IRQn_MIL_STD_15531;
    //
		CLK_CNTR->KEY = _KEY_;
		CLK_CNTR->PER1_CLK |= (1<<19);  //enable clock for MIL0
		CLK_CNTR->PER0_CLK |= (1<<13);  //clock for PORTA
	}
	else if (regs == MIL_STD_15532){
    mko_ptr->irq = IRQn_MIL_STD_15532;
    //
		CLK_CNTR->KEY = _KEY_;
		CLK_CNTR->PER1_CLK |= (1<<21);  //enable clock for MIL1
		CLK_CNTR->PER0_CLK |= (1<<14);  //clock for PORTB
	}
	else{
		// todo: обработчик ошибок
	}
	//
  mko_ptr->regs->CONTROL = 1;  //reset
  switch(mko_ptr->type){
    case (MKO_BC):
      mko_ptr->regs->CONTROL = (1<<20)|(40<<11)|(1<<4)|(1<<2);
    break;
    case (MKO_RT):
      mko_ptr->regs->CONTROL = (1<<20)|(40<<11)|(3<<4)|(2<<2)|((mko_addr & 0x1F)<<6);
      mko_ptr->regs->StatusWord1 = ((mko_addr & 0x1F)<<11);
      break;
  }
	/*enable interrupt*/
	mko_ptr->regs->INTEN = (0x1 << 2) | (0x1 << 3);  //valmess ena, error ena
	NVIC_EnableIRQ(mko_ptr->irq);
	//
  mko_clear_data(mko_ptr);
	mko_ptr->error_cnt = 0;
	mko_ptr->error = 0;
  mko_ptr->need_to_process_flag = 0;
	mko_ptr->aw.whole = (mko_ptr->addr & 0x1F)<<11;
  //
  if ((mko_ptr->addr == 0) && (mko_ptr->type == MKO_RT)){ //отключаем МКО
		mko_ptr->regs->CONTROL = 1;  //reset
	}
}

/**
  * @brief  установка сигнала занято
  * @param  mko_ptr указатель на програмную модель устройства 
  */
void mko_set_busy(typeMKOStruct *mko_ptr)
{
	mko_ptr->aw.field.busy |= 1;
	mko_ptr->regs->StatusWord1 = mko_ptr->aw.whole;
}

/**
  * @brief  снятие сигнала занято
  * @param  mko_ptr указатель на програмную модель устройства 
  */
void mko_release_busy(typeMKOStruct *mko_ptr)
{
	mko_ptr->aw.field.busy &= 0;
  mko_ptr->regs->StatusWord1 = mko_ptr->aw.whole;
}

/**
  * @brief  установка седьмого бита ответного слова
  * @param  mko_ptr указатель на програмную модель устройства
  * @param  val устанавливаемое значение (0/1)
  */
void mko_set_aw_bit_7(typeMKOStruct *mko_ptr, uint8_t val)
{
	if (val & 0x01) mko_ptr->aw.field.mem_empty |= 1;
	else mko_ptr->aw.field.mem_empty = 0;
    mko_ptr->regs->StatusWord1 = mko_ptr->aw.whole;
}

/**
  * @brief  очистка подадресов на чтение
  * @param  mko_ptr указатель на програмную модель устройства
  */
void mko_clear_data(typeMKOStruct *mko_ptr)
{    
    mko_set_busy(mko_ptr);
    memset((uint8_t*)mko_ptr->regs->DATA, 0x00, sizeof(mko_ptr->regs->DATA));
    mko_release_busy(mko_ptr);
}

/**
  * @brief  запись данных на подадрес по номеру для ОУ
  * @param  mko_ptr указатель на програмную модель устройства
  * @param  subaddr номер субадреса
  * @param  data данные для записи (длина - 32 слова)
  */
void mko_rt_write_to_subaddr(typeMKOStruct *mko_ptr, uint8_t subaddr, uint16_t* data)
{    
    uint8_t i;
    mko_set_busy(mko_ptr);
    for (i=0; i<32; i++)
    {
        mko_ptr->regs->DATA[subaddr*32 + i] = (uint32_t)data[i];
    }
    mko_release_busy(mko_ptr);
}

/**
  * @brief  чтение данных с подадреса по номеру для ОУ
  * @param  mko_ptr указатель на програмную модель устройства
  * @param  subaddr номер субадреса
  * @param  data данные для записи (длина - 32 слова)
  */
void mko_rt_read_from_subaddr(typeMKOStruct *mko_ptr, uint8_t subaddr, uint16_t* data)
{    
    uint8_t i;
    for (i=0; i<32; i++)
    {
        data[i] = mko_ptr->regs->DATA[subaddr*32 + i] & 0xFFFF;
    }
}

/**
  * @brief  обработка прерывания при удачной транзакции КШ
  * @param  mko_ptr указатель на програмную модель устройства
  */
void mko_bc_transaction_handler(typeMKOStruct *mko_ptr)
{
	mko_ptr->msg = mko_ptr->regs->MSG;
	mko_ptr->rcv_a = ((mko_ptr->regs->STATUS >> 9) & 0x01);
	mko_ptr->rcv_b = ((mko_ptr->regs->STATUS >> 10) & 0x01);
	mko_ptr->error = (mko_ptr->regs->ERROR & 0x3F);
	if (mko_ptr->regs->ERROR == 0) {
		//
  }
	else{
		mko_ptr->error_cnt ++;
	}
	//
	mko_ptr->regs->STATUS  = 0; // обнуляем статусы
}

/**
  * @brief  чтение данных с подадреса по номеру для КШ
  * @param  mko_ptr указатель на програмную модель устройства
  * @param  addr адресс устройства
  * @param  subaddr номер субадреса
  * @param  bus номер шины для общения
  * @param  data данные для записи (длина - 32 слова)
  * @param  leng количество данных
  */
void mko_bc_transaction_start(typeMKOStruct *mko_ptr, uint8_t mode, uint8_t addr, uint8_t subaddr, uint8_t bus, uint16_t* data, uint8_t leng)
{
  uint8_t i = 0;
	volatile uint32_t cw = 0;
  //заоплняем командное слово 1
	mko_ptr->regs->CONTROL &= ~(0x03 << 4);
  mko_ptr->regs->CONTROL |= (((~bus)&0x01)<<4) | (((bus)&0x01)<<5);
	mko_ptr->cw.whole = (addr << 11) | ((mode & 0x01) << 10) | ((subaddr&0x1F) << 5) | ((leng & 0x1F) << 0);
  mko_ptr->regs->CommandWord1 = mko_ptr->cw.whole;
  mko_ptr->regs->CommandWord2 = 0;
  //
  if (mode == MKO_MODE_WRITE){
    for (i=0; i<leng; i++) {
			mko_ptr->regs->DATA[subaddr*32 + i] = (uint32_t)data[i];
    }
  }
  // запускаем транзакцию
  mko_ptr->regs->CONTROL |= (1<<1);
  // ожидаем окончания транзакции
  while ((mko_ptr->regs->STATUS &= (1<<0)) ==  0) {}; // ожидание окончания активности канала
  // забираем данные с подадреса
  if (mode == MKO_MODE_READ){
    for (i=0; i<leng; i++) {
			data[i] = mko_ptr->regs->DATA[subaddr*32 + i] & 0xFFFF;
    }
  }
}

/**
  * @brief  блокировка работы ядра МКО (дальнейшая работа не возможно до перезагрузки)
  * @param  mko_ptr указатель на програмную модель устройства
  */
void mko_block_transmitter(typeMKOStruct *mko_ptr)
{
	mko_ptr->regs->CONTROL = 1;
}

/**
  * @brief  возврат значения ошибок модуля МКО
  * @param  mko_ptr указатель на програмную модель устройства
	* @param  error указатель на переменную с ошибкой МКО
	* @param  error_cnt указатель на переменную с счетсиком ошибок МКО
  */
void mko_get_error(typeMKOStruct *mko_ptr, uint8_t* error, uint8_t* error_cnt)
{
	*error = mko_ptr->error;
	*error_cnt = mko_ptr->error_cnt;
}

/**
  * @brief  обработка прерывания при удачной транзакции ОУ
  * @param  mko_ptr указатель на програмную модель устройства
  */
void mko_rt_transaction_handler(typeMKOStruct *mko_ptr)
{
  uint8_t i=0;
	mko_ptr->regs->STATUS  &= ~(3 << 9); // обнуляем флаг активности каналов МКО
  //
	mko_ptr->cw.whole = mko_ptr->regs->CommandWord1;
	mko_ptr->msg = mko_ptr->regs->MSG;
	if (mko_ptr->rcv_a == 1 | mko_ptr->rcv_b == 1){
    mko_ptr->mko_rcv_flag |= 0x01;
  } 
	mko_ptr->rcv_a = ((mko_ptr->regs->STATUS >> 9) & 0x01);
	mko_ptr->rcv_b = ((mko_ptr->regs->STATUS >> 10) & 0x01);
	mko_ptr->error = (mko_ptr->regs->ERROR & 0xFFFF);
	if (mko_ptr->regs->ERROR == 0) {
		if (mko_ptr->msg == 0x0410) { // обработка командных сообщений
			__mko_cmd_msg(mko_ptr);
		}
		else if (mko_ptr->cw.field.rd_wr) {  // транзакция на чтение подадреса
      return;
    }
    else if (mko_ptr->cw.field.rd_wr == 0) {  // транзакция на запись в подадрес
			memcpy((char*)&mko_ptr->regs->DATA[mko_ptr->cw.field.sub_addr*32], (char*)&mko_ptr->regs->DATA[mko_ptr->cw.field.sub_addr*32], 32);
      for (i=0; i<32; i++) {
        mko_ptr->data[i] = (uint16_t)mko_ptr->regs->DATA[mko_ptr->cw.field.sub_addr*32 + i];
      }
      mko_ptr->need_to_process_flag = 1;
    }
	}
	else{
		mko_ptr->error_cnt ++;
	}
}

/**
  * @brief  получение адресса из разъема МКО
  * @param  mko_ptr указатель на програмную модель устройства
  * @retval значение адреса на перемычке, 0 - ошибка четности или недопустимое значение
  */
uint8_t mko_get_addr_from_gpio(typeMKOStruct *mko_ptr)
{
	uint8_t mko_addr = 0x00, connector_parity = 0x00, address_parity = 0x00;
  //
	mko_addr =  gpio_get(&mko_ptr->gpio[ADDR0])<<0 |
              gpio_get(&mko_ptr->gpio[ADDR1])<<1 |
              gpio_get(&mko_ptr->gpio[ADDR2])<<2 |
              gpio_get(&mko_ptr->gpio[ADDR3])<<3 |
              gpio_get(&mko_ptr->gpio[ADDR4])<<4 |
              gpio_get(&mko_ptr->gpio[ADDR5])<<5;
  //
  address_parity =  gpio_get(&mko_ptr->gpio[ADDR0])<<0 ^
                    gpio_get(&mko_ptr->gpio[ADDR1])<<1 ^
                    gpio_get(&mko_ptr->gpio[ADDR2])<<2 ^
                    gpio_get(&mko_ptr->gpio[ADDR3])<<3 ^
                    gpio_get(&mko_ptr->gpio[ADDR4])<<4 ^
                    gpio_get(&mko_ptr->gpio[ADDR5])<<5;
  //
  connector_parity = gpio_get(&mko_ptr->gpio[PARITY]);
  //
  // if (connector_parity == address_parity){
    return mko_addr;
  // }
  // else{
  //   return 0;
  // }
}

/**
  * @brief  обработка командных сообщений согласно протоколу МКО
  * @note  в случае приема командного слова длина выступает кодом команды
  * @param  mko_ptr указатель на програмную модель устройства
  */
void __mko_cmd_msg(typeMKOStruct *mko_ptr)
{
    switch (mko_ptr->cw.field.leng){
      case 2: //передать ответное слово
        // ничего не делаем, ответное слово передается ядром
        break;
      case 4: //блокировать передатчик
        if (mko_ptr->rcv_a){
          mko_ptr->regs->CONTROL &= ~(1 << 5); //блокируем передатчик Б
        }
        else if(mko_ptr->rcv_b){
          mko_ptr->regs->CONTROL &= ~(1 << 4); //блокируем передатчик А
        }
        break;
      case 5: //разблокировать передатчик
        if (mko_ptr->rcv_a){
          mko_ptr->regs->CONTROL |= (1 << 5); //разблокируем передатчик Б
        }
        else if(mko_ptr->rcv_b){
          mko_ptr->regs->CONTROL |= (1 << 4); //разблокируем передатчик А
        }
        break;
      case 8: //разблокировать передатчик
        mko_ptr->regs->CONTROL |= 1;  //reset
        mko_ptr->regs->CONTROL &= ~1; //clear reset
        break;
    }
}

/**
  * @brief  проверка необходимости обработки команды
  * @param  mko_ptr указатель на програмную модель устройства
  * @retval 1 - there was a write transaction, 0 - nothing
  */
uint8_t mko_need_to_process(typeMKOStruct *mko_ptr)
{
	if (mko_ptr->need_to_process_flag) {
		mko_ptr->need_to_process_flag = 0;
		return 1;
	}
  return 0;
}

/**
  * @brief  CallBack от обработчика прерывания МКО ОУ, для описания в main с использованием структуры управления МКО ОУ
  */
__weak void INT_MIL0_Callback(void)
{
	//
}

/**
  * @brief  CallBack от обработчика прерывания МКО ОУ, для описания в main с использованием структуры управления МКО ОУ
  */
__weak void INT_MIL1_Callback(void)
{
	//
}

void INT_MIL0_Handler(void) 
{
	INT_MIL0_Callback();
}

void INT_MIL1_Handler(void) 
{
	INT_MIL1_Callback();
}
