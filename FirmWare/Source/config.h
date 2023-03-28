
#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#include "uart.h"

#define FLASH_DATA_ADDRESS      0x0803F800      //адрес для хранения параметров
                                                //последняя страница FLASH памяти (2Kb)
                                                //PM0075.pdf page: 8, table 4
//маски ошибок при сохранении параметров
#define ERR_FLASH_UNLOCK        0x10            //разблокировка памяти
#define ERR_FLASH_ERASE         0x20            //стирание FLASH
#define ERR_FLASH_PROGRAMM      0x40            //сохранение параметров
#define ERR_FLASH_LOCK          0x80            //блокировка памяти

#pragma pack( push, 1 )

//*************************************************************************************************
// Структура для хранения общих параметров системы
//*************************************************************************************************
typedef struct {
    UARTSpeed   debug_speed;                    //скорость отладочного порта (RS232)
    uint16_t    net_pan_id;                     //Personal Area Network ID – идентификатор сети
    uint8_t     net_group;                      //номер группы
    uint8_t     net_key[16];                    //ключ шифрования
    uint16_t    dev_numb;                       //номер уст-ва в сети
    uint16_t    addr_gate;                      //адрес шлюза с сети
 } CONFIG;

//структура хранения блока параметров в FLASH памяти
typedef struct {
    uint8_t  data[128 - sizeof( uint16_t )];
    uint16_t crc;
 } FLASH_DATA;

#pragma pack( pop )

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void ConfigInit( void );
uint8_t ConfigSave( void );
uint8_t ResetSrc( void );
char *FlashReadStat( void );
char *ResetSrcDesc( uint8_t flags );
char *ConfigError( uint8_t error );

#endif 
