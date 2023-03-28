
#ifndef __DATA_H
#define __DATA_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "water.h"
#include "valve.h"
#include "xtime.h"

//ID пакетов передаваемых/принимаемых через радио модуль
typedef enum {
    ZB_PACK_UNDEF,                      //тип пакета не определен
    //входящие ответа
    ZB_PACK_STATE,                      //данные состояния контроллера
    ZB_PACK_DATA,                       //текущие данные расхода/давления/утечки воды
    ZB_PACK_WLOG,                       //журнальные данные расхода/давления/утечки воды
    ZB_PACK_VALVE,                      //состояние электроприводов
    ZB_PACK_LEAKS,                      //состояние датчикоы утечки воды
    //исходящие запросы
    ZB_PACK_SYNC_DTIME,                 //синхронизация даты/времени
    ZB_PACK_REQ_STATE,                  //запрос текущего состояния контроллера
    ZB_PACK_REQ_VALVE,                  //состояние электроприводов подачи воды
    ZB_PACK_REQ_DATA,                   //запрос журнальных/текущих данных расхода/давления/утечки воды
    ZB_PACK_CTRL_VALVE,                 //управление электроприводами подачи воды
    ZB_PACK_ACK                         //подтверждение получение пакета с журнальными данными
 } ZBTypePack;

#pragma pack( push, 1 )

//Структура данных для хранения списка уст-в и их адресов
typedef struct {
    uint16_t        numb_dev;           //номер уст-ва в сети
    uint16_t        addr_dev;           //адрес уст-ва в сети
    uint32_t        last_upd;           //время прошедшее с последнего обновления данных от уст-ва (сек)
} DEV_LIST;

//Структура для хранения параметров уст-ва для формирования 
//подтверждений при запросе журнальных данных по команде ZB_PACK_REQ_DATA 
typedef struct {
    uint16_t dev_numb;
    uint16_t net_addr;
 } DATA_ACK;

//*************************************************************************************************
// Входящие пакеты от уст-ва
// для получения корректного значения адреса отправителя необходимо 
// выполнить перестановку байт: __REVSH( addr_send )
//*************************************************************************************************
//Текущая дата/время, источник сброса, даты/время включения контроллера
typedef struct {
    ZBTypePack      type_pack;          //тип пакета
    uint16_t        dev_numb;           //номер уст-ва в сети
    uint16_t        dev_addr;           //адрес уст-ва в сети
    DATE_TIME       rtc;                //текущее даты/время
    DATE_TIME       start;              //дата/время включения контроллера
    uint8_t         res_src;            //источник сброса
    uint16_t        crc;                //контрольная сумма
    uint16_t        addr_send;          //адрес отправителя (в подсчет контрольной суммы не входит)
 } PACK_STATE;

//Текущие/журнальные данные расхода/давления/утечки воды/состояния электроприводов
typedef struct {
    ZBTypePack      type_pack;          //тип пакета
    uint16_t        dev_numb;           //номер уст-ва в сети
    uint16_t        dev_addr;           //адрес уст-ва в сети
    //значения учета
    DATE_TIME       date_time;          //дата/время события
    uint32_t        count_cold;         //значения счетчика холодной воды
    uint32_t        count_hot;          //значения счетчика горячей воды
    uint32_t        count_filter;       //значения счетчика питьевой воды
    uint16_t        pressr_cold;        //давление холодной воды
    uint16_t        pressr_hot;         //давление горячей воды
    LeakStat        leak1 : 1;          //состояние датчика утечки #1
    LeakStat        leak2 : 1;          //состояние датчика утечки #2
    unsigned        reserv : 4;         //выравнивание до 1 байта
    EventType       type_event : 1;     //признак данных: данные/событие
    DC12VStat       dc12_chk : 1;       //контроль напряжения 12VDc для питания датчиков утечки
    VALVE_STAT_ERR  valve_stat;         //состояния электроприводов
    uint16_t        crc;                //контрольная сумма
    uint16_t        addr_send;          //адрес отправителя (в подсчет контрольной суммы не входит)
 } PACK_DATA;

//Состояния электроприводов
typedef struct {
    ZBTypePack      type_pack;          //тип пакета
    uint16_t        dev_numb;           //номер уст-ва в сети
    uint16_t        dev_addr;           //адрес уст-ва в сети
    //состояния электроприводов
    VALVE_STAT_ERR  valve_stat;         //состояния электроприводов
    uint16_t        crc;                //контрольная сумма
    uint16_t        addr_send;          //адрес отправителя (в подсчет контрольной суммы не входит)
 } PACK_VALVE;

//Состояния датчиков утечки
typedef struct {
    ZBTypePack      type_pack;          //тип пакета
    uint16_t        dev_numb;           //номер уст-ва в сети
    uint16_t        dev_addr;           //адрес уст-ва в сети
    LeakStat        leak1 : 1;          //состояние датчика утечки #1
    LeakStat        leak2 : 1;          //состояние датчика утечки #2
    unsigned        reserv : 5;         //выравнивание до 1 байта
    DC12VStat       dc12_chk : 1;       //контроль напряжения 12VDc для питания датчиков утечки
    uint16_t        crc;                //контрольная сумма
    uint16_t        addr_send;          //адрес отправителя (в подсчет контрольной суммы не входит)
 } PACK_LEAKS;

//*************************************************************************************************
// Исходящие пакеты координатора
// При формировании исходящего пакета в ZigBee модуле автоматически будет добавлен
// адрес отправителя (координатора)
//*************************************************************************************************
//Cинхронизация даты/времени
typedef struct {
    ZBTypePack      type_pack;          //тип пакета
    DATE_TIME       date_time;          //дата/время
    uint16_t        crc;                //контрольная сумма
 } ZB_PACK_RTC;

//Запрос данных ZB_PACK_REQ_STATE, ZB_PACK_REQ_DATA
typedef struct {
    ZBTypePack      type_pack;          //тип пакета
    uint16_t        dev_numb;           //номер уст-ва в сети
    uint16_t        dev_addr;           //адрес уст-ва в сети
    uint8_t         count_log;          //кол-во записей из журнала
    uint16_t        crc;                //контрольная сумма
 } ZB_PACK_REQ;

//Команды управления электроприводами
typedef struct {
    ZBTypePack      type_pack;          //тип пакета
    uint16_t        dev_numb;           //номер уст-ва в сети
    uint16_t        dev_addr;           //адрес уст-ва в сети
    ValveCtrlMode   cold;               //команда управления электропривода горячей воды
    ValveCtrlMode   hot;                //команды управления электропривода холодной воды
    uint16_t        crc;                //контрольная сумма
 } ZB_PACK_CTRL;

//Подтверждение получение данных PACK_DATA 
typedef struct {
    ZBTypePack      type_pack;          //тип пакета
    uint16_t        dev_numb;           //номер уст-ва в сети
    uint16_t        dev_addr;           //адрес уст-ва в сети
    uint16_t        crc;                //контрольная сумма
 } ZB_PACK_ACKDATA;

#pragma pack( pop )

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void DevListUpd( void );
void DevListClr( void );
void DeviceList( void );
void OutData( ZBTypePack id_pack );
uint8_t *CreatePack( ZBTypePack type, uint16_t dev_numb, uint16_t *net_addr, uint8_t count_log, ValveCtrlMode cold, ValveCtrlMode hot, uint8_t *len );
uint8_t CheckPack1( uint8_t *data );
ZBTypePack CheckPack2( uint8_t *data, uint8_t len, DATA_ACK *ptr_data );


#endif 
