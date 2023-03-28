
//*************************************************************************************************
//
// Идентификация и проверка принятых пакетов данных
// Формирование данных для передачи по ZigBee
// Вывод принятой информации
// 
//*************************************************************************************************

#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "cmsis_os2.h"

#include "data.h"
#include "water.h"
#include "config.h"
#include "parse.h"
#include "crc16.h"
#include "xtime.h"
#include "zigbee.h"
#include "message.h"

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern CONFIG config;
extern ZB_CONFIG zb_cfg;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define DEV_LIST_MAX            10          //максимальное кол-во устройств
#define MAX_TIME_UPDATE         120         //максимальное время ожидания периодической (состояния)
                                            //информации от уст-ва, если в течении этого времения 
                                            //информации от уст-ва не приходит - уст-во удалется 
                                            //из списка (сек)

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static char str[80];

static PACK_STATE       pack_state;
static PACK_DATA        pack_data;
static PACK_VALVE       pack_valve;
static PACK_LEAKS       pack_leaks;

static ZB_PACK_RTC      zb_pack_rtc;
static ZB_PACK_REQ      zb_pack_req;
static ZB_PACK_CTRL     zb_pack_ctrl;
static ZB_PACK_ACKDATA  zb_pack_ack;

static DEV_LIST         dev_list[DEV_LIST_MAX];

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static uint16_t DevGetAddr( uint16_t dev_numb );
static ErrorStatus CheckDevList( uint16_t dev_numb, uint16_t dev_addr );

//*************************************************************************************************
// Предваительная идентификация принятого пакета на соответствие: типа пакета
//-------------------------------------------------------------------------------------------------
// uint8_t *data - указатель на буфер принятого пакета
// return        - размер идентифицированного пакета данных или "0" если пакет не идентифицирован
//*************************************************************************************************
uint8_t CheckPack1( uint8_t *data ) {

    ZBTypePack type;
    
    type = (ZBTypePack)*data;
    if ( type == ZB_PACK_STATE )
        return sizeof( PACK_STATE );
    if ( ( type == ZB_PACK_DATA || type == ZB_PACK_WLOG ) )
        return sizeof( PACK_DATA );
    if ( type == ZB_PACK_VALVE )
        return sizeof( PACK_VALVE );
    if ( type == ZB_PACK_LEAKS )
        return sizeof( PACK_LEAKS );
    return 0;
 }

//*************************************************************************************************
// Проверка принятого пакета на соответствие: типа пакета/размера/контрольная сумма
//-------------------------------------------------------------------------------------------------
// uint8_t *data      - указатель на буфер принятого пакета
// uint8_t len        - размер принятого пакета
// DATA_ACK *ptr_data - указатель на структуру в которой размещаются номер и адрес отправителя
//                      для последующего использования при отправки подтверждения
// return             - тип идентифицированного пакета данных или ZB_PACK_UNDEF
//*************************************************************************************************
ZBTypePack CheckPack2( uint8_t *data, uint8_t len, DATA_ACK *ptr_data ) {

    uint16_t crc, addr_send;
    ZBTypePack type;
    
    ptr_data->dev_numb = 0;
    ptr_data->net_addr = 0;
    type = (ZBTypePack)*data;
    if ( type == ZB_PACK_STATE && len == sizeof( pack_state ) ) {
        //текущее состояние контроллера
        memcpy( (uint8_t *)&pack_state, data, sizeof( pack_state ) );
        //КС считаем без полученной КС и net_addr (gate_addr не входит в подсчет КС)
        crc = CalcCRC16( (uint8_t *)&pack_state, sizeof( pack_state ) - ( sizeof( uint16_t ) * 2 ) );
        if ( pack_state.crc != crc ) {
            ZBIncError( ZB_ERROR_CRC );
            return ZB_PACK_UNDEF;
           }
        //проверка адреса отправителя
        addr_send = __REVSH( *( (uint16_t *)&pack_state.addr_send ) );
        if ( pack_state.dev_addr != addr_send ) {
            ZBIncError( ZB_ERROR_ADDR );
            return ZB_PACK_UNDEF;
           }
        //добавим адрес уст-ва в список доступных уст-в
        CheckDevList( pack_state.dev_numb, pack_state.dev_addr );
        return type;
       }
    if ( ( type == ZB_PACK_DATA || type == ZB_PACK_WLOG ) && len == sizeof( pack_data ) ) {
        //текущие данные расхода/давления/утечки воды
        //журнальных данных расхода/давления/утечки воды
        memcpy( (uint8_t *)&pack_data, data, sizeof( pack_data ) );
        //КС считаем без полученной КС и net_addr (gate_addr не входит в подсчет КС)
        crc = CalcCRC16( (uint8_t *)&pack_data, sizeof( pack_data ) - ( sizeof( uint16_t ) * 2 ) );
        if ( pack_data.crc != crc ) {
            ZBIncError( ZB_ERROR_CRC );
            return ZB_PACK_UNDEF;
           }
        //проверка адреса отправителя
        addr_send = __REVSH( *( (uint16_t *)&pack_data.addr_send ) );
        if ( pack_data.dev_addr != addr_send ) {
            ZBIncError( ZB_ERROR_ADDR );
            return ZB_PACK_UNDEF;
           }
        //добавим адрес уст-ва в список доступных уст-в
        CheckDevList( pack_data.dev_numb, pack_data.dev_addr );
        if ( type == ZB_PACK_WLOG && ptr_data != NULL ) {
            //данные для формирования пакета подтверждения ZB_PACK_ACK
            ptr_data->dev_numb = pack_data.dev_numb;
            ptr_data->net_addr = pack_data.dev_addr;
           }
        return type;
       }
    if ( type == ZB_PACK_VALVE && len == sizeof( pack_valve ) ) {
        //текущее состояние уст-ва
        memcpy( (uint8_t *)&pack_valve, data, sizeof( pack_valve ) );
        //КС считаем без полученной КС и net_addr (gate_addr не входит в подсчет КС)
        crc = CalcCRC16( (uint8_t *)&pack_valve, sizeof( pack_valve ) - ( sizeof( uint16_t ) * 2 ) );
        if ( pack_valve.crc != crc ) {
            ZBIncError( ZB_ERROR_CRC );
            return ZB_PACK_UNDEF;
           }
        //проверка адреса отправителя
        addr_send = __REVSH( *( (uint16_t *)&pack_valve.addr_send ) );
        if ( pack_valve.dev_addr != addr_send ) {
            ZBIncError( ZB_ERROR_ADDR );
            return ZB_PACK_UNDEF;
           }
        //добавим адрес уст-ва в список доступных уст-в
        CheckDevList( pack_valve.dev_numb, pack_valve.dev_addr );
        return type;
       }
    if ( type == ZB_PACK_LEAKS && len == sizeof( pack_leaks ) ) {
        //текущее состояние электроприводов
        memcpy( (uint8_t *)&pack_leaks, data, sizeof( pack_leaks ) );
        //КС считаем без полученной КС и net_addr (gate_addr не входит в подсчет КС)
        crc = CalcCRC16( (uint8_t *)&pack_leaks, sizeof( pack_leaks ) - ( sizeof( uint16_t ) * 2 ) );
        if ( pack_leaks.crc != crc ) {
            ZBIncError( ZB_ERROR_CRC );
            return ZB_PACK_UNDEF;
           }
        //проверка адреса отправителя
        addr_send = __REVSH( *( (uint16_t *)&pack_leaks.addr_send ) );
        if ( pack_leaks.dev_addr != addr_send ) {
            ZBIncError( ZB_ERROR_ADDR );
            return ZB_PACK_UNDEF;
           }
        //добавим адрес уст-ва в список доступных уст-в
        CheckDevList( pack_leaks.dev_numb, pack_leaks.dev_addr );
        return type;
       }
    return ZB_PACK_UNDEF;
 }

//*************************************************************************************************
// Функция формирует пакет данных для отправки по ZigBee
//-------------------------------------------------------------------------------------------------
// ZBTypePack type    - тип формируемого пакета
// uint16_t dev_numb  - номер уст-ва
// uint16_t *net_addr - адрес уст-ва
// uint8_t count_log  - кол-во запрашиваемых записей (только для ZB_PACK_REQ_DATA)
// ValveCtrlMode cold - команда управления электроприводом крана холодной воды
// ValveCtrlMode hot  - команды управления электроприводом крана горячей воды
// uint8_t *len       - указатель на переменную для размещения размера сформированного пакета
// return == NULL     - тип пакета не указан
//        != NULL     - указатель на пакет данных
//*************************************************************************************************
uint8_t *CreatePack( ZBTypePack type, uint16_t dev_numb, uint16_t *net_addr, uint8_t count_log, ValveCtrlMode cold, ValveCtrlMode hot, uint8_t *len ) {

    *len = 0;
    if ( type == ZB_PACK_SYNC_DTIME ) {
        //синхронизация дата/время
        zb_pack_rtc.type_pack = ZB_PACK_SYNC_DTIME;     //тип пакета
        GetTimeDate( &zb_pack_rtc.date_time );          //текущие дата/время
        *net_addr = 0;
        //контрольная сумма
        zb_pack_rtc.crc = CalcCRC16( (uint8_t *)&zb_pack_rtc, sizeof( zb_pack_rtc ) - sizeof( zb_pack_rtc.crc ) );
        *len = sizeof( zb_pack_rtc );
        return (uint8_t *)&zb_pack_rtc;
       }
    if ( type == ZB_PACK_REQ_STATE ) {
        //запрос текущего состояния контроллера
        zb_pack_req.type_pack = ZB_PACK_REQ_STATE;      //тип пакета
        zb_pack_req.dev_numb = dev_numb;                //номер уст-ва
        zb_pack_req.dev_addr = DevGetAddr( dev_numb );  //адрес уст-ва в сети
        *net_addr = zb_pack_req.dev_addr;
        zb_pack_req.count_log = 0;                      //кол-во запрашиваемых записей из журнала
        if ( !zb_pack_req.dev_addr || !dev_numb )
            return NULL; //номер и адрес уст-ва не могут быть равны "0"
        zb_pack_req.crc = CalcCRC16( (uint8_t *)&zb_pack_req, sizeof( zb_pack_req ) - sizeof( zb_pack_req.crc ) );
        *len = sizeof( zb_pack_req );
        return (uint8_t *)&zb_pack_req;
       }
    if ( type == ZB_PACK_REQ_DATA ) {
        //запрос журнальных/текущих данных расхода/давления/утечки воды
        zb_pack_req.type_pack = ZB_PACK_REQ_DATA;       //тип пакета
        zb_pack_req.dev_numb = dev_numb;                //номер уст-ва
        zb_pack_req.dev_addr = DevGetAddr( dev_numb );  //адрес уст-ва в сети
        *net_addr = zb_pack_req.dev_addr;
        zb_pack_req.count_log = count_log;              //кол-во запрашиваемых записей из журнала
        if ( !zb_pack_req.dev_addr || !dev_numb )
            return NULL; //номер и адрес уст-ва не могут быть равны "0"
        zb_pack_req.crc = CalcCRC16( (uint8_t *)&zb_pack_req, sizeof( zb_pack_req ) - sizeof( zb_pack_req.crc ) );
        *len = sizeof( zb_pack_req );
        return (uint8_t *)&zb_pack_req;
       }
    if ( type == ZB_PACK_REQ_VALVE ) {
        //запрос текущего состояния электроприводов подачи воды
        zb_pack_req.type_pack = ZB_PACK_REQ_VALVE;      //тип пакета
        zb_pack_req.dev_numb = dev_numb;                //номер уст-ва
        zb_pack_req.dev_addr = DevGetAddr( dev_numb );  //адрес уст-ва в сети
        *net_addr = zb_pack_req.dev_addr;
        zb_pack_req.count_log = 0;                      //кол-во запрашиваемых записей из журнала
        if ( !zb_pack_req.dev_addr || !dev_numb )
            return NULL; //номер и адрес уст-ва не могут быть равны "0"
        zb_pack_req.crc = CalcCRC16( (uint8_t *)&zb_pack_req, sizeof( zb_pack_req ) - sizeof( zb_pack_req.crc ) );
        *len = sizeof( zb_pack_req );
        return (uint8_t *)&zb_pack_req;
       }
    if ( type == ZB_PACK_CTRL_VALVE ) {
        //управление электроприводами подачи воды
        zb_pack_ctrl.type_pack = ZB_PACK_CTRL_VALVE;    //тип пакета
        zb_pack_ctrl.dev_numb = dev_numb;               //номер уст-ва
        zb_pack_ctrl.dev_addr = DevGetAddr( dev_numb ); //адрес уст-ва в сети
        *net_addr = zb_pack_ctrl.dev_addr;
        if ( !zb_pack_ctrl.dev_addr || !dev_numb )
            return NULL; //номер и адрес уст-ва не могут быть равны "0"
        zb_pack_ctrl.cold = cold;                       //команда управления электропривода горячей воды
        zb_pack_ctrl.hot = hot;                         //команды управления электропривода холодной воды
        zb_pack_ctrl.crc = CalcCRC16( (uint8_t *)&zb_pack_ctrl, sizeof( zb_pack_ctrl ) - sizeof( zb_pack_ctrl.crc ) );
        *len = sizeof( zb_pack_ctrl );
        return (uint8_t *)&zb_pack_ctrl;
       }
    if ( type == ZB_PACK_ACK ) {
        //управление электроприводами подачи воды
        zb_pack_ack.type_pack = ZB_PACK_ACK;            //тип пакета
        zb_pack_ack.dev_numb = dev_numb;                //номер уст-ва
        zb_pack_ack.dev_addr = DevGetAddr( dev_numb );  //адрес уст-ва в сети
        *net_addr = zb_pack_ack.dev_addr;
        if ( !zb_pack_ack.dev_addr || !dev_numb )
            return NULL; //номер и адрес уст-ва не могут быть равны "0"
        zb_pack_ack.crc = CalcCRC16( (uint8_t *)&zb_pack_ack, sizeof( zb_pack_ack ) - sizeof( zb_pack_ack.crc ) );
        *len = sizeof( zb_pack_ack );
        return (uint8_t *)&zb_pack_ack;
       }
    return NULL;
 }

//*************************************************************************************************
// Вывод данных принятых в пакете
//-------------------------------------------------------------------------------------------------
// ZBTypePack id_pack - тип пакета
//*************************************************************************************************
void OutData( ZBTypePack id_pack ) {

    if ( id_pack == ZB_PACK_STATE ) {
        //вывод текущих значений уст-ва
        sprintf( str, "\r\nDevice number: ............ %05u (0x%04X)\r\n", pack_state.dev_numb, pack_state.dev_numb );
        UartSendStr( str );
        sprintf( str, "Net Address: .............. 0x%04X\r\n", pack_state.dev_addr );
        UartSendStr( str );
        //источник сброса контроллера
        sprintf( str, "Source reset: ............. %s\r\n", ResetSrcDesc( pack_state.res_src ) );
        UartSendStr( str );
        //дата/время включения контроллера
        sprintf( str, "Date/time of activation: .. %02u.%02u.%04u  %02u:%02u:%02u\r\n", 
                 pack_state.start.day, pack_state.start.month, pack_state.start.year, pack_state.start.hour, pack_state.start.min, pack_state.start.sec );
        UartSendStr( str );
        //дата/время часов удаленного контроллера
        sprintf( str, "Date/time of RTC: ......... %02u.%02u.%04u  %02u:%02u:%02u\r\n", 
                 pack_state.rtc.day, pack_state.rtc.month, pack_state.rtc.year, pack_state.rtc.hour, pack_state.rtc.min, pack_state.rtc.sec );
        UartSendStr( str );
       }
    //вывод текущих данных
    if ( id_pack == ZB_PACK_DATA )
        WaterData( (void *)&pack_data, OUT_DATA );
    //вывод журнальных данных
    if ( id_pack == ZB_PACK_WLOG )
        WaterData( (void *)&pack_data, OUT_LOG );
    //вывод текущих состояний электроприводов
    if ( id_pack == ZB_PACK_VALVE )
        ValveStatus( (VALVE_STAT_ERR *)&pack_valve.valve_stat );
    //вывод состояния датчиков утечки
    if ( id_pack == ZB_PACK_LEAKS )
        LeakData( (VALVE_STAT_ERR *)&pack_leaks );
 }

//*************************************************************************************************
// Проверка наличия уст-ва в списке доступных.
// Если уст-ва нет в списке - оно будет добавлено
// Если уст-во есть в списке - будет обновлен сетевой адрес уст-ва
//-------------------------------------------------------------------------------------------------
// uint16_t numb_dev - логический номер уст-ва
// uint16_t addr_dev - сетевой адрес уст-ва
// return = SUCCESS  - уст-во добавлено или обновлен адрес
//        = ERROR    - уст-во добавить не удалось, нет места
//*************************************************************************************************
static ErrorStatus CheckDevList( uint16_t numb_dev, uint16_t addr_dev ) {

    uint8_t i;
    
    for ( i = 0; i < DEV_LIST_MAX; i++ ) {
        if ( dev_list[i].numb_dev == numb_dev ) {
            //уст-во найдено, обновим сетевой адрес
            dev_list[i].addr_dev = addr_dev;
            dev_list[i].last_upd = 0;
            return SUCCESS;
           }
       }
    //уст-во в списке не найдено, определи место для добавления
    for ( i = 0; i < DEV_LIST_MAX; i++ ) {
        if ( dev_list[i].numb_dev )
            continue;
        //добавляем уст-во в список
        dev_list[i].numb_dev = numb_dev;
        dev_list[i].addr_dev = addr_dev;
        dev_list[i].last_upd = 0;
        return SUCCESS;
       }
    return ERROR;
 }

//*************************************************************************************************
// Функция возращает сетевой адрес по логическому номеру уст-ва 
//-------------------------------------------------------------------------------------------------
// uint16_t numb_dev - логический номер уст-ва
// return = 0        - уст-ва нет в списке
//        > 0        - сетевой адрес уст-ва
//*************************************************************************************************
static uint16_t DevGetAddr( uint16_t numb_dev ) {

    uint8_t i;
    
    if ( !numb_dev )
        return 0;
    for ( i = 0; i < DEV_LIST_MAX; i++ ) {
        //поиск уст-ва по номеру в списке
        if ( dev_list[i].numb_dev == numb_dev ) 
            return dev_list[i].addr_dev;
       }
    return 0;
 }

//*************************************************************************************************
// Обновление времени последнего ответа модуля
// Вызов выполняется из HAL_RTCEx_RTCEventCallback()
//*************************************************************************************************
void DevListUpd( void ) {

    uint8_t i;
    
    for ( i = 0; i < DEV_LIST_MAX && dev_list[i].numb_dev; i++ ) {
        dev_list[i].last_upd++;
        if ( dev_list[i].last_upd > MAX_TIME_UPDATE ) {
            //при превышении времени последнего обновления - удалим уст-во
            dev_list[i].numb_dev = 0;
            dev_list[i].addr_dev = 0;
            dev_list[i].last_upd = 0;
           }
       }
 }

//*************************************************************************************************
// Обнуление списка уст-в
//*************************************************************************************************
void DevListClr( void ) {

    uint8_t i;
    
    for ( i = 0; i < DEV_LIST_MAX; i++ ) {
        dev_list[i].numb_dev = 0;
        dev_list[i].addr_dev = 0;
        dev_list[i].last_upd = 0;
       }
 }

//*************************************************************************************************
// Вывод списка доступных уст-в
//*************************************************************************************************
void DeviceList( void ) {

    uint8_t i;
    char *ptr;
    
    UartSendStr( "Device list ...\r\n" );
    UartSendStr( (char *)msg_str_delim );
    for ( i = 0; i < DEV_LIST_MAX; i++ ) {
        if ( !dev_list[i].numb_dev )
            continue;
        ptr = str;
        ptr += sprintf( ptr, "Device: %05u (0x%04X)  ", dev_list[i].numb_dev, dev_list[i].numb_dev );
        ptr += sprintf( ptr, "NetAddrss: 0x%04X  ", dev_list[i].addr_dev );
        ptr += sprintf( ptr, "Last update: %u (sec)\r\n", dev_list[i].last_upd );
        UartSendStr( str );
       }
    UartSendStr( (char *)msg_str_delim );
 }
