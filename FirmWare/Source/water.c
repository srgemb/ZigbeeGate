
//*************************************************************************************************
//
// Вывод информации по расходу и утечки воды
//
//*************************************************************************************************

#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "cmsis_os2.h"

#include "main.h"
#include "uart.h"
#include "data.h"
#include "valve.h"
#include "water.h"
#include "xtime.h"
#include "message.h"

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static char str[80];

//*************************************************************************************************
// Вывод в консоль значений расхода воды
//-------------------------------------------------------------------------------------------------
// void *ptr    - указатель на данные
// OutType mode - тип вывода данных: текущие/журнальные
//*************************************************************************************************
void WaterData( void *ptr, OutType mode ) {

    PACK_DATA *data = (PACK_DATA *)ptr;
    
    //вывод текущих значений 
    sprintf( str, "\r\nDevice number: ................ %05u (0x%04X)\r\n", data->dev_numb, data->dev_numb );
    UartSendStr( str );
    sprintf( str, "Net Address: .................. 0x%04X\r\n", data->dev_addr );
    UartSendStr( str );
    if ( mode == OUT_LOG ) {
        sprintf( str, "Date time log: ................ %02u.%02u.%04u %02u:%02u:%02u\r\n", 
                 data->date_time.day, data->date_time.month, data->date_time.year, 
                 data->date_time.hour, data->date_time.min, data->date_time.sec );
        UartSendStr( str );
        sprintf( str, "Data type: .................... %s\r\n", data->type_event == EVENT_DATA ? "Event" : "ALARM" );
        UartSendStr( str );
       }
    sprintf( str, "Cold water meter values: ...... %u.%03u\r\n", data->count_cold/1000, data->count_cold%1000 );
    UartSendStr( str );
    sprintf( str, "Hot water meter values: ....... %u.%03u\r\n", data->count_hot/1000, data->count_hot%1000 );
    UartSendStr( str );
    sprintf( str, "Drinking water meter values: .. %u.%03u\r\n", data->count_filter/1000, data->count_filter%1000 );
    UartSendStr( str );
    sprintf( str, "Cold water pressure: .......... %u.%u atm\r\n", data->pressr_cold/100, data->pressr_hot%100 );
    UartSendStr( str );
    sprintf( str, "Hot water pressure: ........... %u.%u atm\r\n", data->pressr_hot/100, data->pressr_hot%100 );
    UartSendStr( str );
    sprintf( str, "Leakage sensor power check: ... %s\r\n", data->dc12_chk == DC12V_OK ? "OK" : "ALARM" );
    UartSendStr( str );
    sprintf( str, "Leak sensor status #1: ........ %s\r\n", data->leak1 == LEAK_NO ? "OK" : "WATER LEAK" );
    UartSendStr( str );
    sprintf( str, "Leak sensor status #2: ........ %s\r\n", data->leak1 == LEAK_NO ? "OK" : "WATER LEAK" );
    UartSendStr( str );
 }

//*************************************************************************************************
// Вывод состояния датчиков утечки
//-------------------------------------------------------------------------------------------------
// void *ptr - указатель на данные
//*************************************************************************************************
void LeakData( void *ptr ) {

    PACK_LEAKS *data = (PACK_LEAKS *)ptr;
    
    //вывод текущих значений 
    sprintf( str, "\r\nDevice number: ................ %05u (0x%04X)\r\n", data->dev_numb, data->dev_numb );
    UartSendStr( str );
    sprintf( str, "Net Address: .................. 0x%04X\r\n", data->dev_addr );
    UartSendStr( str );
    sprintf( str, "Leakage sensor power check: ... %s\r\n", data->dc12_chk == DC12V_OK ? "OK" : "ALARM" );
    UartSendStr( str );
    sprintf( str, "Leak sensor status #1: ........ %s\r\n", data->leak1 == LEAK_NO ? "OK" : "WATER LEAK" );
    UartSendStr( str );
    sprintf( str, "Leak sensor status #2: ........ %s\r\n", data->leak1 == LEAK_NO ? "OK" : "WATER LEAK" );
    UartSendStr( str );
 }
