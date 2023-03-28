
//*************************************************************************************************
//
// Вывод состояния электроприводов
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#include "cmsis_os2.h"

#include "led.h"
#include "main.h"
#include "uart.h"
#include "valve.h"
#include "data.h"
#include "events.h"
#include "parse.h"

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
static char str[80];

//расшифровка состояния электропривода
static char * const status_desc[] = { "UNDEF", "CLOSE", "OPEN" };

//расшифровка ошибок электропривода
static char * const error_desc[] = { "OK", "OVERLOAD", "NO POWER", "TIMEOUT" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static char *ValveStatusDesc( uint8_t value );
static char *ValveErrorDesc( uint8_t value );

//*************************************************************************************************
// Вывод расшифровки состояния электроприводов кранов подачи воды
//-------------------------------------------------------------------------------------------------
// VALVE_STAT_ERR *valve_stat - значение для расшифровки
//*************************************************************************************************
void ValveStatus( VALVE_STAT_ERR *valve_stat ) {

    sprintf( str, "\r\nCold water tap drive status: .. %s\r\n", ValveStatusDesc( (uint8_t)valve_stat->stat_valve_cold ) );
    UartSendStr( str );
    sprintf( str, "Cold water valve error: ....... %s\r\n", ValveErrorDesc( (uint8_t)valve_stat->error_valve_cold ) );
    UartSendStr( str );
    sprintf( str, "Hot water tap drive status: ... %s\r\n", ValveStatusDesc( (uint8_t)valve_stat->stat_valve_hot ) );
    UartSendStr( str );
    sprintf( str, "Hot water valve error: ........ %s\r\n", ValveErrorDesc( (uint8_t)valve_stat->error_valve_hot ) );
    UartSendStr( str );
 }
 
//*************************************************************************************************
// Возвращает расшифровку статуса электропривода
//-------------------------------------------------------------------------------------------------
// Valve valve - тип электропривода: горячая/холодная вода
// return      - указатель на строку с результатом
//*************************************************************************************************
static char *ValveStatusDesc( uint8_t value ) {

    if ( value < SIZE_ARRAY( status_desc )  )
        return status_desc[value];
    return NULL;
 }

//*************************************************************************************************
// Возвращает расшифровку ошибку электропривода
//-------------------------------------------------------------------------------------------------
// Valve valve - тип электропривода: горячая/холодная вода
// return      - указатель на строку с результатом
//*************************************************************************************************
static char *ValveErrorDesc( uint8_t value ) {

    if ( value < SIZE_ARRAY( status_desc )  )
        return error_desc[value];
    return NULL;
 }
