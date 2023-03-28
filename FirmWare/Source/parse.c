
//*************************************************************************************************
//
// Разбор параметров командной строки
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#include "parse.h"

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static uint8_t cnt_par = 0;
static char param_list[MAX_CNT_PARAM][MAX_LEN_PARAM];

//*************************************************************************************************
// Разбор параметров команды. Если параметров указано больше MAX_CNT_PARAM, 
// лишниние параметры игнорируется
//-------------------------------------------------------------------------------------------------
// char *src - строка с параметрами
// return    - количество параметров, в т.ч. команда
//*************************************************************************************************
uint8_t ParseCommand( char *cmnd ) {

    uint8_t i = 0;
    char *str, *token, *saveptr;
   
    cnt_par = 0;
    //уберем коды \r \n
    for ( i = 0; i < strlen( cmnd ); i++ ) {
        if ( *( cmnd + i ) < 32 )
            *( cmnd + i ) = 0x00;
       }
    //обнулим предыдущие параметры
    memset( param_list, 0x00, sizeof( param_list ) );
    //разбор параметров
    str = cmnd;
    for ( i = 0; ; i++ ) {
        token = strtok_r( str, " ", &saveptr );
        if ( token == NULL )
            break;
        str = saveptr;
        strncpy( &param_list[i][0], token, strlen( token ) );
       }
    cnt_par = i;
    return cnt_par;
 }

//*************************************************************************************************
// Возвращает кол-во параметров после разбора ParseCommand()
//-------------------------------------------------------------------------------------------------
// return - кол-во параметров
//*************************************************************************************************
uint8_t GetParamCnt( void ) {

    return cnt_par;
 }

//*************************************************************************************************
// Возвращает указатель на массив параметров
//-------------------------------------------------------------------------------------------------
// return - указатель на массив
//*************************************************************************************************
char *GetParamList( void ) {

    return &param_list[0][0];
 }

//*************************************************************************************************
// Возвращает указатель на значение параметра по индексу
//-------------------------------------------------------------------------------------------------
// uint8_t index - индекс параметра
// return char * - указатель на значение параметра
//*************************************************************************************************
char *GetParamVal( CmndParam index ) {

    if ( index > MAX_CNT_PARAM )
        return NULL;
    return &param_list[index][0];
 }
