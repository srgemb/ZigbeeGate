                                 
//*************************************************************************************************
//
// Общие сообщения
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "cmsis_os2.h"

#include "message.h"

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
const char msg_crlr[]       = "\r\n";
const char msg_prompt[]     = "\r\n>";
const char msg_ok[]         = "OK\r\n";
const char msg_save[]       = "\r\nSaving parameters: ";
const char msg_no_command[] = "\r\nUnknown command.";
const char msg_err_param[]  = "\r\nInvalid parameters.\r\n\r\n";
const char msg_err_dev[]    = "\r\nDevice not found.\r\n\r\n";
const char msg_zb_read[]    = "ZB: read config ...";
const char msg_zb_save[]    = "ZB: save config ...";
const char msg_send_res[]   = "\r\nTransmission result: ";
const char msg_str_delim[]  = "----------------------------------------------------\r\n";


//*************************************************************************************************
// Дополняет строку справа знаками '.' до значения указанного в aligment
//-------------------------------------------------------------------------------------------------
// char *src        - указатель на исходную строку
// uint8_t aligment - максимальная позиция справа для выравнивания
// uint8_t prev_len - длина предыдущей строки (для добавления новой строки к уже имеющейся строке)
// return           - кол-во добавленных символов в т.ч. доп. пробел
//*************************************************************************************************
uint8_t AddDot( char *src, uint8_t aligment, uint8_t prev_len ) {

    uint8_t add_ind, len, add;

    strcat( src, " " );
    if ( strlen( src ) >= aligment && !prev_len )
        return 0;
    len = strlen( src );
    //кол-во символов которые будут добавлены
    add = aligment - ( len - prev_len );
    for ( add_ind = 0; add_ind < add; add_ind++, len++ )
        *(src + len) = '.';
    *(src + len) = ' ';
    return add + 2;
 }
