
//*************************************************************************************************
//
// Обработка команд полученных по UART
//
//*************************************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "cmsis_os.h"

#include "config.h"
#include "command.h"
#include "events.h"
#include "uart.h"
#include "xtime.h"
#include "zigbee.h"
#include "parse.h"
#include "data.h"
#include "message.h"
#include "version.h"

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern CONFIG config;

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
osEventFlagsId_t cmnd_event = NULL;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
//структура хранения и выполнения команд
typedef struct {
    char    name_cmd[20];                              //имя команды
    void    (*func)( uint8_t cnt_par, char *param );   //указатель на функцию выполнения
} CMD;

//расшифровка статуса задач
//#ifdef DEBUG_TARGET
static char * const state_name[] = {
    "Inactive",
    "Ready",
    "Running",
    "Blocked",
    "Terminated",
    "Error"
 };
//#endif

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t task_attr = {
    .name = "Command", 
    .stack_size = 832,
    .priority = osPriorityNormal
 };

static const osEventFlagsAttr_t evn_attr = { .name = "CmndEvents" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void TaskCommand( void *argument );
static void ExecCommand( char *buff );
//static void WaterLog( uint8_t cnt_view );
static ErrorStatus StrHexToBin( char *str, uint8_t *hex, uint8_t size );
static ErrorStatus HexToBin( char *ptr, uint8_t *bin );

//#ifdef DEBUG_TARGET
static char *TaskStateDesc( osThreadState_t state );
//#endif
static char *VersionRtos( uint32_t version, char *str );

static void CmndDate( uint8_t cnt_par, char *param );
static void CmndTime( uint8_t cnt_par, char *param );
static void CmndDateTime( uint8_t cnt_par, char *param );
static void CmndHelp( uint8_t cnt_par, char *param );
static void CmndWater( uint8_t cnt_par, char *param );
static void CmndWtLog( uint8_t cnt_par, char *param );
static void CmndValve( uint8_t cnt_par, char *param );
static void CmndStat( uint8_t cnt_par, char *param );
//static void CmndLog( uint8_t cnt_par, char *param );
static void CmndConfig( uint8_t cnt_par, char *param );
static void CmndZigBee( uint8_t cnt_par, char *param );
static void CmndZbDev( uint8_t cnt_par, char *param );
static void CmndVersion( uint8_t cnt_par, char *param );
//#ifdef DEBUG_TARGET
static void CmndTask( uint8_t cnt_par, char *param );
static void CmndFlash( uint8_t cnt_par, char *param );
static void CmndReset( uint8_t cnt_par, char *param );
//#endif

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static char buffer[128];

static const char *help = {
    "date [dd.mm.yy]                  - Display/set date.\r\n"
    "time [hh:mm[:ss]]                - Display/set time.\r\n"
    "dtime [sync]                     - Display date and time.\r\n"
    "valve num_dev                    - Valve status\r\n"
    "valve num_dev [cold/hot opn/cls] - Drive control\r\n"
    "water num_dev [N]                - Water flow indication\r\n"
    "wtlog num_dev num_logs           - Log data\r\n"
    "dev [N]                          - Device list [stat]\r\n"
    "\r\n"
    "stat                             - Statistics.\r\n"
    "task                             - List task statuses, time statistics.\r\n"
    "flash                            - FLASH config HEX dump.\r\n"
    "zb [res/init/net/save/cfg/chk]   - ZigBee module control.\r\n"
    "config                           - Display of configuration parameters.\r\n"
    "config save                      - Save configuration settings.\r\n"
    "config uart xxxxx                - Setting the speed baud (600 - 115200).\r\n"
    "config panid 0x0000 - 0xFFFE     - Network PANID (HEX format without 0x).\r\n"
    "config netgrp 1-99               - Network group number.\r\n"
    "config netkey XXXX....           - Network key (HEX format without 0x).\r\n"
    "config devnumb 0x0001 - 0xFFFF   - Device number on the network (HEX format without 0x).\r\n"
    "config gate 0x0000- 0xFFF8       - Gateway address (HEX format without 0x).\r\n"
    "version                          - Displays the version number and date.\r\n"
    #ifdef DEBUG_TARGET              
    "reset                            - Reset controller.\r\n"
    #endif                           
    "?                                - Help.\r\n"
  };

//Перечень доступных команд
//*********************************************
//     Имя команды      Функция вызова
//*********************************************
static const CMD cmd[] = {
    { "time",           CmndTime },
    { "date",           CmndDate },
    { "dtime",          CmndDateTime },
    { "water",          CmndWater },
    { "wtlog",          CmndWtLog },
    { "valve",          CmndValve },
    { "stat",           CmndStat },
    { "config",         CmndConfig },
    { "zb",             CmndZigBee },
    { "dev",            CmndZbDev },
    { "version",        CmndVersion },
    { "task",           CmndTask },
    { "flash",          CmndFlash },
    { "reset",          CmndReset },
    { "?",              CmndHelp }
 };

//*************************************************************************************************
// Инициализация задачи
//*************************************************************************************************
void CommandInit( void ) {

    //очередь событий
    cmnd_event = osEventFlagsNew( &evn_attr );
    //создаем задачу обработки команд
    osThreadNew( TaskCommand, NULL, &task_attr );
}

//*************************************************************************************************
// Задача обработки очереди сообщений на выполнение команд полученных по UART1
//*************************************************************************************************
static void TaskCommand( void *argument ) {

    int32_t event;

    for ( ;; ) {
        event = osEventFlagsWait( cmnd_event, EVN_CMND_MASK, osFlagsWaitAny, osWaitForever );
        if ( event & EVN_CMND_EXEC ) {
            //выполнение команды
            ExecCommand( UartBuffer() );
            //разрешаем прием по UART следующей команды
            osEventFlagsSet( uart_event, EVN_UART_START );
           }
        if ( event & EVN_CMND_PROMPT )
            UartSendStr( (char *)msg_prompt );
       }
 }

//*************************************************************************************************
// Обработка команд полученных по UART
//-------------------------------------------------------------------------------------------------
// char *buff - указатель на буфер с командой
//*************************************************************************************************
static void ExecCommand( char *buff ) {

    uint8_t i, cnt_par;

    //разбор параметров команды
    cnt_par = ParseCommand( buff );
    //проверка и выполнение команды
    for ( i = 0; i < SIZE_ARRAY( cmd ); i++ ) {
        if ( strcasecmp( (const char *)&cmd[i].name_cmd, GetParamVal( IND_PAR_CMND ) ) )
            continue;
        UartSendStr( (char *)msg_crlr );
        cmd[i].func( cnt_par, GetParamList() ); //выполнение команды
        UartSendStr( (char *)msg_prompt );
        break;
       }
    if ( i == SIZE_ARRAY( cmd ) ) {
        UartSendStr( (char *)msg_no_command );
        UartSendStr( (char *)msg_prompt );
       }
 }

//*********************************************************************************************
// Вывод статистики по задачам
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров включая команду
// char *param     - указатель на список параметров
//*********************************************************************************************
//#ifdef DEBUG_TARGET
static void CmndTask( uint8_t cnt_par, char *param ) {

    uint8_t i;
    const char *name;
    osThreadState_t state;
    osPriority_t priority;
    osThreadId_t th_id[20];
    uint32_t cnt_task, stack_space, stack_size; 
    
    //вывод шапки параметров
    UartSendStr( "\r\n   Name thread     Priority  State      Stack Unused\r\n" );
    UartSendStr( (char *)msg_str_delim );
    //заполним весь массив th_id значением NULL
    memset( th_id, 0x00, sizeof( th_id ) );
    cnt_task = osThreadGetCount();
    cnt_task = osThreadEnumerate( &th_id[0], sizeof( th_id )/sizeof( th_id[0] ) );
    for ( i = 0; i < cnt_task; i++ ) {
        state = osThreadGetState( th_id[i] );
        priority = osThreadGetPriority( th_id[i] );
        //https://github.com/ARM-software/CMSIS-FreeRTOS/issues/14
        //https://github.com/ARM-software/CMSIS-FreeRTOS/blob/dd7793adcbea0c3c0f3524f86b031ab88b9e2193/DoxyGen/General/src/cmsis_freertos.txt#L299
        //osThreadGetStackSize is not implemented.
        stack_size = osThreadGetStackSize( th_id[i] );
        stack_space = osThreadGetStackSpace( th_id[i] );
        name = osThreadGetName( th_id[i] );
        if ( name != NULL && strlen( name ) )
            sprintf( buffer, "%2u %-16s    %2u    %-10s %5u %5u\r\n", i + 1, name, priority, TaskStateDesc( state ), stack_size, stack_space );
        else sprintf( buffer, "%2u ID = %-11u    %2u    %-10s %5u %5u\r\n", i + 1, (uint32_t)th_id[i], priority, TaskStateDesc( state ), stack_size, stack_space );
        UartSendStr( buffer );
       }
    UartSendStr( (char *)msg_str_delim );
    sprintf( buffer, "Free heap size: %u of %u bytes.\r\n", xPortGetFreeHeapSize(), configTOTAL_HEAP_SIZE );
    UartSendStr( buffer );
 }
//#endif

//*************************************************************************************************
// Вывод параметров настроек контроллера, установка параметров
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndConfig( uint8_t cnt_par, char *param ) {

    char *ptr;
    uint8_t error, ind, bin[sizeof( config.net_key )];
    UARTSpeed uart_speed;
    bool change = false;
    union {
        float    val_float;
        uint8_t  val_uint8;
        uint16_t val_uint16;
        uint32_t val_uint32;
       } value;

    //установка скорости UART порта отладки
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "uart" ) ) {
        value.val_uint32 = atol( GetParamVal( IND_PARAM2 ) );
        //проверка на допустимые значения скорости UART порта
        if ( CheckBaudRate( value.val_uint32, &uart_speed ) == SUCCESS ) {
            change = true;
            config.debug_speed = uart_speed;
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //установка адреса сети в канале
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "panid" ) ) {
        if ( StrHexToBin( GetParamVal( IND_PARAM2 ), (uint8_t *)&value.val_uint16, sizeof( value.val_uint16 ) ) == SUCCESS ) {
            if ( value.val_uint16 <= MAX_NETWORK_PANID ) {
                change = true;
                memcpy( (uint8_t *)&config.net_pan_id, (uint8_t *)&value.val_uint16, sizeof( config.net_pan_id ) );
                config.net_pan_id = __REVSH( config.net_pan_id );
               }
            else UartSendStr( (char *)msg_err_param );
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //установка ключа сети
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "netkey" ) ) {
        if ( StrHexToBin( GetParamVal( IND_PARAM2 ), (uint8_t *)&bin, sizeof( bin ) ) == SUCCESS ) {
            change = true;
            memcpy( config.net_key, bin, sizeof( config.net_key ) );
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //номер устройства в сети
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "devnumb" ) ) {
        if ( StrHexToBin( GetParamVal( IND_PARAM2 ), (uint8_t *)&value.val_uint16, sizeof( value.val_uint16 ) ) == SUCCESS ) {
            if ( value.val_uint16 && value.val_uint16 <= MAX_DEVICE_NUMB ) {
                change = true;
                memcpy( (uint8_t *)&config.dev_numb, (uint8_t *)&value.val_uint16, sizeof( config.dev_numb ) );
                config.dev_numb = __REVSH( config.dev_numb );
               }
            else UartSendStr( (char *)msg_err_param );
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //установка номера группы
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "netgrp" ) ) {
        value.val_uint8 = atoi( GetParamVal( IND_PARAM2 ) );
        if ( value.val_uint8 <= MAX_NETWORK_GROUP ) {
            change = true;
            config.net_group = value.val_uint8;
           }
       }
    //установка адреса шлюза с сети
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "gate" ) ) {
        if ( StrHexToBin( GetParamVal( IND_PARAM2 ), (uint8_t *)&value.val_uint16, sizeof( value.val_uint16 ) ) == SUCCESS ) {
            if ( value.val_uint16 <= MAX_NETWORK_ADDR ) {
                change = true;
                memcpy( (uint8_t *)&config.addr_gate, (uint8_t *)&value.val_uint16, sizeof( config.addr_gate ) );
                config.addr_gate = __REVSH( config.addr_gate );
               }
            else UartSendStr( (char *)msg_err_param );
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //сохранение параметров
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "save" ) ) {
        UartSendStr( (char *)msg_save );
        error = ConfigSave();
        if ( error != HAL_OK ) 
            UartSendStr( ConfigError( error ) );
        else UartSendStr( (char *)msg_ok );
        return;
       }

    sprintf( buffer, "Reading parameters from flash memory: %s\r\n", FlashReadStat() );
    UartSendStr( buffer );
    UartSendStr( (char *)msg_str_delim );
    //вывод значений параметров
    sprintf( buffer, "UART speed: ......................... %u\r\n", UartGetSpeed( (UARTSpeed)config.debug_speed ) );
    UartSendStr( buffer );
    UartSendStr( (char *)msg_str_delim );
    sprintf( buffer, "Network PANID: ...................... 0x%04X\r\n", config.net_pan_id );
    UartSendStr( buffer );
    sprintf( buffer, "Network group number: ............... %u\r\n", config.net_group );
    UartSendStr( buffer );
    ptr = buffer;
    ptr += sprintf( ptr, "Network key: ........................ " );
    for ( ind = 0; ind < sizeof( config.net_key ); ind++ )
        ptr += sprintf( ptr, "%02X", config.net_key[ind] );
    ptr += sprintf( ptr, "\r\n" );
    UartSendStr( buffer );
    sprintf( buffer, "Device number on the network: ....... 0x%04X\r\n", config.dev_numb );
    UartSendStr( buffer );
    sprintf( buffer, "Gateway address: .................... 0x%04X\r\n", config.addr_gate );
    UartSendStr( buffer );
    if ( change == true ) {
        //сохранение параметров
        UartSendStr( (char *)msg_save );
        error = ConfigSave();
        if ( error != HAL_OK ) 
            UartSendStr( ConfigError( error ) );
        else UartSendStr( (char *)msg_ok );
       }
 }

//*************************************************************************************************
// Запрос состояния датчиков утечки и расхода воды
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndWater( uint8_t cnt_par, char *param ) {

    uint16_t dev_numb, net_addr;
    uint8_t *data, len;
    ZBErrorState state;

    if ( cnt_par == 2 ) {
        //вывод состояния давления и расхода воды
        dev_numb = atoi( GetParamVal( IND_PARAM1 ) );
        data = CreatePack( ZB_PACK_REQ_DATA, dev_numb, &net_addr, 0, VALVE_CTRL_NOTHING, VALVE_CTRL_NOTHING, &len );
        if ( data == NULL ) {
            UartSendStr( (char *)msg_err_dev );
            return;
           }
        state = ZBSendPack1( data, len, net_addr, TIME_WAIT_ANSWER );
        sprintf( buffer, "%s %s\r\n", (char *)msg_send_res, ZBErrDesc( state ) );
        UartSendStr( buffer );
        return;
       }
    UartSendStr( (char *)msg_err_param );
 }

//*************************************************************************************************
// 
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndWtLog( uint8_t cnt_par, char *param ) {

    uint16_t dev_numb, net_addr;
    uint8_t *data, len, dev_log;
    ZBErrorState state;

    if ( cnt_par == 3 ) {
        //запрос данных из журнала
        dev_numb = atoi( GetParamVal( IND_PARAM1 ) );
        dev_log = atoi( GetParamVal( IND_PARAM2 ) );
        data = CreatePack( ZB_PACK_REQ_DATA, dev_numb, &net_addr, dev_log, VALVE_CTRL_NOTHING, VALVE_CTRL_NOTHING, &len );
        if ( data == NULL ) {
            UartSendStr( (char *)msg_err_dev );
            return;
           }
        state = ZBSendPack1( data, len, net_addr, TIME_WAIT_ANSWER );
        sprintf( buffer, "%s %s\r\n", (char *)msg_send_res, ZBErrDesc( state ) );
        UartSendStr( buffer );
        return;
       }
    UartSendStr( (char *)msg_err_param );
}

//*************************************************************************************************
// Вывод состояния/управление электроприводами
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndValve( uint8_t cnt_par, char *param ) {

    int opn, cls;
    uint16_t dev_numb, net_addr;
    uint8_t *data, len;
    ZBErrorState state;
    ValveCtrlMode cold = VALVE_CTRL_NOTHING;
    ValveCtrlMode hot  = VALVE_CTRL_NOTHING;
    
    if ( cnt_par == 4 && !strcasecmp( GetParamVal( IND_PARAM2 ), "cold" ) ) {
        //управление электроприводом холодной воды
        dev_numb = atoi( GetParamVal( IND_PARAM1 ) );
        opn = strcasecmp( GetParamVal( IND_PARAM3 ), "opn" );
        cls = strcasecmp( GetParamVal( IND_PARAM3 ), "cls" );
        if ( !opn && cls )
            cold = VALVE_CTRL_OPEN;
        if ( opn && !cls )
            cold = VALVE_CTRL_CLOSE;
        data = CreatePack( ZB_PACK_CTRL_VALVE, dev_numb, &net_addr, 0, cold, hot, &len );
        if ( data == NULL ) {
            UartSendStr( (char *)msg_err_dev );
            return;
           }
        state = ZBSendPack1( data, len, net_addr, TIME_NO_WAIT );
        sprintf( buffer, "%s %s\r\n", (char *)msg_send_res, ZBErrDesc( state ) );
        UartSendStr( buffer );
        return;
       }
    if ( cnt_par == 4 && !strcasecmp( GetParamVal( IND_PARAM2 ), "hot" ) ) {
        //управление электроприводом горячей воды
        dev_numb = atoi( GetParamVal( IND_PARAM1 ) );
        opn = strcasecmp( GetParamVal( IND_PARAM3 ), "opn" );
        cls = strcasecmp( GetParamVal( IND_PARAM3 ), "cls" );
        if ( !opn && cls )
            hot = VALVE_CTRL_OPEN;
        if ( opn && !cls )
            hot = VALVE_CTRL_CLOSE;
        data = CreatePack( ZB_PACK_CTRL_VALVE, dev_numb, &net_addr, 0, cold, hot, &len );
        if ( data == NULL ) {
            UartSendStr( (char *)msg_err_dev );
            return;
           }
        state = ZBSendPack1( data, len, net_addr, TIME_NO_WAIT );
        sprintf( buffer, "%s %s\r\n", (char *)msg_send_res, ZBErrDesc( state ) );
        UartSendStr( buffer );
        return;
       }
    if ( cnt_par == 2 ) {
        //вывод информации о состоянии электроприводов 
        dev_numb = atoi( GetParamVal( IND_PARAM1 ) );
        data = CreatePack( ZB_PACK_REQ_VALVE, dev_numb, &net_addr, 0, cold, hot, &len );
        if ( data == NULL ) {
            UartSendStr( (char *)msg_err_dev );
            return;
           }
        state = ZBSendPack1( data, len, net_addr, TIME_WAIT_ANSWER );
        sprintf( buffer, "%s %s\r\n", (char *)msg_send_res, ZBErrDesc( state ) );
        UartSendStr( buffer );
        return;
       }
    UartSendStr( (char *)msg_err_param );
 }

//*************************************************************************************************
// Вывод/установка текущего значения дата/время
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndDate( uint8_t cnt_par, char *param ) {

    if ( cnt_par == 2 ) {
        if ( DateSet( GetParamVal( IND_PARAM1 ) ) == ERROR ) {
            UartSendStr( (char *)msg_err_param );
            return;
           }
       }
    DateTimeStr( buffer, MASK_DATE );
    UartSendStr( buffer );
}

//*************************************************************************************************
// Вывод/установка текущего значения времени
// Установка времени, входной формат HH:MI:SS
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndTime( uint8_t cnt_par, char *param ) {

    if ( cnt_par == 2 ) {
        if ( TimeSet( GetParamVal( IND_PARAM1 ) ) == ERROR ) {
            UartSendStr( (char *)msg_err_param );
            return;
           }
       }
    DateTimeStr( buffer, MASK_TIME );
    UartSendStr( buffer );
 }

//*************************************************************************************************
// Вывод текущего значения дата/время, синхронизация календаря на всех уст-вах
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndDateTime( uint8_t cnt_par, char *param ) {

    uint16_t net_addr;
    uint8_t *data, len;
    ZBErrorState state;

    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "sync" ) ) {
        //синхронизация даты и времени
        data = CreatePack( ZB_PACK_SYNC_DTIME, 0, &net_addr, 0, VALVE_CTRL_NOTHING, VALVE_CTRL_NOTHING, &len );
        if ( data == NULL )
            return;
        state = ZBSendPack( data, len );
        sprintf( buffer, "%s %s\r\n", (char *)msg_send_res, ZBErrDesc( state ) );
        UartSendStr( buffer );
        return;
       }
    //только вывод текущих даты/время
    DateTimeStr( buffer, MASK_DATE_TIME );
    UartSendStr( buffer );
 }

//*************************************************************************************************
// Вывод списка уст-в зарегистрированных в сети или вывод состояния указанного уст-ва
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndZbDev( uint8_t cnt_par, char *param ) {

    uint16_t dev_numb, net_addr;
    uint8_t *data, len;
    ZBErrorState state;

    if ( cnt_par == 1 ) {
        //вывод списка уст-в
        DeviceList();
        return;
       }
    if ( cnt_par == 2 ) {
        //вывод состояния уст-ва
        dev_numb = atoi( GetParamVal( IND_PARAM1 ) );
        data = CreatePack( ZB_PACK_REQ_STATE, dev_numb, &net_addr, 0, VALVE_CTRL_NOTHING, VALVE_CTRL_NOTHING, &len );
        if ( data == NULL ) {
            sprintf( buffer, "Pointer is NULL.\r\n" );
            UartSendStr( buffer );
            return;
           }
        state = ZBSendPack1( data, len, net_addr, TIME_WAIT_ANSWER );
        sprintf( buffer, "%s %s\r\n", (char *)msg_send_res, ZBErrDesc( state ) );
        UartSendStr( buffer );
        return;
       }
    UartSendStr( (char *)msg_err_param );
 }

//*************************************************************************************************
// Вывод статистики обмена данными ZIGBEE
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndStat( uint8_t cnt_par, char *param ) {

    char str[120];
    uint8_t i, cnt;

    //источник перезапуска контроллера
    sprintf( str, "Source reset: %s\r\n", ResetSrcDesc( ResetSrc() ) );
    UartSendStr( str );
    //статистика протокола ZigBee
    UartSendStr( "\r\nZigBee statistics ...\r\n" );
    UartSendStr( (char *)msg_str_delim );
    cnt = ZBErrCnt( ZB_ERROR_OK );
    for ( i = 0; i < cnt; i++ ) {
        sprintf( buffer, "%s\r\n", ZBErrCntDesc( (ZBErrorState)i, str ) );
        UartSendStr( buffer );
       }
 }

//*************************************************************************************************
// Вывод дампа FLASH памяти (хранение параметров)
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
//#ifdef DEBUG_TARGET
static void CmndFlash( uint8_t cnt_par, char *param ) {
    
    uint8_t dw, dwi, dw_cnt, data_hex[16];
    uint32_t *source_addr, *dest_addr;

    source_addr = (uint32_t *)FLASH_DATA_ADDRESS;
    dw_cnt = sizeof( FLASH_DATA )/sizeof( uint32_t );
    for ( dw = 0; dw < dw_cnt; dw += 4 ) {
        dest_addr = (uint32_t *)&data_hex;
        //читаем значение только как WORD (по 4 байта)
        for ( dwi = 0; dwi < 4; dwi++, dest_addr++, source_addr++ )
            *dest_addr = *(__IO uint32_t *)source_addr;
        //вывод дампа
        DataHexDump( data_hex, HEX_32BIT_ADDR, (uint32_t)source_addr - 16, buffer );
        UartSendStr( buffer );
       }
    UartSendStr( (char *)msg_crlr );
 }
//#endif

//*************************************************************************************************
// Управление радио модулем ZigBee
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndZigBee( uint8_t cnt_par, char *param ) {

    static ZBErrorState state;
    
    if ( cnt_par == 1 ) {
        UartSendStr( (char *)msg_err_param );
        return;
       }
    //аппаратный перезапуск модуля
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "res" ) )
        state = ZBControl( ZB_CMD_DEV_RESET );
    //программный перезапуск модуля
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "init" ) )
        state = ZBControl( ZB_CMD_DEV_INIT );
    //переподключение к сети
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "net" ) )
        state = ZBControl( ZB_CMD_NET_RESTART );
    //запись конфигурации в радио модуль
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "save" ) )
        state = ZBControl( ZB_CMD_SAVE_CONFIG );
    //проверка конфигурации радио модуля
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "chk" ) )
        ZBCheckConfig();
    //чтение и вывод конфигурации
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "cfg" ) ) {
        state = ZBControl( ZB_CMD_READ_CONFIG );
        if ( state == ZB_ERROR_OK )
            ZBConfig(); //вывод параметров конфигурации
       }
    sprintf( buffer, "%s %s\r\n", (char *)msg_send_res, ZBErrDesc( state ) );
    UartSendStr( buffer );
 }

//*************************************************************************************************
// Перезапуск контроллера
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
//#ifdef DEBUG_TARGET
static void CmndReset( uint8_t cnt_par, char *param ) {

    NVIC_SystemReset();
}
//#endif

//*************************************************************************************************
// Вывод номера и даты версий: FirmWare, RTOS, HAL
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndVersion( uint8_t cnt_par, char *param ) {

    char val[32];
    osVersion_t osv;
    char infobuf[40];
    osStatus_t status;

    sprintf( buffer, "FirmWare version: .... %s\r\n", FWVersion( GetFwVersion() ) );
    UartSendStr( buffer );
    sprintf( buffer, "FirmWare date build: . %s\r\n", FWDate( GetFwDate() ) );
    UartSendStr( buffer );
    sprintf( buffer, "FirmWare time build: . %s\r\n", FWTime( GetFwTime() ) );
    UartSendStr( buffer );
    UartSendStr( (char *)msg_crlr );
    sprintf( buffer, "The HAL revision: .... %s\r\n", FWVersion( HAL_GetHalVersion() ) );
    UartSendStr( buffer );
    status = osKernelGetInfo( &osv, infobuf, sizeof( infobuf ) );
    if ( status == osOK ) {
        sprintf( buffer, "Kernel Information: .. %s\r\n", infobuf );
        UartSendStr( buffer );
        sprintf( buffer, "Kernel Version: ...... %s\r\n", VersionRtos( osv.kernel, val ) );
        UartSendStr( buffer );
        sprintf( buffer, "Kernel API Version: .. %s\r\n", VersionRtos( osv.api, val ) );
        UartSendStr( buffer );
       }
}

//*************************************************************************************************
// Вывод подсказки по доступным командам
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndHelp( uint8_t cnt_par, char *param ) {

    UartSendStr( (char *)help );
 }

//*************************************************************************************************
// Расшифровка версии RTOS в текстовый буфер
//-------------------------------------------------------------------------------------------------
// uint32_t version - значение версии в формате: mmnnnrrrr dec
// char *str        - указатель на строку для размещения результата
// return           - указатель на строку с расшифровкой в формате major.minor.rev
//*************************************************************************************************
static char *VersionRtos( uint32_t version, char *str ) {

    uint32_t major, minor, rev;
    
    rev = version%10000;
    version /= 10000;
    minor = version%1000;
    major = version/1000;
    sprintf( str, "%u.%u.%u", major, minor, rev );
    return str; 
 }

//*************************************************************************************************
// Возвращает расшифровку статуса задачи
//-------------------------------------------------------------------------------------------------
// osThreadState_t state - код статуса
//*************************************************************************************************
//#ifdef DEBUG_TARGET
static char *TaskStateDesc( osThreadState_t state ) {

    if ( state == osThreadInactive )
        return state_name[0];
    if ( state == osThreadReady )
        return state_name[1];
    if ( state == osThreadRunning )
        return state_name[2];
    if ( state == osThreadBlocked )
        return state_name[3];
    if ( state == osThreadTerminated )
        return state_name[4];
    if ( state == osThreadError )
        return state_name[5];
    return NULL;
 }
//#endif

//*************************************************************************************************
// Вывод одной строки дампа данных в HEX формате 
// 0x0000: 00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  ................
//-------------------------------------------------------------------------------------------------
// uint8_t *data           - указатель на массив данных 
// HEX_TYPE_ADDR type_addr - разрядность выводимого адреса 16/32 бита
// uint16_t addr           - адрес для вывода 
// char *buff              - указатель на буфер для размещения результата
//*************************************************************************************************
void DataHexDump( uint8_t *data, HEX_TYPE_ADDR type_addr, uint32_t addr, char *buff ) {

    char *ptr, ch;
    uint8_t offset;

    ptr = buff;
    //вывод адреса строки
    if ( type_addr == HEX_16BIT_ADDR )
        ptr += sprintf( ptr, "0x%04X: ", addr );
    else ptr += sprintf( ptr, "0x%08X: ", addr );
    //вывод HEX данных
    for ( offset = 0; offset < 16; offset++ ) {
        ch = *( data + offset );
        //выводим HEX коды
        if ( ( offset & 0x07 ) == 7 )
            ptr += sprintf( ptr, "%02X  ", ch );
        else ptr += sprintf( ptr, "%02X ", ch );
       }
    //выводим символы
    for ( offset = 0; offset < 16; offset++ ) {
        ch = *( data + offset );
        if ( ch >= 32 && ch < 127 )
            ptr += sprintf( ptr, "%c", ch );
        else ptr += sprintf( ptr, "." );
       }
    ptr += sprintf( ptr, "\r\n" );
 }

//*************************************************************************************************
// Преобразует строку с данными в формате HEX в формат BIN
//-------------------------------------------------------------------------------------------------
// char *str        - указатель на исходную строку с HEX значениями
// uint8_t *hex     - указатель для массив для размещения результата
// uint8_t size     - размер переменной для размещения результата
// return = SUCCESS - преобразование выполнено без ошибок
//        = ERROR   - преобразование не выполнено, есть не допустимые символы
//*************************************************************************************************
static ErrorStatus StrHexToBin( char *str, uint8_t *hex, uint8_t size ) {

    uint8_t high, low, len;
    
    len = strlen( str );
    //проверка на длину строки и кратность длины строки = 2 
    if ( ( len/2 ) > size || ( len & 0x01 ) )
        return ERROR;
    while ( *str ) {
        high = low = 0;
        if ( HexToBin( str, &high ) == ERROR )
            return ERROR;
        if ( *++str ) {
            //не конец строки
            if ( HexToBin( str, &low ) == ERROR )
                return ERROR;
            str++; //переход на следующие два байта
           }
        high <<= 4;
        high |= low;
        *hex++ = high;
       }
    return SUCCESS;
 }

//*************************************************************************************************
// Преобразует символ в формате HEX в формат BIN
//-------------------------------------------------------------------------------------------------
// char *ptr        - указатель на HEX символ
// uint8_t *bin     - указатель на переменную для размещения результата
// return = SUCCESS - преобразование выполнено без ошибок
//        = ERROR   - преобразование не выполнено, есть не допустимые символы
//*************************************************************************************************
static ErrorStatus HexToBin( char *ptr, uint8_t *bin ) {

    *bin = 0;
    if ( *ptr >= '0' && *ptr <= '9' ) {
        *bin = *ptr - '0';
        return SUCCESS;
       }
    if ( *ptr >= 'a' && *ptr <= 'f' ) {
        *bin = *ptr - 87;
        return SUCCESS;
       }
    if ( *ptr >= 'A' && *ptr <= 'F' ) {
        *bin = *ptr - 55;
        return SUCCESS;
       }
    return ERROR;
 }
