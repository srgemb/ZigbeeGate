
//*************************************************************************************************
//
// Управление обменом данными c ZigBee модулем
// 
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "cmsis_os2.h"
#include "freertos.h"

#include "main.h"
#include "data.h"
#include "valve.h"
#include "uart.h"
#include "crc16.h"
#include "events.h"
#include "message.h"
#include "parse.h"
#include "config.h"
#include "xtime.h"
#include "zigbee.h"

#define DEBUG_ZIGBEE            0           //вывод принятых/отправленных пакетов в HEX формате
#define DEBUG_MALLOC            0           //вывод отладочной информации для pvPortMalloc()

#define NET_STATUS_SOFT         1           //програмнное определение статуса сети

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern CONFIG config;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart3;

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
ZB_CONFIG zb_cfg;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define BUFFER_CMD              80          //размер буфера для команд управления
#define BUFF_RECV_SIZE          512         //размер буферов приема
#define BUFF_SEND_SIZE          128         //размер буферов передачи

#define ZB_SEND_DATA            0xFC        //код команды передачи данных
#define ZB_SYS_CONFIG           0xFB        //ответ чтения конфигурации

#define OFFSET_DATA_SIZE        1           //смещение для размещения размера пакета
#define ZB_ADDR_SIZE            2           //размер адреса шлюза (координатора)
#define ZB_MODE_SIZE            2           //кол-во байт определяюшие тип передачи пакета

#define TIME_DELAY_RESET        100         //задержка восстановления сигнала сброса (msec)
#define TIME_DELAY_ANSWER       100         //ожидание ответа (msec)
#define TIME_DELAY_CHECK        700         //задержка проверки включения ZigBee модуля (msec)

#define OFFSET_CFG_DATA         3           //смещения для размещения параметров
                                            //конфигурации ZigBee модуля

//Структура для команд управления ZigBee модулем
typedef struct {
    ZBCmnd       id_command;                //ID команды
    uint8_t      code_command[4];           //коды команды
    uint16_t     time_answer;               //время ожидаемого ответа (msec)
    ZBErrorState (*func_exec)( void *ptr ); //указатель на функцию исполнения команды
 } ZB_COMMAND;

//Структура стандартных ответов ZigBee модуля
typedef struct {
    ZBAnswer     id_answer;                 //ID ответа
    uint8_t      code_answer[2];            //коды ответа
 } ZB_ANSWER;

//
typedef struct {
    void        *ptr;                       //указатель на буфер
    uint16_t    len;                        //размер буфера
 } RECV_DATA;

//Расшифровка результата выполнения команд
static char * const error_descr[] = {
    "OK",                                   //ошибок нет
    "Error in command data",                //ошибка в данных команды
    "UART transmission error",              //ошибка передачи данных
    "Module no running",                    //модуль не включен
    "No network",                           //нет сети
    "Command undefined",                    //команда не определена
    "Timed out for response",               //вышло время ожидания ответа на команду
    "Data error",                           //ошибка в данных при вызове функции
    "Checksum error",                       //ошибка контрольной суммы в полученном пакете данных
    "Device number error",                  //ошибка в номере устройства (номер в настройках контроллера 
                                            //не соответствует номеру полученному в пакете данных)
    "Device address error",                 //ошибка в адресе (адрес, присвоенный при подключении к 
                                            //сети не соответствует адресу полученному в пакете)
    "Module response not identified"        //ответ модуля (тип пакета данных) не идентифицирован
 };

static char * const dev_type[] = {
    "Coordinator", "Router", "Terminal"
 };

static char * const nwk_state[] = {
    "No network", "Network exists"
 };

#if ( DEBUG_ZIGBEE == 1 ) && defined( DEBUG_TARGET )
static char * const desc_answer[] = {
    "ANS_UNDEF",
    "ANS_ERROR",
    "ANS_BLD_NET",
    "ANS_JOIN_NET",
    "ANS_NO_NET",
    "ANS_SET_CONFIG",
    "ANS_RESTART",
    "ANS_CFG_FACTORY",
    "ANS_NET_RESTART",
    "ANS_READ_CONFIG"
 };

static char * const desc_pack[] = {
    "PACK_UNDEF",
    "PACK_STATE",
    "PACK_DATA",
    "PACK_WLOG",
    "PACK_VALVE",
    "PACK_LEAKS",
    "PACK_SYNC_DTIME",
    "PACK_REQ_STATE",
    "PACK_REQ_VALVE",
    "PACK_REQ_DATA",
    "PACK_CTRL_VALVE",
    "PACK_ACK"
 };
#endif
                                                                    
static char * const txpower[] = { "-3/16/20", "-1.5/17/22", "0/19/24", "2.5/20/26", "4.5/20/27" };

//*************************************************************************************************
// Прототипы локальных функций вызываемые по ссылке
//*************************************************************************************************
static ZBErrorState SetCfg( void *ptr );
static ZBErrorState Command( void *ptr );

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static char str[100];
#if ( DEBUG_ZIGBEE == 1 ) && defined( DEBUG_TARGET )
static char str2[180];
#endif

static ZBAnswer chk_answ;
static ZBTypePack chk_pack;
static DATA_ACK data_ack;
static bool time_out = false;
static osTimerId_t timer_start;
static osMutexId_t zb_mutex = NULL;
static osMessageQueueId_t msg_recv = NULL;
static osSemaphoreId_t sem_send = NULL, sem_ans = NULL;
static osEventFlagsId_t zb_init = NULL, zb_ctrl = NULL;

static uint16_t recv_ind = 0;
static uint32_t send_cnt = 0, recv_cnt = 0;
static uint32_t error_cnt[SIZE_ARRAY( error_descr )]; //счетчики ошибок
static uint8_t recv, buff_data[BUFFER_CMD]; 
static uint8_t recv_buff[BUFF_RECV_SIZE], send_buff[BUFF_SEND_SIZE];

//Набор команд управления модулем ZigBee
static ZB_COMMAND zb_cmd[] = {
    //код команды, коды команды, время ожидания ответа, функция вызова
    { ZB_CMD_READ_CONFIG,   { 0xFE, 0x01, 0xFE, 0xFF }, 100, Command }, //чтение конфигурации
    { ZB_CMD_SAVE_CONFIG,   { 0xFD, 0x2E, 0xFE, 0xFF }, 200, SetCfg  }, //запись конфигурации
    { ZB_CMD_DEV_INIT,      { 0xFD, 0x01, 0x12, 0xFF }, 100, Command }, //перезапуск модуля
    { ZB_CMD_DEV_FACTORY,   { 0xFD, 0x01, 0x13, 0xFF }, 100, Command }, //установка заводских настроек
    { ZB_CMD_NET_RESTART,   { 0xFD, 0x01, 0x14, 0xFF }, 100, Command }  //переподключение к сети
   };

//Набор ответов модуля ZigBee (для HEX режима обмена данными)
static ZB_ANSWER zb_answr[] = {
    ZB_ANS_ERROR,         { 0xF7, 0xFF },    //ошибка команды
    ZB_ANS_BLD_NET,       { 0xFF, 0xFF },    //Координатор запросил информацию при установлении сети
    ZB_ANS_JOIN_NET,      { 0xFF, 0xAA },    //выполнено  подключение к сети
    ZB_ANS_NO_NET,        { 0xFF, 0x00 },    //сеть потеряна
    ZB_ANS_SET_CONFIG,    { 0xFA, 0xFE },    //запись конфигурации выполнена
    ZB_ANS_RESTART,       { 0xFA, 0x12 },    //модуль ZigBee перезапущен
    ZB_ANS_CFG_FACTORY,   { 0xFA, 0x13 },    //сброс настроек выполнен
    ZB_ANS_NET_RESTART,   { 0xFA, 0x14 }     //модуль переподключен к сети
 };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void TaskZBInit( void *pvParameters );
static void TaskZBFlow( void *pvParameters );
static void TaskZBCtrl( void *pvParameters );
static void Timer1Callback( void *arg );
static ZBErrorState SendData( ZBCmnd cmnd, uint8_t *data, uint8_t len, uint16_t timeout );
static ErrorStatus DevStatus( ZBDevState type );
static ZBErrorState GetAnswer( void );
static ZBAnswer CheckAnswer( uint8_t *answer, uint8_t len );
static void ClearRecv( void );
static void ErrorClr( void );

static char *DevType( ZBDevType dev );
static char *NwkState( ZBNetState state );
static char *TxPower( ZBTxPower id_pwr );

#if ( DEBUG_ZIGBEE == 1 ) && defined( DEBUG_TARGET )
static void SendDebug( ZBDebugMode type, uint8_t *data, uint8_t len );
static char *AnswDesc( ZBAnswer id_answ );
static char *PackDesc( ZBTypePack id_pack );
#endif

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t task1_attr = {
    .name = "ZBInit", 
    .stack_size = 512,
    .priority = osPriorityNormal
 };

static const osThreadAttr_t task2_attr = {
    .name = "ZBFlow", 
    .stack_size = 768,
    .priority = osPriorityNormal
 };

static const osThreadAttr_t task3_attr = {
    .name = "ZBCtrl", 
    .stack_size = 768,
    .priority = osPriorityNormal
 };

static const osSemaphoreAttr_t sem1_attr = { .name = "ZBSemSend" };
static const osSemaphoreAttr_t sem2_attr = { .name = "ZBSemAns" };
static const osEventFlagsAttr_t evn1_attr = { .name = "ZBEvents1" };
static const osEventFlagsAttr_t evn2_attr = { .name = "ZBEvents2" };
static const osMessageQueueAttr_t que_attr = { .name = "Recv" };
static const osTimerAttr_t timer1_attr = { .name = "ZBTimer1" };
static const osMutexAttr_t mutex_attr = { .name = "ZBBee", .attr_bits = osMutexPrioInherit };

//*************************************************************************************************
// Инициализация задачи и очереди событий управления модулем ZigBee
//*************************************************************************************************
void ZBInit( void ) {

    ErrorClr();
    DevListClr();
    GetAnswer();
    //очередь событий
    zb_init = osEventFlagsNew( &evn1_attr );
    zb_ctrl = osEventFlagsNew( &evn2_attr );
    //таймер интервалов
    timer_start = osTimerNew( Timer1Callback, osTimerOnce, NULL, &timer1_attr );
    //семафоры блокировки
    sem_send = osSemaphoreNew( 1, 0, &sem1_attr );
    sem_ans = osSemaphoreNew( 1, 0, &sem2_attr );
    //мьютех ожидания завершения цикла работы
    zb_mutex = osMutexNew( &mutex_attr );
    //очередь сообщений
    msg_recv = osMessageQueueNew( 32, sizeof( RECV_DATA ), &que_attr );
    //создаем задачу
    osThreadNew( TaskZBInit, NULL, &task1_attr );
    osThreadNew( TaskZBFlow, NULL, &task2_attr );
    osThreadNew( TaskZBCtrl, NULL, &task3_attr );
    //т.к. при выполнении HAL_TIM_Base_Start_IT() почти сразу формируется прерывание
    //вызов HAL_TIM_Base_Start_IT() выполняем только один раз, дальнейшее управление
    //TIMER2 выполняется через __HAL_TIM_ENABLE()/__HAL_TIM_DISABLE()
    HAL_TIM_Base_Start_IT( &htim6 );
    //запуск приема
    ClearRecv();
    HAL_UART_Receive_IT( &huart3, (uint8_t *)&recv, sizeof( recv ) );
 }

//*************************************************************************************************
// Задача проверки конфигурации и инициализации ZigBee модуля
//*************************************************************************************************
static void TaskZBInit( void *pvParameters ) {

    int32_t event;

    //проверка конфигурации модуля ZigBee с задержкой после включения
    if ( !osTimerIsRunning( timer_start ) )
        osTimerStart( timer_start, TIME_DELAY_CHECK );
    for ( ;; ) {
        event = osEventFlagsWait( zb_init, EVN_ZB_CONFIG, osFlagsWaitAny, osWaitForever );
        //проверка конфигурации при включении
        if ( event & EVN_ZB_CONFIG ) {
            ZBCheckConfig();
            //проверка конфигурации ZigBee модуля завершена, вывод приглашения в консоль
            osEventFlagsSet( cmnd_event, EVN_CMND_PROMPT );
            osThreadTerminate( osThreadGetId() );
           }
       }
 }
 
//*************************************************************************************************
// Задача управления ZigBee модулем
//*************************************************************************************************
static void TaskZBCtrl( void *pvParameters ) {

    int32_t event;
    void *mem_addr;
    uint8_t *data, len;
    RECV_DATA recv_data;

    for ( ;; ) {
        event = osEventFlagsWait( zb_ctrl, EVN_ZC_MASK, osFlagsWaitAny, osWaitForever );
        if ( event & EVN_ZC_RECV_CHECK ) {
            //индикация о принятии пакета
            osEventFlagsSet( chk_event, EVN_LED_ZB_ACTIVE );
            //выделяем блок памяти для размещения принятых данных
            mem_addr = pvPortMalloc( recv_ind );
            if ( mem_addr != NULL ) {
                recv_data.len = recv_ind;
                recv_data.ptr = mem_addr;
                memcpy( (uint8_t *)mem_addr, recv_buff, recv_ind );
                osMessageQueuePut( msg_recv, &recv_data, NULL, osWaitForever );
                #if ( DEBUG_MALLOC == 1 ) && defined( DEBUG_TARGET )
                sprintf( str, "Allocate: %u, free: %u\r\n", recv_ind, xPortGetFreeHeapSize() );
                UartSendStr( str );
                #endif
               }
            else {
                sprintf( str, "Memory allocation error: %u, free: %u\r\n", recv_ind, xPortGetFreeHeapSize() ); 
                UartSendStr( str );
               }
            //прием завершен, чистим приемный буфер
            ClearRecv();
           }
        if ( event & EVN_ZC_SEND_WLOG ) {
            //формируем подтверждение для получения следующего блока данных журнальных данных
            data = CreatePack( ZB_PACK_ACK, data_ack.dev_numb, &data_ack.net_addr, 0, VALVE_CTRL_NOTHING, VALVE_CTRL_NOTHING, &len );
            if ( data == NULL )
                UartSendStr( (char *)msg_err_dev );
            //отправка подтверждения для получения следующего блока данных
            //тут отправляем пакет без ожидания подтверждения (TIME_NO_WAIT), в случае, если
            //пакет сформирован неправильно, вместо запрашиваемых данных придет код ошибки
            else ZBSendPack1( data, len, data_ack.net_addr, TIME_NO_WAIT );
           }
       }
 }

//*************************************************************************************************
// Задача обработки ответов ZigBee модуля
//*************************************************************************************************
static void TaskZBFlow( void *pvParameters ) {

    osStatus_t status;
    RECV_DATA recv_data;
    uint16_t len_pack, len_chk, offset;

    for ( ;; ) {
        status = osMessageQueueGet( msg_recv, &recv_data, NULL, osWaitForever );
        //проверка принятых данных
        if ( status == osOK ) {
            //проверка системного ответа
            chk_answ = CheckAnswer( recv_data.ptr, recv_data.len );
            #if ( DEBUG_ZIGBEE == 1 ) && defined( DEBUG_TARGET )
            sprintf( str, "Answer SYS: %s\r\n", AnswDesc( chk_answ ) );
            UartSendStr( str );
            #endif
            if ( chk_answ == ZB_ANS_UNDEF ) {
                offset = 0;
                len_pack = recv_data.len;
                do {
                    //полученный блок данных может содержать несколько пакетов
                    //от разных уст-в, каждый пакет разбирается отдельно
                    len_chk = CheckPack1( (uint8_t *)recv_data.ptr + offset );
                    recv_cnt++; //кол-во принятых пакетов
                    //разбор/проверка полученного пакета данных
                    chk_pack = CheckPack2( (uint8_t *)recv_data.ptr + offset, len_chk, &data_ack );
                    #if ( DEBUG_ZIGBEE == 1 ) && defined( DEBUG_TARGET )
                    sprintf( str, " Pack data: %s\r\n", PackDesc( chk_pack ) );
                    UartSendStr( str );
                    #endif
                    if ( chk_pack != ZB_PACK_UNDEF ) {
                        //пакет данных - текущее состояние контроллера
                        if ( chk_pack == ZB_PACK_STATE ) {
                            OutData( ZB_PACK_STATE );
                            //osEventFlagsSet( cmnd_event, EVN_CMND_PROMPT );
                           }
                        //пакет данных - текущие данные расхода/давления/утечки воды
                        if ( chk_pack == ZB_PACK_DATA ) {
                            OutData( ZB_PACK_DATA ); 
                            //osEventFlagsSet( cmnd_event, EVN_CMND_PROMPT );
                           }
                        //пакет данных - состояние электроприводов
                        if ( chk_pack == ZB_PACK_VALVE ) {
                            OutData( ZB_PACK_VALVE ); 
                            //osEventFlagsSet( cmnd_event, EVN_CMND_PROMPT );
                           }
                        //пакет данных - состояние датчиков утечки
                        if ( chk_pack == ZB_PACK_LEAKS ) {
                            OutData( ZB_PACK_LEAKS ); 
                            //osEventFlagsSet( cmnd_event, EVN_CMND_PROMPT );
                           }
                        //пакет данных - журнальные данные расхода/давления/утечки воды
                        if ( chk_pack == ZB_PACK_WLOG ) {
                            OutData( ZB_PACK_WLOG );
                            osEventFlagsSet( zb_ctrl, EVN_ZC_SEND_WLOG );
                           }
                       }
                    //смещение на следующий пакет данных
                    offset += len_chk;
                    //остаток для проверки
                    len_pack -= len_chk;
                  } while ( len_pack && len_chk );
               }
            //проверка принятого пакета завершена, освободим блок памяти
            if ( recv_data.ptr != NULL )
                vPortFree( recv_data.ptr );
            #if ( DEBUG_MALLOC == 1 ) && defined( DEBUG_TARGET )
            sprintf( str, "Memory free: %u, available: %u\r\n", recv_data.len, xPortGetFreeHeapSize() ); 
            UartSendStr( str );
            #endif
            //проверка ожидания ответа
            if ( time_out == true ) {
                time_out = false;
                //снимаем семафор для последующей обработки ответа
                osSemaphoreRelease( sem_ans );
               }
           }
       }
 }

//*************************************************************************************************
// CallBack функция таймера, задержка проверки параметров ZigBee модуля
//*************************************************************************************************
static void Timer1Callback( void *arg ) {

    osEventFlagsSet( zb_init, EVN_ZB_CONFIG );
 }

//*************************************************************************************************
// CallBack функция TIMER6 - пауза в приеме данных, прием пакета завершен
//*************************************************************************************************
void ZBCallBack( void ) {

    //выключаем таймер
    __HAL_TIM_DISABLE( &htim6 );
    //сообщим в задачу для дальнейшей обработки принятых данных
    if ( recv_ind ) {
        osEventFlagsSet( zb_ctrl, EVN_ZC_RECV_CHECK );
       }
 }

//*************************************************************************************************
// CallBack функция при приеме байта по UART2
//*************************************************************************************************
void ZBRecvComplt( void ) {

    //прием одного байта
    if ( recv_ind < sizeof( recv_buff ) )
        recv_buff[recv_ind++] = recv;
    else ClearRecv(); //переполнение буфера
    //продолжаем прием
    HAL_UART_Receive_IT( &huart3, (uint8_t *)&recv, sizeof( recv ) );
    //если таймер выключен - стартуем один раз
    //при приеме каждого следующего байта - только сброс счетчика
    if ( !( htim6.Instance->CR1 & TIM_CR1_CEN ) )
        __HAL_TIM_ENABLE( &htim6 );
    else __HAL_TIM_SetCounter( &htim6, 0 );
}

//*************************************************************************************************
// CallBack функция, вызывается при завершении передачи из UART2
//*************************************************************************************************
void ZBSendComplt( void ) {

    osSemaphoreRelease( sem_send );
 }

//*************************************************************************************************
// Управление ZigBee модулем, кроме команды ZB_SEND_DATA, данные передаются через ZBSendPack()
//-------------------------------------------------------------------------------------------------
// ZBCmnd command      - код команды
// return ZBErrorState - результат выполнения
//*************************************************************************************************
ZBErrorState ZBControl( ZBCmnd command ) {

    uint8_t ind;
    ZBErrorState state;
    
    //проверка включенного ZigBee модуля
    if ( DevStatus( ZB_STATUS_RUN ) == ERROR )
        return ZB_ERROR_RUN;
    //блокировка доступа к ZigBee модулю
    osMutexAcquire( zb_mutex, osWaitForever );
    if ( command == ZB_CMD_DEV_RESET ) {
        //формируем сигнал сброса ZigBee модуля
        HAL_GPIO_WritePin( ZB_RES_GPIO_Port, ZB_RES_Pin, GPIO_PIN_RESET );
        //удержание сигнала сброса на TIME_DELAY_RESET msec
        osDelay( TIME_DELAY_RESET );
        HAL_GPIO_WritePin( ZB_RES_GPIO_Port, ZB_RES_Pin, GPIO_PIN_SET );
        //снимаем блокировку доступа к ZigBee модулю
        osMutexRelease( zb_mutex );
        return ZB_ERROR_OK;
       }
    //проверка и выполнение команды
    for ( ind = 0; ind < SIZE_ARRAY( zb_cmd ); ind++ ) {
        if ( zb_cmd[ind].id_command != command )
            continue;
        //выполнение команды через вызов функций: Command(), SetCfg()
        state = zb_cmd[ind].func_exec( &zb_cmd[ind] );
        break;
       }
    //снимаем блокировку доступа к ZigBee модулю
    osMutexRelease( zb_mutex );
    if ( ind == SIZE_ARRAY( zb_cmd ) )
        return ZB_ERROR_CMD;
    return state;
 }

//*************************************************************************************************
// Формирует команды для управления ZigBee модулем (программный перезапуск, переподключение к сети 
// чтение параметров ) вызывается по ссылке
//-------------------------------------------------------------------------------------------------
// uint8_t *ptr        - указатель на ZB_COMMAND
// return ZBErrorState - результат выполнения
//*************************************************************************************************
static ZBErrorState Command( void *ptr ) {

    ZB_COMMAND *zc_ptr;
    ZBErrorState state;

    zc_ptr = (ZB_COMMAND *)ptr;
    memset( buff_data, 0x00, sizeof( buff_data ) );
    memcpy( buff_data, zc_ptr->code_command, sizeof( zc_ptr->code_command ) );
    //передача данных
    state = SendData( zc_ptr->id_command, buff_data, sizeof( zc_ptr->code_command ), zc_ptr->time_answer );
    ZBIncError( state );
    return state;
 }

//*************************************************************************************************
// Формирует команды записи параметров ZigBee модуля
//-------------------------------------------------------------------------------------------------
// uint8_t *ptr        - указатель на ZB_COMMAND
// return ZBErrorState - результат выполнения
//*************************************************************************************************
static ZBErrorState SetCfg( void *ptr ) {

    uint8_t *dst, *cfg_ptr;
    ZB_COMMAND *zc_ptr;
    ZBErrorState state;
    
    dst = buff_data;
    zc_ptr = (ZB_COMMAND *)ptr;
    //установка параметров, тип уст-ва
    zb_cfg.dev_type = ZB_DEV_COORDINATOR;          
    //PAN ID
    cfg_ptr = (uint8_t *)&config.net_pan_id;
    zb_cfg.pan_id[1] = *cfg_ptr++;
    zb_cfg.pan_id[0] = *cfg_ptr;
    //Network group (NET No)
    zb_cfg.group = config.net_group;
    //для выполнения записи, необходимо обнулить: MAC, Network address, MAC coordinator 
    zb_cfg.short_addr[1] = 0x00;
    zb_cfg.short_addr[0] = 0x00;
    memset( zb_cfg.mac_addr, 0x00, sizeof( zb_cfg.mac_addr ) );
    zb_cfg.coor_short_addr[0] = 0x00;
    zb_cfg.coor_short_addr[1] = 0x00;
    memset( zb_cfg.coor_mac_addr, 0x00, sizeof( zb_cfg.coor_mac_addr ) );
    //ключ шифрования
    memcpy( (uint8_t *)&zb_cfg.key, config.net_key, sizeof( zb_cfg.key ) );
    //копируем всю команду
    memset( buff_data, 0x00, sizeof( buff_data ) );
    memcpy( buff_data, zc_ptr->code_command, sizeof( zc_ptr->code_command ) );
    //адрес для добавления данных с учетом смещения
    dst += OFFSET_CFG_DATA;
    //добавляем данные с учетом смещения
    memcpy( dst, (uint8_t *)&zb_cfg, sizeof( zb_cfg ) );
    //смещение для добавления кода завершения команды
    dst += sizeof( zb_cfg );
    //добавляем код завершения команды с учетом добавленных данных
    *dst = *( zc_ptr->code_command + sizeof( zc_ptr->code_command ) - 1 );
    //передача данных
    state = SendData( zc_ptr->id_command, buff_data, sizeof( zc_ptr->code_command ) + sizeof( zb_cfg ), zc_ptr->time_answer );
    ZBIncError( state );
    return state;
 }

//*************************************************************************************************
// Формирование и отправка пакета данных конкретному уст-ву.
//-------------------------------------------------------------------------------------------------
// uint8_t *data       - указатель на передаваемые данные
// uint8_t len         - размер передаваемых данных
// uint16_t time_answ  - время ожидания ответа
// return ZBErrorState - результат передачи данных
//*************************************************************************************************
ZBErrorState ZBSendPack1( uint8_t *data, uint8_t len, uint16_t addr, uint16_t time_answ ) {

    ZBErrorState state;
    uint8_t *dst, *addr8; 
    uint8_t command[] = { ZB_SEND_DATA, 0x00, ZB_ONDEMAND, ZB_ONDEMAND_ADDRESS };

    //проверка: адреса получателя/размера передаваемых данных
    if ( len > ( sizeof( buff_data ) - sizeof( command ) ) ) {
        ZBIncError( ZB_ERROR_DATA );
        return ZB_ERROR_DATA;
       }
    //проверка включенного ZigBee модуля
    if ( DevStatus( ZB_STATUS_RUN ) == ERROR ) {
        ZBIncError( ZB_ERROR_RUN );
        return ZB_ERROR_RUN;
       }
    //проверка наличия сети ZigBee модуля
    if ( zb_cfg.nwk_state == ZB_NETSTATE_NO ) {
        ZBIncError( ZB_ERROR_NETWORK );
        return ZB_ERROR_NETWORK;
       }
    send_cnt++; //подсчет отправленных пакетов
    //ставим блокировку доступа к ZigBee модулю
    osMutexAcquire( zb_mutex, osWaitForever );
    //подготовка пакета
    dst = buff_data;
    memset( buff_data, 0x00, sizeof( buff_data ) );
    //копируем в буфер команду
    memcpy( buff_data, command, sizeof( command ) );
    //смещение для добавления адреса получателя
    dst += sizeof( command );
    //адрес получателя
    addr8 = (uint8_t *)&addr;
    *dst++ = *( addr8 + 1 );
    *dst++ = *addr8;
    //добавляем данные
    memcpy( dst, data, len );
    //корректируем параметр "размер блока данных"
    *( buff_data + OFFSET_DATA_SIZE ) = len + ZB_MODE_SIZE + ZB_ADDR_SIZE;
    state = SendData( ZB_CMD_SEND_DATA, buff_data, len + sizeof( command ) + ZB_ADDR_SIZE, time_answ );
    ZBIncError( state );
    //передача завершена, снимаем блокировку доступа к ZigBee модулю
    osMutexRelease( zb_mutex );
    return state;
 }

//*************************************************************************************************
// Формирование и отправка широковещательного пакета данных
//-------------------------------------------------------------------------------------------------
// uint8_t *data       - указатель на передаваемые данные
// uint8_t len         - размер передаваемых данных
// return ZBErrorState - результат передачи данных
//*************************************************************************************************
ZBErrorState ZBSendPack( uint8_t *data, uint8_t len ) {

    ZBErrorState state;
    uint8_t *dst;
    uint8_t command[] = { ZB_SEND_DATA, 0x00, ZB_BROADCASTING, ZB_BROADCASTING_MODE1 };

    //проверка: адреса получателя/размера передаваемых данных
    if ( len > ( sizeof( buff_data ) - sizeof( command ) ) ) {
        ZBIncError( ZB_ERROR_DATA );
        return ZB_ERROR_DATA;
       }
    //проверка включенного ZigBee модуля
    if ( DevStatus( ZB_STATUS_RUN ) == ERROR ) {
        ZBIncError( ZB_ERROR_RUN );
        return ZB_ERROR_RUN;
       }
    //проверка наличия сети ZigBee модуля
    if ( DevStatus( ZB_STATUS_NET ) == ERROR ) {
        ZBIncError( ZB_ERROR_NETWORK );
        return ZB_ERROR_NETWORK;
       }
    send_cnt++; //подсчет отправленных пакетов
    //ставим блокировку доступа к ZigBee модулю
    osMutexAcquire( zb_mutex, osWaitForever );
    //подготовка пакета
    dst = buff_data;
    memset( buff_data, 0x00, sizeof( buff_data ) );
    //копируем в буфер команду
    memcpy( buff_data, command, sizeof( command ) );
    //смещение для добавления адреса получателя
    dst += sizeof( command );
    //добавляем данные
    memcpy( dst, data, len );
    //корректируем параметр "размер блока данных"
    *( buff_data + OFFSET_DATA_SIZE ) = len + ZB_MODE_SIZE;
    state = SendData( ZB_CMD_SEND_DATA, buff_data, len + sizeof( command ), TIME_DELAY_ANSWER );
    if ( state == ZB_ERROR_TIMEOUT )
        state = ZB_ERROR_OK; //нет сообщения об ошибке, передача выполнена
    else ZBIncError( state );
    //передача завершена, снимаем блокировку доступа к ZigBee модулю
    osMutexRelease( zb_mutex );
    return state;
 }

//*************************************************************************************************
// Отправка данных/команд в ZigBee модуль
//-------------------------------------------------------------------------------------------------
// ZBCmnd cmnd         - код команды
// uint8_t *data       - указатель на передаваемые данные
// uint8_t len         - размер передаваемых данных
// uint16_t timeout    - время ожидания ответа, при значении = "0" - нет ожидания проверки 
//                       корректности передаваемого пакета.
// return ZBErrorState - результат передачи данных
//*************************************************************************************************
static ZBErrorState SendData( ZBCmnd cmnd, uint8_t *data, uint8_t len, uint16_t timeout ) {

    ZBErrorState state;
    osStatus_t state_sem;
    
    GetAnswer();
    ClearRecv();
    //проверка параметров вызова
    if ( data == NULL || len > sizeof( send_buff ) )
        return ZB_ERROR_DATA;
    //копируем в буфер данные
    memset( send_buff, 0x00, sizeof( send_buff ) );
    memcpy( send_buff, data, len );
    #if ( DEBUG_ZIGBEE == 1 ) && defined( DEBUG_TARGET )
    SendDebug( ZB_DEBUG_TX, data, len );
    #endif
    osEventFlagsSet( chk_event, EVN_LED_ZB_ACTIVE );
    //передача данных
    if ( HAL_UART_Transmit_DMA( &huart3, send_buff, len ) == HAL_OK )
        osSemaphoreAcquire( sem_send, osWaitForever ); //ждем завершение передачи данных
    else return ZB_ERROR_SEND;
    state = ZB_ERROR_OK;
    if ( timeout ) {
        time_out = true;
        //ждем получения ответа
        state_sem = osSemaphoreAcquire( sem_ans, timeout );
        if ( state_sem == osErrorTimeout )
            return ZB_ERROR_TIMEOUT;
        //проверка ответа
        state = GetAnswer();
       }
    return state;
 }

//*************************************************************************************************
// Функция возвращает результат выполнения команды управления (обменом данными) с ZigBee модулем
// При вызове функции переменные: ZBAnswer chk_answ, ZBTypePack chk_pack в которых хранится 
// результат проверки ответа - обнуляются.
//-------------------------------------------------------------------------------------------------
// return ZBErrorState - результат проверки
//*************************************************************************************************
static ZBErrorState GetAnswer( void ) {

    ZBAnswer answ;
    ZBTypePack pack;

    //сохраним текущие значения типов ответов
    answ = chk_answ;
    pack = chk_pack;
    //сбросим значения типов ответов
    chk_answ = ZB_ANS_UNDEF;
    chk_pack = ZB_PACK_UNDEF;
    //формируем результат проверки принятых пакетов
    //тип ответа/пакета данных не определен
    if ( answ == ZB_ANS_UNDEF && pack == ZB_PACK_UNDEF )
        return ZB_ERROR_UNDEF;
    //ошибка выполненение команды (код команды, параметры заданы не верно)
    if ( answ == ZB_ANS_ERROR )
        return ZB_ERROR_EXEC;
    //системная команда выполнена
    if ( answ == ZB_ANS_SET_CONFIG || answ == ZB_ANS_RESTART || 
         answ == ZB_ANS_CFG_FACTORY || answ == ZB_ANS_NET_RESTART || 
         answ == ZB_ANS_READ_CONFIG )
        return ZB_ERROR_OK;
    //входящий пакет идентифицирован и проверен
    if ( pack != ZB_PACK_UNDEF )
        return ZB_ERROR_OK;
    else return ZB_ERROR_UNDEF;
 }

//*************************************************************************************************
// Функция проверяет наличие системных ответов, чтение конфигурации ZigBee модуля
// Проверка выполняется сравнением по данным массива zb_answr[]
//-------------------------------------------------------------------------------------------------
// uint8_t *answer - указатель на буфер с ответом
// uint8_t len     - размер ответа в байтах
// return ZBAnswer - код ответа
//*************************************************************************************************
static ZBAnswer CheckAnswer( uint8_t *answer, uint8_t len ) {

    ZBAnswer answ;
    uint8_t ind, head;

    answ = ZB_ANS_UNDEF;
    head = *answer;

    #if ( DEBUG_ZIGBEE == 1 ) && defined( DEBUG_TARGET )
    SendDebug( ZB_DEBUG_RX, answer, len );
    #endif
    //первый шаг: поиск ответа в списке доступных
    for ( ind = 0; ind < SIZE_ARRAY( zb_answr ); ind++ ) {
        if ( len != sizeof( zb_answr[ind].code_answer ) )
            continue; //сравнивать только ответ заданной длины
        if ( memcmp( answer, zb_answr[ind].code_answer, sizeof( zb_answr[ind].code_answer ) ) == 0 ) {
            answ = zb_answr[ind].id_answer;
            break;
           }
       }
    //второй шаг: актуализация состояния подключения к сети
    if ( answ == ZB_ANS_JOIN_NET || answ == ZB_ANS_BLD_NET )
        zb_cfg.nwk_state = ZB_NETSTATE_OK;
    if ( answ == ZB_ANS_NO_NET )
        zb_cfg.nwk_state = ZB_NETSTATE_NO;
    //обработка конфигурации модуля
    if ( answ == ZB_ANS_UNDEF && head == ZB_SYS_CONFIG && len == sizeof( zb_cfg ) + 1 ) {
        //чтение конфигурации модуля
        memcpy( (uint8_t *)&zb_cfg, answer + 1, sizeof( zb_cfg ) );
        answ = ZB_ANS_READ_CONFIG;
       }
    return answ;
 }

//*************************************************************************************************
// Чтение и установка параметров ZigBee модуля 
// Если параметры модуля не совпадают с параметрами в конфигурации - выполняется установка параметров
//*************************************************************************************************
void ZBCheckConfig( void ) {

    uint8_t *ptr;
    uint16_t cfg_id, cfg_zb;
    ZBErrorState state;
    bool change = false;
    
    //чтение конфигурации
    state = ZBControl( ZB_CMD_READ_CONFIG );
    sprintf( str, "%s %s\r\n", (char *)msg_zb_read, ZBErrDesc( state ) );
    UartSendStr( str );
    if ( state != ZB_ERROR_OK )
        return;
    //установка параметров ZigBee модуля
    if ( zb_cfg.dev_type != ZB_DEV_COORDINATOR ) {
        change = true;
        zb_cfg.dev_type = ZB_DEV_COORDINATOR;
       }
    //PAN ID
    cfg_id = __REVSH( config.net_pan_id );
    cfg_zb = *( (uint16_t *)&zb_cfg.pan_id );
    if ( cfg_id != cfg_zb ) {
        change = true;
        ptr = (uint8_t *)&config.net_pan_id;
        zb_cfg.pan_id[1] = *ptr++;
        zb_cfg.pan_id[0] = *ptr;
       }
    //Network group (NET No)
    if ( zb_cfg.group != config.net_group ) {
        change = true;
        zb_cfg.group = config.net_group;
       }
    //ключ шифрования
    if ( memcmp( zb_cfg.key, config.net_key, sizeof( zb_cfg.key ) ) != 0 ) {
        change = true;
        memcpy( zb_cfg.key, config.net_key, sizeof( zb_cfg.key ) );
       }
    if ( change == false ) {
        UartSendStr( "ZB: parameters match.\r\n" );
        return;
       }
    //запись параметров
    state = ZBControl( ZB_CMD_SAVE_CONFIG );
    sprintf( str, "%s %s\r\n", (char *)msg_zb_save, ZBErrDesc( state ) );
    UartSendStr( str );
    state = ZBControl( ZB_CMD_READ_CONFIG );
    sprintf( str, "%s %s\r\n", (char *)msg_zb_read, ZBErrDesc( state ) );
    UartSendStr( str );
    //вывод конфигурации ZigBee модуля
    ZBConfig();
 }

//*************************************************************************************************
// Обнуление приемного буфера
//*************************************************************************************************
static void ClearRecv( void ) {

    recv_ind = 0;
    memset( (uint8_t *)recv_buff, 0x00, sizeof( recv_buff ) );
 }

//*************************************************************************************************
// Возвращает статус ZigBee модуля для запрашиваемого типа состояния
//-------------------------------------------------------------------------------------------------
// ZBDevStat type   - тип состояния "запущен/в сети"
// return = SUCCESS - успешно
//          ERROR   - ошибка
//*************************************************************************************************
static ErrorStatus DevStatus( ZBDevState type ) {

    if ( type == ZB_STATUS_RUN )
        return (ErrorStatus)HAL_GPIO_ReadPin( ZB_RUN_GPIO_Port, ZB_RUN_Pin );
    #if NET_STATUS_SOFT == 1
    if ( zb_cfg.nwk_state == ZB_NETSTATE_NO )
        return ERROR;
    else return SUCCESS;
    #else
    if ( type == ZB_STATUS_NET )
        return (ErrorStatus)HAL_GPIO_ReadPin( ZB_NET_GPIO_Port, ZB_NET_Pin );
    #endif
 }

//*************************************************************************************************
// Вывод значений параметров ZigBee модуля
//*************************************************************************************************
void ZBConfig( void ) {

    uint8_t ind;
    char *ptr, str[80];

    sprintf( str, "Device type ........................... %s\r\n", DevType( zb_cfg.dev_type ) );
    UartSendStr( str );
    sprintf( str, "Network state ......................... %s\r\n", NwkState( zb_cfg.nwk_state ) );
    UartSendStr( str );
    sprintf( str, "Network PAN_ID ........................ 0x%02X%02X\r\n", zb_cfg.pan_id[0], zb_cfg.pan_id[1] );
    UartSendStr( str );
    ptr = str;
    ptr += sprintf( ptr, "Network key ........................... " );
    for ( ind = 0; ind < sizeof( zb_cfg.key ); ind++ )
        ptr += sprintf( ptr, "%02X", zb_cfg.key[ind] );
    ptr += sprintf( ptr, "\r\n" );
    UartSendStr( str );
    sprintf( str, "Network short address ................. 0x%02X%02X\r\n", zb_cfg.short_addr[0], zb_cfg.short_addr[1] );
    UartSendStr( str );
    ptr = str;
    ptr += sprintf( ptr, "MAC address ........................... " );
    for ( ind = 0; ind < sizeof( zb_cfg.mac_addr ) - 1; ind++ )
        ptr += sprintf( ptr, "%02X:", zb_cfg.mac_addr[ind] );
    ptr += sprintf( ptr, "%02X\r\n", zb_cfg.mac_addr[ind] );
    UartSendStr( str );
    sprintf( str, "Network short address of father node .. 0x%02X%02X\r\n", zb_cfg.coor_short_addr[0], zb_cfg.coor_short_addr[1] );
    UartSendStr( str );
    ptr = str;
    ptr += sprintf( ptr, "MAC address of father node ............ " );
    for ( ind = 0; ind < sizeof( zb_cfg.coor_mac_addr ) - 1; ind++ )
        ptr += sprintf( ptr, "%02X:", zb_cfg.coor_mac_addr[ind] );
    ptr += sprintf( ptr, "%02X\r\n", zb_cfg.coor_mac_addr[ind] );
    UartSendStr( str );
    sprintf( str, "Network group number .................. %u\r\n", zb_cfg.group );
    UartSendStr( str );
    sprintf( str, "Communication channel ................. %u\r\n", zb_cfg.chanel );
    UartSendStr( str );
    sprintf( str, "TX power .............................. %s dbm\r\n", TxPower( zb_cfg.txpower ) );
    UartSendStr( str );
    sprintf( str, "Sleep state ........................... %u\r\n", zb_cfg.sleep_time );
    UartSendStr( str );
 }

//*************************************************************************************************
// Возвращает расшифровку и значения счетчиков ошибок
//-------------------------------------------------------------------------------------------------
// ZBErrorState err_ind - индекс счетчика ошибок, для err_ind = ZB_ERROR_OK 
//                        возвращается общее кол-во принятых пакетов
// char *str            - указатель для размещения результата
// return               - значение счетчика ошибок
//*************************************************************************************************
char *ZBErrCntDesc( ZBErrorState err_ind, char *str ) {

    char *ptr, *prev;
    
    if ( err_ind >= SIZE_ARRAY( error_cnt ) )
        return NULL;
    ptr = str;
    if ( err_ind == ZB_ERROR_OK ) {
        ptr += sprintf( ptr, "Total packages recv" );
        ptr += AddDot( str, 45, 0 );
        ptr += sprintf( ptr, "%6u\r\n", recv_cnt );
        prev = ptr;
        ptr += sprintf( ptr, "Total packages send" );
        ptr += AddDot( str, 45, (uint8_t)( prev - str ) );
        ptr += sprintf( ptr, "%6u", send_cnt );
        return str;
       }
    ptr += sprintf( ptr, "%s", ZBErrDesc( err_ind ) );
    //дополним расшифровку ошибки справа знаком "." до 45 символов
    ptr += AddDot( str, 45, 0 );
    ptr += sprintf( ptr, "%6u ", error_cnt[err_ind] );
    return str;
 }

//*************************************************************************************************
// Возвращает указатель на строку расшифровки результата выполнения запроса по протоколу 
//-------------------------------------------------------------------------------------------------
// ModBusError err_ind - код ошибки обработки принятого ответа от уст-ва
// return              - расшифровка кода ошибки
//*************************************************************************************************
char *ZBErrDesc( ZBErrorState err_ind ) {

    if ( err_ind < SIZE_ARRAY( error_descr ) )
        return error_descr[err_ind];
    return NULL;
 }

//*************************************************************************************************
// Возвращает указатель на строку с расшифровкой значения параметра: тип параметра
//-------------------------------------------------------------------------------------------------
// ZBDevType dev - идентификатор сообщения
// return = NULL - идентификтор не определен, иначе указатель на строку
//*************************************************************************************************
static char *DevType( ZBDevType dev ) {

    if ( dev < SIZE_ARRAY( dev_type ) )
        return dev_type[dev];
    else return NULL;
 }
 
//*************************************************************************************************
// Возвращает указатель на строку с расшифровкой значения параметра: статус сети
//-------------------------------------------------------------------------------------------------
// ZBDevState state - идентификатор сообщения
// return = NULL    - идентификтор не определен, иначе указатель на строку
//*************************************************************************************************
static char *NwkState( ZBNetState state ) {

    if ( state < SIZE_ARRAY( nwk_state ) )
        return nwk_state[state];
    else return NULL;
 }
 
//*************************************************************************************************
// Возвращает указатель на строку с расшифровкой значения параметра: значение мощности
//-------------------------------------------------------------------------------------------------
// uint8_t id_mess - идентификатор сообщения
// return = NULL   - идентификтор не определен, иначе указатель на строку
//*************************************************************************************************
static char *TxPower( ZBTxPower id_pwr ) {

    if ( id_pwr < SIZE_ARRAY( txpower ) )
        return txpower[id_pwr];
    else return NULL;
 }
 
//*************************************************************************************************
// Обнуляет счетчики ошибок
//*************************************************************************************************
static void ErrorClr( void ) {

    recv_cnt = send_cnt = 0;
    memset( (uint8_t *)&error_cnt, 0x00, sizeof( error_cnt ) );
 }

//*************************************************************************************************
// Инкремент счетчиков ошибок
//-------------------------------------------------------------------------------------------------
// ZBErrorState err_ind - код ошибки
//*************************************************************************************************
void ZBIncError( ZBErrorState err_ind ) {

    if ( err_ind < SIZE_ARRAY( error_cnt ) )
        error_cnt[err_ind]++;
 }

//*************************************************************************************************
// Возвращает значение счетчика ошибок
//-------------------------------------------------------------------------------------------------
// ZBErrorState err_ind - индекс счетчика ошибок, для err_ind = ZB_ERROR_OK возвращается 
//                        кол-во счетчиков
// return               - значение счетчика ошибок
//*************************************************************************************************
uint32_t ZBErrCnt( ZBErrorState err_ind ) {

    if ( err_ind == ZB_ERROR_OK )
        return SIZE_ARRAY( error_cnt );
    if ( err_ind < SIZE_ARRAY( error_cnt ) )
        return error_cnt[err_ind];
    return 0;
 }


#if ( DEBUG_ZIGBEE == 1 ) && defined( DEBUG_TARGET )

//*************************************************************************************************
// Возвращает указатель на строку с расшифровкой значения стандартного ответа ZigBee модуля
//-------------------------------------------------------------------------------------------------
// ZBAnswer id_answ - идентификатор сообщения
// return = NULL    - идентификтор не определен, иначе указатель на строку
//*************************************************************************************************
static char *AnswDesc( ZBAnswer id_answ ) {

    if ( id_answ < SIZE_ARRAY( desc_answer ) )
        return desc_answer[id_answ];
    else return NULL;
 }

//*************************************************************************************************
// Возвращает указатель на строку с расшифровкой значения 
//-------------------------------------------------------------------------------------------------
// ZBAnswer id_answ - идентификатор сообщения
// return = NULL    - идентификтор не определен, иначе указатель на строку
//*************************************************************************************************
static char *PackDesc( ZBTypePack id_pack ) {

    if ( id_pack < SIZE_ARRAY( desc_pack ) )
        return desc_pack[id_pack];
    else return NULL;
 }

//*************************************************************************************************
// Вывод отладочной информации при передаче данных в модуль ZigBee
//-------------------------------------------------------------------------------------------------
// uint8_t *data    - указатель на передаваемые данные пакета
// uint8_t len_send - размер передаваемых данных
//*************************************************************************************************
static void SendDebug( ZBDebugMode type, uint8_t *data, uint8_t len ) {

    uint8_t i;
    char *ptr;

    ptr = str2;
    if ( type == ZB_DEBUG_RX )
        ptr += sprintf( ptr, "RECV: " );
    else ptr += sprintf( ptr, "SEND: " );
    for ( i = 0; i < len; i++ )
        ptr += sprintf( ptr, "%02X ", *data++ );
    ptr += sprintf( ptr, "\r\n" );
    UartSendStr( str2 );
 }

#endif
