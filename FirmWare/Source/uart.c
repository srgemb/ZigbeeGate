
//*************************************************************************************************
//
// Управление обменом по UART (отладка), тип очереди передачи: FIFO
// 
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "cmsis_os.h"

#include "main.h"
#include "parse.h"
#include "events.h"
#include "command.h"
#include "uart.h"
#include "vt100.h"

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern UART_HandleTypeDef huart1;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define RECV_BUFF_SIZE      40              //размер приемного буфера
#define SEND_BUFF_SIZE      1024            //размер передающего буфера

#define KEY_BACKSPACE       0x08            //удаление символа
#define KEY_ESC_CMD         0x1B            //

static char const key_up[]   = { 0x1B, 0x5B, 0x41 };

//соответствие значений индексов скорости и значения скорости обмена, 
//длительность передачи одного байта (мкс)
static uint32_t uart_speed[][3] = {
    //ID скорости         скорость   длительность   
    //обмена              обмена       передачи
    { UART_SPEED_600,       600,        16700 },
    { UART_SPEED_1200,      1200,       8350 },
    { UART_SPEED_2400,      2400,       4200 },
    { UART_SPEED_4800,      4800,       2083},
    { UART_SPEED_9600,      9600,       1042 },
    { UART_SPEED_14400,     14400,      694 },
    { UART_SPEED_19200,     19200,      521 },
    { UART_SPEED_28800,     28800,      348 },
    { UART_SPEED_38400,     38400,      260 },
    { UART_SPEED_56000,     56000,      249 },
    { UART_SPEED_57600,     57600,      174 },
    { UART_SPEED_115200,    115200,     87 }
 };

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
osEventFlagsId_t uart_event = NULL;

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static uint16_t esc_ind = 0;
static uint16_t recv_ind = 0, tail = 0, head_tx = 0, tail_tx = 0;
static char recv_ch, recv_temp[RECV_BUFF_SIZE];
static char recv_buff[RECV_BUFF_SIZE], send_buff[SEND_BUFF_SIZE];
static osSemaphoreId_t sem_busy;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t task_attr = {
    .name = "Uart", 
    .stack_size = 432,
    .priority = osPriorityBelowNormal
 };

static const osSemaphoreAttr_t sem_attr = { .name = "UartSemaph" };
static const osEventFlagsAttr_t evn_attr = { .name = "UartEvents" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void TaskUart( void *argument );

//*************************************************************************************************
// Инициализация очереди, задачи для UART1
//*************************************************************************************************
void UartInit( void ) {

    //очередь событий
    uart_event = osEventFlagsNew( &evn_attr );
    //семафор блокировки
    sem_busy = osSemaphoreNew( 1, 0, &sem_attr );
    //создаем задачу обработки команд
    osThreadNew( TaskUart, NULL, &task_attr );
    //инициализация приема по UART
    memset( send_buff, 0x00, sizeof( send_buff ) );
    memset( recv_buff, 0x00, sizeof( recv_buff ) );
    //инициализация приема по UART
    HAL_UART_Receive_IT( &huart1, (uint8_t *)&recv_ch, sizeof( recv_ch ) );
}

//*************************************************************************************************
// Задача обработки очереди сообщений приема данных по UART1.
//*************************************************************************************************
static void TaskUart( void *argument ) {

    int32_t event;

    vt100Init();
    for ( ;; ) {
        event = osEventFlagsWait( uart_event, EVN_UART_MASK, osFlagsWaitAny, osWaitForever );
        if ( event & EVN_UART_START ) {
            //команда выполнена, возобновляем прием команд по UART1
            recv_ind = 0; 
            memset( recv_buff, 0x00, sizeof( recv_buff ) );
            HAL_UART_Receive_IT( &huart1, (uint8_t *)&recv_ch, sizeof( recv_ch ) );
          }
        if ( event & EVN_UART_NEXT ) {
            //передача блока завершена
            if ( tail == tail_tx ) {
                //индекс хвоста буфера и индекс хвоста передачи совпадает - все передано
                tail = head_tx = tail_tx = 0;
                memset( send_buff, 0x00, sizeof( send_buff ) );
                osSemaphoreRelease( sem_busy );
               }
            else {
                //передача продолжается, добавлены данные
                head_tx = tail_tx;  //индекс головы перемещаем в конец переданного ранее блока
                tail_tx = tail;     //индекс своста перемещаем на окончание добавленных данных
                HAL_UART_Transmit_DMA( &huart1, (uint8_t *)( send_buff + head_tx ), tail_tx - head_tx );
               }
          }
       }
 }

//*************************************************************************************************
// Функция вызывается при приеме байта по UART1 (вызов из events.c)
// Принятый байт размещается в приемном буфере, при приеме кода CR 
// передаем событие: EVN_CMND_EXEC в задачу обработки команд
//*************************************************************************************************
void UartRecvComplt( void ) {

    if ( recv_ind >= sizeof( recv_buff ) ) {
        recv_ind = 0;
        memset( recv_buff, 0x00, sizeof( recv_buff ) );
       }
    if ( recv_ch == KEY_BACKSPACE && recv_ind ) {
        recv_ind--;
        recv_buff[recv_ind] = 0x00;
        //продолжаем прием
        HAL_UART_Receive_IT( &huart1, (uint8_t *)&recv_ch, sizeof( recv_ch ) );
        return;
       }
    recv_buff[recv_ind++] = recv_ch;
    if ( recv_ch == KEY_ESC_CMD )
        esc_ind = recv_ind; //признак команды ESC
    if ( memcmp( recv_buff + esc_ind - 1, key_up, sizeof( key_up ) ) == 0 && esc_ind ) {
        //вывод в консоль предыдущей команды
        esc_ind = 0;
        memset( recv_buff, 0x00, sizeof( recv_buff ) );
        vt100CursorDn();
        //копируем в буфер предыдущую команду
        memcpy( recv_buff, recv_temp, strlen( recv_temp ) );
        recv_ind = strlen( recv_temp );
        UartSendStr( recv_buff );
       }
    //проверим последний принятый байт, если CR - обработка команды
    if ( recv_buff[recv_ind - 1] == '\r' ) {
        esc_ind = 0;
        recv_buff[recv_ind - 1] = '\0'; //уберем код CR
        //сохраним команду в буфере
        memset( recv_temp, 0x00, sizeof( recv_temp ) );
        memcpy( recv_temp, recv_buff, recv_ind );
        //выполнение команды в TaskCommand()
        osEventFlagsSet( cmnd_event, EVN_CMND_EXEC );
        return; //не выполняем запуск приема по UART1
       }
    //продолжаем прием
    HAL_UART_Receive_IT( &huart1, (uint8_t *)&recv_ch, sizeof( recv_ch ) );
}

//*************************************************************************************************
// Функция вызывается при завершении передачи из UART1 (вызов из events.c)
//*************************************************************************************************
void UartSendComplt( void ) {

    osEventFlagsSet( uart_event, EVN_UART_NEXT );
 }

//*************************************************************************************************
// Добавляем строку в буфер и запускаем передачу в UART1
//-------------------------------------------------------------------------------------------------
// char *str - указатель на строку для добавления
//*************************************************************************************************
void UartSendStr( char *str ) {

    bool start = false;
    uint16_t length, free, offset = 0;
    
    length = strlen( str );
    do {
        //доступное место в буфере
        free = sizeof( send_buff ) - tail;
        if ( !free ) {
            //ждем свободного места в буфере
            osSemaphoreAcquire( sem_busy, osWaitForever );
            free = sizeof( send_buff ) - tail;
           }
        //проверка места в буфере
        if ( length > free ) {
            //места для размещения всей строки не достаточно, добавляем в буфер часть строки
            memcpy( send_buff + tail, str + offset, free );
            length -= free; //осталось для передачи
            offset += free; //смещение на следующий фрагмент
           }
        else {
            //места для размещения всей строки достаточно, добавляем в буфер всю строку
            memcpy( send_buff + tail, str + offset, length );
            free = length; //размер добавляемого фрагмента
            length = 0;    //размер оставшейся части строки
           }
        if ( !tail )
            start = true;  //признак начала передачи
        else start = false;
        tail += free;      //увеличиваем индекс хвоста 
        if ( start == true ) {
            //начало передачи
            head_tx = 0;
            tail_tx = tail;
            HAL_UART_Transmit_DMA( &huart1, (uint8_t *)send_buff, free );
           }
       } while ( length );
 }

//*************************************************************************************************
// Возвращает адрес приемного буфера UART
//-------------------------------------------------------------------------------------------------
// return - указатель на буфер
//*************************************************************************************************
char *UartBuffer( void ) {

    return recv_buff;
 }
 
//*************************************************************************************************
// Возвращает числовое значение скорости обмена по ID
//-------------------------------------------------------------------------------------------------
// UARTSpeed speed - ID скорости обмена
// return          - значение скорости обмена
//*************************************************************************************************
uint32_t UartGetSpeed( UARTSpeed speed ) {

    if ( speed < SIZE_ARRAY( uart_speed ) )
        return uart_speed[speed][1];
    else return 0;
 }

//*************************************************************************************************
// Проверка значения baud на допустимое значение скорости обмена.
// При успешной проверке в speed возвращается ID скорости обмена
//-------------------------------------------------------------------------------------------------
// uint32_t baud    - скорость обмена
// UARTSpeed *speed - ID скорости обмена
// return = ERROR   - не допустимое значение скорости обмена
//        = SUCCESS - в speed указан ID значения скорости обмена
//*************************************************************************************************
ErrorStatus CheckBaudRate( uint32_t baud, UARTSpeed *speed ) {

    uint8_t i;

    *speed = UART_SPEED_600; //значение по умолчанию
    for ( i = 0; i < SIZE_ARRAY( uart_speed ); i++ ) {
        if ( baud == uart_speed[i][1] ) {
            *speed = (UARTSpeed)uart_speed[i][0];
            return SUCCESS;
           }
       }
    return ERROR;
}

