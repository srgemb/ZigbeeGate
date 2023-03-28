
//*************************************************************************************************
//
// Управление индикацией контроллера
// 
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#include "cmsis_os2.h"

#include "main.h"
#include "led.h"
#include "events.h"

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
//интервал переключения контрольного индикатора
#define TIME_DELAY_NORM         200         //штатный режим работы (msec)
#define TIME_DELAY_ACTIVE       50          //обмен данными - радио модуль (msec)
#define TIME_ACTIVE             300         //длительность сигнала активности (TIME_DELAY_ACTIVE)

#define LED_ON                  0           //светодиод вкл
#define LED_OFF                 1           //светодиод выкл

#define LED_MODE_NONE           0           //светодиод в режиме вкл или выкл
#define LED_MODE_BLK            1           //светодиод в режиме переключения (мигания)
                                            //с интервалом TIME_DELAY

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
osEventFlagsId_t chk_event = NULL;

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static osTimerId_t timer_led;
static uint32_t led_mode = TIME_DELAY_NORM;

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
//static void TaskLed( void *pvParameters );
static void TaskChk( void *pvParameters );
static void TimerCallback( void *arg );

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t task2_attr = {
    .name = "Chk", 
    .stack_size = 256,
    .priority = osPriorityNormal
 };

static const osEventFlagsAttr_t evn_attr = { .name = "LedEvents" };
static const osTimerAttr_t timer_attr = { .name = "TimerLed" };

//*************************************************************************************************
// Инициализация задачи и очереди событий управления индикаторами
//*************************************************************************************************
void LedInit( void ) {

    //очередь событий
    chk_event = osEventFlagsNew( &evn_attr );
    timer_led = osTimerNew( TimerCallback, osTimerOnce, NULL, &timer_attr );
    //создаем задачу
    osThreadNew( TaskChk, NULL, &task2_attr );
 }
 
//*************************************************************************************************
// Задача обработки событий управления контрольным индикатором
//*************************************************************************************************
static void TaskChk( void *pvParameters ) {

    for ( ;; ) {
        osDelay( led_mode );
        if ( osEventFlagsWait( chk_event, EVN_LED_ZB_ACTIVE, osFlagsWaitAny, 0 ) == EVN_LED_ZB_ACTIVE ) {
            led_mode = TIME_DELAY_ACTIVE;
            osTimerStart( timer_led, TIME_ACTIVE );
           }
        //контрольный индикатор
        HAL_GPIO_TogglePin( LED_CHK_GPIO_Port, LED_CHK_Pin );
       }
 }

//*************************************************************************************************
// CallBack функция таймера, восстановление режима индикации
//*************************************************************************************************
static void TimerCallback( void *arg ) {

    led_mode = TIME_DELAY_NORM;
 }
