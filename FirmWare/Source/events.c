
//*************************************************************************************************
//
// Callback функции обработки прерываний
// 
//*************************************************************************************************

#include <string.h>

#include "cmsis_os2.h"

#include "main.h"
#include "data.h"
#include "uart.h"
#include "events.h"
#include "xtime.h"
#include "zigbee.h"

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
//extern IWDG_HandleTypeDef hiwdg;
extern UART_HandleTypeDef huart1, huart3;

//*************************************************************************************************
// CallBack функция, вызывается при приеме байта по UART
//*************************************************************************************************
void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart ) {

    if ( huart == &huart1 )
        UartRecvComplt();
    if ( huart == &huart3 )
        ZBRecvComplt();
}

//*************************************************************************************************
// CallBack функция, вызывается при завершении передачи из UART
//*************************************************************************************************
void HAL_UART_TxCpltCallback( UART_HandleTypeDef *huart ) {

    if ( huart == &huart1 )
        UartSendComplt();
    if ( huart == &huart3 )
        ZBSendComplt();
 }

//*************************************************************************************************
// CallBack функция, секундное прерывание от RTC
//*************************************************************************************************
void HAL_RTCEx_RTCEventCallback( RTC_HandleTypeDef *hrtc ) {

    DATE_TIME date_time;
    
    GetTimeDate( &date_time );
    DevListUpd();
 }

//*************************************************************************************************
// CallBack функция, простой планировщика
//*************************************************************************************************
/*void vApplicationIdleHook( void ) {

    #ifndef DEBUG_TARGET    
    HAL_IWDG_Refresh( &hiwdg );
    #endif
 }*/
