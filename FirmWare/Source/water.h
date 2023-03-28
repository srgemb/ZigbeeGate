
#ifndef __WATER_H
#define __WATER_H

#include <stdint.h>
#include <stdbool.h>

//Источник давления
typedef enum {
    WATER_COLD,                             //холодная вода
    WATER_HOT                               //горячая вода
 } Water;       
        
//Тип учета расхода воды        
typedef enum {      
    COUNT_COLD,                             //холодная вода
    COUNT_HOT,                              //горячая вода
    COUNT_FILTER                            //питьевая вода
 } CountType;       
        
//Состояние датчика утечки      
typedef enum {      
    LEAK_NO,                                //утечки нет
    LEAK_YES                                //есть утечка
 } LeakStat;        
        
//Контроль напряжения 12VDC     
typedef enum {      
    DC12V_ERROR,                            //напряжение отсутствует
    DC12V_OK                                //напряжение есть
 } DC12VStat;       
        
//Тип датчика утечки        
typedef enum {      
    LEAK_ALL,                               //датчик 1+2
    LEAK1,                                  //датчик 1
    LEAK2                                   //датчик 2
 } LeakType;        
        
//Тип данных        
typedef enum {      
    EVENT_DATA,                             //интервальные данные
    EVENT_ALARM                             //событие утечки
 } EventType;       
        
//Тип отображения данных        
typedef enum {      
    OUT_DATA,                               //вывод текущих данных
    OUT_LOG                                 //вывод журнальных данных
 } OutType;

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void WaterData( void *ptr, OutType mode );
void LeakData( void *ptr );

#endif 
