/*
 * fsm_timers.h
 *
 *  Created on:
 *  Author: aethylic
 */

#ifndef FSM_TIMERS_H
#define FSM_TIMERS_H

#include "stm32f0xx_hal.h"

#define MAX_TIMERS 3
#define sec 1000 //сколько тиков в секунде
#define minute 60*sec
#define hour 60*minute
#define day 24*hour

//идентификаторы таймеров
#define RF22_TIMER               0
#define SBUS_TIMER               1
#define FSM_TIMER1               2


//#define USE_GLOBAL_TIMERS

#ifdef USE_GLOBAL_TIMERS
	#define MAX_GTIMERS 	4

	#define GTIMER_1		0
	#define GTIMER_2		1
	#define GTIMER_3		2
	#define GTIMER_4		3
#endif

/*Function prototypes*/
//void ProcessTimers(uint8_t * tick);
void ProcessTimers(void);
void InitTimers(void);
uint32_t GetTimer(uint8_t Timer);
void ResetTimer(uint8_t Timer);

#ifdef USE_GLOBAL_TIMERS
	uint32_t  GetGTimer(uint8_t Timer);
	void StopGTimer(uint8_t Timer);
	void StartGTimer(uint8_t Timer);
	void PauseGTimer(uint8_t Timer);
	void ContinueGTimer(uint8_t Timer);
#endif

#endif // FSM_TIMERS_H
