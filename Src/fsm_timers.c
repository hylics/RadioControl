/*
 * sw_timers.c
 *
 *  Created on:
 *      Author: aethylic
 */

#include "fsm_timers.h"

__IO uint32_t fsm_tick;
__IO uint32_t Timers[MAX_TIMERS];

#ifdef USE_GLOBAL_TIMERS
	#define TIMER_STOPPED		  0
	#define TIMER_RUNNING		  1
	#define TIMER_PAUSED		  2

	__IO uint32_t GTimers[MAX_GTIMERS];
	__IO uint8_t GTStates[MAX_GTIMERS];
#endif

/*
 * Even if main program cycle is longer than system timer ticks period all SW timers will
 * be updated correctly by adding value of ticks from HW system timer accessed by *tick
 * pointer. After all SW timers incremented system timer ticks are cleared.
 * It is easy to change clock division ratio for SW timers from HW timer (default is 1:1).
 */
void ProcessTimers(void) {
	uint8_t i = 0;
	uint32_t x = fsm_tick;

	if (x > 0) {
		for (i=0; i<MAX_TIMERS; i++) {
			Timers[i] += x;
#ifdef USE_GLOBAL_TIMERS
			if (GTStates[i] == TIMER_RUNNING) {
				GTimers[i] += x;
			}
#endif
		}
		fsm_tick=0;
	}
}

void InitTimers(void) {
	uint8_t i;
	for (i=0; i<MAX_TIMERS; i++) {
		Timers[i] = 0;
	}
}

uint32_t GetTimer(uint8_t Timer) {
	return Timers[Timer];
}

void ResetTimer(uint8_t Timer) {
	Timers[Timer] = 0;
}

#ifdef USE_GLOBAL_TIMERS
/// Global Timers functions
uint32_t  GetGTimer(uint8_t Timer){
	return GTimers[Timer];
}

void StopGTimer(uint8_t Timer){
	GTStates[Timer] = TIMER_STOPPED;
}
void StartGTimer(uint8_t Timer){
	if (GTStates[Timer] == TIMER_STOPPED) {
		GTimers[Timer] = 0;
		GTStates[Timer] = TIMER_RUNNING;
	}
}
void PauseGTimer(uint8_t Timer) {
	if (GTStates[Timer] == TIMER_RUNNING) {
		GTStates[Timer] = TIMER_PAUSED;
	}
}
void ContinueGTimer(uint8_t Timer) {
	if (GTStates[Timer] == TIMER_PAUSED) {
		GTStates[Timer] = TIMER_RUNNING;
	}
}
/// *************************
#endif
