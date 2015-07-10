/*
 * fsm.h
 *
 *  Created on:
 *      Author: aethylic
 */

#ifndef FSM_H
#define FSM_H

#include "messages.h"
#include "fsm_timers.h"

//FSMs' states
#define FSM_START       0
#define FSM_STOP        1
#define FSM_CONV        2
#define FSM_WAIT        3

//FSMs' commands




//prototypes
void fsm_init(void);
void fsm_main(void);

#endif // FSM_H
