/*
 * messages.h
 *
 *  Created on:
 *      Author: aethylic
 */
#ifndef MESSAGES_H
#define MESSAGES_H

//#ifndef __stdint_h
//#include <stdint.h>
//#endif

#include "stm32f0xx_hal.h"

//#define USE_BROADCAST_MESSAGES

#define MAX_MESSAGES              6

#define MSG_RF22_IT               0
#define MSG_UART                  1
#define MSG_UART_SBUS             2
#define MSG_UART_TX_END           3
#define MSG_KEY                   4
#define MSG_LCD                   5


#ifdef USE_BROADCAST_MESSAGES
	#define MAX_BROADCAST_MESSAGES 	4

	#define B_MSG_1					0
	#define B_MSG_2					1
	#define B_MSG_3					2
	#define B_MSG_4					3
#endif


//prototypes
void InitMessages(void);
void ProcessMessages(void);
void SendMessageWParam(uint8_t Msg, void * ParamPtr);
void SendMessageWOParam(uint8_t Msg);
uint8_t  GetMessage(uint8_t Msg);
uint8_t  *GetMessageParam_8_t(uint8_t Msg);
uint16_t *GetMessageParam_16_t(uint8_t Msg);
uint32_t *GetMessageParam_32_t(uint8_t Msg);



#ifdef USE_BROADCAST_MESSAGES
	void SendBroadcastMessage(uint8_t Msg);
	uint8_t  GetBroadcastMessage(uint8_t Msg);
#endif

#endif // MESSAGES_H
