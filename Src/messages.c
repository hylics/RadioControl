/*
 * messages.c
 *
 *  Created on: 23.05.2012
 *      Author: aethylic
 */

#include "messages.h"

typedef struct {
	uint8_t Msg;
	void *ParamPtr;
} MSG_DATA;


__IO MSG_DATA Messages[MAX_MESSAGES];

#ifdef USE_BROADCAST_MESSAGES
__IO uint8_t BroadcastMessages[MAX_BROADCAST_MESSAGES];
#endif


void InitMessages(void) {
	//uint8_t i;
	for (uint8_t i=0; i< MAX_MESSAGES; i++) {
		Messages[i].Msg = 0;
	}
#ifdef USE_BROADCAST_MESSAGES
	for (i=0; i<MAX_BROADCAST_MESSAGES; i++) {
		BroadcastMessages[i] = 0;
	}
#endif
}

void SendMessageWParam(uint8_t Msg, void *ParamPtr) {
	if(Messages[Msg].Msg == 0) {
		Messages[Msg].Msg = 1;
		Messages[Msg].ParamPtr = ParamPtr;
	}
}

void SendMessageWOParam(uint8_t Msg) {
	if (Messages[Msg].Msg == 0) {
		Messages[Msg].Msg = 1;
	}
}

uint8_t GetMessage(uint8_t Msg) {
	if (Messages[Msg].Msg == 2) {
		Messages[Msg].Msg = 0;
		return 1;
	}
	return 0;
}

uint8_t * GetMessageParam_8_t(uint8_t Msg) {
	return (uint8_t *)Messages[Msg].ParamPtr;
}

uint16_t * GetMessageParam_16_t(uint8_t Msg) {
	return (uint16_t *)Messages[Msg].ParamPtr;
}

uint32_t * GetMessageParam_32_t(uint8_t Msg) {
	return (uint32_t *)Messages[Msg].ParamPtr;
}



#ifdef USE_BROADCAST_MESSAGES
void SendBroadcastMessage(uint8_t Msg) {
	if (BroadcastMessages[Msg] == 0) {
		BroadcastMessages[Msg] = 1;
	}
}

uint8_t GetBroadcastMessage(uint8_t Msg) {
	if (BroadcastMessages[Msg] == 2) {
		return 1;
	}
	return 0;
}
#endif

void ProcessMessages(void) {
	for (uint8_t i=0; i< MAX_MESSAGES; i++) {
		if (Messages[i].Msg == 2) Messages[i].Msg = 0;
		if (Messages[i].Msg == 1) Messages[i].Msg = 2;
	}
#ifdef USE_BROADCAST_MESSAGES
	for (i=0; i< MAX_BROADCAST_MESSAGES; i++) {
		if (BroadcastMessages[i] == 2) BroadcastMessages[i] = 0;
		if (BroadcastMessages[i] == 1) BroadcastMessages[i] = 2;
	}
#endif
}
