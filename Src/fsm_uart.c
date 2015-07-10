/**
 fsm_uart.c
 Copyright (C) 2014 Aethylic
 $Id: fsm_uart.c,v 0.01 2014/08/26

*/

/* Includes ------------------------------------------------------------------*/
#include "fsm.h"
#include "fsm_uart.h"
#include "usart.h"
#include "rf22.h"
#include "fsm_rf22_defs.h"

/* Macro ---------------------------------------------------------------------*/
#define SBUS_DELAY_STD         14 //standart
#define SBUS_DELAY_HS          7  //high speed

/* External variables --------------------------------------------------------*/
extern __IO uint8_t packet_sbus[PKT_LEN];
/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
__IO uint8_t state_fsm_uart;

/*function implementations ---------------------------------------------------*/

/**----------------------------------------------------------------------------
 * @brief  init finite state machine uart
 * @param  none
 * @retval none
 -----------------------------------------------------------------------------*/
void fsm_uart_init(void) {
	state_fsm_uart=FSM_UART_READY;
	ResetTimer(SBUS_TIMER);
}

/**----------------------------------------------------------------------------
 * @brief  init finite state machine uart
 * @param  none
 * @retval none
 -----------------------------------------------------------------------------*/
void fsm_uart(void) {
	static HAL_StatusTypeDef uart_status;
	
	switch (state_fsm_uart) {
		case FSM_UART_READY:
		  if(GetMessage(MSG_UART_SBUS)) {
				state_fsm_uart=FSM_UART_TX;
			}
		  break;
		case FSM_UART_BUSY:
		  if(GetMessage(MSG_UART_TX_END)) {
				state_fsm_uart=FSM_UART_READY;
			}
		  break;
		case FSM_UART_TX:
			if(GetTimer(SBUS_TIMER)>=SBUS_DELAY_STD) {
				uart_status=HAL_UART_Transmit_DMA(UART_HANDLE, (uint8_t *)packet_sbus, PKT_LEN);
				switch(uart_status) {
					case HAL_OK:
						state_fsm_uart=FSM_UART_BUSY;
					  break;
					default:
						state_fsm_uart=FSM_UART_FAIL;
					  break;
				}
				ResetTimer(SBUS_TIMER);
			}
		  break;
		case FSM_UART_FAIL:
		  break;
		default:
		  state_fsm_uart=FSM_UART_FAIL;
		  break;
	}
	
}

/**----------------------------------------------------------------------------
 * @brief  change state fsm uart after DMA transmit end
 * @param  huart: uart handle
 * @retval none
 -----------------------------------------------------------------------------*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	SendMessageWOParam(MSG_UART_TX_END);
	//state_fsm_uart=FSM_UART_READY;
}

