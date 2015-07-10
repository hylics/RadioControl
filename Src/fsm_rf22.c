/**
 fsm_rf22.c
 Copyright (C) 2014 Aethylic
 $Id: fsm_rf22.c,v 0.01 2014/08/03

*/


/* Includes ------------------------------------------------------------------*/
#include "fsm.h"
#include "fsm_rf22.h"
#include "fsm_uart_defs.h"
#include "rf22.h"


/* Macro ---------------------------------------------------------------------*/

/* External variables --------------------------------------------------------*/
extern __IO uint16_t it_status;
/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
__IO uint8_t state_fsm_rf22;
__IO uint8_t packet_sbus[PKT_LEN];
//uint8_t packet_sbus[PKT_LEN];
/* Private function prototypes -----------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/

/*function implementations ---------------------------------------------------*/

/**----------------------------------------------------------------------------
 * @brief  init finite state machine rf22 modem
 * @param  none
 * @retval none
 -----------------------------------------------------------------------------*/
void fsm_rf22_init(void) {
	RF22_StatusTypeDef init_status;
	
	init_status=rf22_Init();
	
	if (init_status==RF22_OK) {
		state_fsm_rf22=FSM_RF22_READY;
	}
	else {
		state_fsm_rf22=FSM_RF22_FAIL;
	}
}


/**----------------------------------------------------------------------------
 * @brief  process finite state machine rf22 modem
 * @param  none
 * @retval none
 -----------------------------------------------------------------------------*/
void fsm_rf22(void) {
	RF22_StatusTypeDef status_rf22;
	
	switch(state_fsm_rf22) {
		//uint16_t it_status;
		case FSM_RF22_READY:
			//
		  state_fsm_rf22=FSM_RF22_RX;
		  rf22_RxOn();
		  break;
		case FSM_RF22_RX:
			//
		  if(GetMessage(MSG_RF22_IT)) {
				if (it_status&RF22_IPKVALID) {
					state_fsm_rf22=FSM_RF22_FIFO_READ;
				}
			}
		  break;
		case FSM_RF22_TX:
			//
		  break;
		case FSM_RF22_FIFO_READ:
			//
		  status_rf22=rf22_BrstReadFifo((uint8_t *)packet_sbus, PKT_LEN);
		  switch(status_rf22) {
				case RF22_OK:
					SendMessageWOParam(MSG_UART_SBUS);
					state_fsm_rf22=FSM_RF22_READY;
				  break;
				default:
					state_fsm_rf22=FSM_RF22_FAIL;
					break;
			}
		  //rf22_BrstReadFifo(packet_sbus, PKT_LEN);
		  //SendMessageWParam(MSG_UART, (void *)packet_sbus);
		  break;
		case FSM_RF22_FIFO_WRITE:
			//
		  break;
		case FSM_RF22_FAIL:
			//
		  break;
		default:
			//
		  break;
	}
	
	
}


