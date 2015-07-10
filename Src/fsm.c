/*
 * FSM.c
 *
 *  Created on:
 *      Author: aethylic
 */

 #include "fsm.h"
 #include "fsm_rf22_defs.h"
 #include "fsm_uart_defs.h"
 //#include "inc\FSM_onewire.h"

 __IO uint8_t FSM_state=0;


 void fsm_init(void) {
 //
 FSM_state=FSM_STOP;
 ResetTimer(FSM_TIMER1);
 }

void fsm_main(void) {
//
//uint8_t CMD_message;

switch(FSM_state){
case FSM_STOP:
    //
    ResetTimer(FSM_TIMER1);
    //CMD_message=CMD_DS_INIT;
    //SendMessageWParam(MSG_DS, &CMD_message);
    FSM_state=FSM_START;
    break;
case FSM_START:
    //
    if(GetTimer(FSM_TIMER1)>=1000) {
        //Serial.println(GetTimer(FSM_TIMER1));
        ResetTimer(FSM_TIMER1);
        //CMD_message=CMD_DS_TCONV;
        //SendMessageWParam(MSG_DS, &CMD_message);
    }
    break;

default:
    FSM_state=FSM_STOP;
    break;

}
}
