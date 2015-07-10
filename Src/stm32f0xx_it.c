/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @date    03/08/2014 21:24:34
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"
#include "rf22.h"
#include "messages.h"

/* External variables --------------------------------------------------------*/

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
extern __IO uint32_t fsm_tick;
extern __IO uint16_t it_status;


/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles EXTI Line 4 to 15 interrupts.
*/
void EXTI4_15_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(EXTI4_15_IRQn);
  
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/**
* @brief This function handles SPI1 global interrupt.
*/
void SPI1_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(SPI1_IRQn);
  HAL_SPI_IRQHandler(&hspi1);
}

/**
* @brief This function handles DMA1 Channel 4 and Channel 5 Interrupts.
*/
void DMA1_Channel4_5_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(DMA1_Channel4_5_IRQn);
  
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
  HAL_UART_IRQHandler(&huart1);
}

/**
* @brief This function handles Non Maskable Interrupt.
*/
void NMI_Handler(void)
{
  HAL_RCC_NMI_IRQHandler();
}

/******************************************************************************/
/* Implementations callbacks from interrupts handlers   ***********************/
/******************************************************************************/
//weak void HAL_IncTick(void)

/**
* @brief This function increase FSM counter for use in timers. Systick used, independent from using "uwtick"
*/
void HAL_SYSTICK_Callback(void) {
	fsm_tick++;
}

/**
* @brief This function read rf22 interrupt handler when external interrupt on pin PB9 occur and send message to FSM
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin==GPIO_PIN_9) {
		rf22_ItRead(&it_status);
	  SendMessageWParam(MSG_RF22_IT, (void *)&it_status);
	}
}





/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
