/*Author: Alexandre Boyer
 * Date: 14 Aug. 2024
 */
/* GPIO config.
 */

#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_system.h"

//#include "options.h"
#include "gpio.h"



void GPIO_init(void)
{
	  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
	  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
	  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

	  /**/
	  LL_GPIO_ResetOutputPin(GPIOA, nRF_CSN_Pin|nRF_CE_Pin);

	  /**/
	  LL_GPIO_ResetOutputPin(User_LED_GPIO_Port, User_LED_Pin);

	  /**/
	  LL_GPIO_ResetOutputPin(Clock_Monitor_GPIO_Port, Clock_Monitor_Pin);

	  /**/
	  GPIO_InitStruct.Pin = User_Button_Pin;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(User_Button_GPIO_Port, &GPIO_InitStruct);

	  /**/
	  GPIO_InitStruct.Pin = nRF_CSN_Pin|nRF_CE_Pin;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /**/
	  GPIO_InitStruct.Pin = User_LED_Pin;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(User_LED_GPIO_Port, &GPIO_InitStruct);

	  /**/
	  GPIO_InitStruct.Pin = Clock_Monitor_Pin;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(Clock_Monitor_GPIO_Port, &GPIO_InitStruct);

	  //testing pin config
	  GPIO_InitStruct.Pin = NEW_PIN;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(NEW_PIN_Port, &GPIO_InitStruct);
	  /**/
	  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE0);

	  /**/
	  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
	  EXTI_InitStruct.Line_32_63 = LL_EXTI_LINE_NONE;
	  EXTI_InitStruct.LineCommand = ENABLE;
	  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
	  LL_EXTI_Init(&EXTI_InitStruct);

	  /**/
	  LL_GPIO_SetPinPull(nRF_IRQ_GPIO_Port, nRF_IRQ_Pin, LL_GPIO_PULL_UP);

	  /**/
	  LL_GPIO_SetPinMode(nRF_IRQ_GPIO_Port, nRF_IRQ_Pin, LL_GPIO_MODE_INPUT);

	  /* EXTI interrupt init*/
	  NVIC_SetPriority(EXTI0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	  NVIC_EnableIRQ(EXTI0_IRQn);
}


void LED_GREEN( int val )
{
	if	( val )
		LL_GPIO_SetOutputPin(User_LED_GPIO_Port, User_LED_Pin);
	else	LL_GPIO_ResetOutputPin(User_LED_GPIO_Port, User_LED_Pin);
}

int BLUE_BUTTON()
{
	return ( !LL_GPIO_IsInputPinSet(User_Button_GPIO_Port, User_Button_Pin) );
}

//testing pin fonction
void SetPinForDuration(GPIO_TypeDef *port, uint32_t pin, uint32_t duration_ms)
{
    LL_GPIO_SetOutputPin(port, pin);     // Set pin high
    LL_mDelay(duration_ms);              // Wait for the specified duration
    LL_GPIO_ResetOutputPin(port, pin);   // Set pin low
}


