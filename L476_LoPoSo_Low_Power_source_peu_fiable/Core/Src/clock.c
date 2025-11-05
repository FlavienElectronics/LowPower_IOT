/*Author: Alexandre Boyer
 * Date: 14 Aug. 2024
 */
/* Clock config.
 */


#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_rtc.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_utils.h"
#include "clock.h"

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 4
  */
void SystemClock_Config_80M()
{
	/* MSI configuration and activation */
	LL_RCC_MSI_Enable();			// normalement il est deja enabled
	while	(LL_RCC_MSI_IsReady() != 1)	// c'est pour le cas ou on l'aurait change
		{ }

	LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);	// 4 pour 80MHz
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1); //réglage tension régulateur interne

	// demarrer la PLL principale 4MHz --> 80 MHz
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2);
	LL_RCC_PLL_Enable();
	LL_RCC_PLL_EnableDomain_SYS();
	while	( LL_RCC_PLL_IsReady() != 1 )
		{ }

	// connecter Sysclk sur cette PLL
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	while	( LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL )
		{ }

	/* Set APB1 & APB2 prescaler*/
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

	//update global variable SystemCoreClock --> give access to CPU clock frequency.
	LL_SetSystemCoreClock(80000000);
}


// config systick avec interrupt. L'argument periode_en_ticks indique la période de débordement
//du Systick, donnée en nombre de périodes du buc clock.
void mySystick( unsigned int periode_en_ticks )
{
	// periode
	SysTick->LOAD  = periode_en_ticks - 1;

	// priorite
	NVIC_SetPriority( SysTick_IRQn, 7 );
	// init counter
	SysTick->VAL = 0;
	// prescale (0 ===> %8)
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;
	// enable timer, enable interrupt
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

