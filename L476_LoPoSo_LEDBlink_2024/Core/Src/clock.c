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

void SystemClock_Config_80M()   // TEACHER
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

/* Configure system clock to MSI = 4 MHz and switch SYSCLK to MSI */
void SystemClock_Config_MSI_4M(void)
{
	/* Enable MSI */
	LL_RCC_MSI_Enable();
	while (LL_RCC_MSI_IsReady() != 1) { }

	/* Set MSI range to 4 MHz */
	LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6); /* 4 MHz */

	/* Flash latency for low frequency -> lowest latency */
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

	/* Switch system clock to MSI */
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI) { }

	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

	LL_SetSystemCoreClock(4000000);
}


/* Configure system clock to MSI = 24 MHz and switch SYSCLK to MSI */
void SystemClock_Config_MSI_24M(void)
{
	/* Enable MSI */
	LL_RCC_MSI_Enable();
	while (LL_RCC_MSI_IsReady() != 1) { }

	/* Set MSI range to 24 MHz */
	LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_9); /* 24 MHz */

	/* Flash latency: use 1 wait-state for 24 MHz (safe default) */
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

	/* Switch system clock to MSI */
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI) { }

	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

	LL_SetSystemCoreClock(24000000);
}


/* Enable/configure PLL to produce 80 MHz and switch SYSCLK to PLL output.
 * This function configures PLL with MSI as source (MSI must be enabled and set to 4 MHz).
 */
void SystemClock_Enable_PLL_80M(void)
{
	/* Ensure MSI is enabled and set to 4 MHz (PLLVCO input must be in valid range) */
	LL_RCC_MSI_Enable();
	while (LL_RCC_MSI_IsReady() != 1) { }
	LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6); /* 4 MHz */

	/* Flash latency and regulator for 80 MHz */
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

	/* Configure PLL: (MSI / PLLM) * PLLN / PLLR = 80MHz
	 * With MSI=4MHz, PLLM=1, PLLN=40, PLLR=2 -> 4/1*40/2 = 80MHz
	 */
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2);
	LL_RCC_PLL_Enable();
	LL_RCC_PLL_EnableDomain_SYS();
	while (LL_RCC_PLL_IsReady() != 1) { }

	/* Switch system clock to PLL */
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) { }

	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

	LL_SetSystemCoreClock(80000000);
}


/* Switch system clock to MSI (4 MHz) and disable PLL to save power. */
void SystemClock_Disable_PLL(void)
{
	/* Ensure MSI is enabled and set to 4 MHz */
	LL_RCC_MSI_Enable();
	while (LL_RCC_MSI_IsReady() != 1) { }
	LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6); /* 4 MHz */

	/* Switch system clock to MSI before disabling PLL */
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI) { }

	/* Disable PLL domain and PLL itself */
	LL_RCC_PLL_DisableDomain_SYS();
	LL_RCC_PLL_Disable();
	while (LL_RCC_PLL_IsReady() == 1) { }

	/* Restore low flash latency for MSI 4MHz */
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

	LL_SetSystemCoreClock(4000000);
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

