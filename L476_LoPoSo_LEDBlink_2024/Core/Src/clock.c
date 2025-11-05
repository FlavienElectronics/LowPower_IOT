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
#include "stm32l4xx_ll_gpio.h"
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

void Configure_VoltageScaling(uint32_t scale)
{
    if (scale == 1)
        LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    else if (scale == 2)
        LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);

    /* Attente de stabilisation si nécessaire */
    while (LL_PWR_IsActiveFlag_VOS() == 0) {}
}

void Configure_FlashLatency(uint32_t latency)
{
    /* Latence de 0 à 7 selon la fréquence CPU */
    LL_FLASH_SetLatency(latency);

    /* Vérification de l’application effective */
    if (LL_FLASH_GetLatency() != latency)
    {
        // Erreur de configuration Flash, ajouter un assert ou gestion d’erreur ici
        while(1);
    }
}

void Enable_MSI_LSE_Calibration(void)
{
    /* Activer la LSE */
    LL_RCC_LSE_Enable();
    while (LL_RCC_LSE_IsReady() != 1) {}

    /* Connecter la MSI sur la LSE pour calibration */
    LL_RCC_MSI_EnablePLLMode(); // permet la correction auto par LSE
}

void Disable_MSI_LSE_Calibration(void)
{
    LL_RCC_MSI_DisablePLLMode();
}

void Enter_Sleep_100Hz(void)
{
    /* Configuration du timer d’interruption (100 Hz = 10 ms) */
    // Hypothèse : un timer (TIMx) ou SysTick est déjà configuré ailleurs
    // pour générer une IRQ toutes les 10 ms.

	/* === Activation horloge du TIM2 === */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

	/* === Configuration du timer TIM2 pour 100 Hz ===
	 * Fréquence MSI = 4 MHz
	 * PSC = 399 → 4 MHz / (399+1) = 10 kHz
	 * ARR = 99  → 10 kHz / (99+1) = 100 Hz
	 */
    // Reset TIM2 configuration
	 TIM2->CR1 = 0;        // Reset Control Register 1
	 TIM2->CR2 = 0;        // Reset Control Register 2
	 TIM2->DIER = 0;       // Disable interrupts
	 TIM2->SR = 0;         // Clear status register

	 TIM2->PSC = (4000000 / 10000) - 1; // Prescaler to get 10 kHz
	 TIM2->ARR = (10000 / 100) - 1; // Auto-reload for 100 Hz

	 // Enable Update Interrupt
	 TIM2->DIER |= TIM_DIER_UIE;

	 // Enable Timer
	 TIM2->CR1 |= TIM_CR1_CEN;

	/* === Configuration NVIC === */
	NVIC_SetPriority(TIM2_IRQn, 0);
	NVIC_EnableIRQ(TIM2_IRQn);

    LL_LPM_EnableSleep();    // Autorise le mode Sleep
    __WFI();                 // Attente d’interruption → Sleep 10 ms
}

void Exit_Sleep_100Hz(void)
{
    /* === Désactivation des interruptions du timer === */
    NVIC_DisableIRQ(TIM2_IRQn);       // Désactive l'IRQ dans le NVIC
    TIM2->DIER &= ~TIM_DIER_UIE;      // Désactive l'interruption d'update TIM2

    /* === Arrêt du compteur TIM2 === */
    TIM2->CR1 &= ~TIM_CR1_CEN;        // Stoppe le timer

    /* === Réinitialisation optionnelle des registres du timer === */
    TIM2->CR1 = 0;
    TIM2->CR2 = 0;
    TIM2->SR = 0;

    /* === Désactivation de l'horloge du timer (optionnel) === */
    LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM2);

    /* === Désactivation du mode Sleep === */
    //            LL_LPM_DesableSleep();
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

