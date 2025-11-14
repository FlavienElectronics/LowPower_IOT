#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_rtc.h"
#include "stm32l4xx_ll_rcc.h"
#include "main.h"

// RTC/wakeup timer
static void RTC_wakeup_init( int delay )
{
	LL_RTC_DisableWriteProtection( RTC );
	LL_RTC_WAKEUP_Disable( RTC );
	while	( !LL_RTC_IsActiveFlag_WUTW( RTC ) )
		{ }
	// connecter le timer a l'horloge 1Hz de la RTC
	LL_RTC_WAKEUP_SetClock( RTC, LL_RTC_WAKEUPCLOCK_CKSPRE );
	// fixer la duree de temporisation
	LL_RTC_WAKEUP_SetAutoReload( RTC, delay );	// 16 bits
	LL_RTC_ClearFlag_WUT(RTC);
	LL_RTC_EnableIT_WUT(RTC);
	LL_RTC_WAKEUP_Enable(RTC);
	LL_RTC_EnableWriteProtection(RTC);
}

// modes STANDBY/SHUTDOWN,  MPU enable sur reset NVIC disable donc sur IT externe
void RTC_wakeup_init_from_standby_or_shutdown( int delay )
{
	RTC_wakeup_init( delay );
	// enable the Internal Wake-up line
	LL_PWR_EnableInternWU();	// ceci ne concerne que Standby et Shutdown, pas STOPx
}

// Dans le cas des modes STOPx, le MPU sera reveille par interruption
// le module EXTI et une partie du NVIC sont encore alimentes
// le contenu de la RAM et des registres étant préservé, le MPU
// reprend l'execution après l'instruction WFI
void RTC_wakeup_init_from_stop( int delay )
{
	RTC_wakeup_init( delay );
	// valider l'interrupt par la ligne 20 du module EXTI, qui est réservée au wakeup timer
	LL_EXTI_EnableIT_0_31( LL_EXTI_LINE_20 );
	LL_EXTI_EnableRisingTrig_0_31( LL_EXTI_LINE_20 );
	// valider l'interrupt chez NVIC
	NVIC_SetPriority( RTC_WKUP_IRQn, 1 );
	NVIC_EnableIRQ( RTC_WKUP_IRQn );
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
