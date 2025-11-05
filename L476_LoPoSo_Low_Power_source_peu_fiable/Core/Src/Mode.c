#include "main.h"
#include "gpio.h"
#include "clock.h"
#include "spi.h"
#include "usart.h"
#include "RF.h"
#include "stm32l4xx_it.h"
#include "stm32_assert.h"
#include "stm32l476xx.h"
//#include "stm32l4xx_hal.h"

void PWM_50Hz(void);

void INIT_RCC()
{
  RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
  PWR->CR1 |= PWR_CR1_DBP; // enable writing
  RCC->BDCR |=RCC_BDCR_LSEON;
  RCC->BDCR &= ~RCC_BDCR_RTCSEL;
  RCC->BDCR |= RCC_BDCR_RTCSEL_0;
  RCC->BDCR|= RCC_BDCR_RTCEN;
  PWR->CR1 |= PWR_CR1_VOS_0 ;//mode voltage scaling
  PWM_50Hz();

}

void expe1_blue_mode()
{
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_4WS;// flash latency 4

	 RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

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

	 // Configure NVIC for TIM2 Interrupt
	 NVIC_SetPriority(TIM2_IRQn, 0); // Highest priority
	 NVIC_EnableIRQ(TIM2_IRQn);

	NVIC_SetPriority(TIM2_IRQn, 0);
	NVIC_EnableIRQ(TIM2_IRQn);

	LL_LPM_EnableSleep();
	__WFI();//wait for interrupt

}

void expe2()
{
	//RCC->CFGR &= ~RCC_CFGR_SW; //HSI for system clock
	//while((RCC->CFGR & RCC_CFGR_SWS) != 0){}
	RCC->CR &= RCC_CR_MSION; // MSI ON
	//while(RCC->CR & RCC_CR_MSIRDY){}
	RCC->CR &= ~RCC_CR_MSIRANGE;
	RCC->CR |= RCC_CR_MSIRANGE_9;// Set MSI speed to 24MHz
	RCC->CFGR &= ~RCC_CFGR_SW; // Clear SW bits
	RCC->CFGR |= RCC_CFGR_SW_MSI; // Select MSI as system clock
	RCC->CFGR |=RCC_CFGR_SWS_MSI; // MSI used as system clock
	RCC->CR &= ~RCC_CR_PLLON; // PLL OFF
	//while(RCC->CR & RCC_CR_PLLRDY){}
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_1WS;// flash latency 1

	PWM_50Hz();





}

void expe2_blue_mode()
{
	RCC->BDCR |= RCC_BDCR_LSEON; //LSE EN
	//while(!(RCC->BDCR & RCC_BDCR_LSERDY));
	RCC->CR |= RCC_CR_MSIPLLEN; // Sync MSI on LSE


}

void expe3()
{

	RCC->CR &= RCC_CR_MSION; // MSI ON
	//while(RCC->CR & RCC_CR_MSIRDY){}
	RCC->CR &= ~RCC_CR_MSIRANGE;
	RCC->CR |= RCC_CR_MSIRANGE_9;// Set MSI speed to 24MHz
	RCC->CFGR &= ~RCC_CFGR_SW; // Clear SW bits
	RCC->CFGR |= RCC_CFGR_SW_MSI; // Select MSI as system clock
	RCC->CFGR |=RCC_CFGR_SWS_MSI; // MSI used as system clock
	RCC->CR &= ~RCC_CR_PLLON; // PLL OFF
	//while(RCC->CR & RCC_CR_PLLRDY){}
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_3WS; //flash latency 3
	PWR->CR1 |= PWR_CR1_VOS_1 ;//mode voltage scaling one (partially degraded)

}

void expe3_blue_mode()
{
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_4WS;// flash latency 4

	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

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

	// Configure NVIC for TIM2 Interrupt
	NVIC_SetPriority(TIM2_IRQn, 0); // Highest priority
	NVIC_EnableIRQ(TIM2_IRQn);

	NVIC_SetPriority(TIM2_IRQn, 0);
	NVIC_EnableIRQ(TIM2_IRQn);

	LL_LPM_EnableSleep();
	__WFI();
}

void expe4()
{
	    RCC->CR &= RCC_CR_MSION; // MSI ON
		//while(RCC->CR & RCC_CR_MSIRDY){}
		RCC->CR &= ~RCC_CR_MSIRANGE;
		RCC->CR |= RCC_CR_MSIRANGE_9;// Set MSI speed to 24MHz
		RCC->CFGR &= ~RCC_CFGR_SW; // Clear SW bits
		RCC->CFGR |= RCC_CFGR_SW_MSI; // Select MSI as system clock
		RCC->CFGR |=RCC_CFGR_SWS_MSI; // MSI used as system clock
		RCC->CR &= ~RCC_CR_PLLON; // PLL OFF
		//while(RCC->CR & RCC_CR_PLLRDY){}
		FLASH->ACR &= ~FLASH_ACR_LATENCY;
		FLASH->ACR |= FLASH_ACR_LATENCY_3WS;// flash latency 1
		PWR->CR1 |= PWR_CR1_VOS_1 ;//mode voltage scaling one (partially degraded)
		PWM_50Hz();
}
void expe5()
{
	    RCC->CR &= RCC_CR_MSION; // MSI ON
		//while(RCC->CR & RCC_CR_MSIRDY){}
		RCC->CR &= ~RCC_CR_MSIRANGE;
		RCC->CR |= RCC_CR_MSIRANGE_9;// Set MSI speed to 24MHz
		RCC->CFGR &= ~RCC_CFGR_SW; // Clear SW bits
		RCC->CFGR |= RCC_CFGR_SW_MSI; // Select MSI as system clock
		RCC->CFGR |=RCC_CFGR_SWS_MSI; // MSI used as system clock
		RCC->CR &= ~RCC_CR_PLLON; // PLL OFF
		//while(RCC->CR & RCC_CR_PLLRDY){}
		FLASH->ACR &= ~FLASH_ACR_LATENCY;
		FLASH->ACR |= FLASH_ACR_LATENCY_3WS;// flash latency 1
		PWR->CR1 |= PWR_CR1_VOS_1 ;//mode voltage scaling one (partially degraded)
		RCC->BDCR |= RCC_BDCR_LSEON; //LSE EN
		//while(!(RCC->BDCR & RCC_BDCR_LSERDY));
		RCC->CR |= RCC_CR_MSIPLLEN; // Sync MSI on LSE
		FLASH->ACR &= ~FLASH_ACR_LATENCY;
		FLASH->ACR |= FLASH_ACR_LATENCY_4WS;// flash latency 4
		LL_LPM_EnableSleep();
		//__WFI();//wait for interrupt

}

void expe5_blue_mode(void)
{



		    //Config RTC ---------------------------------------------------------------------------------
		      // Activer l'accès aux registres sauvegardés
		      RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN; // Activer l'horloge pour PWR
		      PWR->CR1 |= PWR_CR1_DBP;             // Activer l'accès aux registres sauvegardés
		      // Activer LSE (oscillateur 32.768 kHz)
		      RCC->BDCR |= RCC_BDCR_LSEON;
		      while (!(RCC->BDCR & RCC_BDCR_LSERDY)); // Attendre que LSE soit prêt
		      // Sélectionner LSE comme source d'horloge pour le RTC
		      RCC->BDCR |= RCC_BDCR_RTCSEL_Msk;
		      RCC->BDCR |= RCC_BDCR_RTCEN; // Activer le RTC


		      //------------------------------------------------------------------------------------------------
		    // Désactiver l'alarme A
		    RTC->CR &= ~RTC_CR_ALRAE;
		    while (!(RTC->ISR & RTC_ISR_ALRAWF)); // Attendre que l'alarme soit prête

		    // Configurer l'alarme A pour se déclencher après 'seconds'
		    RTC->ALRMAR = (7 << RTC_ALRMAR_ST_Pos) | RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK2;

		    // Activer l'interruption pour l'alarme A
		    RTC->CR |= RTC_CR_ALRAIE;

		    // Activer l'alarme A
		    RTC->CR |= RTC_CR_ALRAE;

		    // Configurer EXTI pour le RTC
		    EXTI->IMR1 |= EXTI_IMR1_IM18;   // Activer la ligne 18
		    EXTI->RTSR1 |= EXTI_RTSR1_RT18; // Déclenchement sur front montant
		    EXTI->EMR1 |= EXTI_EMR1_EM18;   // Générer un événement pour réveiller le microcontrôleur
		    // Désactiver la protection en écriture pour le RTC
		    		      RTC->WPR = 0xCA;
		    		      RTC->WPR = 0x53;
		    // Activer l'interruption dans le NVIC
		    NVIC_SetPriority(RTC_Alarm_IRQn, 0); // Priorité maximale
		    NVIC_EnableIRQ(RTC_Alarm_IRQn);      // Activer l'interruption

		    // Mettre le RTC en mode init pour configurer ses registres
		   		      RTC->ISR |= RTC_ISR_INIT;
		   		      while (!(RTC->ISR & RTC_ISR_INITF)); // Attendre le mode init
		   		      // Configurer la division pour une fréquence de 1 Hz
		   		      RTC->PRER = (127 << RTC_PRER_PREDIV_A_Pos) | 255;
		   		      // Sortir du mode init
		   		      RTC->ISR &= ~RTC_ISR_INIT;
		   		      // Réactiver la protection en écriture
		   		      RTC->WPR = 0xFF;
    // Mettre en mode Stop0
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;      // Activer le mode SLEEP profond
    PWR->CR1 &= ~PWR_CR1_LPMS;              // Configurer pour STOP0

}

/**
void expe5_blue_mode()
{

	 // Configurer le RTC pour 7 secondes avec une alarme
		RTC->CR &= ~RTC_CR_ALRAE;               // Désactiver l'alarme A
		//while (!(RTC->ISR & RTC_ISR_ALRAWF));   // Attendre la disponibilité du registre
		RTC->ALRMAR = (7 << RTC_ALRMAR_ST_Pos) | RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK2;
		// MSK4, MSK3, MSK2 masquent les bits des heures, minutes, et dixièmes de seconde
		RTC->CR |= RTC_CR_ALRAE;                // Activer l'alarme A
		RTC->CR |= RTC_CR_ALRAIE; // Activer l'interruption pour l'alarme A
		NVIC_EnableIRQ(RTC_Alarm_IRQn); // Activer le vecteur d'interruption
		EXTI->IMR1 |= EXTI_IMR1_IM18;   // Activer l'EXTI pour l'alarme RTC
		EXTI->RTSR1 |= EXTI_RTSR1_RT18; // Déclencher l'EXTI en montée
		NVIC_EnableIRQ(RTC_Alarm_IRQn);
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; // Activer le mode SLEEP profond
		PWR->CR1 &= ~PWR_CR1_LPMS; //STOP 0

}
*/
void PWM_50Hz()
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; // Clock EN GPIOB

	RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN; // Enable LPTIM1 clock

	//Select MSI as LPTIM1 Clock Source
	RCC->CCIPR &= ~RCC_CCIPR_LPTIM1SEL;    // Clear LPTIM1SEL bits
	RCC->CCIPR |= RCC_CCIPR_LPTIM1SEL_0;

	GPIOB->MODER &= ~GPIO_MODER_MODE6; // Clear PB6 mode
	GPIOB->MODER |= (2 << GPIO_MODER_MODE6_Pos); // Alternate function
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL6); // Clear AF
	GPIOB->AFR[0] |= (2 << GPIO_AFRL_AFSEL6_Pos); // Alternate function 2 (TIM4_CH1)


	// Activation de l'horloge pour TIM4
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN; // Enable TIM4 clock

	// Configuration timer TIM4
	//TIM4->PSC = 2400 - 1; // Prescaler 10 kHz
	//TIM4->ARR = 200 - 1;  // ARR for 50Hz
	//TIM4->CCR1 = 100;     // 50% (ARR * 0.5)
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;
	TIM4->PSC = 400 - 1; // Prescaler 10 kHz
	TIM4->ARR = 200 - 1;  // ARR for 50Hz
	TIM4->CCR1 = 100;     // 50% (ARR * 0.5)

	TIM4->CCMR1 &= ~TIM_CCMR1_OC1M; // Clear OC1M bits
	TIM4->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // PWM mode 1
	TIM4->CCMR1 |= TIM_CCMR1_OC1PE; // Preload enable

	TIM4->CCER |= TIM_CCER_CC1E; //Canal 1 EN

	TIM4->CR1 |= TIM_CR1_CEN; // Enable TIM4


}


