/*Author: Alexandre Boyer
 * Date: 14 Aug. 2024
 */
/* Project L476_nRF24L01_MultiCeiver_PTX for STM32L476 mounted on Nucleo board:
 * Demo code for 6:1 Multiceiver network configuration, based on the transceiver Nordic nRF24L01+.
 * The link layer protocol Enhanced ShockBurst (ESB) is used, with automatic acknowledgment.
 * In this network, up to 6 primary transmitters (PTX) can transmit data to 1 primary receiver (PRX)
 * on the same RF channel and 6 logical channels (or data pipes) with unique addresses (known at PTX
 * and PRX side).
 * This project configures the PTX node. It transmits every 2 seconds a packet. The transmission is
 * synchronized on Systick interrupt
 *
 * The clock configuration is the default one (Sysclk = 80 MHz, derived from MSI and PLL).
 * Configured GPIO (even if not used in this project):
 *  > User_LED : PB13
 *  > User_Button (Blue Button) : PC13
 *  > SPI1 to communicate with transceiver nRF24L01 (data rate = 5 Mbps):
 *  	> SPI1_SCK : PA5
 *  	> SPI1_MISO : PA6
 *  	> SPI1_MOSI : PA7
 *  	> NSS not controlled by the SPI1 peripheral but software-controlled (pin nRF_CSN)
 *  > nRF_CSN : PA4 (nRF24L01 SPI Chip Select)
 *  > nRF_CE : PA8 (nRF24L01 Chip Enable pin - TX/RX mode)
 *  > nRF_IRQ : PB0 (EXTI0) (nRF24L01 interrupt pin - not used in this lab)
 *  > USART2_TX : PA2
 *  > USART2_RX : PA3
 */

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */



#include "main.h"
#include "gpio.h"
#include "clock.h"
#include "spi.h"
#include "usart.h"
#include "RadioFunctions.h"
#include "nrf24.h"

#include "stm32l4xx_ll_rtc.h"

#define taille_message 20

uint8_t cptr = 0;
uint8_t cptr_transmit = 1;
uint8_t period_transmit = 5; //10; //période de retransmission du message en multiples de périodes de 100 ms (période
//de débordement du Systick)

volatile unsigned int ticks = 0; //pour la gestion des intervalles de temps. 1 tick = 10 ms.
volatile int blue_mode = 0; //pour savoir si on est dans le mode "Blue mode"
volatile int old_blue = 0;
uint32_t expe = 0; //pour la sauvegarde du numéro de l'expérience

const char entete_message[17] = "Emission paquet #";
uint8_t Message[taille_message];
uint8_t channel_nb = 60; //n° du canal radio utilisé (//channel 60 --> 2460 MHz)
uint8_t adr_data_pipe_used = 1; //numéro du data pipe utilisé pour la transmission (de 0 à 5)


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



void _flavien_MSI_4Mhz(void)
{
	// ATTENTION A NE PAS UTILISER LA PLL LORS DU CHANGEMENT !!!
	RCC->CR |= (0x1); // MSION = 1 (MSI oscilator on)
	while( !((RCC->CR >> 1) & 0x1) ); // Wait for MSIRDY = 1 (MSI oscillator ready)
	RCC->CR &= ~(0xF << 4); // MSIRANGE
	RCC->CR |= 0x6 << 4; 	// 4Mhz
	RCC->CR |= 0x1 << 3;	//MSIREGSEL = 1
	RCC->CFGR &= ~(0x3); //  00: MSI oscillator used as system clock
	while( (RCC->CFGR >> 2 & 0x3) != 0x0); //Set and cleared by hardware to indicate which clock source is used as system clock.
}

void _flavien_MSI_24Mhz(void)
{
	// ATTENTION A NE PAS UTILISER LA PLL LORS DU CHANGEMENT !!!
	RCC->CR |= (0x1); // MSION = 1 (MSI oscilator on)
	while( !((RCC->CR >> 1) & 0x1) ); // Wait for MSIRDY = 1 (MSI oscillator ready)
	RCC->CR = ((RCC->CR & ~(0xF << 4)) | (0x9 << 4)); // MSIRANGE = 1001b = 0x9 = 24Mhz
	RCC->CR |= (0x1 << 3); // MSIREGSEL = 1
	RCC->CFGR &= ~(0x3); //  00: MSI oscillator used as system clock
	while( (RCC->CFGR >> 2 & 0x3) != 0x0); //Set and cleared by hardware to indicate which clock source is used as system clock.
}

void _flavien_PLL_80Mhz(void)
{
	RCC->CFGR &= ~(0x3); //  00: MSI oscillator used as system clock
	while( (RCC->CFGR >> 2 & 0x3) != 0x0); //Set and cleared by hardware to indicate which clock source is used as system clock.
	// Disable the PLL by setting PLLON to 0
	RCC->CR &= ~(0x1 << 24);
	// Wait until PLLRDY is cleared. The PLL is now fully stopped
	while((RCC->CR >> 25) & 0x1);
	// Change the desired parameter
	RCC->PLLCFGR &= ~(0x7F << 8); // clearing PLLN
	RCC->PLLCFGR |= (20 << 8); 	// PLLN = 20

	RCC->PLLCFGR &= ~(0x7 << 4); // clearing PLLM (PLLM = 1)
	// Enable the PLL again by setting PLLON to 1.
	RCC->CR |= 0x1 << 24;
	// Enable the desired PLL outputs by configuring PLLPEN, PLLQEN, PLLREN
	RCC->PLLCFGR |= 0x1 << 24; // PLLREN = 1 (PLLCLK output enable)
	while ( ((RCC->CR >> 25) & 0x1) != 0x1) ; // wait for the PLL to be locked

	RCC->CFGR |= 0x3; //   11: PLL selected as system clock
	while( ((RCC->CFGR >> 2) & 0x3) != 0x3); //Set and cleared by hardware to indicate which clock source is used as system clock.
}

void _flavien_PLL_off(void)
{
	RCC->CFGR &= ~(0x3); //  00: MSI oscillator used as system clock
	while( (RCC->CFGR >> 2 & 0x3) != 0x0); //Set and cleared by hardware to indicate which clock source is used as system clock.
	// Disable the PLL by setting PLLON to 0
	RCC->CR &= ~(0x1 << 24); // PLLON = 0
	// Wait until PLLRDY is cleared. The PLL is now fully stopped
	while((RCC->CR >> 25) & 0x1); // PLLRDY = 0
}

//for 80Mhz frequency
void _flavien_voltage_scaling_1(void)
{
	// Program the VOS bits to “01” in the PWR_CR1 register.
	PWR->CR1 = (PWR->CR1 & ~(0x3 << 9)) | (0x1 << 9); // clearing VOS field + setting the value 01 in VOS field
	// Wait until the VOSF flag is cleared in the PWR_SR2 register.
	while ( ( (PWR->SR2 >> 10) & 0x1) != 0); // waiting for the voltage range to be set (VOSF = 1)
	// Adjust number of wait states according new frequency target in Range 1 (LATENCY bits in the FLASH_ACR).
	FLASH->ACR &= ~(0x7); // clearing LATENCY field
	FLASH->ACR |= 0x4; // setting LATENCY field to 100b (four wait state) according that 80MHz is the frequency
	// Increase the system frequency...
}

//for 80Mhz frequency
void _flavien_voltage_scaling_2(void)
{
	// Program the VOS bits to “10” in the PWR_CR1 register.
	PWR->CR1 = (PWR->CR1 & ~(0x3 << 9)) | (0x2 << 9); // VOS = 10b = range 2
	// Wait until the VOSF flag is cleared in the PWR_SR2 register.
	while ( ( (PWR->SR2 >> 10) & 0x1) != 0); // waiting for the voltage range to be set (VOSF = 0 -> OK)
	// Adjust number of wait states according new frequency target in Range 1 (LATENCY bits in the FLASH_ACR).
	FLASH->ACR &= ~(0x7); // clearing LATENCY field
	FLASH->ACR |= 0x4; // setting LATENCY field to 100b (four wait state) according that 80MHz is the frequency
	// Increase the system frequency...
}

void _flavien_flash_latency(uint8_t latency)
{
	if (latency > 4) return;
	FLASH->ACR &= ~(0x7); // clearing LATENCY field
	FLASH->ACR |= latency;
}

void _flavien_calibration_MSI_vs_LSE(void)
{
	RCC->BDCR |= 0x1; // LSE ocillator ON (LSEON)
	while ( !((RCC->BDCR >> 1) & 0x1) ); // waiting for LSERDY to be == 1
	RCC->CR |= 0x1 << 2; // MSIPLLEN = 1
}

/*Fonction inutile probablement... */
void _flavien_calibration_MSI_vs_LSE_off(void)
{
	RCC->BDCR &= ~(0x1); // LSE ocillator OFF (clear LSEON)
	RCC->CR &= ~(0x1 << 2); // Clearing MSIPLLEN
}

void _flavien_sleep_100Hz_ON(void)
{
	__WFI();
}

/* mode = 0 : Stop 0 mode
* mode = 1 : Stop 1 mode
* mode = 2 : Stop 2 mode
* mode = 3 : Standby mode
* mode = 4 : Shutdown mode
* */
void _flavien_set_stop_mode(uint8_t mode)
{
	if (mode > 4)return;
	PWR->CR1 &= ~(0x7); // Clearing the LPMS field
	if ( ( (PWR->CR1 >> 14) & 0x1 ) && ( mode == 2 ) ) // Checking if LPR = 1 and mode 2 selected
	{
		mode = 1;	//  If LPR bit is set, Stop 2 mode cannot be selected and Stop 1 mode shall be entered instead of Stop 2.
	}
	PWR->CR1 |= mode; // Setting the selected mode into LPMS field
}

void Button_EXTI_Config(void)
{
    // 1. Activer l’horloge GPIOC
    //LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

    // 2. Configurer PC13 en entrée pull-up (souvent bouton user sur Nucleo / STM32 boards)
    //LL_GPIO_SetPinMode(GPIOC, User_Button_Pin, LL_GPIO_MODE_INPUT);
    //LL_GPIO_SetPinPull(GPIOC, User_Button_Pin, LL_GPIO_PULL_UP);

    // 3. Configurer EXTI ligne 13 pour PC13
    //LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

    // Mapper EXTI13 à PC13
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);

    // Configurer EXTI13 interruption front descendant
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_13);
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_13);
    LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_13);

    // 4. Configurer NVIC (EXTI15_10 gère les lignes 10 à 15)
    NVIC_SetPriority(EXTI15_10_IRQn, 2);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}


int main(void)
{

  /*clock domains activation*/
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  // config GPIO
  GPIO_init();
  //config clock
  SystemClock_Config_80M();
  //config bus SPI1 (pour la communication avec le transceiver nRF24L01)
  SPI1_Init();
  //config USART2
  USART2_Init();

  //configuration du transceiver en mode PTX
  Init_Transceiver();
  Config_RF_channel(channel_nb,nRF24_DR_250kbps,nRF24_TXPWR_6dBm);
  Config_CRC(CRC_Field_On, CRC_Field_1byte);
  //Adresse sur 5 bits. Transmission sur le data pipe adr_data_pipe_used.
  Config_PTX_adress(5,Default_pipe_address,adr_data_pipe_used,nRF24_AA_ON);
  Config_ESB_Protocol(nRF24_ARD_1000us,10);
  //on sort du mode power down
  nRF24_SetPowerMode(nRF24_PWR_UP);
  Delay_ms(2); //Attente 2 ms (1.5 ms pour la sortie du mode power down).

  //Entrée en mode TX
  nRF24_SetOperationalMode(nRF24_MODE_TX);
  StopListen();


  // config systick avec interrupt
  mySystick( SystemCoreClock / 100 );	// 100 Hz --> 10 ms

  Button_EXTI_Config();

  expe = LL_RTC_BAK_GetRegister(RTC, 0);
  if (expe == 0){
	  expe = 1;
	  LL_RTC_BAK_SetRegister(RTC, 0, expe);
  }

  if (BLUE_BUTTON()){
	  expe += 1;
	  if (expe > 8){
		  expe = 1;
	  }
	  LL_RTC_BAK_SetRegister(RTC, 0, expe);
  }

  switch (expe) {	//FLAVIEN LE TROUBLE
    	  case 1:
    		  // Code pour expe == 1
    		  /*
    		   * MSI = 4 Mhz
    		   * PLL = 80 Mhz
    		   * Voltage scaling = 1
    		   * Flash latency = 4
    		   * Calibration MSI vs LSE = off
    		   * Sleep (100Hz) = off -> on (when blue button)
    		   * Transceiver = Stand-by I
    		   */
    		_flavien_MSI_4Mhz();
    		_flavien_PLL_80Mhz();
    		_flavien_voltage_scaling_1();
    		_flavien_flash_latency(4);
    		_flavien_calibration_MSI_vs_LSE_off();
    		//Sleep OK dans la routine d'interruption
    		// Transceiver ?
    		Exit_Sleep_100Hz();

    		  break;
    	  case 2:
    		// Code pour expe == 2
    		  /*
    		   * MSI = 24 Mhz
    		   * PLL = off
    		   * Voltage scaling = 1
    		   * Flash latency = 1
    		   * Calibration MSI vs LSE = off -> on (when blue button)
    		   * Sleep (100Hz) = off
    		   * Transceiver = Stand-by I
    		   */
    		_flavien_PLL_off(); // A faire avant de modifier MSI
    		_flavien_MSI_24Mhz();
    		_flavien_voltage_scaling_1();
    		_flavien_flash_latency(1);
    		// Calibration OK dans la routine d'interruption
    		// Pas de sleep
    		// Transceiver ?
    		Exit_Sleep_100Hz();


    		  break;
    	  case 3:
    		  // Code pour expe == 3
    		  /*
    		   * MSI = 24 Mhz
    		   * PLL = off
    		   * Voltage scaling = 2
    		   * Flash latency = 3
    		   * Calibration MSI vs LSE = off
    		   * Sleep (100Hz) = off -> on (when blue button)
    		   * Transceiver = Stand-by I
    		   */
    		_flavien_PLL_off(); // A faire avant de modifier MSI
    		_flavien_MSI_24Mhz();
    		_flavien_voltage_scaling_2();
    		_flavien_flash_latency(3);
    		_flavien_calibration_MSI_vs_LSE_off();
    		//Sleep OK dans la routine d'interruption
    		// Transceiver ?
    		Exit_Sleep_100Hz();

    		  break;
    	  case 4:
    		  // Code pour expe == 4
    		  /*
    		   * MSI = 24 Mhz
    		   * PLL = off
    		   * Voltage scaling = 2
    		   * Flash latency = 3
    		   * Calibration MSI vs LSE = off -> on (when blue button)
    		   * Sleep (100Hz) = off
    		   * Transceiver = Stand-by I
    		   */
    		_flavien_PLL_off();
    		_flavien_MSI_24Mhz();
    		_flavien_voltage_scaling_2();
    		_flavien_flash_latency(3);
    		// Calibration OK dans la routine d'interruption
    		// Transceiver ?
    		Exit_Sleep_100Hz();

    		  break;
    	  case 5:
    		  // Code pour expe == 5
    		  /*
    		   * MSI = 24 Mhz
    		   * PLL = off
    		   * Voltage scaling = 2
    		   * Flash latency = 3
    		   * Calibration MSI vs LSE = on
    		   * Sleep (100Hz) = on
    		   * STOP0, wakeup 7s (when blue button)
    		   * Transceiver = Power-down
    		   */
      		_flavien_PLL_off();
      		_flavien_MSI_24Mhz();
      		_flavien_voltage_scaling_2();
      		_flavien_flash_latency(3);
      		Enable_MSI_LSE_Calibration();
      		// Transceiver ?
      		// Stop mode dans la routine d'interruption
      		Enter_Sleep_100Hz();


    		  break;
    	  case 6:
    		  // Code pour expe == 6
    		  /*
    		   * MSI = 24 Mhz
    		   * PLL = off
    		   * Voltage scaling = 2
    		   * Flash latency = 3
    		   * Calibration MSI vs LSE = on
    		   * Sleep (100Hz) = on
    		   * STOP1, wakeup 7s (when blue button)
    		   * Transceiver = Power-down
    		   */
    		_flavien_PLL_off();
    		_flavien_MSI_24Mhz();
    		_flavien_voltage_scaling_2();
    		_flavien_flash_latency(3);
    		Enable_MSI_LSE_Calibration();
    		// Transceiver ?
    		// Stop mode dans la routine d'interruption
    		Enter_Sleep_100Hz();

    		  break;
    	  case 7:
    		  // Code pour expe == 7
    		  /*
    		   * MSI = 24 Mhz
    		   * PLL = off
    		   * Voltage scaling = 2
    		   * Flash latency = 3
    		   * Calibration MSI vs LSE = on
    		   * Sleep (100Hz) = on
    		   * STOP2, wakeup 7s (when blue button)
    		   * Transceiver = Power-down
    		   */
      		_flavien_PLL_off();
      		_flavien_MSI_24Mhz();
      		_flavien_voltage_scaling_2();
      		_flavien_flash_latency(3);
      		Enable_MSI_LSE_Calibration();
      		// Transceiver ?
      		// Stop mode dans la routine d'interruption
      		Enter_Sleep_100Hz();
    		  break;
    	  case 8:
    		  // Code pour expe == 8
    		  /*
    		   * MSI = 24 Mhz
    		   * PLL = off
    		   * Voltage scaling = 2
    		   * Flash latency = 3
    		   * Calibration MSI vs LSE = on
    		   * Sleep (100Hz) = on
    		   * SHUTDOWN, wakeup 7s (when blue button)
    		   * Transceiver = Power-down
    		   */
    		_flavien_PLL_off();
    		_flavien_MSI_24Mhz();
    		_flavien_voltage_scaling_2();
    		_flavien_flash_latency(3);
    		Enable_MSI_LSE_Calibration();
    		// Transceiver ?
    		// Stop mode dans la routine d'interruption
    		Enter_Sleep_100Hz();
    		  break;
    	  default:
    		  // Code si expe n’est pas entre 1 et 8
    		  break;

  while(1) {

  }

}




// systick interrupt handler --> transmission d'une nouvelle trame toutes les 2 s
//comme l'interuption a lieu toutes les 100 ms, on ajoute un compteur pour transmettre
//quand le compteur atteint 20.
void SysTick_Handler()
{
	uint8_t i;

	if (cptr_transmit == period_transmit) {

		//sortie du mode power down
		nRF24_SetPowerMode(nRF24_PWR_UP);
		//Delay_ms(2); //Attente 2 ms (1.5 ms pour la sortie du mode power down.
		//En fait, on attend 200 ms car la base de temps du systick est 100 ms.
		//LE test montre qu'on n'est pas obligé de mettre un délai. Le temps de compléter le tableau
		//Message et le temps de transmettre sur l'UART prend au moins 5 ms.

		//préparation du message à transmettre
		for (i = 0; i < 17; i++) {
			Message[i] = entete_message[i];
		}
		Message[17] = '_';
		Message[18] = cptr;

		Transmit_Message(Message,taille_message);
		cptr++;
		cptr_transmit = 1;

		//retour dans le mode power down
		nRF24_SetPowerMode(nRF24_PWR_DOWN);
	}
	else {
		cptr_transmit ++;
	}
}

}




/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */

}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
