/*Author: Alexandre Boyer
 * Date: 14 Aug. 2024
 */
/* Project L476_LoPoSo_LEDBlink_2043 for STM32L476 mounted on Nucleo board:
 * Starting project for Low Power Software lab (5ESPE) - Version 2024-25.
 * tThe user LED (mounted on pin PB13) is flashed every 2 seconds for 50 ms.
 * The time base is provided by Systick (100 ticks per second), which generates ISR every 10 ms.
 * The clock configuration is the default one (Sysclk = 80 MHz, derived from MSI and PLL).
 * All the GPIO used in the Low Power Software lab has been configures (even if not used in this project):
 *  > User_LED : PB13
 *  > User_Button (Blue Button) : PC13
 *  > Clock_monitor (to monitor bus clock deviation) : PC10
 *  > SPI1 to communicate with transceiver nRF24L01 (data rate = 5 Mbps):
 *  	> SPI1_SCK : PA5
 *  	> SPI1_MISO : PA6
 *  	> SPI1_MOSI : PA7
 *  	> NSS not controlled by the SPI1 peripheral but software-controlled (pin nRF_CSN)
 *  > nRF_CSN : PA4 (nRF24L01 SPI Chip Select)
 *  > nRF_CE : PA8 (nRF24L01 Chip Enable pin - TX/RX mode)
 *  > nRF_IRQ : PB0 (EXTI0) (nRF24L01 interrupt pin - not used in this lab)
 *  > optional: PA2 and PA3 (USART2_TX and USART2_RX) for RF transceiver debug
 *
 */



#include "main.h"
#include "gpio.h"
#include "clock.h"
#include "spi.h"
#include "usart.h"

#include "stm32l4xx_ll_rtc.h"

volatile unsigned int ticks = 0; //pour la gestion des intervalles de temps. 1 tick = 10 ms.
volatile int blue_mode = 0; //pour savoir si on est dans le mode "Blue mode"
volatile int old_blue = 0;
uint32_t expe = 0; //pour la sauvegarde du numéro de l'expérience

void _flavien_MSI_4Mhz(void)
{
	RCC->CR &= ~0x000000F0; // MSIRANGE
	RCC->CR |= 0x6 << 4; 	// 4Mhz
	RCC->CR |= 0x1 << 3;	//MSIREGSEL = 1
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

//for 80Mhz frequency
void _flavien_voltage_scaling_1(void)
{
	// Program the VOS bits to “01” in the PWR_CR1 register.
	PWR->CR1 = (PWR->CR1 & ~(0x3 << 9)) | (0x1 << 9); // clearing VOS field + setting the value 01 in VOS field
	// Wait until the VOSF flag is cleared in the PWR_SR2 register.
	while ( ( (PWR->SR2 >> 10) & 0x1) != 0); // waiting for the voltage range to be set
	// Adjust number of wait states according new frequency target in Range 1 (LATENCY bits in the FLASH_ACR).
	FLASH->ACR &= ~(0x7); // clearing LATENCY field
	FLASH->ACR |= 0x4; // setting LATENCY field to 100b (four wait state) according that 80MHz is the frequency
	// Increase the system frequency...
}

//for 80Mhz frequency
void _flavien_voltage_scaling_2(void)
{
	// Program the VOS bits to “10” in the PWR_CR1 register.
	PWR->CR1 = (PWR->CR1 & ~(0x3 << 9)) | (0x2 << 9);
	// Wait until the VOSF flag is cleared in the PWR_SR2 register.
	while ( ( (PWR->SR2 >> 10) & 0x1) != 0); // waiting for the voltage range to be set
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

void _flavien_sleep_100Hz_ON(void)
{
	__WFI();
}


int main(void)
{

  RCC->CR |= RCC_CR_MSION;		// Activation du MSI

  /*clock domains activation*/
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  LL_PWR_EnableBkUpAccess();

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  // config GPIO
  GPIO_init();
  //config clock
  SystemClock_Config_80M();
  //config bus SPI1 (pour la communication avec le transceiver nRF24L01)
  SPI1_Init();
  //config USART2
  USART2_Init();

  // config systick avec interrupt
  mySystick( SystemCoreClock / 100 );	// 100 Hz --> 10 ms

  Button_EXTI_Config();

  expe = LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR0);
  if (expe == 0){
	  expe = 1;
	  LL_RTC_BAK_SetRegister(RTC, LL_RTC_BKP_DR0, expe);
  }


  if (BLUE_BUTTON()){
	  expe += 1;
	  if (expe > 8){
		  expe = 1;
	  }
	  LL_RTC_BAK_SetRegister(RTC, LL_RTC_BKP_DR0, expe);
  }

  // ATTENTION À SUPPRIMER !!!!!!!!!!!!
  //expe = 1;
  _flavien_MSI_4Mhz();
  _flavien_PLL_80Mhz();
  _flavien_voltage_scaling_1();
  _flavien_flash_latency(4);
  _flavien_calibration_MSI_vs_LSE();


  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!********!!!!!!!!!!!!


//  switch (expe) {	//TEST
//	  case 1:
//		  // Code pour expe == 1
//		  SystemClock_Config_MSI_4M();		// MSI à 4 Mhz
//		  SystemClock_Enable_PLL_80M();		// PLL à 80 Mhz
//		  Configure_VoltageScaling(1);
//		  Configure_FlashLatency(4);
//		  Disable_MSI_LSE_Calibration();
//		  break;
//	  case 2:
//		  SystemClock_Config_MSI_24M();		// MSI à 24 MHz
//		  SystemClock_Disable_PLL();		// PLL désactivée
//		  Configure_VoltageScaling(1);
//		  Configure_FlashLatency(1);
//		  Disable_MSI_LSE_Calibration();
//		  break;
//	  case 3:
//		  // Code pour expe == 3
//		  SystemClock_Config_MSI_24M();		// MSI à 24 MHz
//		  SystemClock_Disable_PLL();		// PLL désactivée
//		  Configure_VoltageScaling(2);
//		  Configure_FlashLatency(3);
//		  Disable_MSI_LSE_Calibration();
//		  break;
//	  case 4:
//		  // Code pour expe == 4
//		  break;
//	  case 5:
//		  // Code pour expe == 5
//		  break;
//	  case 6:
//		  // Code pour expe == 6
//		  break;
//	  case 7:
//		  // Code pour expe == 7
//		  break;
//	  case 8:
//		  // Code pour expe == 8
//		  break;
//	  default:
//		  // Code si expe n’est pas entre 1 et 8
//		  break;
//  }

  while (1)
  {
	  switch (expe)
	  {
		  case 1:
		  if (blue_mode)
		  {
			  _flavien_sleep_100Hz_ON();
		  }
		  break;
	  }

  }
}

// partie commune a toutes les utilisations du wakeup timer
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

// Dans le cas des modes STANDBY et SHUTDOWN, le MPU sera reveille par reset
// causé par 1 wakeup line (interne ou externe) (le NVIC n'est plus alimenté)
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

// wakeup timer interrupt Handler (inutile mais doit etre defini)
void RTC_WKUP_IRQHandler()
{
	LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_20 );
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

// 5. ISR pour EXTI lignes 10 à 15
void EXTI15_10_IRQHandler(void)
{
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13))
    {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);

        // Ici traitement interruption bouton PC13
        if (blue_mode == 0){
        	blue_mode = 1;
        }else{
        	blue_mode = 0;
        }

        switch (expe) {		// changement de config specifique au blu mode
        	case 1:
        		if (blue_mode == 1){
        			Enter_Sleep_100Hz();
        		}else{
        			Exit_Sleep_100Hz();
        		}
        		break;
        	case 2:
        		Exit_Sleep_100Hz();
        		Enable_MSI_LSE_Calibration();
        		break;
        	case 3:
        		if (blue_mode == 1){
					Enter_Sleep_100Hz();
				}else{
					Exit_Sleep_100Hz();
				}
        		break;
        	case 4:
        		Exit_Sleep_100Hz();
        		Enable_MSI_LSE_Calibration();
        		break;
        	case 5:
        		RTC_wakeup_init_from_stop(7);
        		break;
        	case 6:
        		RTC_wakeup_init_from_stop(7);
        		break;
        	case 7:
        		RTC_wakeup_init_from_stop(7);
        		break;
        	case 8:
        		RTC_wakeup_init_from_standby_or_shutdown(7);
        		break;
		    default:
			    // Code si expe n’est pas entre 1 et 8
			    break;
        }

    }
}


// systick interrupt handler --> allumage LED toutes les 2 s pendant 50 ms.
//Scrutation de l'état du bouton bleu  (pas d'action à ce stade).
void SysTick_Handler()
{
	unsigned int subticks;

	//scrutation bouton bleu
	ticks += 1;

	if (ticks % 2){
		// etat bas
		PWM_STATE(0);
	}else{
		// etat haut
		PWM_STATE(1);
	}

	if	( BLUE_BUTTON() )
		{
		if	( old_blue == 0 )
			blue_mode = 1;
		old_blue = 1;
		}
	else 	old_blue = 0;

	//gestion de l'allumage de la LED
	subticks = ticks % 200;
	if	( subticks == 0 )
		LED_GREEN(1);
	else if	( subticks == 15*expe )
		LED_GREEN(0);
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
