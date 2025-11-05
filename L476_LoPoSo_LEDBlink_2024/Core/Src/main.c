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

  switch (expe) {
	  case 1:
		  // Code pour expe == 1
		  RCC->ICSCR &= ~RCC_ICSCR_MSIRANGE;                 // Clear le champ MSIRANGE
		  RCC->ICSCR |= RCC_ICSCR_MSIRANGE_2 | RCC_ICSCR_MSIRANGE_1;  // 0b0110 = 4 MHz
		  break;
	  case 2:
		  // Code pour expe == 2
		  RCC->ICSCR &= ~RCC_ICSCR_MSIRANGE;
		  RCC->ICSCR |= RCC_ICSCR_MSIRANGE_3 | RCC_ICSCR_MSIRANGE_0;  // 0b1001 = 24 MHz
		  break;
	  case 3:
		  // Code pour expe == 3
		  break;
	  case 4:
		  // Code pour expe == 4
		  break;
	  case 5:
		  // Code pour expe == 5
		  break;
	  case 6:
		  // Code pour expe == 6
		  break;
	  case 7:
		  // Code pour expe == 7
		  break;
	  case 8:
		  // Code pour expe == 8
		  break;
	  default:
		  // Code si expe n’est pas entre 1 et 8
		  break;



  }

  while (1)
  {

  }
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
	else if	( subticks == 5*expe )
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
