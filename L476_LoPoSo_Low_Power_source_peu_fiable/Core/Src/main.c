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
#include "RF.h"
#include "stm32l4xx_it.h"
#include "stm32_assert.h"
#include "Mode.h"

volatile unsigned int ticks = 0; //pour la gestion des intervalles de temps. 1 tick = 10 ms.
volatile int blue_mode = 0; //pour savoir si on est dans le mode "Blue mode"
volatile int old_blue = 0;
volatile int expe = 0; //pour la sauvegarde du numéro de l'expérience



int main(void)
{
	if (LL_RCC_LSE_IsReady() == 1) {

	} else {


	}

  blue_mode=0;
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
  // config systick avec interrupt
  mySystick( SystemCoreClock / 100 );	// 100 Hz --> 10 ms
  //Config RCC
  INIT_RCC();



  //incrementation expe
  expe = 0;
  expe = RTC->BKP0R;
  if (RTC->BKP1R)
  {
	  expe = expe+1;
  }
  if (expe > 8 )
  {
	  expe = 1;
  }
  RTC->BKP0R = expe; // changement expe OK
  //liste des expe
  if(expe==1)
  {

  }
  if(expe==1 && blue_mode)
  {
	  expe1_blue_mode();
  }
  if(expe==2)
    {
	  expe2();
    }
  if(expe==3)
    {
	  expe3();
    }
  if(expe==4)
    {
	  expe4();
    }
  if(expe==5)
      {
  	  expe5();
      }



  while (1)
  {
	  if(expe > 4 && expe < 9)
	  {
		  PWR->CR1 &= ~PWR_CR1_LPMS;
		  __WFI();
	  }
	  if(expe==1 && blue_mode)
	  {
		  expe1_blue_mode();
	  }
	  if(expe==2 && blue_mode)
	   {
	  	  expe2_blue_mode();
	   }
	  if(expe==3 && blue_mode)
	  {
	  	  expe3_blue_mode();
	  }
	  if(expe==4 && blue_mode)
	   {
	  	  expe2_blue_mode();
	   }
	  if(expe==5 && blue_mode)
	  {
		 blue_mode=0;
	  	 expe5_blue_mode();
	  	__WFI();//wait for interrupt



	   }
	  		unsigned int subticks;
	  		//gestion de l'allumage de la LED
	  		subticks = ticks % 200;
	  		if	( subticks == 0 )
	  			LED_GREEN(1);
	  		else if	( subticks == 5 )
	  			LED_GREEN(0);
  }
}

// systick interrupt handler --> allumage LED toutes les 2 s pendant 50 ms.
//Scrutation de l'état du bouton bleu  (pas d'action à ce stade).
void SysTick_Handler()
{
	//PWM_50Hz();
	//scrutation bouton bleu
	ticks += 1;
	if	( BLUE_BUTTON() )
		{
		if	( old_blue == 0 )
			blue_mode = 1;
		old_blue = 1;
		}
	else 	old_blue = 0;

	RTC->BKP1R = old_blue;

	/*if (old_blue)
		RTC->BKP1R = 1;
	else
		RTC->BKP1R = 0; */


}

void RTC_Alarm_IRQHandler(void)
{
    if (RTC->ISR & RTC_ISR_ALRAF) // Vérifier si l'alarme A est déclenchée
    {
        RTC->ISR &= ~RTC_ISR_ALRAF; // Effacer le drapeau d'alarme
        EXTI->PR1 |= EXTI_PR1_PIF18; // Effacer le drapeau EXTI
        // Ajouter ici le code pour reprendre l'activité normale

    }
    RTC->BKP0R = 6;
    NVIC_SystemReset(); // Effectue un reset logiciel

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
