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
#include "RadioFunctions.h"
#include "nrf24.h"
#include "rtc.h"
#include "stm32l4xx_ll_rtc.h"

volatile unsigned int ticks = 0;	//1tick = 10 ms.
volatile int blue_mode = 0;			//bouton bleu
volatile int expe = 1;				//nb expérience

uint8_t cptr = 0;
uint8_t cptr_transmit = 1;
uint8_t period_transmit = 200;	//période de retransmission du message en multiples de périodes de 10 ms (période de débordement du Systick)

#define taille_message 32			// Init message
const char entete_message[15] = "BOUT-CARV-LESP-";
uint8_t Message[taille_message];
uint8_t channel_nb = 60;		//n° du canal radio utilisé (//channel 60 --> 2460 MHz)
uint8_t adr_data_pipe_used = 1; //numéro du data pipe utilisé pour la transmission (de 0 à 5)

int main(void)
{
  /*clock domains activation*/
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  LL_PWR_EnableBkUpAccess(); // droit Write

  if (LL_RCC_LSE_IsReady() == 1) { // hot start
	  LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
	  LL_RCC_EnableRTC();
  }
  else // cold start
  {
	  LL_RCC_ForceBackupDomainReset();
	  LL_RCC_ReleaseBackupDomainReset();
	  LL_RCC_LSE_Enable();
	  while (!LL_RCC_LSE_IsReady()) {
	  }
	  LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
	  LL_RCC_EnableRTC();
	  LL_RTC_DisableWriteProtection(RTC);
	  LL_RTC_EnableInitMode(RTC);
	  LL_RTC_SetAsynchPrescaler(RTC,0x7F);		//div 128
	  LL_RTC_SetSynchPrescaler(RTC,0xFF);		//div 256
	  LL_RTC_DisableInitMode(RTC);
	  LL_RTC_EnableWriteProtection(RTC);
  }

  Button_EXTI_Config();	// Création de l'interruption du bouton pour le Blue Mode
  GPIO_init();			// Configuration GPIO
  SPI1_Init();			// Configuration du BUS SPI1

  expe = LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR0);		// Récupération du numéro d'expérience dans le registre RTC

  if (expe < 1 || expe > 8){		// Au cas où l'expérience n'est pas définie correctement
	  expe = 1;
  }

  if (BLUE_BUTTON()){
	expe += 1;
	if (expe == 9){
		expe = 1;
	}
	blue_mode = 0;
  }

    while (BLUE_BUTTON()){}			// Eviter le "saut" du bouton si appuyé
    LL_RTC_BAK_SetRegister(RTC, LL_RTC_BKP_DR0, expe);

    // config clock tree
    if (expe == 1){
	  SystemClock_Config_80M();
    } else if (expe == 2) {
    	SystemClock_2();
    } else if (expe == 3) {
    	SystemClock_3();
    }else if (expe == 4) {
    	SystemClock_4();
    } else {
    	SystemClock_5_6_7_8();
    }

    mySystick( SystemCoreClock / 100);		// Config du Systick with IT

    Init_Transceiver();		//config transceiver / mode PTX
    Config_RF_channel(channel_nb,nRF24_DR_250kbps,nRF24_TXPWR_6dBm);
    Config_CRC(CRC_Field_On, CRC_Field_1byte);
    //Adresse sur 5 bits. Transmission sur le data pipe adr_data_pipe_used.
    Config_PTX_adress(5,Default_pipe_address,adr_data_pipe_used,nRF24_AA_ON);
    Config_ESB_Protocol(nRF24_ARD_1000us,10);
    //on sort du mode power down
    if (expe < 5){
    	nRF24_SetPowerMode(nRF24_PWR_UP);
    	Delay_ms(2); //Attente 2 ms (1.5 ms pour la sortie du mode power down).
    }

    nRF24_SetOperationalMode(nRF24_MODE_TX);		//Entrée en mode TX
    StopListen();

  while (1)
  {

  }
}

void Button_EXTI_Config(void)
{
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_13);
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_13);
    LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_13);
    NVIC_SetPriority(EXTI15_10_IRQn, 2);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void EXTI15_10_IRQHandler(void)
{
	if (blue_mode == 0){
		blue_mode = 1;

	if (blue_mode == 1){
		switch (expe) {
		  case 1:
			  LL_LPM_EnableSleep();
			  blue_mode = 0;
			  __WFI();
			  break;

		  case 2:
			  while (LL_RCC_LSE_IsReady() != 1) {
				  LL_RCC_LSE_Enable();
			  }
			  LL_RCC_MSI_EnablePLLMode();
			  blue_mode = 0;
			  break;

		  case 3:
			  LL_LPM_EnableSleep();
			  blue_mode = 0;
			  __WFI();
			  break;

		  case 4:
			  while (LL_RCC_LSE_IsReady() != 1) {
				  LL_RCC_LSE_Enable();
			  }
			  LL_RCC_MSI_EnablePLLMode();
			  blue_mode = 0;
			  break;

		  case 5:
			  RTC_wakeup_init_from_stop(7);
			  LL_PWR_SetPowerMode(LL_PWR_MODE_STOP0);
			  LL_LPM_EnableDeepSleep();
			  __WFI();
			  blue_mode = 0;
			  break;

		  case 6:
			  RTC_wakeup_init_from_stop(7);
			  LL_PWR_SetPowerMode(LL_PWR_MODE_STOP1);
			  LL_LPM_EnableDeepSleep();
			  __WFI();
			  SystemClock_5_6_7_8();
			  blue_mode = 0;
			  break;

		  case 7:
			  RTC_wakeup_init_from_stop(7);
			  LL_PWR_SetPowerMode(LL_PWR_MODE_STOP2);
			  LL_LPM_EnableDeepSleep();
			  __WFI();
			  blue_mode = 0;
			  break;

		  case 8:
			  RTC_wakeup_init_from_standby_or_shutdown(7);
			  LL_PWR_SetPowerMode(LL_PWR_MODE_SHUTDOWN);
			  LL_LPM_EnableDeepSleep();
			  __WFI();
			  blue_mode = 0;
			  break;

		  default:
			  if (expe != 1 && expe != 2 && expe != 3 && expe != 4) {
				  LL_LPM_EnableSleep();
				  __WFI();
			  }
			  break;
			}
		}
	}
}

// systick interrupt handler --> allumage LED toutes les 2 s pendant 50 ms.
void SysTick_Handler()
{
	unsigned int subticks;
	ticks += 1;

	//gestion de l'allumage de la LED
	subticks = ticks % 200;
	if	( subticks == 0 ) LED_GREEN(1);
    else if	( subticks == (expe * 5) ){
	LED_GREEN(0);}

	//Partie pour le transceiver
		if (cptr_transmit == period_transmit) {

			//sortie du mode power down
			if (expe > 5){
			nRF24_SetPowerMode(nRF24_PWR_UP);}
			//Delay_ms(2); //Attente 2 ms (1.5 ms pour la sortie du mode power down.
			//En fait, on attend 200 ms car la base de temps du systick est 100 ms.
			//LE test montre qu'on n'est pas obligé de mettre un délai. Le temps de compléter le tableau
			//Message et le temps de transmettre sur l'UART prend au moins 5 ms.

			//préparation du message à transmettre
			for (uint8_t i = 0; i < 15; i++) {
				Message[i] = entete_message[i];
			}
			Message[15] = '-';
			Message[16] = cptr;

			Transmit_Message(Message,taille_message);
			cptr++;
			cptr_transmit = 1;
			if (expe > 5) nRF24_SetPowerMode(nRF24_PWR_DOWN); // mode low power
		}
		else {
			cptr_transmit ++;
		}
}

// wakeup timer interrupt Handler (inutile mais doit etre defini)
void RTC_WKUP_IRQHandler()
{
	LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_20 );
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

// ========= IFDEF =========

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
