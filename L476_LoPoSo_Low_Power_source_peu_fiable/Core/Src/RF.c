/*
 * RF.C
 *
 *  Created on: Oct 14, 2024
 *      Author: guilhem CHAMBAUD
 */

#include "RF.h"

#include "main.h"
#include "gpio.h"
#include "clock.h"
#include "spi.h"
#include "usart.h"
#include "RadioFunctions.h"
#include "nrf24.h"

#define taille_message 20

uint8_t cptr = 0;
uint8_t cptr_transmit = 1;
uint8_t period_transmit = 5; //10; //période de retransmission du message en multiples de périodes de 100 ms (période
//de débordement du Systick)

const char entete_message[17] = "Emission paquet #";
uint8_t Message[taille_message];
uint8_t channel_nb = 60; //n° du canal radio utilisé (//channel 60 --> 2460 MHz)
uint8_t adr_data_pipe_used = 1; //numéro du data pipe utilisé pour la transmission (de 0 à 5)

void Transmission_RF (void)
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


	  //configuration interruption Systick (attention, il n'y a que 23 bits dans le registre load ...
	  //mySystick( SystemCoreClock * 2 );	// 0.5 Hz --> 2 s
	  //on va partir sur une période de 100 ms
	  mySystick( SystemCoreClock /10 ); //10 Hz --> 0.1 s
}

// systick interrupt handler --> transmission d'une nouvelle trame toutes les 2 s
//comme l'interuption a lieu toutes les 100 ms, on ajoute un compteur pour transmettre
//quand le compteur atteint 20.
void SysTick_Handler2()
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






/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler2(void)
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
