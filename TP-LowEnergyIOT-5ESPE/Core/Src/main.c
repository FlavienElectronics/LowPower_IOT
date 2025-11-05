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
#include <stdio.h>


volatile unsigned int ticks = 0; //pour la gestion des intervalles de temps. 1 tick = 10 ms.
unsigned int subticks = 0;
volatile int blue_mode = 0; //pour savoir si on est dans le mode "Blue mode"
volatile int old_blue = 0;
volatile int expe = 1; //pour la sauvegarde du numéro de l'expérience
uint32_t testing ;
int status;
LL_RTC_InitTypeDef RTC_InitStruct;

uint8_t channel_nb = 60; //n° du canal radio utilisé (//channel 60 --> 2460 MHz)
uint8_t adr_data_pipe_used = 1; //numéro du data pipe utilisé pour la transmission (de 0 à 5)

void expe_counter() {

	if (BLUE_BUTTON()) {
		expe = LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR0);
		expe++;
		if (9 == expe){ expe = 1; }
		LL_RTC_BAK_SetRegister(RTC, LL_RTC_BKP_DR0, expe);
	}
}


int main(void)
{

	/*clock domains activation*/
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);


	// config GPIO
	GPIO_init();
	//config clock
	SystemClock_Config_80M();
	//config bus SPI1 (pour la communication avec le transceiver nRF24L01)
	SPI1_Init();
	//config USART2
	USART2_Init();
	//INIT RTC AVEC REGISTER
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN; //Enable GRP1_CLOCK
	PWR->CR1 |= PWR_CR1_DBP; //Enable BkUpAccess
	RCC->BDCR |= RCC_BDCR_LSEON; //Enable RCC_LSE
	RCC->BDCR &= ~RCC_BDCR_RTCSEL; //Set RTCClockSource
	RCC->BDCR |= RCC_BDCR_RTCSEL_0;
	RCC->BDCR |= RCC_BDCR_RTCEN; // Enable RTC

	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);


	if (LL_RCC_LSE_IsReady() == 1) {
		hot_start();
	} else {
		cold_start();
	}


	expe_counter();
	expe = LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR0);


	if (expe == 1) {
		SystemClock_Config_80M();
	}
	if (expe == 2) {
		SystemClock_Config_Expe2();
	}
	if (expe == 3) {
		SystemClock_Config_ExpeReste();

	}
	if (expe ==4){
		LL_LPM_EnableSleep();
		SystemClock_Config_ExpeReste();
	}
	if (expe == 5) {
		SystemClock_Config_ExpeReste();
		LL_LPM_EnableSleep();
		LL_RCC_MSI_EnablePLLMode();
	}
	if (expe == 6) {
		SystemClock_Config_ExpeReste();
		LL_RCC_MSI_EnablePLLMode();
		LL_LPM_EnableSleep();
	}
	if (expe == 7) {
		SystemClock_Config_ExpeReste();
		LL_RCC_MSI_EnablePLLMode();
		LL_LPM_EnableSleep();

	}
	if (expe == 8) {
		SystemClock_Config_ExpeReste();
		LL_RCC_MSI_EnablePLLMode();
		LL_LPM_EnableSleep();

	}

	NVIC_SetPriority(SysTick_IRQn, -1);
	// config systick avec interrupt
	mySystick(SystemCoreClock / 100 );	// 100 Hz --> 10 ms





//	//// ----------------------------------------------------------------------------
//	//// 									PTX
//	//// 					ENVOI DE MESSAGES VIA LE TRANCEIVER RF
//	//// ----------------------------------------------------------------------------
//	//configuration du transceiver en mode PTX
//	Init_Transceiver();
//	Config_RF_channel(channel_nb,nRF24_DR_250kbps,nRF24_TXPWR_18dBm);
//	Config_CRC(CRC_Field_On, CRC_Field_1byte);
//	//Adresse sur 5 bits. Transmission sur le data pipe adr_data_pipe_used.
//	Config_PTX_adress(5,Default_pipe_address,adr_data_pipe_used,nRF24_AA_ON);
//	Config_ESB_Protocol(nRF24_ARD_1000us,10);
//	//on sort du mode power down
//	nRF24_SetPowerMode(nRF24_PWR_UP);
//	Delay_ms(2); //Attente 2 ms (1.5 ms pour la sortie du mode power down).
//
//	//Entrée en mode TX
//	nRF24_SetOperationalMode(nRF24_MODE_TX);
//	StopListen();
//
//	//configuration interruption Systick (attention, il n'y a quue 23 bits dans le registre load ...
//	mySystick( SystemCoreClock * 2 );	// 0.5 Hz --> 2 s
//	//on va partir sur une période de 100 ms
//	//mySystick( SystemCoreClock /10 ); //10 Hz --> 0.1 s
//
//	int expNumber = expe;
//	int packetNumber = 0;
//
//	// 1 char = 1 octet, donc chaque message a une taille max de 32 char
//	char messageToSend[33];  // 32 char + caractère de fin de chaîne
//
//	// Création du message à envoyer			    |   NOMS DU BINOME   | EXPN | NBPAQUET |
//	snprintf(messageToSend, sizeof(messageToSend), "O-LOPES_TETAZ_CHALHOUB_EXP%d_%d   ", expNumber, packetNumber);
//
//	// Appel de la fonction Transmit_Message (exemple d'appel)
//	Transmit_Message((uint8_t *)messageToSend, 32);
while (1)
	{
		if (expe == 1) {
			if (blue_mode){
				Sleep();
			}
		}
		if (expe == 2) {
			if (blue_mode){
				configMsiLse();
			}
		}
		if (expe == 3) {
			if (blue_mode){
				Sleep();
			}
		}
		if (expe ==4){
			if (blue_mode){
				configMsiLse();
			}
		}
		if (expe == 5) {
			if (blue_mode){
				Stop_0();
				blue_mode=0;
			}
		}
		if (expe == 6) {
			if (blue_mode){
				Stop_1();
				blue_mode=0;
			}
		}
		if (expe == 7) {
			if (blue_mode){
				Stop_2();
				blue_mode=0;
			}
		}
		if (expe == 8) {
			if (blue_mode){
				Shutdown();
				blue_mode=0;
			}
		}


	}
}
// systick interrupt handler --> allumage LED toutes les 2 s pendant 50 ms.
//Scrutation de l'état du bouton bleu  (pas d'action à ce stade).
void SysTick_Handler()
{

	ticks += 1;
	GPIOC->ODR ^= (1 << 1);
	if	( BLUE_BUTTON() )
	{
		if	(old_blue == 0)
			blue_mode = 1;
		old_blue =1;
	}
	else 	old_blue = 0;

	subticks = ticks %200;
	if(subticks ==0)
	{
		LED_GREEN(1);
	}else if (subticks == 5*expe){
		LED_GREEN(0);
	}

	//gestion du toogle PC10
	if(ticks % 1 == 0){
		 LL_GPIO_TogglePin (GPIOC , LL_GPIO_PIN_10);
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


// partie commune a toutes les utilisations du wakeup timer
void RTC_wakeup_init( int delay )
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
void RTC_WKUP_IRQHandler() {
	LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_20);
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



//	RTC AVEC REGISTER
//	//Init to RTC
//	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN; //Enable GRP1_CLOCK
//	PWR->CR1 |= PWR_CR1_DBP; //Enable BkUpAccess
//	RCC->BDCR |= RCC_BDCR_LSEON; //Enable RCC_LSE
//	RCC->BDCR &= ~RCC_BDCR_RTCSEL; //Set RTCClockSource
//	RCC->BDCR |= RCC_BDCR_RTCSEL_0;
//	RCC->BDCR |= RCC_BDCR_RTCEN; // Enable RTC
//
//	 expe= RTC->BKP0R;
//	 if ( 1 == RTC->BKP1R)
//	 	{
//		 	 expe++;
//	 	}
//	 if ( 8 < expe)
//		{
//		 	 expe++;
//		}
//	RTC->BKP0R = expe;



// LL_RTC_BAK_SetRegister(RTC, 3, expe);
// testing = LL_RTC_BAK_GetRegister(RTC, 3);
// expe = LL_RTC_BAK_GetRegister(RTC, 3);
// // config systick avec interrupt
// mySystick( SystemCoreClock / 100 );	// 100 Hz --> 10 ms
// while (expe < 7)
// {
// 	status = BLUE_BUTTON();
// 	if ( 1 == status)
// 	{
// 		LL_mDelay(100);
// 		expe++;
// 		LL_RTC_BAK_SetRegister(RTC, 3, expe);
// 		testing = LL_RTC_BAK_GetRegister(RTC, 3);
// 	}
// }


//void Expe1()
//{
//    //Config clock deja faite
//	//Mode sleep
//	Sleep();
//}
//
//void Expe2(){
//	//Config clock
//	SystemClock_Config_Expe2();
//
//	//Config MSI
//	configMsiLse();
//}
//
//void Expe3()
//{
//	Sleep();
//}
//
//
//void Expe4(){
//	//Config clock
//	SystemClock_Config_ExpeReste();
//
//	//Config MSI
//	configMsiLse();
//}
//
//void Expe5()
//{
//	//Config clock
//	SystemClock_Config_ExpeReste();
//	//Config mode sleep
//	SCB->SCR |= (1 << 2);	//SLEEPDEEP à 1
//
//	//Mode STOP0
//	PWR->CR &= ~(1 << 2);
//
//	__WFI();
//}










//	//// ----------------------------------------------------------------------------
//	//// 									PRX
//	//// 				RECEPTION DE MESSAGES VIA LE TRANCEIVER RF
//	//// ----------------------------------------------------------------------------
//	//configuration du transceiver en mode PRX
//	Init_Transceiver();
//	Config_RF_channel(channel_nb,nRF24_DR_250kbps,nRF24_TXPWR_12dBm);
//	Config_CRC(CRC_Field_On, CRC_Field_1byte);
//	Config_PRX_adress(5,nRF24_AA_ON,Default_pipe_address); //Adresse sur 5 bits
//	Config_ESB_Protocol(nRF24_ARD_500us,10);
//	//on sort du mode power down
//	nRF24_SetPowerMode(nRF24_PWR_UP);
//	Delay_ms(2); //Attente 2 ms (1.5 ms pour la sortie du mode power down).
//
//	//Entrée en mode RX
//	nRF24_SetOperationalMode(nRF24_MODE_RX);
//	StartListen();
//
//	//Ecoute continue
//	Continuous_RX_Listen(500);
