#include "flavien.h"

extern volatile int blue_mode;

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


void experience(uint8_t numero_experience)
{
    switch (numero_experience) {	//FLAVIEN LE TROUBLE
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
    }
}

void blue_button_pressed(uint8_t numero_experience)
{

	if(blue_mode == 0)
		blue_mode = 1;
	if (blue_mode == 1)
	{
		switch (numero_experience) {
			  case 1:
				  LL_LPM_EnableSleep();
				  __WFI();
				  blue_mode = 0;
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
				  __WFI();
				  blue_mode = 0;
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
//				  LL_PWR_SetPowerMode(LL_PWR_MODE_STOP0);
				  _flavien_set_stop_mode(0);
				  LL_LPM_EnableDeepSleep();

				  __WFI();
				  blue_mode = 0;
				  break;

			  case 6:
				  RTC_wakeup_init_from_stop(7);
//				  LL_PWR_SetPowerMode(LL_PWR_MODE_STOP1);
				  _flavien_set_stop_mode(1);
				  LL_LPM_EnableDeepSleep();
				  __WFI();
				  SystemClock_5_6_7_8();
				  blue_mode = 0;
				  break;

			  case 7:
				  RTC_wakeup_init_from_stop(7);
//				  LL_PWR_SetPowerMode(LL_PWR_MODE_STOP2);
				  _flavien_set_stop_mode(2);
				  LL_LPM_EnableDeepSleep();
				  __WFI();
				  blue_mode = 0;
				  break;

			  case 8:
				  RTC_wakeup_init_from_standby_or_shutdown(7);
//				  LL_PWR_SetPowerMode(LL_PWR_MODE_SHUTDOWN);
				  _flavien_set_stop_mode(4);
				  NVIC_DisableIRQ(SysTick_IRQn);
				  LL_LPM_EnableDeepSleep();
				  __WFI();
				  blue_mode = 0;
				  break;

			  default:
				  if (numero_experience != 1 && numero_experience != 2 && numero_experience != 3 && numero_experience != 4) {
					  LL_LPM_EnableSleep();
					  __WFI();
					  blue_mode = 0;
				  }
				  break;
				}
	}

}

