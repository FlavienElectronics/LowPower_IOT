/*Author: Alexandre Boyer
 * Date: 14 Aug. 2024
 */
/* Clock config.
 */


// main clock config functions
void SystemClock_Config_80M(void);
void SystemClock_Config_MSI_4M(void);
void SystemClock_Config_MSI_24M(void);
void SystemClock_Enable_PLL_80M(void);
void SystemClock_Disable_PLL(void);


// config systick avec interrupt
void mySystick( unsigned int periode_en_ticks );
