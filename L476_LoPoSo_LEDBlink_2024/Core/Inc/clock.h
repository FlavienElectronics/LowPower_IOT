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
void Configure_VoltageScaling(uint32_t scale);
void Configure_FlashLatency(uint32_t latency);
void Enable_MSI_LSE_Calibration(void);
void Disable_MSI_LSE_Calibration(void);
void Enter_Sleep_100Hz(void);
void Exit_Sleep_100Hz(void);


// config systick avec interrupt
void mySystick( unsigned int periode_en_ticks );
