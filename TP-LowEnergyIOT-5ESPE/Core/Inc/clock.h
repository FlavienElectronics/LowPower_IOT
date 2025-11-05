/*Author: Alexandre Boyer
 * Date: 14 Aug. 2024
 */
/* Clock config.
 */


// main clock config functions
void SystemClock_Config_80M(void);
void RTC_wakeup_init( int delay );
void RTC_WKUP_IRQHandler();

// config systick avec interrupt
void mySystick( unsigned int periode_en_ticks );
