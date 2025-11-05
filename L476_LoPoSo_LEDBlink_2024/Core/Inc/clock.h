/*Author: Alexandre Boyer
 * Date: 14 Aug. 2024
 */
/* Clock config.
 */


// main clock config functions
void SystemClock_Config_80M(void);
void SystemClock_Config_4M(void);
void SystemClock_Config_24M(void);

/* Enable PLL and switch system clock to PLL output (configured for 80 MHz)
 * and update SystemCoreClock. This config assumes MSI at 4 MHz as PLL source.
 */
void SystemClock_EnablePLL_80M(void);

/* Disable PLL and switch system clock back to MSI (4 MHz). */
void SystemClock_DisablePLL(void);


// config systick avec interrupt
void mySystick( unsigned int periode_en_ticks );
