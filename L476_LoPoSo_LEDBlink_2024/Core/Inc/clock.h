/*Author: Alexandre Boyer
 * Date: 14 Aug. 2024
 */
/* Clock config.
 */


// main clock config functions
void SystemClock_Config_80M(void);


// config systick avec interrupt
void mySystick( unsigned int periode_en_ticks );

// Init des clocks pour les experiences
void SystemClock_2(void);
void SystemClock_3(void);
void SystemClock_4(void);
void SystemClock_5_6_7_8(void);
