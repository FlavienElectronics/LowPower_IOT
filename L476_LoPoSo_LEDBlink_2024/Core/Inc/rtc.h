/*
 * rtc.h
 *
 *  Created on: Nov 14, 2025
 *      Author: lespiaucq
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_

void RTC_wakeup_init( int delay );
void RTC_wakeup_init_from_standby_or_shutdown( int delay );
void RTC_wakeup_init_from_stop( int delay );
void Enable_MSI_LSE_Calibration(void);
void Disable_MSI_LSE_Calibration(void);
void Enter_Sleep_100Hz(void);
void Exit_Sleep_100Hz(void);

#endif /* INC_RTC_H_ */
