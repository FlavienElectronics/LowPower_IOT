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

#endif /* INC_RTC_H_ */
