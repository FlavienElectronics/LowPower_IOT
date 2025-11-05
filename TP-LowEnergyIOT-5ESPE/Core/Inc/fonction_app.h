/*
 * fonction_app.h
 *
 *  Created on: Dec 4, 2024
 *      Author: yorgo
 */

#ifndef INC_FONCTION_APP_H_
#define INC_FONCTION_APP_H_

#include "stm32l4xx_ll_crs.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_rtc.h"
#include "stm32l4xx_ll_spi.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_gpio.h"

#include "gpio.h"
#include "main.h"
#include "clock.h"
#include <stdio.h>

void cold_start();
void hot_start();
void  Sleep();
void configMsiLse();

void Stop_0();
void Stop_1();
void Stop_2();
void Shutdown();

#endif /* INC_FONCTION_APP_H_ */
