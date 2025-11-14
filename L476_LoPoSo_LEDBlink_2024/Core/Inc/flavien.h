#include "main.h"
#include "rtc.h"
#include "clock.h"

void _flavien_MSI_4Mhz(void);

void _flavien_MSI_24Mhz(void);

void _flavien_PLL_80Mhz(void);

void _flavien_PLL_off(void);

void _flavien_voltage_scaling_1(void);

void _flavien_voltage_scaling_2(void);

void _flavien_flash_latency(uint8_t latency);

void _flavien_calibration_MSI_vs_LSE(void);

void _flavien_calibration_MSI_vs_LSE_off(void);

void _flavien_sleep_100Hz_ON(void);

void _flavien_set_stop_mode(uint8_t mode);

void experience(uint8_t numero_experience);

void blue_button_pressed(uint8_t numero_experience);

