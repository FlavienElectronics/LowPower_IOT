/*Author: Alexandre Boyer
 * Date: 14 Aug. 2024
 */
/* GPIO config.
 */
#include <stdint.h>
#include "stm32l4xx_ll_gpio.h"

#define User_Button_Pin LL_GPIO_PIN_13
#define User_Button_GPIO_Port GPIOC
#define nRF_CSN_Pin LL_GPIO_PIN_4
#define nRF_CSN_GPIO_Port GPIOA
#define SPI1_SCK_Pin LL_GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin LL_GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin LL_GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define nRF_IRQ_Pin LL_GPIO_PIN_0
#define nRF_IRQ_GPIO_Port GPIOB
#define nRF_IRQ_EXTI_IRQn EXTI0_IRQn
#define User_LED_Pin LL_GPIO_PIN_13
#define User_LED_GPIO_Port GPIOB
#define nRF_CE_Pin LL_GPIO_PIN_8
#define nRF_CE_GPIO_Port GPIOA
#define Clock_Monitor_Pin LL_GPIO_PIN_10
#define Clock_Monitor_GPIO_Port GPIOC

//testing PIN
#define NEW_PIN_Port GPIOB
#define NEW_PIN LL_GPIO_PIN_6

// config
void GPIO_init(void);

// utilisation
void LED_GREEN( int val );
int BLUE_BUTTON();
//test pin
//void SetPinForDuration(GPIO_TypeDef *port, uint32_t pin, uint32_t duration_ms);
