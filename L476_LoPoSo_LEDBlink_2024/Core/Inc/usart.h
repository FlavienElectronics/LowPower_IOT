/*Author: Alexandre Boyer
 * Date: 14 Aug. 2024
 */
/* USART config.
 */

#ifndef INC_USART_H_
#define INC_USART_H_

#include <stdint.h>

#define HEX_CHARS      "0123456789ABCDEF"

void USART2_Init(void);
void UART_SendStr(char *string);
void UART_SendBufHex(char *buf, uint16_t bufsize);
void UART_SendHex8(uint8_t num);
void UART_SendInt(uint32_t num);


#endif /* INC_USART_H_ */
