/*Author: Alexandre Boyer
 * Date: 14 Aug. 2024
 */
/* Radio functions to create a 6:1 star network based on the nRF24L01.
 */


#ifndef INC_RADIOFUNCTIONS_H_
#define INC_RADIOFUNCTIONS_H_

#include "gpio.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_spi.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_utils.h"

#define Default_pipe_address 0 //pour utiliser les adresses de pipe par défaut
#define Custom_pipe_address 1 //pour définir ses propres adresses de pipe

//La définition des adresses logiques des 6 data pipes pour la TX (3 à 5 caractères).
//Attention, les adresses des pipes 2, 3, 4, 5 ne doivent différer de celle du pipe 1
//que d'un octet (de poids faible).
#define adr_TX_pipe0 "PIPE0"
#define adr_TX_pipe1 "PIPE1"
#define adr_TX_pipe2 "PIPE2"
#define adr_TX_pipe3 "PIPE3"
#define adr_TX_pipe4 "PIPE4"
#define adr_TX_pipe5 "PIPE5"

//La définition des adresses logiques des 6 data pipes pour la RX (3 à 5 caractères).
//Attention, les adresses des pipes 2, 3, 4, 5 ne doivent différer de celle du pipe 1
//que d'un octet (de poids faible).
#define adr_RX_pipe0 "PIPE0"
#define adr_RX_pipe1 "PIPE1"
#define adr_RX_pipe2 "2"
#define adr_RX_pipe3 "3"
#define adr_RX_pipe4 "4"
#define adr_RX_pipe5 "5"

#define CRC_Field_Off 0
#define CRC_Field_On 1
#define CRC_Field_1byte 0
#define CRC_Field_2byte 1

#define NRF_SPI SPI1

static inline void nRF24_CE_L() {
    LL_GPIO_ResetOutputPin(nRF_CE_GPIO_Port, nRF_CE_Pin);
}

static inline void nRF24_CE_H() {
    LL_GPIO_SetOutputPin(nRF_CE_GPIO_Port, nRF_CE_Pin);
}

static inline void nRF24_CSN_L() {
    LL_GPIO_ResetOutputPin(nRF_CSN_GPIO_Port, nRF_CSN_Pin);
}

static inline void nRF24_CSN_H() {
    LL_GPIO_SetOutputPin(nRF_CSN_GPIO_Port, nRF_CSN_Pin);
}


static inline uint8_t nRF24_LL_RW(uint8_t data) {
    LL_SPI_SetRxFIFOThreshold(NRF_SPI,LL_SPI_RX_FIFO_TH_QUARTER);
    LL_SPI_Enable(NRF_SPI);
    // Wait until TX buffer is empty
    while (LL_SPI_IsActiveFlag_BSY(NRF_SPI));
    while (!LL_SPI_IsActiveFlag_TXE(NRF_SPI));
    LL_SPI_TransmitData8(NRF_SPI, data);
    while (!LL_SPI_IsActiveFlag_RXNE(NRF_SPI));
    return LL_SPI_ReceiveData8(NRF_SPI);
}


static inline void Delay_ms(uint32_t ms) { LL_mDelay(ms); }

void Init_Transceiver();
void Config_RF_channel(uint8_t channel_nb, uint8_t DataRate, uint8_t TX_Power);
void Config_CRC(uint8_t CRC_On, uint8_t size_CRC);
void Config_PRX_adress(uint8_t Address_width, uint8_t autoAck_on, uint8_t Addr_Default);
void Config_PTX_adress(uint8_t Address_width, uint8_t Addr_Default, uint8_t num_data_pipe, uint8_t autoAck_on);
void Config_ESB_Protocol(uint8_t delay_retries, uint8_t count_retries);
void StartListen();
void StopListen();

void Continuous_RX_Listen(int wait_time_ms);
void Transmit_Message(uint8_t *Message,uint8_t longueur);

#endif /* INC_RADIOFUNCTIONS_H_ */
