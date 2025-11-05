/*Author: Alexandre Boyer
 * Date: 14 Aug. 2024
 */
/* Radio functions to create a 6:1 star network based on the nRF24L01. The functions concern the PTX and the PRX.
 */

#include "nrf24.h"
#include "RadioFunctions.h"
#include "usart.h"


uint8_t nRF24_payload_recu[32];  //pour le stockage du message reçu (32 caractères max). Uniquement pour le PRX.
uint8_t payload_length = 32; //longueur du payload (32 max) (reçu par le PRXou à transmettre par le PTX)

//stockage des adresses des 6 data pipes
uint8_t addresses_TX[][6] = {adr_TX_pipe0, adr_TX_pipe1, adr_TX_pipe2, adr_TX_pipe3, adr_TX_pipe4, adr_TX_pipe5};
uint8_t addresses_RX[][6] = {adr_RX_pipe0, adr_RX_pipe1, adr_RX_pipe2, adr_RX_pipe3, adr_RX_pipe4, adr_RX_pipe5};

// Numéro du pipe utilisé par le PTX
nRF24_RXResult pipe_TX;

// pour stocker le numero du pipe sur lequel le PRX a reçu son dernier message
nRF24_RXResult pipe_message_recu;

// Timeout counter (depends on the CPU speed)
// Used for not stuck waiting for IRQ
#define nRF24_WAIT_TIMEOUT         (uint32_t)0x000FFFFF

// Result of packet transmission
typedef enum {
	nRF24_TX_ERROR  = (uint8_t)0x00, // Unknown error
	nRF24_TX_SUCCESS,                // Packet has been transmitted successfully
	nRF24_TX_TIMEOUT,                // It was timeout during packet transmit
	nRF24_TX_MAXRT                   // Transmit failed with maximum auto retransmit count
} nRF24_TXResult;
nRF24_TXResult tx_result;



// Function to transmit data packet
// input:
//   pBuf - pointer to the buffer with data to transmit
//   length - length of the data buffer in bytes
// return: one of nRF24_TX_xx values
nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length) {
	volatile uint32_t wait = nRF24_WAIT_TIMEOUT;
	uint8_t status;

	// Deassert the CE pin (in case if it still high)
	nRF24_CE_L();

	// Transfer a data from the specified buffer to the TX FIFO
	nRF24_WritePayload(pBuf, length);

	// Start a transmission by asserting CE pin (must be held at least 10us)
	nRF24_CE_H();

	// Poll the transceiver status register until one of the following flags will be set:
	//   TX_DS  - means the packet has been transmitted
	//   MAX_RT - means the maximum number of TX retransmits happened
	// note: this solution is far from perfect, better to use IRQ instead of polling the status
	do {
		status = nRF24_GetStatus();
		if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
			break;
		}
	} while (wait--);

	// Deassert the CE pin (Standby-II --> Standby-I)
	nRF24_CE_L();

	if (!wait) {
		// Timeout
		return nRF24_TX_TIMEOUT;
	}

	// Check the flags in STATUS register
	UART_SendStr("[");
	UART_SendHex8(status);
	UART_SendStr("] ");

	// Clear pending IRQ flags
    nRF24_ClearIRQFlags();

	if (status & nRF24_FLAG_MAX_RT) {
		// Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
		return nRF24_TX_MAXRT;
	}

	if (status & nRF24_FLAG_TX_DS) {
		// Successful transmission
		return nRF24_TX_SUCCESS;
	}
	// Some banana happens, a payload remains in the TX FIFO, flush it
	nRF24_FlushTX();

	return nRF24_TX_ERROR;
}

//Initialisation du transceiver --> paramètres par défaut, raz des IRQ, passage en mode power down
//et vérification que le transceiver fonctionne.
//Les registres du nRF24L01 sont accessibles par SPI même en mode power down.
void Init_Transceiver() {
	uint8_t check_OK;

	nRF24_Init(); // Initialize the nRF24L01 to its default state
	nRF24_ClearIRQFlags(); // Clear any pending IRQ flags
	nRF24_SetPowerMode(nRF24_PWR_DOWN); //passage en mode power down.
	//même si le passage en mode power down est rapide, on laisse un délai de 1 ms (suppose que
	//le Systick soit cadencé pour déborder toutes les 1 ms).
	Delay_ms(1);

	check_OK = nRF24_Check(); //vérif de la présence du nRF24L01
	if (check_OK == 1) {
		UART_SendStr("The transceiver nRF24L01 is on-line.\r\n");
	} else {
		UART_SendStr("The transceiver nRF24L01 is not responding.\r\n");
	}
}

//Configuration du canal RF : fréq du canal RF, puissance RF et data rate
void Config_RF_channel(uint8_t channel_nb, uint8_t DataRate, uint8_t TX_Power) {
	uint8_t reg;

	// Set RF channel
	nRF24_SetRFChannel(channel_nb);
	// Set data rate
	nRF24_SetDataRate(DataRate);
	// Set TX power
	nRF24_SetTXPower(TX_Power);

	//verification des registres.
	//reg = ReadReg(0x05); //registre RF_CH
	//reg = ReadReg(0x06); //registre RF_SETUP
	//reg = ReadReg(0x00); //registre config
}

//configuration du CRC :
//CRC_On = 0 --> desactivation du champ CRC, = 1 --> activation du champ CRC
//size_CRC = 0 --> CRC sur 1 octet. size_CRC = 1 --> CRC sur 2 octets
void Config_CRC(uint8_t CRC_On, uint8_t size_CRC) {
	uint8_t scheme_CRC = 0;

	if (CRC_On == CRC_Field_On) {
		if (size_CRC == CRC_Field_1byte) {
			scheme_CRC = 0x8;
		}
		else {
			scheme_CRC = 0xC;
		}
	}

	nRF24_SetCRCScheme(scheme_CRC);
}

//configure les 6 adresses des data pipes. Configure aussi la largeur de l'adresse (3 à 5 octets)
//Addr_Default = Default_pipe_address ou Custom_pipe_address.
void Config_PRX_adress(uint8_t Address_width, uint8_t autoAck_on, uint8_t Addr_Default) {
	uint8_t verif_RX0_Adr[5];

	if ((Address_width < 3) | (Address_width > 5)) { //par défaut, adresse sur 5 octets
		nRF24_SetAddrWidth(5);
	} else {
		nRF24_SetAddrWidth(Address_width);
	}

	//ouverture des 6 data pipes (bits du registre EN_RXADDR)
	nRF24_SetRXPipe(nRF24_PIPE0,autoAck_on,payload_length);
	nRF24_SetRXPipe(nRF24_PIPE1,autoAck_on,payload_length);
	nRF24_SetRXPipe(nRF24_PIPE2,autoAck_on,payload_length);
	nRF24_SetRXPipe(nRF24_PIPE3,autoAck_on,payload_length);
	nRF24_SetRXPipe(nRF24_PIPE4,autoAck_on,payload_length);
	nRF24_SetRXPipe(nRF24_PIPE5,autoAck_on,payload_length);

	//configuration des adresses des 6 data pipes :
	if (Addr_Default == Custom_pipe_address) {
		nRF24_SetAddr(nRF24_PIPE0, addresses_RX[0]);
		nRF24_SetAddr(nRF24_PIPE1, addresses_RX[1]);
		nRF24_SetAddr(nRF24_PIPE2, addresses_RX[2]);
		nRF24_SetAddr(nRF24_PIPE3, addresses_RX[3]);
		nRF24_SetAddr(nRF24_PIPE4, addresses_RX[4]);
		nRF24_SetAddr(nRF24_PIPE5, addresses_RX[5]);
	}
	//else : on ne fait rien. Après l'extinction de l'alim du nRF24L01, les adresses
	//de chaque pipe reprendront leurs valeurs par défaut.


	//vérification de l'écriture des adresses de réception
	ReadMBReg(0x0A, verif_RX0_Adr, 5);
}

//configure l'adresse du data pipe utilisé par le PTX. On indique le numéro du data pipe num_data_pipe.
//num_data_pipe est compris entre 0 et 5. Addr_Default = Default_pipe_address ou Custom_pipe_address.
void Config_PTX_adress(uint8_t Address_width, uint8_t Addr_Default, uint8_t num_data_pipe, uint8_t autoAck_on) {
	uint8_t reg;
	uint8_t verif_TX_Adr[5];
	uint8_t verif_RX0_Adr[5];
	uint8_t pipeAddrVar[5];

	if ((Address_width < 3) | (Address_width > 5)) { //par défaut, adresse sur 5 octets
			nRF24_SetAddrWidth(5);
	} else {
			nRF24_SetAddrWidth(Address_width);
	}

	//On rend identique l'adresse TX et l'adresse RX_P0, lorsque l'auto-acknowledgment est activé.
	//Par défaut, les adresses sont sur 5 octets, AA activés et data pipe 0 et 1 activés par défaut.
	nRF24_SetRXPipe(nRF24_PIPE0,autoAck_on,payload_length);
	//on vérifie la largeur du champ d'adresse
	reg = ReadReg(0x03);

	if (autoAck_on == nRF24_AA_ON) {  //l'adresse TX = l'adresse RX_P0.

		if (Addr_Default == Default_pipe_address) {
			switch (num_data_pipe) {
				case 0 : pipeAddrVar[4]=0xE7; pipeAddrVar[3]=0xE7; pipeAddrVar[2]=0xE7; pipeAddrVar[1]=0xE7; pipeAddrVar[0]=0xE7; break; //adr défaut RX_ADDR_P0
				case 1 : pipeAddrVar[4]=0xC2; pipeAddrVar[3]=0xC2; pipeAddrVar[2]=0xC2; pipeAddrVar[1]=0xC2; pipeAddrVar[0]=0xC2; break;//adr défaut RX_ADDR_P1
				case 2 : pipeAddrVar[4]=0xC3; pipeAddrVar[3]=0xC2; pipeAddrVar[2]=0xC2; pipeAddrVar[1]=0xC2; pipeAddrVar[0]=0xC2; break;//adr défaut RX_ADDR_P2
				case 3 : pipeAddrVar[4]=0xC4; pipeAddrVar[3]=0xC2; pipeAddrVar[2]=0xC2; pipeAddrVar[1]=0xC2; pipeAddrVar[0]=0xC2; break;//adr défaut RX_ADDR_P3
				case 4 : pipeAddrVar[4]=0xC5; pipeAddrVar[3]=0xC2; pipeAddrVar[2]=0xC2; pipeAddrVar[1]=0xC2; pipeAddrVar[0]=0xC2; break;//adr défaut RX_ADDR_P4
				case 5 : pipeAddrVar[4]=0xC6; pipeAddrVar[3]=0xC2; pipeAddrVar[2]=0xC2; pipeAddrVar[1]=0xC2; pipeAddrVar[0]=0xC2; break;//adr défaut RX_ADDR_P5
			}
			nRF24_SetAddr(nRF24_PIPE0,pipeAddrVar);
			nRF24_SetAddr(nRF24_PIPETX,pipeAddrVar);
		}
		//sinon, on fixe les adresses TX et RX0 à partir du contenu du tableau addresses.
		else {
			nRF24_SetAddr(nRF24_PIPE0,addresses_TX[num_data_pipe]);
			nRF24_SetAddr(nRF24_PIPETX,addresses_TX[num_data_pipe]);
		}

		//vérification des adresses :
		ReadMBReg(0x0A, verif_RX0_Adr, 5);
		ReadMBReg(0x10, verif_TX_Adr, 5);

	} else {//dans ce cas, on ne configure pas le récepteur

		if (Addr_Default == Default_pipe_address) {
			switch (num_data_pipe) {
				case 0 : pipeAddrVar[4]=0xE7; pipeAddrVar[3]=0xE7; pipeAddrVar[2]=0xE7; pipeAddrVar[1]=0xE7; pipeAddrVar[0]=0xE7; break; //adr défaut RX_ADDR_P0
				case 1 : pipeAddrVar[4]=0xC2; pipeAddrVar[3]=0xC2; pipeAddrVar[2]=0xC2; pipeAddrVar[1]=0xC2; pipeAddrVar[0]=0xC2; break; //adr défaut RX_ADDR_P1
				case 2 : pipeAddrVar[4]=0xC3; pipeAddrVar[3]=0xC2; pipeAddrVar[2]=0xC2; pipeAddrVar[1]=0xC2; pipeAddrVar[0]=0xC2; break; //adr défaut RX_ADDR_P2
				case 3 : pipeAddrVar[4]=0xC4; pipeAddrVar[3]=0xC2; pipeAddrVar[2]=0xC2; pipeAddrVar[1]=0xC2; pipeAddrVar[0]=0xC2; break; //adr défaut RX_ADDR_P3
				case 4 : pipeAddrVar[4]=0xC5; pipeAddrVar[3]=0xC2; pipeAddrVar[2]=0xC2; pipeAddrVar[1]=0xC2; pipeAddrVar[0]=0xC2; break; //adr défaut RX_ADDR_P4
				case 5 : pipeAddrVar[4]=0xC6; pipeAddrVar[3]=0xC2; pipeAddrVar[2]=0xC2; pipeAddrVar[1]=0xC2; pipeAddrVar[0]=0xC2; break; //adr défaut RX_ADDR_P5
			}
			nRF24_SetAddr(nRF24_PIPETX,pipeAddrVar);
		}
		else {
			nRF24_SetAddr(nRF24_PIPETX, addresses_TX[num_data_pipe]);
		}
	}
}

//Active le protocole Enhanced ShockBurst, notamment l'auto acknowledgment et l'auto retransmit.
//On spécifie, pour le PTX uniquement, le nb de retransmission (ARC --> count_retries, de 0 à 15) et le
//délai entre retransmission (ARD --> delay_retries, de 0 à 15)
void Config_ESB_Protocol(uint8_t delay_retries, uint8_t count_retries) {
	uint8_t reg;

	//autorisation de l'auto acknowledgement pour tous les data pipes
	nRF24_EnableAA(0);
	nRF24_EnableAA(1);
	nRF24_EnableAA(2);
	nRF24_EnableAA(3);
	nRF24_EnableAA(4);
	nRF24_EnableAA(5);

	//réglage paramètres de retransmission
	nRF24_SetAutoRetr(delay_retries,count_retries);

	//verification des registres
	//reg = ReadReg(0x01); //registre EN_AA
	//reg = ReadReg(0x04); //registre SETUP_RETR
}

//la fonction active la fonction RX (mise à 1 de la broche CE).
//Le transceiver doit être au préalable sorti du mode power down (donc en mode StandBy I) et
//et le mode RX doit avoir été sélectionné (PRIM_RX = 1).
void StartListen() {
	nRF24_CE_H();
}

//la fonction active la fonction TX (mise à 0 de la broche CE).
//Le transceiver doit être au préalable sorti du mode power down (donc en mode StandBy I) et
//et le mode TX doit avoir été sélectionné (PRIM_RX = 0).
void StopListen() {
	nRF24_CE_L();
}

//attention : fonction bloquante ! La fonction permet une écoute permanente et l'affichage des messages
//reçues sur le canal RF via l'UART. Ici, la fonction n'utilise pas d'interruption, mais un polling
//du statut du RX_FIFO. Le délai wait_time_ms est ajouté entre chaque interrogation de l'état du
//RX_FIFO
void Continuous_RX_Listen(int wait_time_ms) {
	while (1) {
		if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
			// Get a payload from the transceiver
			pipe_message_recu = nRF24_ReadPayload(nRF24_payload_recu, &payload_length);

			// Clear all pending IRQ flags
			nRF24_ClearIRQFlags();

			// Print a payload contents to UART
			UART_SendStr("Message reçu sur le data pipe #");
			UART_SendInt(pipe_message_recu);
			UART_SendStr(" de ");
			UART_SendInt(payload_length);
			UART_SendStr(" octets:>");
			UART_SendBufHex((char *)nRF24_payload_recu, payload_length);
			UART_SendStr("<\r\n");
		}
		else {
			UART_SendStr("Wait for message \n");
			Delay_ms(900);
		}
	}
}

//La fonction transmet le message Message de longueur donnée par l'argument longueur.
//La fonction indique aussi sur l'UART si la transmission s'est faite.
void Transmit_Message(uint8_t *Message,uint8_t longueur) {

	payload_length = longueur;

	UART_SendStr("Message a transmettre:>");
	UART_SendBufHex((char *)Message,payload_length);
	UART_SendStr("<: ");

	// Transmit a packet
	tx_result = nRF24_TransmitPacket(Message, payload_length);
	switch (tx_result) {
		case nRF24_TX_SUCCESS:
			UART_SendStr("OK");
			break;
		case nRF24_TX_TIMEOUT:
			UART_SendStr("TIMEOUT");
			break;
		case nRF24_TX_MAXRT:
			UART_SendStr("MAX RETRANSMIT");
			break;
		default:
			UART_SendStr("ERROR");
			break;
	}
	UART_SendStr("\r\n");
}

