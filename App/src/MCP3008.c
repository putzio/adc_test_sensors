/*
 * MCP3008.c
 *
 *  Created on: 14 Feb 2020
 *      Author: Daniel Mårtensson
 */

#include "../../App/inc/MCP3008.h"

 /*
  * Set the MISO, MOSI, SCK and CS
  * SPI settings:
  * CPHA = 1 Edge
  * Prescaler = 8
  * First bit = MBS first
  * CPOL = Low
  */
void MCP_Init(MCP3008_SPI* spi, SPI_HandleTypeDef* hspi, GPIO_TypeDef* CS_PORT, uint16_t CS_PIN) {
	spi->hspi = hspi;
	spi->CS_PORT = CS_PORT;
	spi->CS_PIN = CS_PIN;
	//  HAL_GPIO_Init(CS_PORT, CS_PIN);
}

// Read the channels from 0 to 7
uint16_t MCP_Read_Channel(MCP3008_SPI* spi, uint8_t channel) {

	// Declare data that we will send
	uint8_t pTxData[3] = { 0 };
	pTxData[0] = ((0x01 << 7) |		// start bit
		(1 << 6) |			// SGL
		((channel & 0x07) << 3)); 	// channel number
	pTxData[1] = 0x00;
	pTxData[2] = 0x00;

	// Data that we will get
	uint8_t pRxData[3] = { 0 };

	// CS low, Send and receive, CS high
	HAL_GPIO_WritePin(spi->CS_PORT, spi->CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(spi->hspi, pTxData, pRxData, 3, 10);
	HAL_GPIO_WritePin(spi->CS_PORT, spi->CS_PIN, GPIO_PIN_SET);

	// Compute the ADC
	return 0x3FF & ((pRxData[0] & 0x01) << 9 | (pRxData[1] & 0xFF) << 1 | (pRxData[2] & 0x80) >> 7);
}
