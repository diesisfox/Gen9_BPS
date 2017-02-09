/*
 * mcp3909.c
 *
 *  Created on: Jan 2, 2017
 *      Author: frank
 */

#include "mcp3909.h"

uint8_t inline mcp3909_SPI_WriteReg(MCP3909HandleTypeDef * hmcp, uint8_t address, uint8_t length) {

  // Aseemble CONTROL BYTE
  // | 0 | 1 | A4 | A3 | A2 | A1 | R/!W |
  uint8_t CONTROL_BYTE = 0;
  CONTROL_BYTE = address << 1;
  CONTROL_BYTE |= 1 << 6;
  CONTROL_BYTE |= 1;    // Write to address

  hmcp->pTxBuf[0] = CONTROL_BYTE;	// Set the first byte of the Tx Buffer to CONTROL_BYTE

  // NOTE: Don't necessarily need DMA since we want this to be synchronous
  // If the chip times out, then put the mcp3909 handle into error state
  if( HAL_SPI_Transmit(hmcp->hspi,hmcp->pTxBuf, length, SPI_TIMEOUT) != HAL_OK){
	  return 0;
  } else {
	  return 1;
  }
}

// Reads a register or a set of registers starting from the given address
uint8_t inline mcp3909_SPI_ReadReg(MCP3909HandleTypeDef * hmcp, uint8_t address, uint8_t readType){

  // Set the register read behavior
  (hmcp->readType) &= 0x3FFFFF;			// Clear the readType bits
  (hmcp->readType) |= readType << 22;	// Set the readType bits
  for(uint8_t i = 2; i >= 0; i--){
  	  (hmcp->pTxBuf)[i] = 0;
  	  (hmcp->pTxBuf)[i] = ((hmcp->readType)  >> 8*(2-i)) & (0xFF);
  }
  mcp3909_SPI_WriteReg(hmcp, STATUS, REG_LEN);

  // Aseemble CONTROL BYTE
  // | 0 | 1 | A4 | A3 | A2 | A1 | R/!W |
  uint8_t CONTROL_BYTE = 0;
  CONTROL_BYTE = address << 1;
  CONTROL_BYTE |= 1 << 6;
  CONTROL_BYTE |= 0;    // Read from address

  hmcp->pTxBuf[0] = CONTROL_BYTE;	// Set the first byte of the Tx Buffer to CONTROL_BYTE

  // NOTE: Don't necessarily need DMA since we want this to be synchronous
  // If the chip times out, then put the mcp3909 handle into error state
  // Only transmit the CONTROL_BYTE to the chip to request for data transfer
  if(HAL_SPI_TransmitReceive(hmcp->hspi,hmcp->pTxBuf, hmcp->pRxBuf, 1, SPI_TIMEOUT) != HAL_OK){
	  return 0;
  } else {
	  return 1;
  }
}

uint8_t mcp3909_init(SPI_HandleTypeDef * hspi, MCP3909HandleTypeDef * hmcp){
  hmcp->hspi = hspi;
  // Set up temporary register containers for modification
  uint32_t REG_PHASE = 0;
  uint32_t REG_GAIN = 0;
  uint32_t REG_STATUS = 0;
  uint32_t REG_CONFIG = 0;

  // Global chip settings:
  // First byte of REG_CONFIG
  REG_CONFIG |= hmcp->extCLK;			// Clock source setting
  REG_CONFIG |= (hmcp->extVREF) << 1;	// Voltage reference setting
  REG_CONFIG |= (hmcp->prescale) << 2;	// Prescaler setting
  REG_CONFIG |= (hmcp->osr) << 4;		// Over Sampling Ratio setting

  REG_STATUS |= (hmcp->readType) << 22;	// Read configuration to register type
  REG_STATUS |= 1 << 14;				// 3 Cycle latency to let sinc3 settle

  // Channel specific settings:
  for(uint8_t i = 0; i < MAX_CHANNELS; i++){
	  REG_CONFIG |= (hmcp->channel[i].dither) << (6+i);	// Dither controls

	  // Set mode at last; ADC_SHUTDOWN will override clock and vref source settings
	  switch((hmcp->channel[i]).mode){
	  case ADC_SHUTDOWN:
		  REG_CONFIG |= EXT_CLK;
		  REG_CONFIG |= EXT_VREF << 1;
		  REG_CONFIG |= 1 << (12+i);
		  REG_CONFIG |= 0 << (18+i);
		  break;

	  case	ADC_RESET:
		  REG_CONFIG |= 0 << (12+i);
		  REG_CONFIG |= 1 << (18+i);
		  break;

	  case ADC_ON:
		  REG_CONFIG |= 0 << (12+i);
		  REG_CONFIG |= 0 << (18+i);
		  break;
	  }

	  // Set resolution
	  REG_STATUS |= ((hmcp->channel[i]).resolution) << (15+i);

	  // Set channel gain
	  REG_GAIN |= ((hmcp->channel[i]).PGA) << (4*i);
	  REG_GAIN |= ((hmcp->channel[i]).boost) << (4*i + 3);
  }

  // Set phase registers
  REG_PHASE |= (hmcp->phase[0]);		// CH4 & CH5
  REG_PHASE |= (hmcp->phase[1]) << 8;	// CH3 & CH2
  REG_PHASE |= (hmcp->phase[2]) << 16;	// CH1 & CH0

  // TODO: Check the conversion code below
  // TODO: Check the SPI write functions
  // Directly write into the transmission buffer
  for(uint8_t i = 2; i >= 0; i--){
	  (hmcp->pTxBuf)[i] = 0;
	  (hmcp->pTxBuf)[i] = (REG_PHASE >> 8*(2-i)) & (0xFF);
  }
  mcp3909_SPI_WriteReg(hmcp, PHASE, REG_LEN);

  for(uint8_t i = 2; i >= 0; i--){
	  (hmcp->pTxBuf)[i] = 0;
	  (hmcp->pTxBuf)[i] = (REG_GAIN >> 8*(2-i)) & (0xFF);
  }
  mcp3909_SPI_WriteReg(hmcp, GAIN, REG_LEN);

  for(uint8_t i = 2; i >= 0; i--){
	  (hmcp->pTxBuf)[i] = 0;
	  (hmcp->pTxBuf)[i] = (REG_STATUS >> 8*(2-i)) & (0xFF);
  }
  mcp3909_SPI_WriteReg(hmcp, STATUS, REG_LEN);

  for(uint8_t i = 2; i >= 0; i--){
	  (hmcp->pTxBuf)[i] = 0;
	  (hmcp->pTxBuf)[i] = (REG_CONFIG >> 8*(2-i)) & (0xFF);
  }
  mcp3909_SPI_WriteReg(hmcp, CONFIG, REG_LEN);

  return mcp3909_verify(hmcp);
}

// Returns 1 if verificaiton success
// Returns 0 if verification failed or error
uint8_t mcp3909_verify(MCP3909HandleTypeDef * hmcp){
	if(mcp3909_SPI_ReadReg(hmcp, MOD, READ_TYPE)){
	  // Set up temporary register containers for modification
	  uint32_t REG_PHASE = 0;
	  uint32_t REG_GAIN = 0;
	  uint32_t REG_STATUS = 0;
	  uint32_t REG_CONFIG = 0;

	  // Global chip settings:
	  // First byte of REG_CONFIG
	  REG_CONFIG |= hmcp->extCLK;			// Clock source setting
	  REG_CONFIG |= (hmcp->extVREF) << 1;	// Voltage reference setting
	  REG_CONFIG |= (hmcp->prescale) << 2;	// Prescaler setting
	  REG_CONFIG |= (hmcp->osr) << 4;		// Over Sampling Ratio setting

	  REG_STATUS |= (hmcp->readType) << 22;	// Read configuration to register type
	  REG_STATUS |= 1 << 14;				// 3 Cycle latency to let sinc3 settle

	  // Channel specific settings:
	  for(uint8_t i = 0; i < MAX_CHANNELS; i++){
		  REG_CONFIG |= (hmcp->channel[i].dither) << (6+i);	// Dither controls

		  // Set mode at last; ADC_SHUTDOWN will override clock and vref source settings
		  switch((hmcp->channel[i]).mode){
		  case ADC_SHUTDOWN:
			  REG_CONFIG |= EXT_CLK;
			  REG_CONFIG |= EXT_VREF << 1;
			  REG_CONFIG |= 1 << (12+i);
			  REG_CONFIG |= 0 << (18+i);
			  break;

		  case	ADC_RESET:
			  REG_CONFIG |= 0 << (12+i);
			  REG_CONFIG |= 1 << (18+i);
			  break;

		  case ADC_ON:
			  REG_CONFIG |= 0 << (12+i);
			  REG_CONFIG |= 0 << (18+i);
			  break;
		  }

		  // Set resolution
		  REG_STATUS |= ((hmcp->channel[i]).resolution) << (15+i);

		  // Set channel gain
		  REG_GAIN |= ((hmcp->channel[i]).PGA) << (4*i);
		  REG_GAIN |= ((hmcp->channel[i]).boost) << (4*i + 3);
	  }

	  // Set phase registers
	  REG_PHASE |= (hmcp->phase[0]);		// CH4 & CH5
	  REG_PHASE |= (hmcp->phase[1]) << 8;	// CH3 & CH2
	  REG_PHASE |= (hmcp->phase[2]) << 16;	// CH1 & CH0

	  uint32_t tempRegs[5];		// Holds all the register configurations to be assembled
	  for(uint8_t i = 1; i < 5; i++){
		  tempRegs[i]=0;
		  // Assemble uint32_t from 3 uint8_t
		  for(uint8_t j = 2; j >=0; j--){
			  tempRegs[i] |= (hmcp->pRxBuf[i*3 + j]) << (8*(2-j));
		  }
	  }

	  tempRegs[3] &= ~(1 << 21);	// Set bit 21 of STATUS register to 0 - state of internal use register unpredictable

	  if(tempRegs[1] != REG_PHASE){
		  return 0;
	  }
	  if(tempRegs[2] != REG_GAIN){
		  return 0;
	  }
	  if(tempRegs[3] != REG_STATUS){
		  return 0;
	  }
	  if(tempRegs[4] != REG_CONFIG){
		  return 0;
	  }
	  return 1;		// All registers match configurations
	} else {
		return 0;
	}
}

// Put all channels into shutdown mode, vref=clk=1;
uint8_t mcp3909_shutdown_all_channels(MCP3909HandleTypeDef * hmcp){

}

uint8_t mcp3909_channel_dataReady(MCP3909HandleTypeDef * hmcp, uint8_t channelNum){
	if(mcp3909_SPI_readReg(hmcp, STATUS, READ_SINGLE)){
		if(((hmcp->pRxBuf[2] >> channelNum) & 0x1)){
			return 1;
		}
	}
	return 0;
}

// Read the value of a single channle
uint32_t readChannel(MCP3909HandleTypeDef * hmcp, uint8_t channelNum, uint32_t buf){
	// Wake channels

	// Check data ready

	// Shutdown channels
}

// Read the value of a pair of channels
uint8_t readChannelPair(MCP3909HandleTypeDef * hmcp, uint8_t channelGroup, uint32_t * buf){
	// Wake channels

	// Check for data ready

	// Shutdown channels
}
