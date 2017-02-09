/*
 * mcp3909.h
 *
 *  Created on: Jan 2, 2017
 *      Author: frank
 */

#ifndef MCP3909_H_
#define MCP3909_H_

#include "stm32l4xx_hal.h"

// NOTE: the Tx Buffer should always have the first byte empty for the CONTROL BYTE!
// Tx and Rx buffers must both be declared and allocated globally before the functions

// States: INIT, SHUTDOWN

// Register Addresses
#define CHANNEL_0   0x00
#define CHANNEL_1   0x01
#define CHANNEL_2   0x02
#define CHANNEL_3   0x03
#define CHANNEL_4   0x04
#define Channle_5   0x05
#define MOD         0x06U
#define PHASE       0x07U
#define GAIN        0x08U
#define STATUS      0x09
#define CONFIG      0x0A

#define SPI_TIMEOUT	100

// Programmable gain amplifier ratio
typedef enum {
  PGA_1 = 0,
  PGA_2,
  PGA_4,
  PGA_8,
  PGA_16,
  PGA_32
} PGA_Conf;

typedef enum {
  ADC_SHUTDOWN,           // Converters and biases off (low-power mode)
  ADC_RESET,              // Converters active, output forced to 0
  ADC_ON                  // Converters active, output adc values
} ADC_mode;

// Analog main clock prescaler
typedef enum {
  PRESCALE_1 = 0,         // Default
  PRESECLE_2,
  PRESCALE_4,
  PRESCALE_8
} prescale_CONF;

// Oversampling ratio
typedef enum {
  OSR_32 = 0,
  OSR_64,                 // Default
  OSR_128,
  OSR_256
} OSR_CONF;

// ADC dither control
#define DITHER_ON     1
#define DITHER_OFF    0

// ADC resolution mode
#define RES_24        1
#define RES_16        0

// ADC boost mode
#define BOOST_ON      1
#define BOOST_OFF     0

// Reference voltage source
#define EXT_VREF      1    // External Vref
#define INT_VREF      0    // Internal Vref (2.35V +- 2%)

// Clock source setting
#define EXT_CLK       1    // External high speed clock to OSC1
#define OSC_CLK       0    // Crystal oscillator between OSC1 and OSC2

#define READ_SINGLE   0    // Read a single register
#define READ_GROUP    1    // Read a register group
#define READ_TYPE     2    // Read a register type
#define REAL_ALL      3    // Read all onboard registers

#define REG_LEN		  3	   // Register data length

// Register read lengths
#define READ_SINGLE_LEN			3
#define READ_STATUS_GROUP_LEN	6
#define READ_CHN_GROUP_LEN		6
#define READ_MOD_GROUP_LEN		9
#define READ_TYPE_LEN			18
#define READ_ALL_LEN			33

#define MAX_CHANNELS  6    // 6 ADC Channels on MCP3909

typedef struct {
  uint8_t   channel;      // Channel number
  uint8_t	readType;
  PGA_Conf  PGA;          // ADC gain setting
  ADC_mode  mode;          // ADC operating mode selection
  uint8_t   dither;       // ADC dither filter
  uint8_t   resolution;   // ADC resolution
  uint8_t   boost;        // ADC boost mode
} channel_Conf;

typedef struct {
  SPI_HandleTypeDef *	hspi;	// SPI Handle object
  uint8_t		readType;		// Read single, type, group, all registers
  prescale_CONF prescale;
  OSR_CONF      osr;
  uint8_t       extCLK;
  uint8_t       extVREF;
  channel_Conf  channel[6];
  uint8_t *		phase;
  uint8_t *     pRxBuf;     // Rx Buffer
  uint8_t *     pTxBuf;     // Tx Buffer
} MCP3909HandleTypeDef;


// Initializes mcp3909
// Sets clock source, reference source, prescaler, and OSR
// Writes the commands to SPI
uint8_t mcp3909_init(SPI_HandleTypeDef * hspi, MCP3909HandleTypeDef * hmcp);

// Reads back all configuration registers from MCP3909 and verifies against the defined handle
uint8_t mcp3909_verify(MCP3909HandleTypeDef * hmcp);

// Put all channels into shutdown mode, vref=clk=1;
uint8_t mcp3909_shutdown_all_channels(MCP3909HandleTypeDef * hmcp);

// SPI Utilities
uint8_t mcp3909_SPI_WriteReg(MCP3909HandleTypeDef * hmcp, uint8_t address, uint8_t length);

uint8_t mcp3909_SPI_ReadReg(MCP3909HandleTypeDef * hmcp, uint8_t length, uint8_t readType);

uint8_t mcp3909_channel_dataRead(MCP3909HandleTypeDef * hmcp, uint8_t channelNum);

uint32_t readChannel(MCP3909HandleTypeDef * hmcp, uint8_t channelNum, uint32_t buf);

uint8_t readChannelPair(MCP3909HandleTypeDef * hmcp, uint8_t channelGroup, uint32_t * buf);

#endif /* MCP3909_H_ */
