#pragma once
#ifndef ADS1256_DRIVER_H
#define ADS1256_DRIVER_H

#include <bitset>
#include <iostream>

/* BCM2835.h Provided by AirSpayce
* https://www.airspayce.com/mikem/bcm2835/
*/
#include <bcm2835.h>
#include "BCM2835_Driver.h"

#define ADS_RST_PIN 18
#define ADS_CS_PIN 22
#define ADS_DRDY_PIN 8

#define DRDY_TIMEOUT 5000000

#define SENSOR_RESISTANCE 50

#define RPI4_CLOCK_RATE 1500000000	// 1.5 GHz
#define MASTER_CLOCK_PERIOD 1 / (RPI4_CLOCK_RATE / CLOCK_DIVIDER)

#define MAXIMUM_POWER_READING 5000

#define OUTPUT_CONSOLE true

/**
* Register map as defined in the datasheet
* https://www.ti.com/lit/ds/symlink/ads1256.pdf (Page 30)
*/
typedef enum {
	REG_STATUS			= 0x00,		// b(ID3, ID2, ID1, ID0, ORDER, ACAL, BUFEN, DRDY)
	REG_MUX				= 0x01,		// b(PSEL3, PSEL2, PSEL1, PSEL0, NSEL3, NSEL2, NSEL1, NSEL0)
	REG_ADCON			= 0x02,		// b(0, CLK1, CLK0, SDCS1, SDCS0, PGA2, PGA1, PGA0)
	REG_DRATE			= 0x03,		// b(DR7, DR6, DR5, DR4, DR3, DR2, DR1, DR0)
	REG_IO				= 0x04,		// b(DIR3, DIR2, DIR1, DIR0, DIO3, DIO2, DIO1, DIO0)
	REG_OFC0			= 0x05,		// b(OFC07, OFC06, OFC05, OFC04, OFC03, OFC02, OFC01, OFC00)
	REG_OFC1			= 0x06,		// b(OFC15, OFC14, OFC13, OFC12, OFC11, OFC10, OFC09, OFC08)
	REG_OFC2			= 0x07,		// b(OFC23, OFC22, OFC21, OFC20, OFC19, OFC18, OFC17, OFC16)
	REG_FSC0			= 0x08,		// b(FSC07, FSC06, FSC05, FSC04, FSC03, FSC02, FSC01, FSC00)
	REG_FSC1			= 0x09,		// b(FSC15, FSC14, FSC13, FSC12, FSC11, FSC10, FSC09, FSC08)
	REG_FSC2			= 0x0A,		// b(FSC23, FSC22, FSC21, FSC20, FSC19, FSC18, FSC17, FSC16)
} ADS1256_REGISTERS;


/**
* Command definitions as defined in the datasheet
* https://www.ti.com/lit/ds/symlink/ads1256.pdf (Page 34)
*/
typedef enum {
	CMD_WAKEUP			= 0x00,		// Completes SYNC and exits standby mode
	CMD_RDATA			= 0x01,		// Read Data
	CMD_RDATAC			= 0x03,		// Read Data Continuously
	CMD_SDATAC			= 0x0F,		// Stop Read Data Continuously
	CMD_RREG			= 0x10,		// Read from REG rrrr
	CMD_WREG			= 0x50,		// Write from REG rrrr
	CMD_SELFCAL			= 0xF0,		// Offset and Gain Self-Calibration
	CMD_SELFOCAL		= 0xF1,		// Offset Self-Calibration
	CMD_SELFGCAL		= 0xF2,		// Gain Self-Calibration
	CMD_SYSOCAL			= 0xF3,		// System Offset Calibration
	CMD_SYSGCAL			= 0xF4,		// System Gain Calibration
	CMD_SYNC			= 0xFC,		// Synchronize the A/D Conversion
	CMD_STANDBY			= 0xFD,		// Begin Standby Mode
	CMD_RESET			= 0xFE,		// Reset to Power-Up Values
	CMD_WAKEUP_1		= 0xFF,		// Completes SYNC and exits standby mode
} ADS1256_COMMANDS;


/**
* Programmable gain amplifier levels with respective register values
* https://www.ti.com/lit/ds/symlink/ads1256.pdf (Page 31)
*/
typedef enum {
	ADS1256_GAIN_1		= 0x00,		// Default
	ADS1256_GAIN_2		= 0x01,
	ADS1256_GAIN_4		= 0x02,
	ADS1256_GAIN_8		= 0x03,
	ADS1256_GAIN_16		= 0x04,
	ADS1256_GAIN_32		= 0x05,
	ADS1256_GAIN_64		= 0x06,
	ADS1256_GAIN_MAX	= 0x07,		// Equals 64 Gain
} ADS1256_GAIN;


/**
* Available data rates with their respective register values
* https://www.ti.com/lit/ds/symlink/ads1256.pdf (Page 18)
*/
typedef enum {
	ADS1256_30000_SPS	= 0xF0,		// 1 Average (Bypassed) Default
	ADS1256_15000_SPS	= 0xE0,		// 2 Averages
	ADS1256_7500_SPS	= 0xD0,		// 4 Averages
	ADS1256_3750_SPS	= 0xC0,		// 8 Averages
	ADS1256_2000_SPS	= 0xB0,		// 15 Averages
	ADS1256_1000_SPS	= 0xA1,		// 30 Averages
	ADS1256_500_SPS		= 0x92,		// 60 Averages
	ADS1256_100_SPS		= 0x82,		// 300 Averages
	ADS1256_60_SPS		= 0x72,		// 500 Averages
	ADS1256_50_SPS		= 0x63,		// 600 Averages
	ADS1256_30_SPS		= 0x53,		// 1000 Averages
	ADS1256_25_SPS		= 0x43,		// 1200 Averages
	ADS1256_15_SPS		= 0x33,		// 2000 Averages
	ADS1256_10_SPS		= 0x23,		// 3000 Averages
	ADS1256_5_SPS		= 0x13,		// 6000 Averages
	ADS1256_2d5_SPS		= 0x03,		// 12000 Averages
} ADS1256_DRATE;


/**
* Available channels and their MUX values
* https://www.ti.com/lit/ds/symlink/ads1256.pdf (Page 31)
*/
typedef enum {
	ADS1256_AIN0		= 0x00,
	ADS1256_AIN1		= 0x01,
	ADS1256_AIN2		= 0x02,
	ADS1256_AIN3		= 0x03,
	ADS1256_AIN4		= 0x04,
	ADS1256_AIN5		= 0x05,
	ADS1256_AIN6		= 0x06,
	ADS1256_AIN7		= 0x07,
	ADS1256_AINCOM		= 0x08,
} ADS1256_CHANNELS;


/**
 * Sets the Chip Select pin on the ADS
 */
void ADS1256_Chip_Select(bool signal);

/**
* Checks if the DRDY pin is low
* 
* @return bool True if low else false
*/
bool ADS1256_Data_Ready();

/**
* Poll for the DRDY pin until low
* 
* @return int 1 if DRDY went low, 0 if timed out, see DRDY_TIMEOUT constant
*/
int ADS1256_Poll_Data_Ready();


/**
 * Send a byte via SPI with a little delay
 */
void ADS1256_Send_Byte(uint8_t byte);

/**
 * Read a byte via SPI
 * 
 * @return uint8_t Read data byte from the SPI slave
 */
uint8_t ADS1256_Read_Byte();

/*
* Read 3 bytes via SPI and pack them into a word
* 
* @param is_RDATAC is the chip in read data continously mode?
* If the chip is not in RDATAC, the function will call RDATA and read the next byte manually
*/
uint32_t ADS1256_Read_Word(bool is_RDATAC);

/**
* Resets the ADS1256 chip
*/
void ADS1256_Reset();

/**
* Configures the ADS1256's gain and data rate by parameters
* 
* @param gain level from the ADS1256_GAIN type
* @param rate from the ADS1256_DRATE type
*/
void ADS1256_Configure(ADS1256_GAIN gain, ADS1256_DRATE rate);

/**
* Send a command to the ADS1256
* 
* Pulls the CS pin low then sends the command and raises the CS pin high
* 
* @param ADS1256_COMMANDS The command defined in the ADS1256_COMMANDS enum
*/
void ADS1256_Send_Cmd(ADS1256_COMMANDS CMD);

/**
* Write to a register on the ADS1256
* 
* Pulls the CS pin low then sends the WREG command 
* followed by the data and then raises the CS pin high
*
* @param ADS1256_REGISTERS The register defined in the ADS1256_REGISTERS enum
* @param uint8_t The value to set the register
*/
void ADS1256_WREG(ADS1256_REGISTERS REG, uint8_t DATA);

/**
* Write to a register on the ADS1256
* 
* Pulls the CS pin low then sends the WREG command
* followed by the data and then raises the CS pin high
*
* @param ADS1256_REGISTERS REG. The register defined in the ADS1256_REGISTERS enum
* @return uint8_t The value from REG
*/
uint8_t ADS1256_RREG(ADS1256_REGISTERS REG);

/*
* Sets the ADS1256 chip into Read Data Continuous mode
* This enables the continuous output of new data on each DRDY without subsequent read commands
* @return int32_t 24 bits of output data packed into a single word
*/
void ADS1256_Enable_RDATAC();

/**
* Set the channel to be read
* @param ADS1256_CHANNELS A channel as defined in the ADS1256_CHANNELS enum
*/
void ADS1256_Set_Channel(uint8_t CHANNEL);

void ADS1256_Set_Channel_Differential(uint8_t POS_CHANNEL, uint8_t NEG_CHANNEL);

/**
* Dump the contents of all registers on the ADS1256 chip in binary form to cout
*/
void ADS1256_Dump_Registers();

/**
* Reads the ADS1256 status register via RREG and isolates the ID bits
* @return int Chip ID
*/
int ADS1256_Get_Id();

/**
* Reads the ADS1256 status register via RREG and isolates the BUFFEN (Buffer Enabled) bit
* @return bool Buffer enabled
*/
bool ADS1256_BUFFEN();

/**
* Reads the ADS1256 status register via RREG and isolates the ACAL (Auto calibration) bit
* @return bool Auto calibration enabled
*/
bool ADS1256_ACAL();

/**
* Reads the ADS1256 DRATE register via RREG
* @return uint8_t Datarate in samples per second
*/
uint8_t ADS1256_GET_DRATE();

#endif
