#include "ADS1256_Driver.h"

void ADS1256_Chip_Select(bool signal) {
	bcm2835_gpio_write(ADS_CS_PIN, signal);
}

bool ADS1256_Data_Ready() {
	return (bcm2835_gpio_lev(ADS_DRDY_PIN) == LOW);
}

int ADS1256_Poll_Data_Ready() {
	bcm2835_delayMicroseconds(5);

	for (int i = 0; i < DRDY_TIMEOUT; i++) {
		if (ADS1256_Data_Ready()) {
			return 1;
		}
	}

	return 0;
}

void ADS1256_Send_Byte(uint8_t byte) {
	bcm2835_delayMicroseconds(2);
	bcm2835_spi_transfer(byte);
}

uint8_t ADS1256_Read_Byte() {
	return bcm2835_spi_transfer(CMD_WAKEUP);
}

uint32_t ADS1256_Read_Word(bool is_RDATAC) {
	uint32_t word;

	ADS1256_Poll_Data_Ready();

	ADS1256_Chip_Select(LOW);

	if (!is_RDATAC) {
		ADS1256_Send_Byte(CMD_RDATA);
		bcm2835_delayMicroseconds(1000);
	}

	// Packing bytes into a 32 bit word, 24 bits are used
	word = ((uint32_t)ADS1256_Read_Byte() << 16) & 0x00FF0000;	// MSB
	word |= ((uint32_t)ADS1256_Read_Byte() << 8);				// MID BYTE
	word |= ADS1256_Read_Byte();								// LSB

	ADS1256_Chip_Select(HIGH);

	// Check if the word is signed
	if (word & 0x800000) {

		// Extend the sign
		word &= (0xFF000000);

	}

	return word;
}

void ADS1256_Reset() {
	bcm2835_gpio_write(ADS_RST_PIN, 0);
	bcm2835_delay(5);

	bcm2835_gpio_write(ADS_RST_PIN, 1);
	bcm2835_delay(5);
}

void ADS1256_Configure(ADS1256_GAIN gain, ADS1256_DRATE rate) {
	ADS1256_Poll_Data_Ready();

	uint8_t config[4] = { 0, 0, 0, 0 };

	// Status Register	| Auto Calibration, Analog Input Buffer Disabled
	config[0] = 0x4;

	// MUX Register		| AIN 0 and AINCOM
	config[1] = 0x8;

	// ADCON Register	| PGA Settings from gain variable
	config[2] = gain;

	// DRATE Register	| Data rate from rate variable
	config[3] = rate;

	ADS1256_Chip_Select(LOW);

	// Start writing from register 0 and continue writing 3 registers deep
	ADS1256_Send_Byte(CMD_WREG);
	ADS1256_Send_Byte(0x03);

	for (int i = 0; i < 4; i++) {
		ADS1256_Send_Byte(config[i]);
	}

	ADS1256_Chip_Select(HIGH);

	bcm2835_delayMicroseconds(500);
}

void ADS1256_Send_Cmd(ADS1256_COMMANDS CMD) {
	ADS1256_Chip_Select(LOW);

	ADS1256_Send_Byte(CMD);

	ADS1256_Chip_Select(HIGH);
}

void ADS1256_WREG(ADS1256_REGISTERS REG, uint8_t DATA) {
	ADS1256_Chip_Select(LOW);

	// Send Write Register command
	ADS1256_Send_Byte(CMD_WREG | REG);
	ADS1256_Send_Byte(0x00);	// Single byte to write

	// Data to be written to register
	ADS1256_Send_Byte(DATA);

	ADS1256_Chip_Select(HIGH);
}

uint8_t ADS1256_RREG(ADS1256_REGISTERS REG) {
	uint8_t value = 0;

	ADS1256_Poll_Data_Ready();

	ADS1256_Chip_Select(LOW);

	ADS1256_Send_Byte(CMD_RREG | REG);	// 1-ST Control Byte: Read from REG
	ADS1256_Send_Byte(0x00);			// 2-ND Control Byte: Read from a single register

	bcm2835_delayMicroseconds(30);

	value = ADS1256_Read_Byte();

	ADS1256_Chip_Select(HIGH);

	return value;
}

void ADS1256_Enable_RDATAC() {
	// Requires DRDY to be low to issue command
	ADS1256_Poll_Data_Ready();

	ADS1256_Send_Cmd(CMD_RDATAC);

	bcm2835_delayMicroseconds(5);
}

void ADS1256_Set_Channel(uint8_t CHANNEL) {
	// Single-ended mode:
	ADS1256_WREG(REG_MUX, (CHANNEL << 4) | (1 << 3));

	// Synchronize the new MUX register value
	ADS1256_Send_Cmd(CMD_SYNC);

	// Complete sync and wakeup
	ADS1256_Send_Cmd(CMD_WAKEUP);
}

void ADS1256_Set_Channel_Differential(uint8_t POS_CHANNEL, uint8_t NEG_CHANNEL) {
	// Differential mode:
	ADS1256_WREG(REG_MUX, ((POS_CHANNEL << 4) | (1 << 3)) | NEG_CHANNEL);

	// Synchronize the new MUX register value
	ADS1256_Send_Cmd(CMD_SYNC);

	// Complete sync and wakeup
	ADS1256_Send_Cmd(CMD_WAKEUP);
}

void ADS1256_Dump_Registers() {
	// Iterate all registers
	for (uint8_t i = REG_STATUS; i < REG_FSC2; i++) {
		std::bitset<8> reg(ADS1256_RREG((ADS1256_REGISTERS)i));

		std::cout << (ADS1256_REGISTERS)i << ":  " << reg << std::endl;
	}
}

int ADS1256_Get_Id() {
	// Shift the MSB 4 bits, since they represent the chip ID
	return (ADS1256_RREG(REG_STATUS) >> 4);
}

bool ADS1256_BUFFEN() {
	// Bit 1 is the BUFFEN toggle bit
	return (ADS1256_RREG(REG_STATUS) & 0x2);
}

bool ADS1256_ACAL() {
	// Bit 2 is the ACAL toggle bit
	return (ADS1256_RREG(REG_STATUS) & 0x4);
}

uint8_t ADS1256_GET_DRATE() {
	return ADS1256_RREG(REG_DRATE);
}
