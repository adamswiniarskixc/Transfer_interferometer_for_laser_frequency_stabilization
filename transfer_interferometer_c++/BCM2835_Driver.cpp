#include "BCM2835_Driver.h"

void BCM2835_GPIO_Initialize() {
	// Set pins to output
	bcm2835_gpio_fsel(ADS_RST_PIN, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_write(ADS_RST_PIN, 1);

	bcm2835_gpio_fsel(ADS_CS_PIN, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_write(ADS_CS_PIN, 1);

	// Set pins to input
	bcm2835_gpio_fsel(ADS_DRDY_PIN, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_set_pud(ADS_DRDY_PIN, BCM2835_GPIO_PUD_UP);
}

void BCM2835_GPIO_Exit() {
	bcm2835_spi_end();
	bcm2835_close();
}

int BCM2835_SPI_Initialize() {
	if (bcm2835_init()) {

		bcm2835_spi_begin();
		bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);	// SPI Mode 0 only supports MSB bit order
		bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);					// Set SPI Mode 0, hardware SPI chips
		bcm2835_spi_setClockDivider(CLOCK_DIVIDER);					// 6.250 Mhz SPI frequency

		// The ADS1256 supports 0.1 to 10 MHz external clock signal for SPI

		// Initialize GPIO with the ADS1256 chip
		BCM2835_GPIO_Initialize();

		return 1;
	}
	else {
		std::cout << "Failed to initialize BCM2835 GPIO." << std::endl;
	}

	return 0;
}