#pragma once
#ifndef BCM2835_DRIVER_H
#define BCM2835_DRIVER_H

#include <iostream>

/* BCM2835.h Provided by AirSpayce
* https://www.airspayce.com/mikem/bcm2835/
*/
#include <bcm2835.h>

#define ADS_RST_PIN 18
#define ADS_CS_PIN 22
#define ADS_DRDY_PIN 8

#define CLOCK_DIVIDER 64	// Advisably product of 2^n in range (1 - 32786)

/**
 * Begin SPI operations and assign pins required
*/
void BCM2835_GPIO_Initialize();

/**
 * End SPI operations and return assigned pins to default input
 * Close BCM2835 library
*/
void BCM2835_GPIO_Exit();

/**
 * Initialize BCM2835 library and SPI protocol
 * @return 1 if successful otherwise 0
*/
int BCM2835_SPI_Initialize();
#endif
