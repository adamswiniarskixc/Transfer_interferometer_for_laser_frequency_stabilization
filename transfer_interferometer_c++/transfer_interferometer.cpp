// Basic code for generating sawtooth signal and reading data (without fitting sine and extracting phases)

//g++ -o transfer_interferometer transfer_interferometer.cpp BCM2835_Driver.o ADS1256_Driver.o -lbcm2835

#include <cstdio>
#include <iostream>
#include <fstream>

#include <stdlib.h>
#include <stdio.h>
#include <signal.h>

#include <string.h>
#include <math.h>

#include "bcm2835.h"

#include "BCM2835_Driver.h"
#include "ADS1256_Driver.h"

using namespace std;

// MCP4728 DAC address
#define MCP4728_ADDRESS 0x60

// DAC channels
#define DAC_CHANNEL_A 0x40

// Amplitude of the sawtooth waveform
#define Amplitude 4095 // 12-bit resolution

// Number of steps for the sawtooth waveform
#define StepsUp 50
#define StepsDown 20

// Custom analog write function for MCP4728
void analogWrite(uint8_t chan, uint16_t value) {
    uint8_t data[3];

    // Limit check value
    value = (value > Amplitude) ? Amplitude : value;

    // MCP4728 expects a 12-bit data stream in three bytes
    data[0] = DAC_CHANNEL_A | chan;
    data[1] = (value >> 8) & 0xFF;
    data[2] = value & 0xFF;

    // Write the data to the DAC
    bcm2835_i2c_write((const char *)data, 3);
}

void exit_handler(int signum) {
    BCM2835_GPIO_Exit();
    exit(0);
}

int main() {
	if (!bcm2835_init()) {
        printf("bcm2835_init failed. Are you running as root?\n");
        return 1;
    }

    if (!bcm2835_i2c_begin()) {
        printf("bcm2835_i2c_begin failed. Are you running as root?\n");
        bcm2835_close();
        return 1;
    }

    // Set I2C clock speed (MCP4728 supports up to 3.4MHz)
    bcm2835_i2c_setClockDivider(300);
	
    // Create the sawtooth wave table
    int tab[StepsUp + StepsDown];
    int up = Amplitude / StepsUp;
    int down = Amplitude / StepsDown;
    for (int i = 0; i < StepsUp; ++i) {
        tab[i] = i * up;
    }
    for (int i = 0; i < StepsDown; ++i) {
        tab[i + StepsUp] = Amplitude - i * down;
    }
	
	
    signal(SIGINT, exit_handler);
	
    if (!BCM2835_SPI_Initialize()) {
	    return EXIT_FAILURE;
    }
	
    ADS1256_Reset();
	
    ADS1256_Configure(ADS1256_GAIN_1, ADS1256_30000_SPS);
	
    ADS1256_Set_Channel_Differential(ADS1256_AIN1, ADS1256_AIN0);

    uint32_t rword;
    double voltage;
    
    // Open a file for writing
    ofstream outFile("test_data.txt");
    	
    while (true) {
	for (int i = 0; i < StepsUp; ++i) {
	    analogWrite(0, tab[i]);
	    rword = ADS1256_Read_Word(false);
	    voltage = rword * 5.12 / 0x7FFFFF;
	    cout << voltage << endl;
	    outFile << voltage << endl;
	}
	
	for (int i = StepsUp; i < StepsUp + StepsDown; ++i) {
	    analogWrite(0, tab[i]);
	}
    }
    outFile.close();
	
	
    bcm2835_i2c_end();
    bcm2835_close();
    
    return 0;
}
