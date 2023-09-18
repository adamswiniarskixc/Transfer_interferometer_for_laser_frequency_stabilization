// Generating sawtooth signal and reading data without fitting
// Multithreading was tested but without a success

//g++ -o transfer_interferometer_v2 transfer_interferometer_v2.cpp BCM2835_Driver.o ADS1256_Driver.o -lbcm2835 -lpthread

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

#include <thread>
#include <mutex>

#include <vector>

using namespace std;

// Global mutex for synchronization
mutex mtx;

// MCP4728 DAC address
#define MCP4728_ADDRESS 0x60

// DAC channels
#define DAC_CHANNEL_A 0x40

// Amplitude of the sawtooth waveform
#define Amplitude 4095 // 12-bit resolution

// Number of steps for the sawtooth waveform
#define StepsUp 50
#define StepsDown 20

#define MONITOR_ERR_REF 1
#define MONITOR_FIT_REF 2
#define MONITOR_SET_PHASE_REF 3
#define MONITOR_RESIDUE_REF 4

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

// Function to write to DAC
void dacTask(const int* waveform, int startIndex, int batchSize) {
	for (int i = 0; i < batchSize; ++i) {
		analogWrite(0, waveform[startIndex + i]);
	}
}

// Function to read from ADC
void adcTask(vector<uint32_t>& adcReadings, int startIndex, int batchSize) {
    for (int i = 0; i < batchSize; ++i) {
        uint32_t rword = ADS1256_Read_Word(false);
        mtx.lock(); // Lock the mutex before writing to shared data
        adcReadings[startIndex + i] = rword;
        mtx.unlock(); // Unlock the mutex after writing
    }
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
    int tab1[StepsUp + StepsDown];
    int up = Amplitude / StepsUp;
    int down = Amplitude / StepsDown;
    for (int i = 0; i < StepsUp; ++i) {
        tab1[i] = i * up;
    }
    for (int i = 0; i < StepsDown; ++i) {
        tab1[i + StepsUp] = Amplitude - i * down;
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
    int batchSize = 1;

    while (true) {
	for (int batchStart = 0; batchStart < StepsUp + StepsDown; batchStart += batchSize) {
	    int remainingSteps = StepsUp + StepsDown - batchStart;
	    int currentBatchSize = min(batchSize, remainingSteps);
		
	    // Create a vector to store ADC readings for the current batch
	    vector<uint32_t> adcReadings(currentBatchSize);
	    // Create threads for DAC write and ADC read tasks
	    thread dacThread(dacTask, tab1, batchStart, currentBatchSize);
	    thread adcThread(adcTask, ref(adcReadings), batchStart, currentBatchSize);

	    // Wait for threads to finish
	    dacThread.join();
	    adcThread.join();
		}
    }
	
	
    bcm2835_i2c_end();
    bcm2835_close();
    
    return 0;
}
