//g++ -Wall -o MCP4728_sawtooth MCP4728_sawtooth.cpp -lwiringPi

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <cstdio>

// MCP4728 DAC address
#define MCP4728_ADDRESS 0x60

// DAC channels
#define DAC_CHANNEL_A 0x40

// Amplitude of the sawtooth waveform
#define Amplitude 4095 // or 65535?

// Number of steps for the sawtooth waveform
#define StepsUp 80
#define StepsDown 20

int main() {
    // Setup WiringPi
    if (wiringPiSetup() == -1) {
        perror("Error initializing WiringPi");
        return 1;
    }

    // Initialize the I2C interface
    int i2c_fd = wiringPiI2CSetup(MCP4728_ADDRESS);
    if (i2c_fd == -1) {
        perror("Error initializing I2C");
        return 1;
    }

    // Create the sawtooth wave tables
    int tab1[StepsUp];
    int tab2[StepsDown];
    int up = static_cast<int>(Amplitude / StepsUp);
    int down = static_cast<int>(Amplitude / StepsDown);
    for (int i = 0; i < StepsUp; ++i) {
        tab1[i] = i * up;
        //printf("%d\n", tab1[i]);
    }
    for (int i = 0; i < StepsDown; ++i) {
        tab2[i] = Amplitude - i * down;
		//printf("%d\n", tab2[i]);
    }

    // Main loop to generate the sawtooth waveform
    while (true) {
        // Output the values from tab1
        for (int i = 0; i < StepsUp; ++i) {
            int value = tab1[i];
            wiringPiI2CWriteReg16(i2c_fd, DAC_CHANNEL_A, value);
            delayMicroseconds(10000);
        }

        // Output the values from tab2
        for (int i = 0; i < StepsDown; ++i) {
            int value = tab2[i];
            wiringPiI2CWriteReg16(i2c_fd, DAC_CHANNEL_A, value);
            delayMicroseconds(10000);
        }
    }

    return 0;
}
