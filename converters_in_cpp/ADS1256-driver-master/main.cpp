//g++ -o main main.cpp BCM2835_Driver.cpp ADS1256_Driver.cpp MetricLogger.cpp -lbcm2835
//g++ -o main main.cpp BCM2835_Driver.o ADS1256_Driver.o MetricLogger.o -lbcm2835

#include <cstdio>
#include <iostream>

#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <string.h>
#include <math.h>

#include "bcm2835.h"

#include "BCM2835_Driver.h"
#include "ADS1256_Driver.h"
#include "MetricLogger.h"

void exit_handler(int signum) {
    BCM2835_GPIO_Exit();
    exit(0);
}

int main() {
    signal(SIGINT, exit_handler);

    if (!BCM2835_SPI_Initialize()) {
        return EXIT_FAILURE;
    }

    // Reset the chip to default values, avoid unexpected configurations
    ADS1256_Reset();
/*
    // Keep trying to read the chip ID, which is set by the manufacturer as 0x3
    while (ADS1256_Get_Id() != 3) {
        std::cout << "Attemping to establish SPI communications.." << std::endl;
        bcm2835_delay(1000);
    }
*/
    ADS1256_Configure(ADS1256_GAIN_1, ADS1256_30000_SPS);

    ADS1256_Set_Channel(ADS1256_AIN0);

    if (OUTPUT_CONSOLE) {

        std::cout
            << "# Chip ID           : " << (int)ADS1256_Get_Id() << std::endl
            << "# Buffer            : " << (ADS1256_BUFFEN() ? "Enabled" : "Disabled") << std::endl
            << "# Auto calibration  : " << (ADS1256_ACAL() ? "Enabled" : "Disabled") << std::endl;
    
    }

    uint32_t rword;
    double dataSum, dataAverage, vRms, i_sec, num_samples;
    double power, acc_power;
    double voltageSum, voltageAverage;

    for (;;) {

        num_samples = 0;
        dataSum = 0;
        dataAverage = 0;
        
        voltageSum = 0;
        voltageAverage = 0;

        time_t now = time(0);
        time_t next = now + 1;
/*
        // Collect exactly one second of samples

        while (time(0) < next) {

            // Sample a series of channels to reduce sinc filter decimation of the signal
            // This is due to the signal often resembling noise

            for (uint8_t i = ADS1256_AIN2; i < ADS1256_AINCOM; i++) {

                ADS1256_Set_Channel(i);

                rword = ADS1256_Read_Word(false);

                if (i == ADS1256_AIN2) {

                    num_samples++;
                    //dataSum += pow((rword * 5.0 / 0x7FFFFF) - 2.5, 2);
                    dataSum += pow((rword * 5.12 / 0x7FFFFF) - 2.5, 2);
                    voltageSum += rword * 5.12 / 0x7FFFFF;
                }

            }

        }
*/

        while (time(0) < next) {
            ADS1256_Set_Channel_Differential(ADS1256_AIN1, ADS1256_AIN0);
            
            rword = ADS1256_Read_Word(false);
            
            num_samples++;
            dataSum += pow((rword * 5.12 / 0x7FFFFF) - 2.5, 2);
            voltageSum += rword * 5.12 / 0x7FFFFF;
        }

        // Calculate power readings

        dataAverage = dataSum / num_samples;
        voltageAverage = voltageSum / num_samples;
        
        vRms = sqrt(dataAverage);

        i_sec = vRms / SENSOR_RESISTANCE;

        // 240V * Secondary current * Inductor Windings
        power = 240 * (i_sec * 2000);

        // Current drawn power for one second
        acc_power += (power / 1000) / (60 * 60);

        if (OUTPUT_CONSOLE) {
            std::cout << "# Number of samples : " << num_samples << std::endl;
            //std::cout << "# Burden current    : " << i_sec << std::endl;
            //std::cout << "# Primary current   : " << i_sec * 2000 << std::endl;
            //std::cout << "# Power             : " << power << std::endl;
            std::cout << "# Voltage           : " << voltageAverage << std::endl;
            
            // Move the cursor up 5 places
            for (size_t i = 0; i < 2; i++) {
                std::cout << "\x1b[A";
            }
            
        }

        // Simple spike filter, should be adjusted based on circumstances
        if (power < MAXIMUM_POWER_READING) {
            MetricLogger::AppendToLiveData(power);
            MetricLogger::AppendToHistoricData(power);
            MetricLogger::SetMetrics(power, acc_power);
        }

    }

    BCM2835_GPIO_Exit();

    return 0;
}
