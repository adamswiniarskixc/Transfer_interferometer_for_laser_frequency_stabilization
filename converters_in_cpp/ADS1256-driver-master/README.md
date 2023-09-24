# ADS1256-driver
A driver for an analog to digital converter named ADS1256 created by Texas Instruments on RPi using the BCM2835 SoC Broadcom chip.

## Configuration

### ADS1256 Driver
Various configuration are located at the top of the header file -
* Data ready timeout
* Sensor resistance (The resistance on the sensor fed into the converter)
* Maximum power reading (For clipping)
* Write to console (Disable for less overhead)

### MetricLogger
Configure the paths to your liking in the MetricLogger.h file to log to a specific directory.
