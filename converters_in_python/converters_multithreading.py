import ADS1256
import time
import numpy as np
import RPi.GPIO as GPIO
import config
import board
import adafruit_mcp4728
from multiprocessing import Process, Queue

GPIO.setwarnings(False)

# MCP4728 initialization
i2c = board.I2C()   # uses board.SCL and board.SDA
dac = adafruit_mcp4728.MCP4728(i2c)

# ADS1256 pins
RST_PIN = 18
CS_PIN = 22
DRDY_PIN = 8

# ADS1256 initialization
ADC = ADS1256.ADS1256()

gain = ADS1256.ADS1256_GAIN_E['ADS1256_GAIN_1']
drate = ADS1256.ADS1256_DRATE_E['ADS1256_30000SPS']

# ADC channel to read
channel = 0

# GPIO initialization
GPIO.setmode(GPIO.BCM)
config.module_init()  # Initialize the config module
GPIO.setup(DRDY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ADS1256 setup
ADC.ADS1256_reset()
ADC.ADS1256_ConfigADC(gain, drate)

ADC.ADS1256_SetMode(1) #differential mode

# Fitting matrix for extracting phase
def get_fitting_matrix(n_steps, fit_freq, n_steps_skip=10):
    #n_steps - number of data points
    #fit_freq - frequency of the sine and cosine function to fit to
    #n_steps_skip - skip the first n_steps_skip number of points for fitting

    n_use = n_steps - n_steps_skip

    time = n_steps_skip + np.arange(n_use, dtype=float)
    sin_t = np.sin(2*np.pi*time/n_steps*fit_freq)
    cos_t = np.cos(2*np.pi*time/n_steps*fit_freq)
    offset = np.ones(len(time))

    D = np.vstack([sin_t, cos_t, offset]).T

    DTD = np.dot(D.T, D)
    compute_matrix = np.dot(np.linalg.inv(DTD),D.T)[:2, :]
    compute_matrix_padded = np.zeros((2, n_steps))
    compute_matrix_padded[:, n_steps_skip:] = compute_matrix
    flattened_array = np.array(compute_matrix_padded.flatten(), dtype='float32')
    return flattened_array    

# Sawtooth signal

Amplitude = 65535

steps_up = 50  # ramp up
steps_down = 20  # ramp down

tab1 = range(0, Amplitude, int(Amplitude/steps_up))
tab2 = range(Amplitude, 0, -int(Amplitude/steps_down))

# Queue for reading the signal using ADC
value_queue = Queue()

def read_value(queue):
    while True:
        value = ADC.ADS1256_GetChannalValue(channel)
        queue.put(value)
        print(value)

value_process = Process(target=read_value, args=(value_queue,))
value_process.start()

while True:
    for i in tab1:
        dac.channel_a.value = i
        # Read the value from the queue
        if not value_queue.empty():
            value = value_queue.get()

    for i in tab2:
        dac.channel_a.value = i

value_process.terminate()
