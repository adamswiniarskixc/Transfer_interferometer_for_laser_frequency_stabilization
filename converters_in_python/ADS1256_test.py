import ADS1256
import time
import RPi.GPIO as GPIO
import config
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

GPIO.setwarnings(False)

# Pin definition
RST_PIN = 18
CS_PIN = 22
DRDY_PIN = 8

# Initialize ADS1256 ADC
ADC = ADS1256.ADS1256()

# Set ADC gain and data rate
gain = ADS1256.ADS1256_GAIN_E['ADS1256_GAIN_1']
drate = ADS1256.ADS1256_DRATE_E['ADS1256_30000SPS']

# Set channel to read
channel = 0

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
config.module_init()  # Initialize the config module
GPIO.setup(DRDY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Initialize ADS1256 ADC
ADC.ADS1256_reset()
ADC.ADS1256_ConfigADC(gain, drate)

ADC.ADS1256_SetMode(1) #differential mode

'''
# Animation

x_len = 100  # Maximum number of data points to display on the x-axis
y_range = [0, 6000000]

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = list(range(0, x_len))
ys = [0]*x_len
ax.set_ylim(y_range)

line, = ax.plot(xs, ys)

plt.xlabel('Samples')
plt.ylabel('Voltage [V]')

def animate(i, ys):
    value = ADC.ADS1256_GetChannelValue(channel)
    
    ys.append(value)

    # Limit y list to a set number of items
    ys = ys[-x_len:]

    line.set_ydata(ys)

    return line,

ani = FuncAnimation(fig,
                    animate,
                    fargs = (ys,),
                    interval = 1, # [ms]
                    blit = True)
'''
while(True):
    print(ADC.ADS1256_GetChannelValue(channel))
#plt.show()


'''
plt.ion()

x = []
y = []

def graph(value):
    y.append(value)
    x.append(time.time())
    plt.clf()
    plt.scatter(x, y)
    plt.plot(x, y)
    plt.draw()

while True:
    value = ADC.ADS1256_GetChannelValue(channel)
    graph(value)
'''
