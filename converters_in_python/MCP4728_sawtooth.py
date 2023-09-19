import board
import adafruit_mcp4728
from time import sleep

i2c = board.I2C()   # uses board.SCL and board.SDA
dac = adafruit_mcp4728.MCP4728(i2c)

Amplitude = 65535

steps_up = 50
steps_down = 20

tab1 = range(0,Amplitude,int(Amplitude/steps_up))
tab2 = range(Amplitude,0,-int(Amplitude/steps_down))

while True:          
    for i in tab1:
        dac.channel_a.value = i
        #sleep(0.001)
    for i in tab2:
        dac.channel_a.value = i
        #sleep(0.001)

