import sys
import pandas as pd
import numpy as np
from scipy.optimize import curve_fit
from scipy.signal import argrelextrema
import matplotlib.pyplot as plt
import time

if len(sys.argv) < 2:
    print("Usage: python script_name.py <csv_file>")
    sys.exit(1)

filename = sys.argv[1]
df = pd.read_csv(filename)

x = np.array(df['X'].values[1:], dtype = int)
dt = df['Increment'].values[0]
t = dt*x*1000 # [ms]

V1 = np.array(df['CH1'].values[1:], dtype = float) # [V]
V2 = np.array(df['CH2'].values[1:], dtype = float)
#V1_err = V1*0.03 + 1 # [mV]

plt.scatter(t, V1, s = 10, color = 'b')

Visibility = (max(V1) - min(V1))/(max(V1) + min(V1))

'''
offset = 0
for elem in V1:
    offset += elem
offset /= len(V1)
print(offset)

V1 -= offset
'''

def sine_function(t, A, B, C, w, phase):
    return A * np.sin(w * np.exp(C*t) + phase) + B 

#for full range and f(x) = A * np.sin(w * t **C + phase) + B:
#                  A       B      C      w          phase
#for sin^2: p0 = [9e-02, 9e-03, 1.16, 2*np.pi*9923, 1.95] in V and s
#for sin: p0 = [4.8e-02, 5.7e-02, 1.16, 124701., 8.62] in V and s
#in ms: w/1000

#fitting range:
a = 400
b = -400

t = t[a:b]
V1 = V1[a:b]
x = np.arange(0, len(t))
# V1_err = V1_err[a:b]

# a = 170, b = -85, p0 = [48.02, 57.87, 5.43e-4, -62.34, 63.8] for 800 hz 26.04 3 and 4
# a = 120, b = -130 for 5
# a = 60, b = -180 for 6
start = time.perf_counter()
popt, pcov = curve_fit(sine_function, t, V1, p0 = [48.02, 57.87, 5.43e-4, -62.34, 63.8])
end = time.perf_counter()
#popt, pcov = curve_fit(sine_function, t, V1, p0 = [1.5, 1.5, 1e-3, -500, 500])
print('computation time: ', (end - start)*1000000, ' us')
print('Number of points: ', len(x))

params = {
    'Visibility': Visibility,
    'A': popt[0],
    'A_err': pcov[0, 0]**0.5,
    'B': popt[1],
    'B_err': pcov[1,1]**0.5,
    'C': popt[2],
    'C_err': pcov[2, 2]**0.5,
    'w': popt[3],
    'w_err': pcov[3, 3]**0.5,
    'phase': popt[4],
    'phase_err': pcov[4,4]**0.5,
}

#min = argrelextrema(V1, np.less)
#max = argrelextrema(V1, np.greater)

y_fit = sine_function(t, params['A'], params['B'], params['C'], params['w'], params['phase'])

n = len(t) #number of data points
p = 5       #number of parameters

chi_squared = np.sum((V1 - y_fit[x])**2)
ndf = n-p
chi_sq_ndf = chi_squared/ndf
params['chi_sq_ndf'] = chi_sq_ndf

#R-squared (COD - Coefficient of Determination)

SSR = np.sum((V1 - y_fit[x])**2)
mean_y = np.mean(V1)
SST = np.sum((V1-mean_y)**2)
R_square = 1 - SSR/SST
params['R-square (COD)'] = R_square

adj_R = 1 - (1-R_square)*(n-1)/(n-p-1)
params['Adjusted R-square'] = adj_R

with open('fits\{}_params.txt'.format(filename[:-4]), 'w') as file:
    for key, value in params.items():
        print(f'{key} = {value}\n')
        file.write(f'{key} = {value}\n')

# plt.scatter(t, V1, s = 10, color = 'b')
# plt.errorbar(t, V1, V1_err, fmt = 'b.', ecolor='y')
plt.plot(t, y_fit, color = 'r', label='fit')
#plt.plot(min*dt, V1[min], 'x', c = 'orange')
plt.xlabel(r'$t \ [ms]$')
plt.ylabel(r'$Photodiode \ signal \ $[arbitrary units]')
plt.savefig('fits\{}.pdf'.format(filename[:-4]))
plt.show()
