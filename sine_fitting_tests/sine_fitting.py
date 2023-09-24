import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import time

data = pd.read_csv(r'C:\Users\Adam\Desktop\fizyka indywidualna\praca licencjacka\Laboratorium Ultrazimnych Atomów\Laser frequency stabilization project\algorithms\sine_fitting_tests\26_04_800hz_ramp_4.csv')

x = np.array(data['X'].values[1:], dtype = int)
dt = data['Increment'].values[0]
dt = dt * 1000000 # [us]
t = dt*x # [us]

V1 = np.array(data['CH1'].values[1:], dtype = float)*1000 # [mV]
V1 = V1 - sum(V1)/len(V1)
V2 = np.array(data['CH2'].values[1:], dtype = float)
V1_err = V1*0.03 + 1 # [mV]

def get_fitting_matrix(steps, f, skip_left, skip_right):
    n = steps - skip_left - skip_right
    t = skip_left + np.arange(n, dtype = float)
    sin_t = np.sin(2*np.pi*f*t/steps)
    cos_t = np.cos(2*np.pi*f*t/steps)
    offset = np.ones(len(t))
    
    D = np.vstack([sin_t, cos_t, offset]).T
    
    DTD = np.dot(D.T, D)
    compute_matrix = np.dot(np.linalg.inv(DTD), D.T)[:2, :]
    compute_matrix_padded = np.zeros((2, steps))
    compute_matrix_padded[:, skip_left:-skip_right] = compute_matrix
    flattened_array = np.array(compute_matrix_padded.flatten(), dtype = 'float32')
    return flattened_array

def generate_signal(amplitude, frequency, duration, sampling_rate, noise_amplitude):
    t = np.arange(0, duration, 1/sampling_rate)
    sine_wave = amplitude * np.sin(2 * np.pi * frequency * t)
    noise = np.random.normal(scale=noise_amplitude, size=len(t))
    sine_with_noise = sine_wave + noise
    return t, sine_with_noise
'''
amplitude = 5
freq = 4
duration = 1
sampling_rate = 30000
noise_amplitude = 0.2

time_, signal = generate_signal(amplitude, freq, duration, sampling_rate, noise_amplitude)
'''
time_ = x
signal = V1

skip_left = 500
skip_right = 100
steps = len(signal)

print('Number of points: ', steps - skip_left - skip_right)

start = time.perf_counter()
# Perform FFT
fft_result = np.fft.fft(signal)
fft_freqs = np.fft.fftfreq(len(signal), d=1/len(time_))
magnitude_spectrum = np.abs(fft_result)

# Find the index of the maximum magnitude in the spectrum
max_magnitude_index = np.argmax(magnitude_spectrum)

# Find the corresponding frequency
peak_frequency = fft_freqs[max_magnitude_index]

fit_freq = peak_frequency

sin_t = np.sin(2*np.pi*time_/steps*fit_freq)
cos_t = np.cos(2*np.pi*time_/steps*fit_freq)

a = get_fitting_matrix(steps, fit_freq, skip_left, skip_right)
amp_sin = np.sum(a[:steps]*signal)
amp_cos = np.sum(a[steps:]*signal)
end = time.perf_counter()

print("Peak Frequency from FFT:", peak_frequency)
print('computation time: ', (end - start)*1000000, ' us')

# Plot the signal and the FFT magnitude spectrum
plt.subplot(2, 1, 1)
plt.plot(time_, signal)
plt.plot(time_[skip_left:-skip_right], amp_sin*sin_t[skip_left:-skip_right] + amp_cos*cos_t[skip_left:-skip_right])
plt.title("Signal")

plt.subplot(2, 1, 2)
plt.plot(fft_freqs[0:10], magnitude_spectrum[0:10])
plt.title("FFT Magnitude Spectrum")
plt.xlabel("Frequency (Hz)")

plt.tight_layout()
plt.show()
