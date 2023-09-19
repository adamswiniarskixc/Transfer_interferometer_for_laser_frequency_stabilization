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

#include <fftw3.h>

#include "Eigen/Dense" // for matrix operations
#include <vector>

using namespace std;
using namespace Eigen;

// MCP4728 DAC address
#define MCP4728_ADDRESS 0x60

// DAC channels
#define DAC_CHANNEL_A 0x40

#define ZEROV 0;
#define V2P5 2047;

// Amplitude of the sawtooth waveform
#define Amplitude 4095 // 12-bit resolution

// Number of steps for the sawtooth waveform
#define STEPS_UP 50
#define STEPS_DOWN 20

#define PI M_PI
#define PI2 2*M_PI

// Monitor reference laser
#define MONITOR_ERR_REF 1
#define MONITOR_FIT_REF 2
#define MONITOR_SET_PHASE_REF 3
#define MONITOR_RESIDUE_REF 4

// Monitor slave laser
#define MONITOR_ERR_SLAVE 11
#define MONITOR_FIT_SLAVE 12
#define MONITOR_SET_PHASE_SLAVE 13
#define MONITOR_RESIDUE_SLAVE 14

struct Params {
    int ramp_amplitude;
    
    // laser frequencies
    float freq_ref, freq_slave;
    
    // PI gains
    float gain_p_ref, gain_i_ref;
    float gain_p_slave, gain_i_slave;
    
    float set_phase_ref, set_phase_slave;
    int lock_state_ref, lock_state_slave;
    
    int output_offset_slave;
    
    int monitor_channel;
    
    int ramp_amplitude_slave;
    int ramp_n_steps_slave;
    int ramp_state_slave;
};

Params params;

int in0, in1; // ref, slave interference
int out0, out1, out2; // ramp, error signal for slave, monitor channel

int out1_pid;

int ramp_mean;
int ramp_offset;

int ramp_offset_slave;

int cycle_up = 0;
int cycle_up_slave = 0;

int cycle_down = 0;
int cycle_down_slave = 0;

int ramp_step_up;
int ramp_step_down;

int ramp_step_slave;

bool ramp_direction;
bool ramp_direction_slave;

int in0_array[STEPS_UP];
int in0_array2[STEPS_UP];

int in0_buffer;

int in1_array[STEPS_UP];
int in1_array2[STEPS_UP];

// Number of steps to skip
const int SKIP_LEFT = 10;
const int SKIP_RIGHT = 10;
const int STEPS_USE = STEPS_UP - SKIP_LEFT - SKIP_RIGHT;

// 2 matrices, one for each wavelength, each of size 2xSTEPS_UP
float estimation_matrix[2*2*STEPS_UP];

//float estimation_matrix[1*2*STEPS_UP];

float p_ref[2];
float p_ref_sum[2];
float p_ref_set[2];
float phase_ref;
float accumulator_ref;

float freq_ratio;

float p_slave[2];
float p_slave_sum[2];
float p_slave_set[2];
float phase_slave;
float accumulator_slave;

float sin_array_ref[STEPS_UP];
float cos_array_ref[STEPS_UP];

float sin_array_slave[STEPS_UP];
float cos_array_slave[STEPS_UP];

float phase_damp_slave;

// Pre-compute sin and cos required for monitoring the fit
void computeData() {
    for (int i=0; i < STEPS_UP; i++) {
	sin_array_ref[i] = sin(PI2*((float)i)*params.freq_ref/STEPS_UP);
	cos_array_ref[i] = cos(PI2*((float)i)*params.freq_ref/STEPS_UP);
	
	sin_array_slave[i] = sin(PI2*((float)i)*params.freq_slave/STEPS_UP);
	cos_array_slave[i] = cos(PI2*((float)i)*params.freq_slave/STEPS_UP);
    }
}

void evaluate_sin_cos() {
    p_ref_set[0] = sin(params.set_phase_ref);
    p_ref_set[1] = cos(params.set_phase_ref);

    p_slave_set[0] = sin(params.set_phase_slave);
    p_slave_set[1] = cos(params.set_phase_slave);
}


double get_fit_freq(int array[], int arraySize, int SKIP_LEFT, int STEPS_USE) {
    vector<double> in_array(array, array + arraySize);
    
    int dataSize = STEPS_USE;

    // Perform FFTW setup
    fftw_complex* in = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * dataSize);
    fftw_complex* out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * dataSize);
    fftw_plan plan = fftw_plan_dft_1d(dataSize, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

    for (int i = 0; i < dataSize; ++i) {
        in[i][0] = in_array[SKIP_LEFT + i];
        in[i][1] = 0.0;
    }

    // Execute FFT
    fftw_execute(plan);

    double SampleRate = 1.0;  // You can modify this if your actual sampling rate is different

    // Calculate frequency bins
    double* fft_freqs = new double[dataSize];
    for (int i = 0; i < dataSize; ++i) {
        fft_freqs[i] = i * SampleRate / dataSize;
    }

    // Find the index of the maximum magnitude in the spectrum
    int max_magnitude_index = 0;
    for (int i = 1; i < dataSize/2; ++i) {
        if (abs(out[i][0]) > abs(out[max_magnitude_index][0])) {
            max_magnitude_index = i;
        }
    }

    double peak_frequency = fft_freqs[max_magnitude_index];

    // Clean up FFTW resources
    fftw_destroy_plan(plan);
    fftw_free(in);
    fftw_free(out);
    delete[] fft_freqs;

    return peak_frequency;
}

vector<float> get_fitting_matrix(int n_steps, float fit_freq, int skip_left = 10, int skip_right = 10) {
    // steps to use
    int n_use = n_steps - skip_left - skip_right;

    MatrixXf D(n_use, 3);
    for (int i = skip_left; i < n_steps-skip_right; ++i) {
        float time = static_cast<float>(i);
        float theta = PI2 * time / n_steps * fit_freq;
        D(i - skip_left, 0) = sin(theta);
        D(i - skip_left, 1) = cos(theta);
        D(i - skip_left, 2) = 1.0f;
    }

    MatrixXf DTD = D.transpose() * D;
    MatrixXf compute_matrix = (DTD.inverse() * D.transpose()).block(0, 0, 2, n_use);

    vector<float> flattened_array(2 * n_steps, 0.0f);

    for (int i = skip_left; i < n_steps - skip_right; ++i) {
        flattened_array[i]  = compute_matrix(0, i-skip_left);
        flattened_array[n_steps + i] = compute_matrix(1, i-skip_left);
    }

    return flattened_array;
}

void processParams() {
    ramp_direction = true;
    ramp_offset = 0;
    ramp_step_up = static_cast<int>(static_cast<float>(params.ramp_amplitude) / static_cast<float>(STEPS_UP));
    ramp_step_down = static_cast<int>(static_cast<float>(params.ramp_amplitude) / static_cast<float>(STEPS_DOWN));
    cycle_up = 0;
    cycle_down = 0;
    
    ramp_direction_slave = true;
    ramp_offset_slave = 0;
    ramp_step_slave = static_cast<int>(static_cast<float>(params.ramp_amplitude_slave) / static_cast<float>(params.ramp_n_steps_slave));
    cycle_up_slave = 0;
    cycle_down_slave = 0;

    evaluate_sin_cos();
    computeData();
}

double get_phase_difference(float *p_act, float *p_set) {
    float sinp, cosp;
    float l2 = p_act[0]*p_act[0] + p_act[1]*p_act[1];
    
    float invhypot = 1./sqrt(l2);
    
    sinp = invhypot*(p_act[0]*p_set[0] - p_act[1]*p_set[1]);
    cosp = invhypot*(p_act[0]*p_set[1] + p_act[1]*p_set[0]);
    
    return atan2(sinp, cosp);
}

// Analog write function for MCP4728
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

bool Exit = false; // Global flag variable to exit the program

void exit_handler(int signum) {
    (void)signum; // To suppress the unused parameter warning
    bcm2835_i2c_end();
    BCM2835_GPIO_Exit();
    bcm2835_close();
    Exit = true;
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
	
    signal(SIGINT, exit_handler);
	
    if (!BCM2835_SPI_Initialize()) {
	    return EXIT_FAILURE;
    }
	
    ADS1256_Reset();
	
    ADS1256_Configure(ADS1256_GAIN_1, ADS1256_30000_SPS);
	
    //ADS1256_Set_Channel_Differential(ADS1256_AIN1, ADS1256_AIN0);

    //uint32_t rword;
    //double voltage;
    
    in0_buffer = 0;
    
    in0 = ZEROV;
    in1 = ZEROV;
    
    out0 = ZEROV;
    out1 = ZEROV;
    out2 = ZEROV;
    
    ramp_mean = V2P5;  //???
    
    params.ramp_amplitude = Amplitude;
    
    /*
    
    ramp_step_up = static_cast<int>(static_cast<float>(params.ramp_amplitude) / static_cast<float>(STEPS_UP));
    ramp_step_down = static_cast<int>(static_cast<float>(params.ramp_amplitude) / static_cast<float>(STEPS_DOWN));

    // Create the sawtooth wave table
    int tab[STEPS_UP + STEPS_DOWN];
    
    for (int i = 0; i < STEPS_UP; ++i) {
        tab[i] = i * ramp_step_up;
    }
    for (int i = 0; i < STEPS_DOWN; ++i) {
        tab[i + STEPS_UP] = Amplitude - i * ramp_step_down;
    }
    
    */
    
    params.gain_p_ref = 100.;
    params.gain_i_ref = 200.;
    
    params.gain_p_slave = 100.;
    params.gain_i_slave = 200.;
    
    phase_ref = 0.0;
    accumulator_ref = 0.0;
    
    params.monitor_channel = MONITOR_ERR_REF;
    
    params.freq_ref = 2.2; // to be calculated using fft
    params.freq_slave = 2.2/(852/767);

    params.set_phase_ref = 0.0;
    params.set_phase_slave = 0.0;
    
    params.lock_state_ref = 0;
    params.lock_state_slave = 0;
    
    params.output_offset_slave = ZEROV;
    
    phase_damp_slave = 0.0;
    
    for (int i=0; i < STEPS_UP; i++)
    {
	in0_array[i] = ZEROV;
	in1_array[i] = ZEROV;
    }
    
    ramp_direction = true;
    params.lock_state_ref = false;
    params.lock_state_slave = false;
    
    params.ramp_amplitude_slave = Amplitude;
    params.ramp_n_steps_slave = 100;
    params.ramp_state_slave = 0;
    
    processParams();
    
    out1_pid = 0;
    
    //ofstream outFile("test_data.txt");
    
    while (!Exit) {
	ADS1256_Set_Channel_Differential(ADS1256_AIN1, ADS1256_AIN0);
	in0 = ADS1256_Read_Word(false);
	//outFile << in0 << endl;
	
	ADS1256_Set_Channel_Differential(ADS1256_AIN3, ADS1256_AIN2);
	in1 = ADS1256_Read_Word(false);
	
	if (ramp_direction) {
	    if (in0_buffer) {
		in0_array2[cycle_up] = in0;
		in1_array2[cycle_up] = in1;
	    }
	    else {
		in0_array[cycle_up] = in0;
		in1_array[cycle_up] = in1;
	    }
		
	    p_ref[0] += estimation_matrix[cycle_up]*((float)in0);
	    p_ref[1] += estimation_matrix[cycle_up+STEPS_UP]*((float)in0);
	    
	    p_slave[0] += estimation_matrix[cycle_up+2*STEPS_UP]*((float)in1);
	    p_slave[1] += estimation_matrix[cycle_up+3*STEPS_UP]*((float)in1);
	    
	    cycle_up += 1;
	    ramp_offset += ramp_step_up;
	    //ramp_offset = tab[cycle_up-1];
	    
	    if (cycle_up == STEPS_UP)
	    {
		// ********************************************************
		// Find reference and slave fit frequencies
		params.freq_ref = get_fit_freq(in0_array, sizeof(in0_array) / sizeof(in0_array[0]), SKIP_LEFT, STEPS_USE);
		params.freq_slave = get_fit_freq(in1_array, sizeof(in1_array) / sizeof(in1_array[0]), SKIP_LEFT, STEPS_USE);
		
		//outFile << params.freq_ref << endl;
		
		freq_ratio = params.freq_slave / params.freq_ref;
		
		vector<float> result_ref = get_fitting_matrix(STEPS_UP, params.freq_ref);
		vector<float> result_slave = get_fitting_matrix(STEPS_UP, params.freq_slave);
		
		for (int i = 0; i < STEPS_UP; ++i) {
		    estimation_matrix[i] = result_ref[i];
		    estimation_matrix[STEPS_UP + i] = result_slave[i];
		}
		
		//*********************************************************
		
		cycle_up = 0;
		ramp_direction = false; // switch ramp direction
		// ramp done, reset fit parameters
		
		p_ref_sum[0] = p_ref[0];
		p_ref_sum[1] = p_ref[1];
		
		p_slave_sum[0] = p_slave[0];
		p_slave_sum[1] = p_slave[1];
		
		p_ref[0] = 0.0;
		p_ref[1] = 0.0;
		
		p_slave[0] = 0.0;
		p_slave[1] =0.0;
		
		phase_ref = get_phase_difference(p_ref_sum, p_ref_set);
		//outFile << phase_ref << endl;
		phase_slave = get_phase_difference(p_slave_sum, p_slave_set);
		
		if (params.lock_state_ref) {
		    accumulator_ref += params.gain_i_ref * phase_ref;
		    ramp_mean = V2P5 + (int)(accumulator_ref + params.gain_p_ref * phase_ref);
		    
		    // if interferometer is locked, then subtract residual error in phase
		    phase_slave -= freq_ratio*phase_ref;
		}
		else {
		    accumulator_ref = 0.0;
		    ramp_mean = ZEROV;
		}
		
		if (params.lock_state_slave) {
		    float pd = phase_slave - phase_damp_slave; // ???
		    phase_damp_slave *= 0.99; // ???
		    accumulator_slave += params.gain_i_slave * pd;
		    out1_pid = (int)(accumulator_slave + params.gain_p_slave*pd);
		}
		else {
		    accumulator_slave = 0.0;
		    out1_pid = 0;
		    phase_damp_slave = phase_slave;
		}
		
		// Write to monitor channel
		switch(params.monitor_channel) {
		    case MONITOR_ERR_REF:
			out2 = (int)(phase_ref/PI2*Amplitude); // divide by 2pi and multiplied by Amplitude such that 0 <= out3 <= Amplitude
			break;
		    case MONITOR_ERR_SLAVE:
			out2 = (int)(phase_slave/PI2*Amplitude);
			break;
		    case MONITOR_SET_PHASE_REF:
			out2 = (int)(params.set_phase_ref/PI2*Amplitude);
			break;
		    case MONITOR_SET_PHASE_SLAVE:
			out2 = (int)(params.set_phase_slave/PI2*Amplitude);
			break;
		}
	    }
	    
	    // Write fitted points to monitor channel
	    switch(params.monitor_channel) {
	      case MONITOR_FIT_REF:
		out2 = ZEROV + p_ref_sum[0]*sin_array_ref[cycle_up] + p_ref_sum[1]*cos_array_ref[cycle_up];
		break;
	       case MONITOR_RESIDUE_REF:
		 out2 = in0 - (p_ref_sum[0]*sin_array_ref[cycle_up] + p_ref_sum[1]*cos_array_ref[cycle_up]);
		 break;
	      case MONITOR_FIT_SLAVE:
		out2 = ZEROV + p_slave_sum[0]*sin_array_slave[cycle_up] + p_slave_sum[1]*cos_array_slave[cycle_up];
		break;
	      case MONITOR_RESIDUE_SLAVE:
		out2 = in1 - (p_slave_sum[0]*sin_array_slave[cycle_up] + p_slave_sum[1]*cos_array_slave[cycle_up]);
		break;
	    }
	}
	else {
	    // we are cycling down
	    cycle_down += 1;
	    //ramp_offset = tab[STEPS_UP + cycle_down-1];
	    ramp_offset -=ramp_step_down;
	    
	    if (cycle_down == STEPS_DOWN) {
		cycle_down = 0;
		ramp_direction = true;
		if (in0_buffer)
		    in0_buffer = 0;
		else
		    in0_buffer = 1;
	    }
	}
	
	out1 = params.output_offset_slave - out1_pid;
	
	// ramp the slave channel
	if(params.ramp_state_slave) {
	    if(ramp_direction_slave) {
		cycle_up_slave += 1;
		ramp_offset_slave += ramp_step_slave;
	    
		if(cycle_up_slave == params.ramp_n_steps_slave) {
		    cycle_up_slave = 0;
		    ramp_direction_slave = false;
		}
	    }
	    else {
		cycle_down_slave += 1;
		ramp_offset_slave -= ramp_step_slave;
		if (cycle_down_slave == params.ramp_n_steps_slave) {
		    cycle_down_slave = 0;
		    ramp_direction_slave = true;
		}
	    }
	    out1 += ramp_offset_slave;
	}
	else {
	    cycle_up_slave = 0;
	    cycle_down_slave = 0;
	    ramp_direction_slave = true;
	    ramp_offset_slave = ZEROV;
	}
	
	out0 = ramp_mean + ramp_offset;
	//out0 = ramp_offset;
	analogWrite(0, out0); // channel 0
	analogWrite(2, out1); // channel 1
	analogWrite(4, out2); // channel 2
    }
    
    //outFile.close();
    
    bcm2835_i2c_end();
    bcm2835_close();
    
    return 0;
}
