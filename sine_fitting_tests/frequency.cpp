#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <fftw3>

int main() {
    // Load data from CSV file
    std::ifstream file("path_to_your_file.csv");
    std::vector<double> x, V1;
    double dt;

    if (!file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }

    std::string line;
    getline(file, line); // Skip header line

    // Read data from the file
    while (getline(file, line)) {
        double valueX, valueV1;
        if (sscanf(line.c_str(), "%lf,%lf", &valueX, &valueV1) == 2) {
            x.push_back(valueX);
            V1.push_back(valueV1);
        }
    }

    file.close();

    // Print the read data
    for (size_t i = 0; i < x.size(); ++i) {
        std::cout << "x[" << i << "] = " << x[i] << ", V1[" << i << "] = " << V1[i] << std::endl;
    }

    // Perform FFTW setup
    int dataSize = V1.size();
    fftw_complex* in = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * dataSize);
    fftw_complex* out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * dataSize);
    fftw_plan plan = fftw_plan_dft_1d(dataSize, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

    // Populate the input array for FFT
    for (int i = 0; i < dataSize; ++i) {
        in[i][0] = V1[i];
        in[i][1] = 0.0;
    }

    // Execute FFT
    fftw_execute(plan);

    // Calculate frequency bins
    double* fft_freqs = new double[dataSize];
    for (int i = 0; i < dataSize; ++i) {
        fft_freqs[i] = i / (dt * dataSize);
    }

    // Find the index of the maximum magnitude in the spectrum
    int max_magnitude_index = 0;
    for (int i = 1; i < dataSize; ++i) {
        if (std::abs(out[i][0]) > std::abs(out[max_magnitude_index][0])) {
            max_magnitude_index = i;
        }
    }

    // Find the corresponding frequency
    double peak_frequency = fft_freqs[max_magnitude_index];

    std::cout << "Peak Frequency from FFT: " << peak_frequency << std::endl;

    // Clean up FFTW resources
    fftw_destroy_plan(plan);
    fftw_free(in);
    fftw_free(out);
    delete[] fft_freqs;

    return 0;
}
