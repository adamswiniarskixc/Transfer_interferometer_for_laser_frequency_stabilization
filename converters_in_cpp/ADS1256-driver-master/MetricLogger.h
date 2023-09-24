#pragma once
#include <ctime>
#include <fstream>
#include <string>
#include <sstream>
#include <cmath>
#include <iomanip> // SetPrecision
#include <deque>

#include <string.h>

#define PATH_TO_DATA "~/Desktop/interferometer/c/ADS1256-driver-master/"

#define LIVE_DATA_FILE "live_data.csv"
#define METRICS_DATA_FILE "metrics.csv"
#define HISTORIC_DATA_FILE "historical_data.csv"

#define MAX_LIVE_DATA_SAMPLES 60

// Wrapper for timestamps
struct Timestamp {
    char ts[80];
};

// The currently displayed live data measures
static std::deque <std::pair< Timestamp, double> > live_data_pool;

class MetricLogger {
public:
    static void AppendToLiveData(double power);
    static void AppendToHistoricData(double power);
    static void SetMetrics(double cur_power, double acc_power);

};

