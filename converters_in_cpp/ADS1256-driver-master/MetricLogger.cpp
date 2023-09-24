#include "MetricLogger.h"

/*
* Keeps the most recent amount LIVE_DATA_SAMPLES in the live log
*/
void MetricLogger::AppendToLiveData(double power) {
    time_t now;
    time(&now);

    Timestamp t;

    // ISO 8601 Timestamp
    strftime(t.ts, sizeof(t.ts), "%FT%TZ", gmtime(&now));

    std::stringstream path;

    // Construct path
    path << PATH_TO_DATA << LIVE_DATA_FILE;

    if  (live_data_pool.size() > MAX_LIVE_DATA_SAMPLES) {
        live_data_pool.pop_back();
    }

    live_data_pool.push_front({ t, power });

    std::ofstream live_data(path.str().c_str(), std::ios::out);

    std::stringstream data;

    for (auto measurement : live_data_pool) {
        data << std::fixed << std::setprecision(2) << measurement.first.ts << "," << measurement.second << "\n";
    }

    live_data << data.str();
}

/*
* Appends a power reading to the historic data log, this log is full scale
*/
void MetricLogger::AppendToHistoricData(double power) {
    time_t now;
    time(&now);

    char buffer[80];

    // ISO 8601 Timestamp
    strftime(buffer, sizeof(buffer), "%FT%TZ", gmtime(&now));

    std::stringstream path;

    // Construct path
    path << PATH_TO_DATA << HISTORIC_DATA_FILE;

    std::ofstream live_data(path.str().c_str(), std::ios::app | std::ios::out);

    std::stringstream data;

    data << buffer << "," << std::fixed << std::setprecision(2) << power << "\n";

    live_data << data.str();
}

/*
* Updates the metrics displayed
*/
void MetricLogger::SetMetrics(double cur_power, double acc_power) {
    std::stringstream path;

    // Construct path
    path << PATH_TO_DATA << METRICS_DATA_FILE;

    std::ofstream metrics_data(path.str().c_str(), std::ios::out);

    std::stringstream data;

    data << std::fixed << std::setprecision(2) << cur_power << "," << acc_power;

    metrics_data << data.str();
}
