#ifndef ADAPTIVE_FILTER_PERIOD_H
#define ADAPTIVE_FILTER_PERIOD_H

#include <cmath>
#include <limits>

class AdaptiveFilterPeriod {

    private:

	    double period_mean;                   // current period mean
        double period_M2;                     // sum of squares of differences from period mean
        uint32_t sample_count = 0;            // number of samples measured for a period of statistics

public:

    AdaptiveFilterPeriod() : period_mean(0.0), period_M2(0.0) { }

    void update(int32_t value) {
        sample_count++; // TODO: use uint64
        double delta = value - period_mean;        // update Welford's algorithm variables for running variance
        period_mean += delta / sample_count;
        double delta2 = value - period_mean;
        period_M2 += delta * delta2;
    }

    double getStdDev() {
        double std_dev = 0.0;
        if (sample_count > 1) { std_dev = std::sqrt(period_M2 / sample_count); }
        period_mean = 0.0;
        period_M2 = 0.0;
        sample_count = 0;
        return std_dev;
    }

    double getAverage() const { return period_mean; }
    uint32_t getSampleCount() const { return sample_count; }

};

#endif