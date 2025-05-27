#ifndef ADAPTIVE_FILTER_STDDEV_H
#define ADAPTIVE_FILTER_STDDEV_H

#include <cmath>
#include <limits>

class AdaptiveFilter {

    private:

        uint32_t period_sample_count;         // number of samples measured for a period of statistics
	    double period_mean;                   // current period mean
        double period_M2;                     // sum of squares of differences from period mean

public:

    AdaptiveFilter() :
        period_sample_count(0),
        period_mean(0.0),
        period_M2(0.0) { }

    void update(int32_t nanos_diff) {

        period_sample_count++;           // update Welford's algorithm variables for running variance

        // update Welford's algorithm variables for running variance
        double delta = nanos_diff - period_mean;
        period_mean += delta / period_sample_count;

        double delta2 = nanos_diff - period_mean;
        period_M2 += delta * delta2;

    }

    double getStdDev() {
        double std_dev = 0.0;
        if (period_sample_count > 1) {
            std_dev = std::sqrt(period_M2 / period_sample_count);
        }
        period_mean = 0.0;
        period_M2 = 0.0;
        period_sample_count = 0;
        return std_dev;
    }

    double getAverage() const { return period_mean; }

};

#endif