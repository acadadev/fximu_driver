#ifndef ADAPTIVE_FILTER_STDDEV_H
#define ADAPTIVE_FILTER_STDDEV_H

#include <cmath>
#include <limits>

class AdaptiveFilter {

    private:

        const double initial_alpha = 0.3;     // starting learning rate (more aggressive)
        const double final_alpha = 0.001;     // final learning rate
        const uint32_t warmup_samples = 128;  // number of samples for initial adaptation

        double avg_nanos_diff;                // nano second delta
        bool warmed_up;                       // if filter has converged
        uint32_t filter_sample_count;         // number of samples measured
        uint32_t period_sample_count;         // number of samples measured for a period of statistics
	    double period_mean;                   // current period mean
        double period_M2;                     // sum of squares of differences from period mean


public:

    AdaptiveFilter(double initial_value = 0.0) :
        avg_nanos_diff(initial_value),
        warmed_up(false),
        filter_sample_count(0),
        period_sample_count(0),
        period_mean(0.0),
        period_M2(0.0) { }

    void update(int32_t nanos_diff) {

        filter_sample_count++;           // filter sample count
        period_sample_count++;           // update Welford's algorithm variables for running variance

        // update Welford's algorithm variables for running variance
        double delta = nanos_diff - period_mean;
        period_mean += delta / period_sample_count;

        double delta2 = nanos_diff - period_mean;
        period_M2 += delta * delta2;

        // calculate dynamic alpha
        double alpha;
        if(warmed_up) {
            alpha = final_alpha;
        } else if (filter_sample_count < warmup_samples) {
            alpha = initial_alpha - (initial_alpha - final_alpha) * (static_cast<double>(filter_sample_count) / warmup_samples);
        } else {
            alpha = final_alpha;
            warmed_up = true;
        }

        // apply the adaptive filter
        avg_nanos_diff = (avg_nanos_diff * (1.0 - alpha)) + (nanos_diff * alpha);
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

    double getAverage() const { return avg_nanos_diff; }
    uint32_t getSampleCount() const { return filter_sample_count; }
    bool isWarmedUp() const { return warmed_up; }
};

#endif