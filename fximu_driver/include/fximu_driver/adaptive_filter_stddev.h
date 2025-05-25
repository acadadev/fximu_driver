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
        uint32_t sample_count;                // number of samples measured
        bool warmed_up;                       // if filter has converged

        double period_M2;                     // sum of squares of differences from current mean
        uint32_t period_sample_count;

public:

    AdaptiveFilter(double initial_value = 0.0) :
        avg_nanos_diff(initial_value),
        sample_count(0),
        warmed_up(false),
        period_M2(0.0),
        period_sample_count(0) { }

    void update(int32_t nanos_diff) {

        sample_count++;

        // update Welford's algorithm variables for running variance
        period_sample_count++;
        double delta = nanos_diff - getAverage();
        period_M2 += delta * delta;

        // calculate dynamic alpha
        double alpha;
        if(warmed_up) {
            alpha = final_alpha;
        } else if (sample_count < warmup_samples) {
            alpha = initial_alpha - (initial_alpha - final_alpha) * (static_cast<double>(sample_count) / warmup_samples);
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
        period_M2 = 0.0;
        period_sample_count = 0;
        return std_dev;
    }

    double getAverage() const { return avg_nanos_diff; }
    uint32_t getSampleCount() const { return sample_count; }
    bool isWarmedUp() const { return warmed_up; }
};

#endif