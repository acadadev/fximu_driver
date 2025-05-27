#ifndef ADAPTIVE_FILTER_STDDEV_H
#define ADAPTIVE_FILTER_STDDEV_H

#include <cmath>
#include <limits>

class AdaptiveFilter {

    private:

        const double initial_alpha = 0.3;     // starting learning rate (more aggressive)
        const double final_alpha = 0.001;     // final learning rate
        const uint32_t warmup_samples = 128;  // number of samples for initial adaptation

        uint64_t sample_count;         		  // number of samples measured

        double mean_nanos_diff;               // nano second delta
		double sum_deviations;                // sum of deviations from mean
        bool warmed_up = false;               // if filter has converged

public:

    AdaptiveFilter(double initial_value = 0.0) :
		sample_count(0),
        mean_nanos_diff(initial_value),
	    sum_deviations(0.0),
        warmed_up(false)
         {}

    void update(int32_t nanos_diff) {

        sample_count++;           			  // increment sample count

		double delta = nanos_diff - mean_nanos_diff;
		sum_deviations += delta * delta;

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
        mean_nanos_diff = (mean_nanos_diff * (1.0 - alpha)) + (nanos_diff * alpha);
    }

    double getStdDev() {
        double std_dev = 0.0;
        if (sample_count > 1) {
            std_dev = std::sqrt(sum_deviations / sample_count);
        }
        return std_dev;
    }

    double getAverage() const { return mean_nanos_diff; }
    uint32_t getSampleCount() const { return sample_count; }
    bool isWarmedUp() const { return warmed_up; }
};

#endif