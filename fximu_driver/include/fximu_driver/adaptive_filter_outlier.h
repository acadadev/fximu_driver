#ifndef ADAPTIVE_FILTER_OUTLIER_H
#define ADAPTIVE_FILTER_OUTLIER_H

#include <cmath>

class AdaptiveFilterOutlier {
private:
    double avg_value;                // average value
    double initial_alpha;            // starting learning rate
    double final_alpha;              // final learning rate
    uint32_t warmup_samples;         // number of samples for initial adaptation
    bool warmed_up = false;          // filter warmed_up
    uint32_t sample_count = 0;       // sample count
    double outlier_threshold;        // threshold for outlier detection (in standard deviations)
    double m2 = 0.0;                 // for variance calculation (Welford's algorithm)
    double variance = 0.0;           // current variance estimate
    double last_raw_value = 0.0;     // last raw input value

public:
    AdaptiveFilter(double initial_avg = 0.0,
                  double initial_alpha = 0.25,
                  double final_alpha = 0.0625,
                  uint32_t warmup_samples = 16,
                  double outlier_threshold = 4.0) :
        avg_value(initial_avg),
        initial_alpha(initial_alpha),
        final_alpha(final_alpha),
        warmup_samples(warmup_samples),
        outlier_threshold(outlier_threshold) { }

    bool update(int32_t value) {
        last_raw_value = value;
        sample_count++;

        // Update variance estimate using Welford's algorithm
        double delta = value - avg_value;
        avg_value += delta / sample_count;
        double delta2 = value - avg_value;
        m2 += delta * delta2;
        variance = (sample_count > 1) ? m2 / (sample_count - 1) : 0.0;

        // Check for outlier (only after we have some variance estimate)
        if (sample_count > 10 && variance > 0) {
            double z_score = std::abs(value - avg_value) / std::sqrt(variance);
            if (z_score > outlier_threshold) {
                return false; // outlier detected
            }
        }

        double alpha;
        if(warmed_up) {
            alpha = final_alpha;
        } else if (sample_count < warmup_samples) {
            alpha = initial_alpha - (initial_alpha - final_alpha) * (static_cast<double>(sample_count) / warmup_samples);
        } else {
            alpha = final_alpha;
            warmed_up = true;
        }

        avg_value = (avg_value * (1.0 - alpha)) + (value * alpha);
        return true; // value accepted
    }

    double getAverage() const { return avg_value; }
    double getVariance() const { return variance; }
    double getStdDev() const { return std::sqrt(variance); }
    uint32_t getSampleCount() const { return sample_count; }
    bool isWarmedUp() const { return warmed_up; }
    double getLastRawValue() const { return last_raw_value; }
};

#endif //ADAPTIVE_FILTER_H

#endif
