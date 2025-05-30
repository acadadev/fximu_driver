#ifndef ADAPTIVE_FILTER_H
#define ADAPTIVE_FILTER_H

#include <cmath>

class AdaptiveFilter {

    private:

        double avg_value;                                                         // average value
        double initial_alpha;                                                     // starting learning rate (more aggressive)
        double final_alpha;                                                       // final learning rate
        uint32_t warmup_samples;                                                  // number of samples for initial adaptation
        bool warmed_up = false;                                                   // filter warmed_up
        uint32_t sample_count = 0;                                                // sample count

    public:

        AdaptiveFilter(double initial_avg = 0.0, double initial_alpha = 0.25, double final_alpha = 0.001, int warmup_samples = 128) :
            avg_value(initial_avg),
            initial_alpha(initial_alpha),
            final_alpha(final_alpha),
            warmup_samples(warmup_samples) { }

        void update(int32_t value) {
            sample_count++;
            double alpha;                                                          // calculate dynamic alpha based on sample count
            if(warmed_up) {
                alpha = final_alpha;
            } else if (sample_count < warmup_samples) {                            // linear interpolation from initial_alpha to final_alpha
                alpha = initial_alpha - (initial_alpha - final_alpha) * (static_cast<double>(sample_count) / warmup_samples);
            } else {
                alpha = final_alpha;
                warmed_up = true;
            }
            avg_value = (avg_value * (1.0 - alpha)) + (value * alpha);              // apply the adaptive filter
        }

        double getAverage() const { return avg_value; }
        uint32_t getSampleCount() const { return sample_count; }
		bool isWarmedUp() const { return warmed_up; }

};

#endif //ADAPTIVE_FILTER_H