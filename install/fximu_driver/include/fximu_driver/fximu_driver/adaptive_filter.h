#ifndef ADAPTIVE_FILTER_H
#define ADAPTIVE_FILTER_H

#include <cmath>

// TODO: TEST: change input value to negative suddenly
// TODO: consider making the nanos diff cyclic. as in distance from a to b.
// TODO: REVISE: normal
class AdaptiveFilter {

    private:

        const double initial_alpha = 0.3;     // starting learning rate (more aggressive)
        const double final_alpha = 0.001;     // final learning rate
        const uint32_t warmup_samples = 128;  // number of samples for initial adaptation

        double avg_nanos_diff;
        uint32_t sample_count;
        bool warmed_up;

    public:

        AdaptiveFilter(double initial_value = 0.0) : avg_nanos_diff(initial_value), sample_count(0), warmed_up(false) { }

        void update(int32_t nanos_diff) {

            sample_count++;
            double alpha;                                    // calculate dynamic alpha based on sample count

            if(warmed_up) {
                alpha = final_alpha;
            } else if (sample_count < warmup_samples) {    // linear interpolation from initial_alpha to final_alpha
                alpha = initial_alpha - (initial_alpha - final_alpha) * (static_cast<double>(sample_count) / warmup_samples);
            } else {
                alpha = final_alpha;
                warmed_up = true;
            }

            // apply the adaptive filter
            avg_nanos_diff = (avg_nanos_diff * (1.0 - alpha)) + (nanos_diff * alpha);

        }
        /*
        bool normal(int32_t nanos_diff) {

            if (!warmed_up) { return true; }                  // return true if not warmed up
            if (nanos_diff == 0) { return true; }             // return true if nanos_diff is zero

            // check if nanos_diff is more than 128 times larger than avg_nanos_diff
            if (static_cast<int64_t>(abs(nanos_diff)) > static_cast<int64_t>(abs(avg_nanos_diff)) * 128) {
                return false;
            }

            // check if avg_nanos_diff is more than 128 times larger than nanos_diff
            if (static_cast<int64_t>(abs(avg_nanos_diff)) > static_cast<int64_t>(abs(nanos_diff)) * 128) {
                return false;
            }

            return true;
        }*/

        double getAverage() const { return avg_nanos_diff; }
        uint32_t getSampleCount() const { return sample_count; }

};

#endif //ADAPTIVE_FILTER_H
