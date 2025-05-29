#ifndef ADAPTIVE_FILTER_RTT_H
#define ADAPTIVE_FILTER_RTT_H

#include <cmath>
#include <limits>

class AdaptiveFilterRTT {


    private:

        const double initial_alpha = 0.4;     // starting learning rate (more aggressive)
        const double final_alpha = 0.1;       // final learning rate
        const uint32_t warmup_samples = 8;    // number of samples for initial adaptation
        double rtt;                           // round trip time
        double offset;                        // time offset
        uint64_t sample_count;         		  // number of samples measured
        bool warmed_up = false;               // if filter has converged

public:

    AdaptiveFilterRTT() : rtt(0.0), offset(0.0), sample_count(0),warmed_up(false) {}

    void update(int32_t sigma, int32_t phi) {

        sample_count++;           			  // increment sample count

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
        rtt = (rtt * (1.0 - alpha)) + (sigma * alpha);
        offset = (offset * (1.0 - alpha)) + (phi * alpha);
    }

    double getRTT() const { return rtt; }
    double getOffset() const { return offset; }

};

#endif