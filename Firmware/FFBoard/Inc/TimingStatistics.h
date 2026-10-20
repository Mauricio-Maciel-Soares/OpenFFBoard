#pragma once
#include <cstdint>
#include <cmath>
#include <algorithm>

// Non-overlapping windows of at most 1024 actual intervals. No allocation,
// clock extrapolation or PC polling enters these statistics.
struct TimingStatistics {
    uint32_t lastTimeUs = 0, lastUs = 0, minUs = 0, maxUs = 0, count = 0;
    float meanUs = 0, m2Us = 0;
    bool initialized = false;
    void record(uint32_t now) {
        if (!initialized) { lastTimeUs = now; initialized = true; return; }
        const uint32_t dt = now - lastTimeUs;
        lastTimeUs = now;
        lastUs = dt;
        if (count == 1024) { count = 0; meanUs = m2Us = 0; }
        minUs = count ? std::min(minUs, dt) : dt;
        maxUs = count ? std::max(maxUs, dt) : dt;
        ++count;
        const float delta = float(dt) - meanUs;
        meanUs += delta / float(count);
        m2Us += delta * (float(dt) - meanUs);
    }
    float stddevUs() const { return count ? std::sqrt(std::max(0.0f, m2Us / count)) : 0; }
};
