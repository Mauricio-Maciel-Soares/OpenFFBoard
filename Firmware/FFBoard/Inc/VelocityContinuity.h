#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

// Change sources without a speed step. Only the source-offset is removed over
// 5 ms; changes in the new source pass through immediately. The synthetic
// offset decay is excluded from acceleration (no artificial inertia impulse).
class VelocityContinuity {
public:
    struct Result { float speed, acceleration; bool transitioning; };
    void reset() { initialized = false; offset = 0; remainingUs = 0; }
    Result update(float source, bool encoder, uint32_t nowUs, float nominalHz) {
        if (!initialized) {
            initialized = true; lastUs = nowUs; lastSource = lastOutput = source;
            wasEncoder = encoder;
            return {source, 0, false};
        }
        const uint32_t dt = nowUs - lastUs;
        lastUs = nowUs;
        const bool changed = encoder != wasEncoder;
        if (changed) { offset = lastOutput - source; remainingUs = transitionUs; ++switches; }
        else if (remainingUs) remainingUs -= std::min(remainingUs, dt);
        const float speed = source + offset * (float(remainingUs) / transitionUs);
        // A stalled loop does not invent a large acceleration on resumption.
        const float acceleration = !changed && dt && dt <= 100000
            ? (source - lastSource) * std::min(1000000.0f / dt, nominalHz * 2.0f) : 0;
        lastSource = source; lastOutput = speed; wasEncoder = encoder;
        return {speed, acceleration, remainingUs != 0};
    }
    uint32_t switches = 0;
private:
    static constexpr uint32_t transitionUs = 5000;
    bool initialized = false, wasEncoder = false;
    uint32_t lastUs = 0, remainingUs = 0;
    float lastSource = 0, lastOutput = 0, offset = 0;
};
