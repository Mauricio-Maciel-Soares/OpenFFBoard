#pragma once

#include <cstdint>
#include <cmath>
#include <cstring>
#include "TimingStatistics.h"

// Caller prevents the HAL tick ISR from changing tick/UIF during this snapshot.
// Hardware keeps counting: retry if an overflow occurs between register reads.
template<class ReadTick, class ReadCounter, class ReadPending>
uint32_t readDiagnosticMicros(ReadTick tick, ReadCounter counter, ReadPending pending) {
    const uint32_t ms = tick();
    bool before, after;
    uint32_t us;
    do {
        before = pending();
        us = counter();
        after = pending();
    } while (before != after);
    return (ms + (after ? 1U : 0U)) * 1000U + us;
}

// Read-only diagnostics. Never used to scale, limit or time the torque output.
struct AxisDiagnostics {
    uint32_t sequence = 0;
    uint32_t timeUs = 0;
    uint32_t dtUs = 0;
    uint32_t minDtUs = UINT32_MAX;
    uint32_t maxDtUs = 0;
    uint32_t clipped = 0;
    uint32_t limited = 0;
    int32_t effect = 0;
    int32_t requested = 0;
    int32_t clippedTarget = 0;
    int32_t output = 0;
    float positionSpeed = 0;
    float encoderSpeed = 0;
    float speed = 0;
    float accel = 0;
    bool encoderValid = false;
    bool encoderUsed = false;
    bool driverReady = false;
    bool timeInitialized = false;
    TimingStatistics timing;
    uint32_t sourceSwitches = 0;
    bool velocityTransition = false;

    void recordTime(uint32_t now) {
        timing.record(now);
        if (timeInitialized) {
            dtUs = now - timeUs; // unsigned subtraction also handles timer rollover
            if (dtUs < minDtUs) minDtUs = dtUs;
            if (dtUs > maxDtUs) maxDtUs = dtUs;
        }
        timeUs = now;
        timeInitialized = true;
        ++sequence;
    }
};

// ODrive 0.5.6 CANSimple: two little-endian IEEE754 floats in an 8-byte frame.
// Used on the little-endian STM32. Reject incomplete/RTR/non-finite samples.
inline bool decodeODrivePair(const uint8_t* data, uint32_t length, bool rtr,
                             float& first, float& second) {
    if (rtr || length != 8) return false;
    float a, b;
    std::memcpy(&a, data, sizeof(a));
    std::memcpy(&b, data + 4, sizeof(b));
    if (!std::isfinite(a) || !std::isfinite(b)) return false;
    first = a;
    second = b;
    return true;
}

struct ODriveTelemetry {
    float iqSetpoint = 0;
    float iqMeasured = 0;
    float velocity = 0; // turns/s, motor coordinates
    float queuedTorque = 0; // Nm, accepted into CAN TX mailbox (not an ACK)
    uint32_t iqTimeMs = 0;
    uint32_t encoderTimeMs = 0;
    uint32_t torqueTimeMs = 0;
    uint32_t iqSequence = 0;
    uint32_t encoderSequence = 0;
    uint32_t txSequence = 0;
    uint32_t txErrors = 0;
    TimingStatistics iqTiming, encoderTiming, txTiming;
    uint32_t invalidEncoder = 0, invalidIq = 0, busErrors = 0;
    uint32_t voltageTimeUs = 0;
    bool voltageValid = false;
    uint8_t freshEncoderStreak = 0;

    void recordEncoder(uint32_t now) {
        const bool continuous = encoderTiming.initialized && uint32_t(now - encoderTiming.lastTimeUs) <= 5000;
        freshEncoderStreak = continuous ? std::min<unsigned>(2, freshEncoderStreak + 1) : 1;
        encoderTiming.record(now);
    }
    bool velocityFreshUs(uint32_t now) const {
        return freshEncoderStreak >= 2 && encoderTiming.initialized
            && uint32_t(now - encoderTiming.lastTimeUs) <= 5000;
    }

    bool velocityFresh(uint32_t now) const {
        return encoderSequence != 0 && uint32_t(now - encoderTimeMs) <= 5;
    }
};
