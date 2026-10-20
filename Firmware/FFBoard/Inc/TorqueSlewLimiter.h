#pragma once

#include <algorithm>
#include <cstdint>

// Counts/ms, timed in microseconds. The caller supplies the last command passed
// to the driver and clips both the target and the final output to the power limit.
class TorqueSlewLimiter {
public:
    void reset() {
        initialized = false;
        lastRate = 0;
        clearFraction();
    }

    void clearFraction() {
        remainder = 0;
        direction = 0;
    }

    int32_t apply(int32_t target, int32_t previous, uint32_t rate,
                  uint32_t nowUs, uint32_t nominalPeriodUs) {
        // Zero is a complete bypass, including timing and fractional history.
        if (rate == 0) {
            reset();
            return target;
        }
        uint32_t dtUs = initialized ? uint32_t(nowUs - lastTimeUs) : 0;
        lastTimeUs = nowUs;
        initialized = true;

        // No lower clamp: duplicate/early calls must not invent elapsed time.
        // A stall must not build up permission for an arbitrarily large step.
        const uint64_t nominal = std::max<uint32_t>(nominalPeriodUs, 1);
        const uint32_t maxDtUs = static_cast<uint32_t>(
            std::min<uint64_t>(nominal + nominal / 2, UINT32_MAX));
        if (dtUs > maxDtUs) {
            dtUs = maxDtUs;
            clearFraction();
        }
        if (rate != lastRate) {
            clearFraction();
            lastRate = rate;
        }

        const int64_t delta = int64_t(target) - previous;
        const int8_t nextDirection = delta > 0 ? 1 : delta < 0 ? -1 : 0;
        if (direction != nextDirection) clearFraction();
        direction = nextDirection;
        const uint64_t distance = delta < 0 ? -delta : delta;
        const uint64_t budget = uint64_t(rate) * dtUs + remainder;
        const uint64_t step = budget / 1000;
        if (step >= distance) {
            clearFraction(); // no stored credit once the target is reached
            return target;
        }
        // Retain only the sub-count part, with no credit across reversals.
        remainder = budget % 1000;
        return static_cast<int32_t>(int64_t(previous) + direction * int64_t(step));
    }

private:
    uint32_t lastTimeUs = 0;
    uint32_t lastRate = 0;
    uint16_t remainder = 0; // 0..999 thousandths of one count
    int8_t direction = 0;
    bool initialized = false;
};
