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
        lastRiseRate = 0;
        lastFallRate = 0;
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

    // Magnitude-based asymmetric path. Reversals release to zero with fallRate;
    // the opposite polarity can build only on the following update.
    int32_t applyAsymmetric(int32_t target, int32_t previous,
                            uint32_t riseRate, uint32_t fallRate,
                            uint32_t nowUs, uint32_t nominalPeriodUs) {
        if (riseRate == 0 && fallRate == 0) {
            reset();
            return target;
        }
        uint32_t dtUs = initialized ? uint32_t(nowUs - lastTimeUs) : 0;
        lastTimeUs = nowUs;
        initialized = true;
        const uint64_t nominal = std::max<uint32_t>(nominalPeriodUs, 1);
        const uint32_t maxDtUs = static_cast<uint32_t>(
            std::min<uint64_t>(nominal + nominal / 2, UINT32_MAX));
        if (dtUs > maxDtUs) {
            dtUs = maxDtUs;
            clearFraction();
        }
        if (riseRate != lastRiseRate || fallRate != lastFallRate) {
            clearFraction();
            lastRiseRate = riseRate;
            lastFallRate = fallRate;
        }

        const bool crossingZero = (previous > 0 && target < 0)
                               || (previous < 0 && target > 0);
        uint32_t activeRate = 0;
        int8_t nextDirection = 0;
        uint64_t distance = 0;
        if (crossingZero) {
            activeRate = fallRate;
            nextDirection = previous > 0 ? -1 : 1;
            distance = previous > 0 ? uint64_t(previous)
                                    : uint64_t(-int64_t(previous));
        } else {
            const int64_t previousMagnitude = previous >= 0 ? int64_t(previous) : -int64_t(previous);
            const int64_t targetMagnitude = target >= 0 ? int64_t(target) : -int64_t(target);
            const int64_t delta = int64_t(target) - previous;
            nextDirection = delta > 0 ? 1 : delta < 0 ? -1 : 0;
            distance = delta < 0 ? uint64_t(-delta) : uint64_t(delta);
            activeRate = targetMagnitude > previousMagnitude ? riseRate : fallRate;
        }
        if (direction != nextDirection) clearFraction();
        direction = nextDirection;
        if (activeRate == 0) {
            clearFraction();
            return crossingZero ? 0 : target;
        }
        const uint64_t budget = uint64_t(activeRate) * dtUs + remainder;
        const uint64_t step = budget / 1000;
        if (step >= distance) {
            clearFraction();
            return crossingZero ? 0 : target;
        }
        remainder = budget % 1000;
        return static_cast<int32_t>(int64_t(previous) + direction * int64_t(step));
    }

private:
    uint32_t lastTimeUs = 0;
    uint32_t lastRate = 0;
    uint32_t lastRiseRate = 0;
    uint32_t lastFallRate = 0;
    uint16_t remainder = 0; // 0..999 thousandths of one count
    int8_t direction = 0;
    bool initialized = false;
};
