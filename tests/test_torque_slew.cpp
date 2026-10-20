#include "TorqueSlewLimiter.h"
#include <cassert>
#include <cstdint>
#include <limits>

int main() {
    // Same elapsed time -> same command ramp at 1000, 500, 250 and 125 Hz.
    for (uint32_t period : {1000U, 2000U, 4000U, 8000U}) {
        TorqueSlewLimiter slew;
        int32_t output = slew.apply(30000, 0, 1000, 50, period);
        assert(output == 0); // no invented time on initialization
        for (uint32_t elapsed = period; elapsed <= 24000; elapsed += period) {
            output = slew.apply(30000, output, 1000, 50 + elapsed, period);
            assert(output == int32_t(elapsed));
        }
        assert(slew.apply(-30000, output, 1000, 50 + 24000 + period, period)
               == 24000 - int32_t(period));
    }

    // Jitter below the cap is integrated, including sub-count updates.
    TorqueSlewLimiter fractional;
    int32_t output = fractional.apply(100, 0, 1, 0, 1000);
    uint32_t now = 0;
    for (unsigned i = 0; i < 300; ++i) {
        now += i % 3 == 0 ? 100 : i % 3 == 1 ? 500 : 1400;
        output = fractional.apply(100, output, 1, now, 1000);
        assert(output == std::min<int32_t>(100, now / 1000));
    }
    assert(output == 100);

    // Zero elapsed time, zero rate, sign reversal and no banked idle credit.
    TorqueSlewLimiter s;
    assert(s.apply(100, 5, 10, 100, 1000) == 5);
    assert(s.apply(100, 5, 10, 100, 1000) == 5);
    assert(s.apply(100, 5, 0, 100, 1000) == 100);
    assert(s.apply(-100, 100, 0, 100, 1000) == -100);
    assert(s.apply(100, -100, 10, 10000, 1000) == -100); // re-enable establishes time
    s.reset();
    assert(s.apply(100, 0, 1, 0, 1000) == 0);
    assert(s.apply(100, 0, 1, 900, 1000) == 0);
    assert(s.apply(-100, 0, 1, 1000, 1000) == 0); // discard positive fraction
    assert(s.apply(-100, 0, 1, 1900, 1000) == -1);
    assert(s.apply(-1, -1, 1, 2900, 1000) == -1); // at target: no credit
    assert(s.apply(-100, -1, 1, 3000, 1000) == -1);

    // A long stall is capped relative to the selected loop rate.
    for (uint32_t period : {1000U, 2000U, 4000U, 8000U}) {
        s.reset();
        assert(s.apply(30000, 0, 1000, 0, period) == 0);
        assert(s.apply(30000, 0, 1000, 10000000, period) == int32_t(period * 3 / 2));
    }
    // No fractional credit survives a forced final power clip.
    s.reset();s.apply(100, 0, 1, 0, 1000);s.apply(100, 0, 1, 900, 1000);
    s.clearFraction();
    assert(s.apply(100, 0, 1, 1000, 1000) == 0);

    // Full 32-bit microsecond rollover uses unsigned subtraction.
    s.reset();
    assert(s.apply(10000, 0, 1000, UINT32_MAX - 499, 1000) == 0);
    assert(s.apply(10000, 0, 1000, 500, 1000) == 1000);
    // Maximum command-rate values cannot overflow signed output bounds.
    s.reset();s.apply(INT32_MAX, INT32_MIN, UINT32_MAX, 0, 1000);
    assert(s.apply(INT32_MAX, INT32_MIN, UINT32_MAX, 1000, 1000) == INT32_MAX);
    assert(s.apply(INT32_MIN, INT32_MAX, UINT32_MAX, 2000, 1000) == INT32_MIN);
}
