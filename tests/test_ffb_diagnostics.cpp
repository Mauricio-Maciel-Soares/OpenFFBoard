#include "FFBDiagnostics.h"
#include <cassert>
#include <limits>
#include <initializer_list>

int main() {
    assert(readDiagnosticMicros([]{return 42U;}, []{return 999U;}, []{return false;}) == 42999U);
    assert(readDiagnosticMicros([]{return 42U;}, []{return 0U;}, []{return true;}) == 43000U);
    assert(readDiagnosticMicros([]{return 43U;}, []{return 0U;}, []{return false;}) == 43000U);
    // Overflow after the first flag read: discard the mixed snapshot and retry.
    unsigned flags = 0, counters = 0;
    assert(readDiagnosticMicros([]{return 42U;}, [&]{return counters++ ? 1U : 999U;},
                                [&]{return flags++ != 0;}) == 43001U);
    assert(counters == 2);
    AxisDiagnostics axis;
    axis.recordTime(UINT32_MAX - 499);
    assert(axis.dtUs == 0);
    axis.recordTime(500);
    assert(axis.dtUs == 1000 && axis.minDtUs == 1000);
    axis.recordTime(1750);
    assert(axis.maxDtUs == 1250 && axis.sequence == 3);
    axis.sequence = UINT32_MAX;
    axis.recordTime(2750);
    axis.recordTime(3750);
    assert(axis.dtUs == 1000 && axis.sequence == 1);
    assert(axis.minDtUs == 1000 && axis.maxDtUs == 1250);
    AxisDiagnostics startup;
    startup.recordTime(0);
    startup.recordTime(1000);
    startup.recordTime(2000);
    assert(startup.minDtUs == 1000 && startup.maxDtUs == 1000);

    // Known wire bytes for +1.5 A and -2.25 A (not native float serialization).
    uint8_t wire[8] = {0, 0, 0xc0, 0x3f, 0, 0, 0x10, 0xc0};
    float a = 9, b = 9;
    assert(decodeODrivePair(wire, 8, false, a, b));
    assert(a == 1.5f && b == -2.25f);
    for (unsigned length = 0; length < 8; ++length) {
        assert(!decodeODrivePair(wire, length, false, a, b));
        assert(a == 1.5f && b == -2.25f);
    }
    assert(!decodeODrivePair(wire, 8, true, a, b));
    for (float invalid : {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
        std::memcpy(wire + 4, &invalid, 4);
        assert(!decodeODrivePair(wire, 8, false, a, b));
        assert(a == 1.5f && b == -2.25f); // no partially updated pair
    }

    ODriveTelemetry drive;
    assert(!drive.velocityFresh(0));
    drive.encoderSequence = 1;
    drive.encoderTimeMs = UINT32_MAX - 2;
    assert(drive.velocityFresh(2)); // exactly 5 ms across rollover
    assert(!drive.velocityFresh(3));
}
