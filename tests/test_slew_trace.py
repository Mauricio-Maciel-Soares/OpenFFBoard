"""Host regression tests for the bounded high-frequency SlewTrace and observations."""
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
INC = ROOT / "Firmware/FFBoard/Inc"

CPP = r'''
#include "SlewTrace.h"
#include "TorqueSlewLimiter.h"
#include <array>
#include <cassert>
#include <cstdint>

int main() {
    TorqueSlewLimiter limiter;
    TorqueSlewObservation observation;
    assert(limiter.applyAsymmetric(20000, 0, 8192, 4096, 1000, 1000, &observation) == 0);
    assert(observation.dtRawUs == 0);
    assert(observation.dtUsedUs == 0);
    assert(observation.flags & TORQUE_SLEW_OBS_FIRST_SAMPLE);
    assert(limiter.applyAsymmetric(20000, 0, 8192, 4096, 2000, 1000, &observation) == 8192);
    assert(observation.dtRawUs == 1000 && observation.dtUsedUs == 1000);
    assert(observation.activeRate == 8192);
    assert(limiter.applyAsymmetric(0, 8192, 8192, 4096, 3000, 1000, &observation) == 4096);
    assert(observation.activeRate == 4096);

    // Ring retains the newest 256 samples and reports monotonic sequence IDs.
    SlewTrace trace;
    trace.start();
    for (uint32_t i = 0; i < 300; ++i) {
        SlewTraceSample sample;
        sample.timeUs = i * 1000;
        sample.finalOutput = static_cast<int32_t>(i);
        trace.record(sample);
    }
    const SlewTraceStatus status = trace.status();
    assert(status.recording && status.count == 256 && status.capacity == 256);
    assert(status.oldest == 45 && status.newest == 300 && status.overwritten == 44);
    std::array<SlewTraceSample, SlewTrace::maxBatch> batch{};
    const SlewTraceBatchInfo info = trace.readAfter(-1, batch);
    assert(info.count == 16 && info.gap == 0);
    assert(batch[0].sequence == 45 && batch[15].sequence == 60);
    const SlewTraceBatchInfo after = trace.readAfter(290, batch);
    assert(after.count == 10 && batch[0].sequence == 291 && batch[9].sequence == 300);
    const SlewTraceBatchInfo lost = trace.readAfter(1, batch);
    assert(lost.gap == 43 && lost.count == 16 && batch[0].sequence == 45);
    trace.stop();
    assert(!trace.isRecording());
    trace.start();
    assert(trace.status().count == 0 && trace.status().newest == 0);
}
'''

class SlewTraceTests(unittest.TestCase):
    def test_trace_ring_and_limiter_observation(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / "test.cpp"
            binary = Path(temp) / "test"
            source.write_text(CPP)
            subprocess.run([
                "g++", "-std=c++20", "-Wall", "-Wextra", "-Werror",
                "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
                "-no-pie", "-I", str(INC), str(source), "-o", str(binary)
            ], check=True)
            subprocess.run([str(binary)], check=True)

if __name__ == "__main__":
    unittest.main()
