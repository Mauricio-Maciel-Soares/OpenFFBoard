"""Sanitizer-backed regressions for magnitude-based asymmetric torque slew."""
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
INC = ROOT / "Firmware/FFBoard/Inc"

CPP = r'''
#include "TorqueSlewLimiter.h"
#include <cassert>
#include <cstdint>
#include <climits>

int main() {
    TorqueSlewLimiter slew;
    assert(slew.applyAsymmetric(10000, 0, 1000, 200, 100, 1000) == 0);
    assert(slew.applyAsymmetric(10000, 0, 1000, 200, 1100, 1000) == 1000);
    assert(slew.applyAsymmetric(5000, 1000, 1000, 200, 2100, 1000) == 2000);
    assert(slew.applyAsymmetric(1500, 2000, 1000, 200, 3100, 1000) == 1800);

    // Positive-to-negative reversal releases to zero with fallRate.
    TorqueSlewLimiter reverse;
    assert(reverse.applyAsymmetric(900, 0, 1000, 200, 0, 1000) == 0);
    assert(reverse.applyAsymmetric(-9000, 900, 1000, 200, 1000, 1000) == 700);
    assert(reverse.applyAsymmetric(-9000, 700, 1000, 200, 2000, 1000) == 500);
    assert(reverse.applyAsymmetric(-9000, 500, 1000, 200, 3000, 1000) == 300);
    assert(reverse.applyAsymmetric(-9000, 300, 1000, 200, 4000, 1000) == 100);
    assert(reverse.applyAsymmetric(-9000, 100, 1000, 200, 5000, 1000) == 0);
    assert(reverse.applyAsymmetric(-9000, 0, 1000, 200, 6000, 1000) == -1000);

    // Mirrored reversal: no left/right sign bias.
    TorqueSlewLimiter negative;
    assert(negative.applyAsymmetric(-5000, 0, 500, 100, 0, 1000) == 0);
    assert(negative.applyAsymmetric(-5000, 0, 500, 100, 1000, 1000) == -500);
    assert(negative.applyAsymmetric(5000, -500, 500, 100, 2000, 1000) == -400);

    TorqueSlewLimiter fastRelease;
    assert(fastRelease.applyAsymmetric(1000, 1000, 500, 5000, 0, 1000) == 1000);
    assert(fastRelease.applyAsymmetric(-1000, 1000, 500, 5000, 1000, 1000) == 0);
    assert(fastRelease.applyAsymmetric(-1000, 0, 500, 5000, 2000, 1000) == -500);

    TorqueSlewLimiter unlimitedRise;
    assert(unlimitedRise.applyAsymmetric(100, 0, 0, 100, 0, 1000) == 100);
    assert(unlimitedRise.applyAsymmetric(-100, 100, 0, 100, 1000, 1000) == 0);

    TorqueSlewLimiter fractional;
    assert(fractional.applyAsymmetric(10, 0, 1, 1, 0, 1000) == 0);
    assert(fractional.applyAsymmetric(10, 0, 1, 1, 500, 1000) == 0);
    assert(fractional.applyAsymmetric(10, 0, 1, 1, 1000, 1000) == 1);
    assert(fractional.applyAsymmetric(-10, 1, 1, 1, 1500, 1000) == 1);

    TorqueSlewLimiter stalled;
    assert(stalled.applyAsymmetric(30000, 0, 1000, 1000, 0, 1000) == 0);
    assert(stalled.applyAsymmetric(30000, 0, 1000, 1000, 10000000, 1000) == 1500);

    TorqueSlewLimiter extremes;
    assert(extremes.applyAsymmetric(INT32_MAX, INT32_MIN, 1000, 1000, 0, 1000) == INT32_MIN);
    assert(extremes.applyAsymmetric(INT32_MAX, INT32_MIN, 1000, 1000, 1000, 1000) == INT32_MIN + 1000);
    assert(extremes.applyAsymmetric(INT32_MIN, INT32_MAX, 1000, 1000, 2000, 1000) == INT32_MAX - 1000);

    // The legacy symmetric entry point remains unchanged.
    TorqueSlewLimiter legacy;
    assert(legacy.apply(10000, 0, 1000, 0, 1000) == 0);
    assert(legacy.apply(10000, 0, 1000, 1000, 1000) == 1000);
}
'''

class AsymmetricSlewTests(unittest.TestCase):
    def test_magnitude_ramp_and_zero_crossing(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "test.cpp"
            binary = root / "test"
            source.write_text(CPP)
            subprocess.run([
                "g++", "-std=c++20", "-Wall", "-Wextra", "-Werror",
                "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
                "-no-pie", "-I", str(INC), str(source), "-o", str(binary)
            ], check=True)
            subprocess.run([str(binary)], check=True)

if __name__ == "__main__":
    unittest.main()
