#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

enum SlewTraceFlag : uint16_t {
    SLEW_TRACE_ASYMMETRIC       = 1U << 0,
    SLEW_TRACE_LIMITER_CALLED   = 1U << 1,
    SLEW_TRACE_DT_CLAMPED       = 1U << 2,
    SLEW_TRACE_CROSSING_ZERO    = 1U << 3,
    SLEW_TRACE_FIRST_DT         = 1U << 4,
    SLEW_TRACE_DRIVER_READY     = 1U << 5,
    SLEW_TRACE_OUTPUT_ENABLED   = 1U << 6,
    SLEW_TRACE_DRIVER_TURN_CALL = 1U << 7,
    SLEW_TRACE_FINAL_POWER_CLIP = 1U << 8,
    SLEW_TRACE_PREVIOUS_VALID   = 1U << 9,
    SLEW_TRACE_NO_ACTIVE_RATE   = 1U << 10,
    SLEW_TRACE_LIMITER_BYPASSED = 1U << 11
};

// One record per FFB update. Values are integer counts/times so offline
// validation can reproduce the limiter's exact integer budget without floats.
struct SlewTraceSample {
    uint32_t sequence = 0;
    uint32_t timeUs = 0;
    uint32_t dtRawUs = 0;
    uint32_t dtUsedUs = 0;
    uint32_t nominalPeriodUs = 0;
    int32_t previousOutput = 0;
    int32_t requested = 0;
    int32_t clippedTarget = 0;
    int32_t slewOutput = 0;
    int32_t finalOutput = 0;
    uint32_t activeRate = 0;
    uint16_t remainderBefore = 0;
    uint16_t remainderAfter = 0;
    uint16_t flags = 0;
};

struct SlewTraceStatus {
    bool recording = false;
    uint32_t count = 0;
    uint32_t capacity = 0;
    uint32_t oldest = 0;
    uint32_t newest = 0;
    uint32_t overwritten = 0;
};

struct SlewTraceBatchInfo {
    uint32_t oldest = 0;
    uint32_t newest = 0;
    uint32_t gap = 0;
    uint32_t count = 0;
};

// Bounded, RAM-only capture. All mutating/read operations except isRecording()
// must be called from the caller's short critical section.
class SlewTrace {
public:
    static constexpr uint32_t capacity = 256;
    static constexpr uint32_t maxBatch = 16;

    bool isRecording() const { return recording; }

    void start() {
        writeIndex = 0;
        stored = 0;
        overwritten = 0;
        recording = true;
    }

    void stop() { recording = false; }

    void record(const SlewTraceSample& source) {
        if (!recording) return;
        SlewTraceSample sample = source;
        sample.sequence = ++sequence;
        samples[writeIndex] = sample;
        writeIndex = (writeIndex + 1U) % capacity;
        if (stored < capacity) {
            ++stored;
        } else if (overwritten != UINT32_MAX) {
            ++overwritten;
        }
    }

    SlewTraceStatus status() const {
        SlewTraceStatus result;
        result.recording = recording;
        result.count = stored;
        result.capacity = capacity;
        result.newest = sequence;
        result.oldest = stored ? sequence - stored + 1U : sequence + 1U;
        result.overwritten = overwritten;
        return result;
    }

    // cursor=-1 starts at the oldest retained sample. Otherwise returns records
    // after cursor. A nonzero gap means samples were overwritten before reading.
    SlewTraceBatchInfo readAfter(int64_t cursor,
                                 std::array<SlewTraceSample, maxBatch>& output) const {
        SlewTraceBatchInfo result;
        result.newest = sequence;
        result.oldest = stored ? sequence - stored + 1U : sequence + 1U;
        if (!stored || cursor < -1 || cursor > int64_t(UINT32_MAX)) return result;

        uint32_t wanted = result.oldest;
        if (cursor != -1) {
            const uint32_t behind = sequence - static_cast<uint32_t>(cursor);
            if (behind >= 0x80000000U) {
                result.gap = UINT32_MAX; // Cursor is from a future/reset sequence.
            } else if (behind > stored) {
                result.gap = behind - stored;
            } else {
                wanted = static_cast<uint32_t>(cursor) + 1U;
            }
            if (result.gap) wanted = result.oldest;
        }

        const uint32_t remaining = sequence - wanted;
        if (remaining >= 0x80000000U) return result;
        result.count = std::min<uint32_t>(maxBatch, remaining + 1U);
        for (uint32_t i = 0; i < result.count; ++i) {
            const uint32_t sampleSequence = wanted + i;
            const uint32_t age = sequence - sampleSequence;
            if (age >= stored) {
                result.count = i;
                break;
            }
            const uint32_t index = (writeIndex + capacity - 1U - age) % capacity;
            output[i] = samples[index];
        }
        return result;
    }

private:
    std::array<SlewTraceSample, capacity> samples{};
    uint32_t sequence = 0;
    uint32_t writeIndex = 0;
    uint32_t stored = 0;
    uint32_t overwritten = 0;
    volatile bool recording = false;
};
