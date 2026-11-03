#pragma once
#include <cstdint>
#include <array>

// An observation made at CAN RX, not a synchronized ODrive control-loop sample.
// Commands were accepted by the local CAN driver; there is no motor ACK.
struct ODriveTraceSample {
    uint32_t sequence = 0, timeUs = 0;
    float iqSetpoint = 0, iqMeasured = 0, queuedTorque = 0, velocity = 0, busVoltage = 0;
    uint32_t torqueAgeUs = UINT32_MAX, encoderAgeUs = UINT32_MAX, voltageAgeUs = UINT32_MAX;
    uint32_t txSequence = 0, encoderSequence = 0;
    uint32_t axisErrors = 0, axisState = 0, encoderValid = 0;
};

class ODriveTrace {
public:
    static constexpr unsigned capacity = 64;
    void record(ODriveTraceSample sample) {
        sample.sequence = ++sequence; // wrap is handled by unsigned distance
        entries[writeIndex] = sample;
        writeIndex = (writeIndex + 1) % capacity;
        if (size < capacity) ++size;
    }
    // Copy one record while interrupts are masked. Formatting happens outside.
    bool read(uint32_t wanted, ODriveTraceSample& out) const {
        const uint32_t distance = sequence - wanted;
        if (distance >= size) return false;
        out = entries[(writeIndex + capacity - 1 - distance) % capacity];
        return true;
    }
    uint32_t newest() const { return sequence; }
    unsigned count() const { return size; }
    void clear() { size = 0; writeIndex = 0; } // retain sequence across node changes
private:
    std::array<ODriveTraceSample, capacity> entries{};
    uint32_t sequence = 0;
    unsigned writeIndex = 0, size = 0;
};
