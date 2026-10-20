#pragma once

#include <cstdint>
#include "flash_helpers.h"

struct AxisSavedSettings {
    uint32_t maxTorqueRate = 0;
    bool encoderVelocity = false;
};

// Seven consecutive virtual EEPROM words per axis: two banks of
// {rate low, rate high, tagged encvel}, followed by a tagged bank selector.
// Write/verify the inactive bank before selecting it. An interrupted bank
// update leaves the previously selected bank intact. The EEPROM backend owns
// power-failure recovery and atomicity of each individual word record.
class AxisSettingsStorage {
public:
    static bool load(uint16_t base, AxisSavedSettings& settings) {
        uint16_t selected;
        if (!readSelector(base, selected)) return false;
        const uint16_t bank = base + 3 * (selected & 1);
        uint16_t lo, hi, flags;
        if (!Flash_Read(bank, &lo, false) || !Flash_Read(bank + 1, &hi, false)
            || !Flash_Read(bank + 2, &flags, false) || (flags & 0xfffe) != flagsTag) return false;
        settings.maxTorqueRate = uint32_t(lo) | (uint32_t(hi) << 16);
        settings.encoderVelocity = (flags & 1) != 0;
        return true;
    }

    static bool save(uint16_t base, const AxisSavedSettings& settings) {
        AxisSavedSettings saved;
        if (load(base, saved) && saved.maxTorqueRate == settings.maxTorqueRate
            && saved.encoderVelocity == settings.encoderVelocity) return true;

        uint16_t selected;
        const uint16_t next = readSelector(base, selected) ? ((selected & 1) ^ 1) : 0;
        const uint16_t bank = base + 3 * next;
        if (!writeVerified(bank, uint16_t(settings.maxTorqueRate))
            || !writeVerified(bank + 1, uint16_t(settings.maxTorqueRate >> 16))
            || !writeVerified(bank + 2, flagsTag | uint16_t(settings.encoderVelocity))) return false;
        return writeVerified(base + 6, selectorTag | next);
    }

private:
    static constexpr uint16_t flagsTag = 0xa500;
    static constexpr uint16_t selectorTag = 0x5a00;

    static bool readSelector(uint16_t base, uint16_t& selected) {
        return Flash_Read(base + 6, &selected, false) && (selected & 0xfffe) == selectorTag;
    }

    static bool writeVerified(uint16_t address, uint16_t value) {
        uint16_t stored;
        // Flash_Write returns false both on failure and when a word is unchanged.
        if (Flash_Read(address, &stored, false) && stored == value) return true;
        Flash_Write(address, value);
        return Flash_Read(address, &stored, false) && stored == value;
    }
};
