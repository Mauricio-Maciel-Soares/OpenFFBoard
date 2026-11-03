#pragma once
#include "AxisSettingsStorage.h"
#include <array>

struct AxisSlewSettings : AxisSavedSettings {
    std::array<uint32_t, 3> slots{};
    uint8_t active = 0; // 0: custom; 1..3: stored slot
    uint8_t validSlots = 0;
    uint32_t torqueRiseRate = 0; // 0: inherit maxTorqueRate
    uint32_t torqueFallRate = 0; // 0: inherit maxTorqueRate
    bool asymmetric = false;
};

// New addresses leave the previous seven-word record intact for migration.
// One transaction covers rate, encvel, all three slots and the active choice.
class AxisSlewProfiles {
public:
    static bool valid(const AxisSlewSettings& s) {
        return s.active <= 3 && s.validSlots <= 7 && (!s.active ||
            ((s.validSlots & (1U << (s.active - 1))) && s.maxTorqueRate == s.slots[s.active - 1]));
    }
    static bool load(uint16_t base, uint16_t legacy, AxisSlewSettings& s) {
        uint16_t selector;
        if (!Flash_Read(base + 18, &selector, false)) {
            AxisSavedSettings old;
            if (!AxisSettingsStorage::load(legacy, old)) return false;
            s = {}; s.maxTorqueRate = old.maxTorqueRate; s.encoderVelocity = old.encoderVelocity;
            loadAsymmetric(base, s);
            return true;
        }
        if ((selector & 0xfffe) != selectTag) return false;
        uint16_t words[9];
        for (unsigned i = 0; i < 9; ++i)
            if (!Flash_Read(base + 9 * (selector & 1) + i, &words[i], false)) return false;
        if ((words[8] & 0xffc0) != flagsTag) return false;
        AxisSlewSettings result;
        result.maxTorqueRate = uint32_t(words[0]) | uint32_t(words[1]) << 16;
        for (unsigned i = 0; i < 3; ++i) result.slots[i] = uint32_t(words[2 + i * 2]) | uint32_t(words[3 + i * 2]) << 16;
        result.encoderVelocity = words[8] & 1;
        result.active = (words[8] >> 1) & 3;
        result.validSlots = (words[8] >> 3) & 7;
        if (!valid(result)) return false;
        loadAsymmetric(base, result);
        s = result;
        return true;
    }
    static bool save(uint16_t base, const AxisSlewSettings& s) {
        if (!valid(s)) return false;
        uint16_t words[9] = {uint16_t(s.maxTorqueRate), uint16_t(s.maxTorqueRate >> 16)};
        for (unsigned i = 0; i < 3; ++i) {
            words[2 + i * 2] = uint16_t(s.slots[i]);
            words[3 + i * 2] = uint16_t(s.slots[i] >> 16);
        }
        words[8] = flagsTag | uint16_t(s.encoderVelocity) | uint16_t(s.active << 1) | uint16_t(s.validSlots << 3);
        uint16_t selector;
        const bool selected = Flash_Read(base + 18, &selector, false) && (selector & 0xfffe) == selectTag;
        if (selected) {
            bool same = true;
            for (unsigned i = 0; i < 9; ++i) {
                uint16_t value;
                same = Flash_Read(base + 9 * (selector & 1) + i, &value, false) && value == words[i] && same;
            }
            if (same) return true;
        }
        const uint16_t next = selected ? ((selector & 1) ^ 1) : 0;
        for (unsigned i = 0; i < 9; ++i)
            if (!write(base + next * 9 + i, words[i])) return false;
        return write(base + 18, selectTag | next);
    }

    // Independent extension: bank A +19..+23, bank B +24..+28, selector +29.
    // The existing 19-word profile journal at +0..+18 remains unchanged.
    static bool loadAsymmetric(uint16_t base, AxisSlewSettings& s) {
        s.torqueRiseRate = 0;
        s.torqueFallRate = 0;
        s.asymmetric = false;
        uint16_t selector;
        if (!Flash_Read(base + 29, &selector, false) || (selector & 0xfffe) != asymSelectTag)
            return false;
        uint16_t words[5];
        const uint16_t bank = base + 19 + 5 * (selector & 1);
        for (unsigned i = 0; i < 5; ++i)
            if (!Flash_Read(bank + i, &words[i], false)) return false;
        if ((words[4] & 0xfffe) != asymFlagsTag) return false;
        s.torqueRiseRate = uint32_t(words[0]) | (uint32_t(words[1]) << 16);
        s.torqueFallRate = uint32_t(words[2]) | (uint32_t(words[3]) << 16);
        s.asymmetric = (words[4] & 1) != 0;
        return true;
    }

    static bool saveAsymmetric(uint16_t base, const AxisSlewSettings& s) {
        uint16_t selector;
        const bool selected = Flash_Read(base + 29, &selector, false)
            && (selector & 0xfffe) == asymSelectTag;
        const uint16_t words[5] = {
            uint16_t(s.torqueRiseRate), uint16_t(s.torqueRiseRate >> 16),
            uint16_t(s.torqueFallRate), uint16_t(s.torqueFallRate >> 16),
            uint16_t(asymFlagsTag | uint16_t(s.asymmetric))
        };
        if (!selected && !s.asymmetric && s.torqueRiseRate == 0 && s.torqueFallRate == 0)
            return true;
        if (selected) {
            bool same = true;
            const uint16_t bank = base + 19 + 5 * (selector & 1);
            for (unsigned i = 0; i < 5; ++i) {
                uint16_t value;
                same = Flash_Read(bank + i, &value, false) && value == words[i] && same;
            }
            if (same) return true;
        }
        const uint16_t next = selected ? ((selector & 1) ^ 1) : 0;
        const uint16_t bank = base + 19 + 5 * next;
        for (unsigned i = 0; i < 5; ++i)
            if (!write(bank + i, words[i])) return false;
        return write(base + 29, asymSelectTag | next);
    }

private:
    static constexpr uint16_t selectTag = 0x6b00, flagsTag = 0xc600;
    static constexpr uint16_t asymSelectTag = 0x7d00, asymFlagsTag = 0xd700;
    static bool write(uint16_t address, uint16_t value) {
        uint16_t read;
        if (Flash_Read(address, &read, false) && read == value) return true;
        Flash_Write(address, value);
        return Flash_Read(address, &read, false) && read == value;
    }
};
