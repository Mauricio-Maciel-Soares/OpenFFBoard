"""Host regressions for real axis save/restore, setters and storage code."""
import importlib.util
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
INC = ROOT/'Firmware/FFBoard/Inc'
EXT = ROOT/'Firmware/FFBoard/UserExtensions'


def method(text, signature):
    start = text.index(signature)
    end = text.index('{', start) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


HARNESS = r'''
#include <cassert>
#include <cstdint>
#include <map>
#include <vector>
#include <stdexcept>
// Supply the storage backend, leaving AxisSettingsStorage itself unmodified.
#define FLASH_HELPERS_H_
std::map<uint16_t,uint16_t> flash;
unsigned writes=0;
int failAfter=-1;
bool cutPower=false;
bool Flash_Read(uint16_t a,uint16_t* value,bool checkempty=true){
    auto p=flash.find(a);if(p==flash.end())return false;
    if(checkempty && p->second==0xffff)return false; // also exercise external EEPROM empty-word semantics
    *value=p->second;return true;
}
bool Flash_Write(uint16_t a,uint16_t value){
    ++writes;
    if(failAfter==0){if(cutPower)throw std::runtime_error("power cut");return false;}
    if(failAfter>0)--failAfter;
    auto p=flash.find(a);if(p!=flash.end() && p->second==value)return false;
    flash[a]=value;return true;
}
#include "AxisSettingsStorage.h"
#include "eeprom_addresses.h"
void pulseErrLed(){}
enum class CommandStatus {OK,ERR,NOT_FOUND};
enum class CMDtype {get,set,getat,setat,other};
enum class Axis_commands {encvel,maxtorquerate};
struct ParsedCommand {Axis_commands cmdId;CMDtype type;int64_t val;};
using CommandReply=int64_t;
FLASH_ADDRESSES
struct Filter {void setFc(float){} void setQ(float){}};
struct Axis {
    AxisFlashAddrs flashAddrs;
    struct Config {uint8_t drvtype=0,enctype=0;} conf;
    static Config decodeConfFromInt(uint16_t){return {};}
    static uint16_t encodeConfToInt(const Config&){return 0;}
    void setDrvType(uint8_t){} void setEncType(uint8_t){}
    uint16_t maxSpeedDegS=0,power=5000,degreesOfRotation=900;
    uint8_t fx_ratio_i=204,endstopStrength=127,idlespringstrength=127,damperIntensity=30,frictionIntensity=0,inertiaIntensity=0;
    uint8_t filterProfileId=1;
    int8_t expoValInt=0;
    bool invertAxis=true,useEncoderVelocity=false;
    uint32_t maxTorqueRateMS=0;
    struct {unsigned resets=0;void reset(){++resets;}} torqueSlew;
    float filter_f=1000;
    struct FilterSetting {float freq=40,q=70;} filterSpeedCst[4],filterAccelCst[4];
    Filter speedFilter,accelFilter,damperFilter,frictionFilter,inertiaFilter;
    struct {uint8_t numerator=1,denominator=1;} gearRatio;
    void setPower(uint16_t p){power=p;}
    void setDegrees(uint16_t d){degreesOfRotation=d;}
    void setIdleSpringStrength(uint8_t v){idlespringstrength=v;}
    void setFxStrengthAndFilter(uint8_t v,uint8_t& member,Filter&){member=v;}
    void setGearRatio(uint8_t n,uint8_t d){gearRatio.numerator=n;gearRatio.denominator=d;}
    void setExpo(int8_t value){expoValInt=value;}
    void restoreFlash();void saveFlash();
    void restoreAxisTuningSettings();
    bool saveAxisTuningSettings(uint32_t,bool);
    CommandStatus setEncoderVelocity(int64_t);
    CommandStatus setMaxTorqueRate(int64_t);
    CommandStatus command(const ParsedCommand& cmd,std::vector<CommandReply>& replies){
        switch(cmd.cmdId){COMMAND_CASES default:return CommandStatus::NOT_FOUND;}
        return CommandStatus::OK;
    }
};
METHODS
CommandStatus set(Axis& a,Axis_commands name,int64_t value){
    std::vector<CommandReply> reply;return a.command({name,CMDtype::set,value},reply);
}
int64_t get(Axis& a,Axis_commands name){
    std::vector<CommandReply> reply;assert(a.command({name,CMDtype::get,0},reply)==CommandStatus::OK);
    assert(reply.size()==1);return reply[0];
}
int main(){
    // Legacy addresses do not opt in to the new features on first boot.
    flash[ADR_AXIS1_MAX_ACCEL]=12345;
    Axis initial;initial.restoreFlash();assert(!initial.useEncoderVelocity && initial.maxTorqueRateMS==0 && writes==0);
    assert(set(initial,Axis_commands::encvel,1)==CommandStatus::OK);
    assert(set(initial,Axis_commands::maxtorquerate,10922)==CommandStatus::OK);
    Axis reboot;reboot.restoreFlash();
    assert(get(reboot,Axis_commands::encvel)==1 && get(reboot,Axis_commands::maxtorquerate)==10922);
    assert(flash[ADR_AXIS1_MAX_ACCEL]==12345);
    writes=0;
    assert(set(reboot,Axis_commands::encvel,1)==CommandStatus::OK);
    assert(set(reboot,Axis_commands::maxtorquerate,10922)==CommandStatus::OK);
    get(reboot,Axis_commands::encvel);get(reboot,Axis_commands::maxtorquerate);assert(writes==0);
    std::vector<CommandReply> atReply;
    assert(reboot.command({Axis_commands::maxtorquerate,CMDtype::setat,10922},atReply)==CommandStatus::OK);
    assert(reboot.command({Axis_commands::maxtorquerate,CMDtype::getat,0},atReply)==CommandStatus::OK);
    assert(atReply.size()==1 && atReply[0]==10922 && writes==0);
    for(int64_t bad:{-1LL,2LL,4294967296LL})assert(set(reboot,Axis_commands::encvel,bad)==CommandStatus::ERR);
    for(int64_t bad:{-1LL,4294967296LL})assert(set(reboot,Axis_commands::maxtorquerate,bad)==CommandStatus::ERR);
    assert(writes==0 && reboot.maxTorqueRateMS==10922 && reboot.useEncoderVelocity);
    // Full uint32 range, including 0xffff halves; values are never truncated.
    for(uint32_t rate:{0U,5461U,16383U,65535U,65536U,0x1234abcdU,UINT32_MAX}){
        assert(set(reboot,Axis_commands::maxtorquerate,rate)==CommandStatus::OK);
        Axis next;next.restoreFlash();assert(next.maxTorqueRateMS==rate && next.useEncoderVelocity);
    }
    assert(set(reboot,Axis_commands::encvel,0)==CommandStatus::OK);
    assert(set(reboot,Axis_commands::maxtorquerate,0)==CommandStatus::OK);
    Axis disabled;disabled.restoreFlash();assert(!disabled.useEncoderVelocity && disabled.maxTorqueRateMS==0);

    // The existing Save button path also includes both settings.
    disabled.maxTorqueRateMS=0xdeadbeef;disabled.useEncoderVelocity=true;disabled.saveFlash();
    Axis manual;manual.restoreFlash();assert(manual.maxTorqueRateMS==0xdeadbeef && manual.useEncoderVelocity);

    // Independent per-axis records, with no overwrite of X or the old map.
    Axis y;y.flashAddrs.tuning=ADR_AXIS2_TUNING_A_LO;
    Axis z;z.flashAddrs.tuning=ADR_AXIS3_TUNING_A_LO;
    assert(set(y,Axis_commands::maxtorquerate,99)==CommandStatus::OK);
    assert(set(z,Axis_commands::encvel,1)==CommandStatus::OK);
    y.restoreAxisTuningSettings();z.restoreAxisTuningSettings();manual.restoreAxisTuningSettings();
    assert(y.maxTorqueRateMS==99 && !y.useEncoderVelocity && z.maxTorqueRateMS==0 && z.useEncoderVelocity);
    assert(manual.maxTorqueRateMS==0xdeadbeef && manual.useEncoderVelocity);

    // Failed writes return ERR before changing the active RAM setting.
    const auto stable=flash;
    failAfter=0;writes=0;
    assert(set(manual,Axis_commands::encvel,0)==CommandStatus::ERR);
    assert(manual.useEncoderVelocity && manual.maxTorqueRateMS==0xdeadbeef);
    assert(set(manual,Axis_commands::maxtorquerate,1)==CommandStatus::ERR);
    assert(flash==stable && manual.maxTorqueRateMS==0xdeadbeef);
    failAfter=-1;

    // Simulate power loss before each word write and restore a fresh object.
    // Both initial save and repeated bank reuse must select only complete data.
    for(unsigned generation=0;generation<4;++generation){
        flash.clear();writes=0;
        Axis a;
        for(unsigned i=0;i<=generation;++i){
            a.maxTorqueRateMS=0x12340000+i;a.useEncoderVelocity=(i&1)!=0;
            assert(a.saveAxisTuningSettings(a.maxTorqueRateMS,a.useEncoderVelocity));
        }
        const auto before=flash;
        for(int cut=0;cut<=4;++cut){
            flash=before;failAfter=cut;cutPower=true;
            bool committed=false;
            try{committed=a.saveAxisTuningSettings(0xabcd4321,!a.useEncoderVelocity);}catch(const std::runtime_error&){}
            failAfter=-1;cutPower=false;
            Axis recovered;recovered.restoreAxisTuningSettings();
            // Unchanged inactive-bank words can be skipped. After the selector
            // commits, a reboot must restore the new complete pair instead.
            assert(recovered.maxTorqueRateMS==(committed?0xabcd4321:a.maxTorqueRateMS));
            assert(recovered.useEncoderVelocity==(committed?!a.useEncoderVelocity:a.useEncoderVelocity));
        }
    }
    flash.clear();failAfter=2;cutPower=true;
    try{Axis a;a.setMaxTorqueRate(0x12345678);}catch(const std::runtime_error&){}
    failAfter=-1;cutPower=false;
    Axis firstIncomplete;firstIncomplete.restoreAxisTuningSettings();assert(firstIncomplete.maxTorqueRateMS==0 && !firstIncomplete.useEncoderVelocity);
    assert(set(firstIncomplete,Axis_commands::maxtorquerate,0x12345678)==CommandStatus::OK);
    Axis retried;retried.restoreAxisTuningSettings();assert(retried.maxTorqueRateMS==0x12345678);

    // Explicitly selecting a default on blank storage still creates a record.
    flash.clear();Axis defaults;
    assert(set(defaults,Axis_commands::maxtorquerate,0)==CommandStatus::OK);
    AxisSavedSettings saved;assert(AxisSettingsStorage::load(ADR_AXIS1_TUNING_A_LO,saved));
    assert(saved.maxTorqueRate==0 && !saved.encoderVelocity);
    // A missing or invalid committed record must not enable either option.
    flash[ADR_AXIS1_TUNING_A_FLAGS]=0xffff;
    defaults.maxTorqueRateMS=99;defaults.useEncoderVelocity=true;
    defaults.restoreAxisTuningSettings();assert(defaults.maxTorqueRateMS==0 && !defaults.useEncoderVelocity);
    flash[ADR_AXIS1_TUNING_A_FLAGS]=0xa500;flash.erase(ADR_AXIS1_TUNING_A_HI);
    defaults.restoreAxisTuningSettings();assert(defaults.maxTorqueRateMS==0 && !defaults.useEncoderVelocity);
}
'''


class PersistenceTests(unittest.TestCase):
    def test_reboot_commands_and_interrupted_save(self):
        text=(ROOT/'Firmware/FFBoard/Src/Axis.cpp').read_text()
        header=(INC/'Axis.h').read_text()
        cases='\n'.join(text[text.index('case Axis_commands::'+name+':'):text.index('case Axis_commands::'+following+':')]
                        for name,following in [('encvel','degrees'),('maxtorquerate','fxratio')])
        methods='\n'.join(method(text,signature) for signature in [
            'void Axis::restoreFlash(', 'void Axis::saveFlash(', 'void Axis::restoreAxisTuningSettings(',
            'bool Axis::saveAxisTuningSettings(', 'CommandStatus Axis::setEncoderVelocity(', 'CommandStatus Axis::setMaxTorqueRate('])
        program=HARNESS.replace('FLASH_ADDRESSES',method(header,'struct AxisFlashAddrs')+';').replace('COMMAND_CASES',cases).replace('METHODS',methods)
        with tempfile.TemporaryDirectory() as temp:
            d=Path(temp);(d/'main.h').write_text('#pragma once\n');(d/'test.cpp').write_text(program)
            exe=d/'test'
            subprocess.run(['g++','-std=c++20','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-no-pie','-I',str(d),'-I',str(INC),'-I',str(EXT/'Inc'),str(d/'test.cpp'),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

    def test_memory_map_and_generated_files(self):
        spec=importlib.util.spec_from_file_location('memory',ROOT/'Firmware/scripts/generate_memory.py')
        memory=importlib.util.module_from_spec(spec);spec.loader.exec_module(memory)
        entries=memory.parse_csv(ROOT/'Firmware/scripts/memory_map.csv')
        addresses=[int(e['adr'],0) for e in entries if e['adr']]
        self.assertEqual(len(addresses),len(set(addresses)))
        self.assertEqual(memory.generate_header(entries),(EXT/'Inc/eeprom_addresses.h').read_text())
        self.assertEqual(memory.generatesrc(entries),(EXT/'Src/eeprom_addresses.c').read_text())
        names=['TUNING_A_LO','TUNING_A_HI','TUNING_A_FLAGS','TUNING_B_LO','TUNING_B_HI','TUNING_B_FLAGS','TUNING_SELECT']
        for axis,base in [(1,0x30d),(2,0x34d),(3,0x38d)]:
            for offset,name in enumerate(names):
                entry=next(e for e in entries if e['name']==f'ADR_AXIS{axis}_{name}')
                self.assertEqual(int(entry['adr'],0),base+offset)
                self.assertEqual((entry['enable'],entry['export']),('1','1'))


if __name__=='__main__':
    unittest.main()
