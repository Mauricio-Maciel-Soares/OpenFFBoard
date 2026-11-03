"""Compile the actual HidFFB class and USB callback with hardware test doubles."""
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
INC = ROOT / 'Firmware/FFBoard/Inc'
SRC = ROOT / 'Firmware/FFBoard/Src'


def method(text, signature):
    start = text.index(signature)
    end = text.index('{', start) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


COMMON = r'''
#pragma once
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
#define MAX_AXIS 2
#define CFG_TUD_HID_EP_BUFSIZE 64
enum hid_report_type_t {HID_REPORT_TYPE_INVALID, HID_REPORT_TYPE_INPUT, HID_REPORT_TYPE_OUTPUT, HID_REPORT_TYPE_FEATURE};
inline uint32_t tick = 0;
inline uint32_t HAL_GetTick(){return tick;}
template<class T,class C> T clip(T v,C lo,C hi){return v<lo?lo:v>hi?hi:v;}
inline bool tud_hid_report(uint8_t,const void*,uint16_t){return true;}
'''
USB = r'''
#pragma once
#include "cppmain.h"
struct UsbHidHandler {
    inline static UsbHidHandler* globalHidHandler=nullptr;
    virtual ~UsbHidHandler()=default;
    virtual void hidOut(uint8_t,hid_report_type_t,const uint8_t*,uint16_t)=0;
    virtual uint16_t hidGet(uint8_t,hid_report_type_t,uint8_t*,uint16_t)=0;
    void registerHidCallback(){globalHidHandler=this;}
};
'''
CALCULATOR = r'''
#pragma once
#include "ffb_defs.h"
struct EffectsCalculator {
    static constexpr uint32_t max_effects=MAX_EFFECTS;
    std::array<FFB_Effect,max_effects> effects;
    bool active=false;
    uint8_t gain=255;
    void updateSamplerate(float){}
    void setActive(bool v){active=v;}
    void setGain(uint8_t v){gain=v;}
    void setFilters(FFB_Effect*){}
    void logEffectType(uint8_t,bool=false){}
    void logEffectState(uint8_t,uint8_t){}
    int32_t find_free_effect(uint8_t);
    void free_effect(uint16_t);
};
struct EffectsControlItf {void fxUpdateEvent(){} void cfUpdateEvent(){}};
'''
HARNESS = r'''
#include "HidFFB.h"
struct HID_CMD_Data_t {uint8_t bytes[25];};
struct HID_CommandInterface {
    inline static HID_CommandInterface* globalInterface=nullptr;
    unsigned calls=0;
    void hidCmdCallback(HID_CMD_Data_t*){++calls;}
};
CALLBACK
ALLOCATOR

void out(HidFFB& h, const void* data, uint16_t len){
    const auto b=static_cast<const uint8_t*>(data);
    h.hidOut(b[0],HID_REPORT_TYPE_OUTPUT,b,len);
}
void control(HidFFB& h,uint8_t value){uint8_t b[]={HID_ID_CTRLREP,value};out(h,b,sizeof b);}
FFB_BlockLoad_Feature_Data_t block(HidFFB& h){
    FFB_BlockLoad_Feature_Data_t b{};
    assert(h.hidGet(HID_ID_BLKLDREP,HID_REPORT_TYPE_FEATURE,reinterpret_cast<uint8_t*>(&b),sizeof b)==sizeof b);
    return b;
}
uint8_t allocate(HidFFB& h,uint8_t type=FFB_EFFECT_CONSTANT){
    FFB_CreateNewEffect_Feature_Data_t b{type,0};
    h.hidOut(HID_ID_NEWEFREP,HID_REPORT_TYPE_FEATURE,reinterpret_cast<uint8_t*>(&b),sizeof b);
    return block(h).effectBlockIndex;
}
uint8_t status(HidFFB& h){
    uint8_t value=0;
    assert(h.hidGet(HID_ID_STATE,HID_REPORT_TYPE_INPUT,&value,1)==1);
    return value;
}
void operation(HidFFB& h,uint8_t id,uint8_t op){FFB_EffOp_Data_t b{HID_ID_EFOPREP,id,op,1};out(h,&b,sizeof b);}
void configure(HidFFB& h,uint8_t id){
    FFB_SetEffect_t b{};b.effectBlockIndex=id;b.effectType=FFB_EFFECT_CONSTANT;b.enableAxis=1;
    out(h,&b,sizeof b);
}
int main(int argc,char** argv){
    assert(argc==2);
    const std::string test=argv[1];
    auto ec=std::make_shared<EffectsCalculator>();
    HidFFB h(ec,1);
    if(test=="pool"){
        const auto full=block(h).ramPoolAvailable;
        for(unsigned i=1;i<=MAX_EFFECTS;++i){assert(allocate(h)==i);assert(block(h).ramPoolAvailable==full-i*sizeof(FFB_Effect));}
        assert(allocate(h)==0 && block(h).loadStatus==2);
        uint8_t free[]={HID_ID_BLKFRREP,7};out(h,free,2);
        assert(block(h).ramPoolAvailable==sizeof(FFB_Effect));
        out(h,free,2);assert(block(h).ramPoolAvailable==sizeof(FFB_Effect));
        free[1]=0;out(h,free,2);free[1]=255;out(h,free,2);
        assert(allocate(h)==7 && block(h).ramPoolAvailable==0);
        control(h,8);assert(block(h).ramPoolAvailable==full && h.getFfbActive());
        assert(allocate(h,FFB_EFFECT_CUSTOM)==0 && block(h).loadStatus==3);
        assert(allocate(h,0)==0 && block(h).loadStatus==3);
        assert(allocate(h)==1);
        h.stop_FFB();h.reset_ffb();assert(!h.getFfbActive()); // USB suspend path
    }else if(test=="control"){
        allocate(h);allocate(h);configure(h,1);operation(h,1,1);operation(h,2,1);
        assert(status(h)&HID_EFFECT_PLAYING);
        control(h,4);assert(h.getFfbActive() && !ec->effects[0].state && !ec->effects[1].state);
        assert(!(status(h)&HID_EFFECT_PLAYING));
        configure(h,1);assert(!ec->effects[0].state); // stopped effects must not restart
        operation(h,1,1);operation(h,2,2);assert(!ec->effects[0].state && ec->effects[1].state);
        operation(h,2,3);operation(h,2,0);operation(h,2,4);operation(h,3,1);
        assert(!ec->effects[1].state && !ec->effects[2].state);
        control(h,2);configure(h,1);assert(!h.getFfbActive());
        assert(!(status(h)&(HID_ENABLE_ACTUATORS|HID_EFFECT_PAUSE)));
        control(h,32);assert(!h.getFfbActive());
        control(h,1);assert(h.getFfbActive());
    }else if(test=="pause"){
        allocate(h);configure(h,1);tick=100;operation(h,1,1);
        tick=120;control(h,16);assert(!h.getFfbActive());
        assert((status(h)&(HID_EFFECT_PAUSE|HID_ENABLE_ACTUATORS))==(HID_EFFECT_PAUSE|HID_ENABLE_ACTUATORS));
        tick=130;control(h,16);configure(h,1);assert(!h.getFfbActive());
        tick=220;control(h,32);assert(h.getFfbActive() && ec->effects[0].startTime==200);
        tick=0xfffffff0;control(h,16);operation(h,1,1);
        tick=20;control(h,32);assert(ec->effects[0].startTime==20); // timer wrap
        control(h,16);control(h,2);control(h,32);assert(!h.getFfbActive());
    }else if(test=="condition"){
        for(uint8_t axes: {1,2}){
            auto calc=std::make_shared<EffectsCalculator>();HidFFB device(calc,axes);allocate(device,FFB_EFFECT_SPRING);
            for(unsigned raw=0;raw<256;++raw){
                for(auto& c:calc->effects[0].conditions)c.cpOffset=0;
                FFB_SetCondition_Data_t b{};b.reportId=HID_ID_CONDREP;b.effectBlockIndex=1;
                b.parameterBlockOffset=raw;b.cpOffset=123;b.positiveCoefficient=100;
                out(device,&b,sizeof b);
                const unsigned axis=raw&(axes==1?0x3f:0x0f);
                for(unsigned i=0;i<MAX_AXIS;++i){
                    assert(calc->effects[0].conditions[i].cpOffset==((axis<axes && i==axis)?123:0));
                    assert(calc->effects[0].conditions[i].positiveSaturation==0);
                }
            }
        }
    }else if(test=="transport"){
        allocate(h);
        FFB_SetConstantForce_Data_t b{HID_ID_CONSTREP,1,-1234};
        auto bytes=reinterpret_cast<const uint8_t*>(&b);
        tud_hid_set_report_cb(0,0,HID_REPORT_TYPE_OUTPUT,bytes,sizeof b);assert(ec->effects[0].magnitude==-1234);
        b.magnitude=1234;
        tud_hid_set_report_cb(0,b.reportId,HID_REPORT_TYPE_OUTPUT,bytes+1,sizeof b-1);assert(ec->effects[0].magnitude==1234);
        uint8_t newEffect[]={FFB_EFFECT_SPRING,0,0};
        tud_hid_set_report_cb(0,HID_ID_NEWEFREP,HID_REPORT_TYPE_FEATURE,newEffect,sizeof newEffect);
        assert(ec->effects[1].type==FFB_EFFECT_SPRING);
        tud_hid_set_report_cb(0,0,HID_REPORT_TYPE_OUTPUT,nullptr,0);
        uint8_t tooLong[65]{};tud_hid_set_report_cb(0,HID_ID_GAINREP,HID_REPORT_TYPE_OUTPUT,tooLong,65);assert(ec->gain==255);
        HID_CommandInterface commands;HID_CommandInterface::globalInterface=&commands;
        uint8_t command[25]={HID_ID_HIDCMD};
        tud_hid_set_report_cb(0,0,HID_REPORT_TYPE_OUTPUT,command,1);assert(commands.calls==0);
        tud_hid_set_report_cb(0,0,HID_REPORT_TYPE_OUTPUT,command,25);assert(commands.calls==1);
        tud_hid_set_report_cb(0,HID_ID_HIDCMD,HID_REPORT_TYPE_OUTPUT,command+1,24);assert(commands.calls==2);
        tud_hid_set_report_cb(0,HID_ID_HIDCMD,HID_REPORT_TYPE_FEATURE,command,25);assert(commands.calls==2);
    }else if(test=="bounds"){
        // Every supported output must reject every truncated prefix without reading beyond it.
        const std::pair<uint8_t,unsigned> sizes[]={{1,18},{2,14},{3,15},{4,12},{5,4},{6,6},{10,4},{11,2},{12,2},{13,2}};
        allocate(h);ec->effects[0].magnitude=99;
        for(auto [id,size]:sizes){
            for(unsigned len=0;len<size;++len){
                std::vector<uint8_t> b(len,1);if(len)b[0]=id;
                h.hidOut(id,HID_REPORT_TYPE_OUTPUT,b.data(),len);
            }
        }
        assert(ec->effects[0].magnitude==99 && ec->effects[0].type==FFB_EFFECT_CONSTANT && !h.getFfbActive());
        for(unsigned len=0;len<3;++len){std::vector<uint8_t> b(len,1);h.hidOut(HID_ID_NEWEFREP,HID_REPORT_TYPE_FEATURE,b.data(),len);}
        assert(ec->effects[1].type==FFB_EFFECT_NONE);
        uint8_t b[]={HID_ID_GAINREP,3};h.hidOut(HID_ID_GAINREP,HID_REPORT_TYPE_FEATURE,b,2);assert(ec->gain==255);
        b[0]=0;h.hidOut(HID_ID_GAINREP,HID_REPORT_TYPE_OUTPUT,b,2);assert(ec->gain==255);
        auto calc=std::make_shared<EffectsCalculator>();HidFFB two(calc,2);allocate(two);
        uint8_t set[22]={1,1,1};
        for(unsigned len=0;len<22;++len)two.hidOut(1,HID_REPORT_TYPE_OUTPUT,set,len);
        assert(!two.getFfbActive());two.hidOut(1,HID_REPORT_TYPE_OUTPUT,set,22);assert(two.getFfbActive());
    }else if(test=="get"){
        uint8_t b[8];
        for(uint8_t id:{HID_ID_BLKLDREP,HID_ID_POOLREP}){
            for(unsigned len=0;len<8;++len){
                memset(b,0xa5,sizeof b);
                const auto n=h.hidGet(id,HID_REPORT_TYPE_FEATURE,b,len);assert(n==std::min(len,4u));
                for(unsigned i=n;i<8;++i)assert(b[i]==0xa5);
            }
            assert(h.hidGet(id,HID_REPORT_TYPE_INPUT,b,8)==0);
        }
        assert(h.hidGet(HID_ID_STATE,HID_REPORT_TYPE_FEATURE,b,8)==0);
        assert(h.hidGet(HID_ID_POOLREP,HID_REPORT_TYPE_FEATURE,nullptr,8)==0);
        assert(h.hidGet(HID_ID_STATE,HID_REPORT_TYPE_INPUT,b,0)==0);
        assert(!(status(h)&HID_EFFECT_PLAYING));
    }else{assert(false);}
}
'''


class HidPidTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        d = Path(cls.temp.name)
        for name in ['HidFFB.h', 'ffb_defs.h']:
            (d/name).write_text((INC/name).read_text())
        (d/'cppmain.h').write_text(COMMON)
        (d/'UsbHidHandler.h').write_text(USB)
        (d/'EffectsCalculator.h').write_text(CALCULATOR)
        (d/'Filters.h').write_text('#pragma once\nstruct Biquad {};\n')
        for name in ['constants.h', 'PersistentStorage.h', 'FastAvg.h', 'flash_helpers.h', 'hid_device.h']:
            (d/name).write_text('#pragma once\n')
        callback = method((SRC/'global_callbacks.cpp').read_text(), 'void tud_hid_set_report_cb(')
        calculator = (SRC/'EffectsCalculator.cpp').read_text()
        allocator = '\n'.join(method(calculator, signature) for signature in ['void EffectsCalculator::free_effect(', 'int32_t EffectsCalculator::find_free_effect('])
        (d/'main.cpp').write_text(HARNESS.replace('CALLBACK', callback).replace('ALLOCATOR', allocator))
        (d/'HidFFB.cpp').write_text((SRC/'HidFFB.cpp').read_text())
        cls.exe = d/'test'
        subprocess.run(['g++', '-std=c++17', '-g', '-O1', '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer', '-no-pie', '-I', str(d), str(d/'main.cpp'), str(d/'HidFFB.cpp'), '-o', str(cls.exe)], check=True)

    def test_descriptor_contract(self):
        # Read compiled descriptors rather than assuming struct sizes match the wire.
        d = Path(self.temp.name)
        (d/'hid.h').write_text('#define HID_INPUT(x) 0x81,x\n#define HID_OUTPUT(x) 0x91,x\n#define BYTES32(x) ((x)&255),(((x)>>8)&255),(((x)>>16)&255),(((x)>>24)&255)\n#define HID_LOGICAL_MIN_N(x,n) 0x17,BYTES32(x)\n#define HID_LOGICAL_MAX_N(x,n) 0x27,BYTES32(x)\n' + '\n'.join(f'#define HID_USAGE_DESKTOP_{name} {value}' for name, value in
            [('X',48),('Y',49),('Z',50),('RX',51),('RY',52),('RZ',53),('SLIDER',54),('DIAL',55)]))
        desc = ROOT/'Firmware/FFBoard/UserExtensions'
        (d/'usb_hid_ffb_desc.h').write_text((desc/'Inc/usb_hid_ffb_desc.h').read_text())
        for axes in [1, 2]:
            for bits in [16, 32]:
                with self.subTest(axes=axes, bits=bits):
                    suffix = '_32b' if bits == 32 else ''
                    flag = f'AXIS{axes}_FFB_HID_DESC' + ('_32B' if bits == 32 else '')
                    symbol = f'hid_{axes}ffb_desc{suffix}'
                    (d/'descriptor.cpp').write_text(
                        f'#define {flag}\n#define __ALIGN_BEGIN\n#define __ALIGN_END\n'
                        + (desc/f'Src/usb_hid_{axes}ffb_desc.c').read_text()
                        + f'\n#include <cstdio>\nint main(){{fwrite({symbol},1,sizeof({symbol}),stdout);}}\n')
                    subprocess.run(['g++','-std=c++17','-I',str(d),str(d/'descriptor.cpp'),'-o',str(d/'descriptor')],check=True)
                    data = subprocess.check_output([str(d/'descriptor')])
                    report_id = size = count = logical_max = usage_page = 0
                    totals = {}
                    enable_max = []
                    offset_width = []
                    usages = []
                    pos = 0
                    while pos < len(data):
                        prefix = data[pos];pos += 1
                        n = (0,1,2,4)[prefix & 3]
                        value = int.from_bytes(data[pos:pos+n], 'little');pos += n
                        tag = prefix & 0xfc
                        if tag == 0x84: report_id = value
                        elif tag == 0x74: size = value
                        elif tag == 0x94: count = value
                        elif tag == 0x24: logical_max = value
                        elif tag == 0x04: usage_page = value
                        elif tag == 0x08: usages.append((usage_page, value))
                        if tag in [0x80,0x90,0xb0]:
                            key = (tag,report_id)
                            totals[key] = totals.get(key,0) + size * count
                            if tag == 0x90 and report_id == 1 and any(u in usages for u in [(1,0x30),(15,0x56)]):
                                enable_max.append(logical_max)
                            if tag == 0x90 and report_id == 3 and (15,0x23) in usages:
                                offset_width.append(size)
                        if (prefix & 0x0c) == 0: usages = [] # local items end at each Main item
                    expected = {1:14+4*axes,2:14,3:15,4:12,5:4,6:6,10:4,11:2,12:2,13:2}
                    for report, length in expected.items():
                        self.assertEqual(totals[(0x90,report)],(length-1)*8)
                    self.assertEqual(enable_max,[1,1])
                    self.assertEqual(offset_width,[6 if axes == 1 else 4])
                    self.assertEqual(totals[(0x80,2)],8)
                    self.assertEqual(totals[(0xb0,17)],24)
                    self.assertEqual(totals[(0xb0,18)],32)
                    self.assertEqual(totals[(0xb0,19)],32)

    def test_pid_regressions(self):
        for case in ['pool', 'control', 'pause', 'condition', 'transport', 'bounds', 'get']:
            with self.subTest(case=case):
                subprocess.run([str(self.exe), case], check=True)


if __name__ == '__main__':
    unittest.main()
