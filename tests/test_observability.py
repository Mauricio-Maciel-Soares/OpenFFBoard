"""Compile actual CAN receive/command paths and bounded telemetry helpers."""
import subprocess
import tempfile
import unittest
from pathlib import Path
from test_diagnostics import ROOT, INC, method

HARNESS = r'''
#include "FFBDiagnostics.h"
#include "ODriveTrace.h"
#include <vector>
#include <string>
#include <cstdio>
#include <cassert>
#include <limits>
namespace cpp_freertos {struct CriticalSection {static void Enter(){} static void Exit(){}};}
uint32_t clockUs=0;
uint32_t coherentMicros(){return clockUs;}
uint32_t HAL_GetTick(){return clockUs/1000;}
enum class ODriveState:uint32_t {AXIS_STATE_IDLE=1,AXIS_STATE_CLOSED_LOOP_CONTROL=8};
enum class ODriveLocalState {WAIT_READY};
enum class ODriveCAN_commands:uint32_t {diag,diag2,trace};
enum class CMDtype {get,getat,set};
enum class CommandStatus {OK,ERR};
struct ParsedCommand {uint32_t cmdId; CMDtype type; int64_t adr=0;};
using CommandReply=std::string;
struct CANPort {} bus;
struct CAN_rx_msg {struct {uint32_t id=0,length=8;bool extId=false,rtr=false;} header;uint8_t data[8]={};};
struct ODriveCAN {
    CANPort* port=&bus;uint8_t nodeId=0;
    ODriveTelemetry telemetry;ODriveTrace torqueTrace;
    uint32_t lastCanMessage=0,lastPosTime=0,lastVoltageUpdate=0;
    float lastPos=0,lastSpeed=0,lastVoltage=0;
    uint32_t errors=0,odriveMotorFlags=0,odriveEncoderFlags=0,odriveControllerFlags=0;
    bool waitReady=false,posWaiting=false,queueOk=true,heartbeatSeen=true;uint32_t lastHeartbeatMs=0;
    bool velocityIsValid(uint32_t,uint32_t) const;
    ODriveState odriveCurrentState=ODriveState::AXIS_STATE_CLOSED_LOOP_CONTROL;
    ODriveLocalState state=ODriveLocalState::WAIT_READY;
    template<class T>bool sendMsg(uint8_t,T){return queueOk;}
    void setTorque(float);bool getVelocity_f(float&);
    void canRxPendCallback(CANPort*,CAN_rx_msg&);void canErrorCallback(CANPort*,uint32_t);
    CommandStatus command(const ParsedCommand& cmd,std::vector<CommandReply>& replies){
        switch(static_cast<ODriveCAN_commands>(cmd.cmdId)){CASES default:return CommandStatus::ERR;}
        return CommandStatus::OK;
    }
};
METHODS
void rx(ODriveCAN& d,uint8_t cmd,float a,float b){
    CAN_rx_msg m;m.header.id=cmd;std::memcpy(m.data,&a,4);std::memcpy(m.data+4,&b,4);d.canRxPendCallback(&bus,m);
}
std::string get(ODriveCAN& d,ODriveCAN_commands cmd,int64_t adr=0){
    std::vector<CommandReply> replies;
    assert(d.command({uint32_t(cmd),cmd==ODriveCAN_commands::trace?CMDtype::getat:CMDtype::get,adr},replies)==CommandStatus::OK);
    assert(replies.size()==1);return replies[0];
}
int main(){
    ODriveCAN d;float velocity;
    clockUs=1000;rx(d,0x09,1,2);assert(!d.getVelocity_f(velocity));
    clockUs=2000;rx(d,0x09,1.002,2);assert(d.getVelocity_f(velocity)&&velocity==2);
    clockUs=2100;d.setTorque(3.5);
    clockUs=2400;rx(d,0x17,48,2);
    clockUs=3000;rx(d,0x14,3,2.5);
    ODriveTraceSample sample;assert(d.torqueTrace.read(1,sample));
    assert(sample.iqMeasured==2.5 && sample.queuedTorque==3.5 && sample.torqueAgeUs==900);
    assert(sample.encoderAgeUs==1000 && sample.voltageAgeUs==600 && sample.encoderValid);
    auto trace=get(d,ODriveCAN_commands::trace,-1);
    assert(trace.find("1,1,1,0,1\n1,3000,3,2.5,3.5,2,48,900,1000,600,1,2,0,8,1") == 0);
    assert(get(d,ODriveCAN_commands::trace,1)=="1,1,1,0,0");
    auto diag=get(d,ODriveCAN_commands::diag2);
    assert(std::count(diag.begin(),diag.end(),',')==41 && diag.substr(0,2)=="2,");
    auto legacy=get(d,ODriveCAN_commands::diag);assert(std::count(legacy.begin(),legacy.end(),',')==12 && legacy.substr(0,2)=="1,");
    clockUs=7001;assert(!d.getVelocity_f(velocity));
    clockUs=8000;rx(d,0x09,2,3);assert(!d.getVelocity_f(velocity));
    clockUs=9000;rx(d,0x09,2,3);assert(d.getVelocity_f(velocity));
    rx(d,0x09,2,std::numeric_limits<float>::infinity());assert(!d.getVelocity_f(velocity));
    assert(d.telemetry.invalidEncoder==1);
    clockUs=10000;rx(d,0x09,2,3);clockUs=11000;rx(d,0x09,2,3);assert(d.getVelocity_f(velocity));
    d.heartbeatSeen=false;assert(!d.getVelocity_f(velocity));d.heartbeatSeen=true;
    clockUs=2012000;rx(d,0x09,2,3);assert(!d.getVelocity_f(velocity));
    d.lastHeartbeatMs=HAL_GetTick();clockUs+=1000;rx(d,0x09,2,3);assert(d.getVelocity_f(velocity));
    d.canErrorCallback(&bus,1);assert(!d.getVelocity_f(velocity)&&d.telemetry.busErrors==1);
    CAN_rx_msg malformed;malformed.header.id=9;malformed.header.length=4;d.canRxPendCallback(&bus,malformed);
    assert(d.telemetry.invalidEncoder==2);
    d.queueOk=false;d.setTorque(9);assert(d.telemetry.txSequence==1&&d.telemetry.queuedTorque==3.5&&d.telemetry.txErrors==1);
    for(unsigned i=0;i<70;++i){clockUs+=1000;rx(d,0x14,1,1);}
    assert(get(d,ODriveCAN_commands::trace,1).find("1,8,71,6,16\n8,")==0);
    assert(get(d,ODriveCAN_commands::trace,100).find("1,8,71,4294967295,16\n8,")==0);
}
'''


class ObservabilityTests(unittest.TestCase):
    def test_real_can_handlers(self):
        source = (ROOT/'Firmware/FFBoard/UserExtensions/Src/ODriveCAN.cpp').read_text()
        cases = source[source.index('case ODriveCAN_commands::diag:'):source.index('case ODriveCAN_commands::vbus:')]
        methods = '\n'.join(method(source,s) for s in ('bool ODriveCAN::velocityIsValid(', 'bool ODriveCAN::getVelocity_f(', 'void ODriveCAN::setTorque(', 'void ODriveCAN::canRxPendCallback(', 'void ODriveCAN::canErrorCallback('))
        program = HARNESS.replace('CASES',cases).replace('METHODS',methods)
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp);(path/'can.cpp').write_text(program)
            for src in (path/'can.cpp',ROOT/'tests/test_observability.cpp'):
                exe=path/src.stem
                subprocess.run(['g++','-std=c++20','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-no-pie','-I',str(INC),str(src),'-o',str(exe)],check=True)
                subprocess.run([str(exe)],check=True)


if __name__ == '__main__':
    unittest.main()
