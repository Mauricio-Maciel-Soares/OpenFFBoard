#include "TimingStatistics.h"
#include "VelocityContinuity.h"
#include "ODriveTrace.h"
#include "FFBDiagnostics.h"
#include <cassert>

int main() {
    TimingStatistics t;
    t.record(UINT32_MAX - 999); t.record(0); t.record(1100); t.record(2000);
    assert(t.count == 3 && t.lastUs == 900 && t.minUs == 900 && t.maxUs == 1100);
    assert(std::abs(t.meanUs - 1000) < .01 && std::abs(t.stddevUs() - 81.6497) < .01);
    for (unsigned i=0; i<1021; ++i) t.record(t.lastTimeUs + 1000);
    assert(t.count == 1024);
    t.record(t.lastTimeUs + 8000);
    assert(t.count == 1 && t.minUs == 8000 && t.stddevUs() == 0);

    VelocityContinuity v;
    assert(v.update(100, false, 0, 1000).speed == 100);
    auto r=v.update(-100, true, 1000, 1000);
    assert(r.speed==100 && r.acceleration==0 && r.transitioning);
    r=v.update(-90, true, 2000, 1000);
    assert(r.speed==70 && r.acceleration==10000); // offset decay excluded from inertia
    r=v.update(-90, true, 6000, 1000);
    assert(r.speed==-90 && r.acceleration==0 && !r.transitioning);
    r=v.update(500, false, 7000, 1000);
    assert(r.speed==-90 && r.acceleration==0);
    assert(v.switches==2);

    ODriveTelemetry drive;
    drive.recordEncoder(UINT32_MAX - 1999); assert(!drive.velocityFreshUs(UINT32_MAX - 1999));
    drive.recordEncoder(UINT32_MAX - 999); assert(drive.velocityFreshUs(4000));
    assert(!drive.velocityFreshUs(4001));
    drive.recordEncoder(6000); assert(!drive.velocityFreshUs(6000));
    drive.recordEncoder(7000); assert(drive.velocityFreshUs(7000));
    drive.freshEncoderStreak=0;drive.recordEncoder(8000);assert(!drive.velocityFreshUs(8000));
    drive.recordEncoder(9000);assert(drive.velocityFreshUs(9000));

    ODriveTrace trace; ODriveTraceSample s;
    assert(!trace.read(0,s));
    for(unsigned i=1;i<=70;++i){s.timeUs=i*1000;s.iqMeasured=float(i);trace.record(s);}
    assert(trace.count()==64 && trace.newest()==70 && !trace.read(6,s));
    assert(trace.read(7,s) && s.sequence==7 && s.timeUs==7000 && s.iqMeasured==7);
    assert(trace.read(70,s) && s.sequence==70 && !trace.read(71,s));
    trace.clear();assert(trace.count()==0 && !trace.read(70,s));
    trace.record({});assert(trace.newest()==71 && trace.read(71,s));
}
