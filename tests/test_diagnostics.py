"""Host regressions: actual Axis method bodies with hardware/filter test doubles."""
import importlib.util
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
INC = ROOT / "Firmware/FFBoard/Inc"
spec = importlib.util.spec_from_file_location("capture", ROOT / "tools/capture_odrive_diagnostics.py")
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


def method(text, signature):
    start = text.index(signature)
    opening = text.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[start:end]


HARNESS = r'''
#include "FFBDiagnostics.h"
#include <cassert>
#include <algorithm>
#include <cmath>
#include <tuple>
using std::abs;
namespace cpp_freertos { struct CriticalSection { static void Enter(){} static void Exit(){} }; }
void pulseClipLed() {}
template<class T,class C> T clip(T v,C l,C h) { return v > h ? h : v < l ? l : v; }
struct Metrics { float posDegrees=0,pos_f=0,speed=0,accel=0; int32_t pos_scaled_16b=0,torque=0; };
struct Filter { float process(float v) { return v; } }; // isolate source selection, not biquad behavior
struct Encoder { bool fresh=true; float velocity=2; bool getVelocity_f(float& v){v=velocity; return fresh;} };
struct Driver { bool motorReady(){return true;} };
struct Axis {
    AxisDiagnostics diagnostics,workingDiagnostics;
    struct {Metrics current,previous;} metric;
    struct {float gearRatio=1;} gearRatio;
    Driver driver; Driver* drv=&driver; Encoder encoder;
    Filter speedFilter,accelFilter;
    float filter_f=1000,_lastSpeed=0;
    bool useEncoderVelocity=false,encoderVelocityWasUsed=false;
    uint16_t degreesOfRotation=900;
    int32_t effectTorque=0,axisEffectTorque=0;
    float expo=1,effect_margin_scaler=1,torqueScaler=1;
    uint16_t maxSpeedDegS=0,power=10000;
    uint32_t maxTorqueRateMS=1000;
    float speedLimiterP=.3,speedLimiterI=.03,spdlimitreducerI=0;
    bool outOfBounds=false,invertAxis=false;
    float forceFadeCurMult=1,forceFadeTime=1;
    Encoder* getEncoder(){return &encoder;}
    bool isInverted(){return invertAxis;}
    std::pair<int32_t,float> scaleEncValue(float p,uint16_t){return {int32_t(p),p};}
    int32_t calculateExpoTorque(int32_t t){return t;}
    int16_t updateEndstop(){return 0;}
    bool updateTorque(int32_t*);
    void updateMetrics(float);
};
'''

CHECKS = r'''
int main(){
    Axis a; int32_t output;
    a.effectTorque=50000;
    for(int n=1;n<=15;++n){
        a.updateMetrics(0); a.updateTorque(&output);
        assert(output==std::min(n*1000,10000)); // ramp must not get stuck or wind up
    }
    assert(a.diagnostics.clippedTarget==10000 && a.diagnostics.output==10000);
    assert(a.diagnostics.limited==9 && a.diagnostics.clipped==15);
    a.effectTorque=-50000;
    a.updateMetrics(0); a.updateTorque(&output); assert(output==9000);
    a.power=1000;
    a.updateMetrics(0); a.updateTorque(&output); assert(output==1000); // reduced power immediately
    a.outOfBounds=true;
    a.updateMetrics(0); a.updateTorque(&output); assert(output==0); // safety bypass

    Axis v;
    v.updateMetrics(1); assert(v.metric.current.speed==1000); // default path unchanged
    assert(!v.diagnostics.encoderUsed);
    v.useEncoderVelocity=true; v.gearRatio.gearRatio=0.5f; v.invertAxis=true;
    v.updateMetrics(2);
    assert(v.metric.current.speed==-360 && v.metric.current.accel==0); // sign, turns/s and ratio
    assert(v.workingDiagnostics.encoderUsed);
    v.encoder.velocity=3;
    v.updateMetrics(3); assert(v.metric.current.speed==-540 && v.metric.current.accel==-180000);
    v.encoder.fresh=false;
    v.updateMetrics(4); assert(v.metric.current.speed==1000 && v.metric.current.accel==0);
    assert(!v.workingDiagnostics.encoderUsed);
    v.encoder.fresh=true;
    v.updateMetrics(5); assert(v.metric.current.speed==-540 && v.metric.current.accel==0);
}
'''


class DiagnosticsTests(unittest.TestCase):
    def test_cpp(self):
        with tempfile.TemporaryDirectory() as temp:
            temp = Path(temp)
            text = (ROOT / "Firmware/FFBoard/Src/Axis.cpp").read_text()
            axis = temp / "axis.cpp"
            axis.write_text(HARNESS + method(text, "void Axis::updateMetrics(")
                            + method(text, "bool Axis::updateTorque(") + CHECKS)
            for source in (ROOT / "tests/test_ffb_diagnostics.cpp", axis):
                exe = temp / source.stem
                subprocess.run(["g++", "-std=c++20", "-Wall", "-Wextra", "-Werror", "-I", str(INC), str(source), "-o", str(exe)], check=True)
                subprocess.run([str(exe)], check=True)

    def test_no_torque_estimate_from_stale_or_missing_iq(self):
        drive = dict(iq_sequence=0, iq_age_ms=0, iq_setpoint_a=2, iq_measured_a=1)
        self.assertEqual(capture.torque_estimates(drive, 1.11, 50)["estimated_torque_nm"], "")
        drive.update(iq_sequence=1, iq_age_ms=51)
        self.assertEqual(capture.torque_estimates(drive, 1.11, 50)["iq_fresh"], 0)
        drive["iq_age_ms"] = 10
        result = capture.torque_estimates(drive, 1.11, 50)
        self.assertAlmostEqual(result["estimated_torque_nm"], 1.11)
        self.assertAlmostEqual(result["iq_tracking_error_nm"], 1.11)

    def test_schema_validation(self):
        with self.assertRaises(ValueError):
            capture.parse_payload("ERR", capture.AXIS_FIELDS)
        with self.assertRaises(ValueError):
            capture.parse_payload("2,10", ["version", "value"])
        with self.assertRaises(ValueError):
            capture.parse_payload("1,nan", ["version", "value"])

    def test_fragmented_serial_and_unrelated_reply(self):
        class Port:
            in_waiting = 0
            chunks = iter([b"[sys.version?|x]\n[axis.0.", b"diag?|1,23", b"]\n"])
            def write(self, data):
                self.written = data
            def read(self, _):
                return next(self.chunks, b"")
        port = Port()
        self.assertEqual(capture.read_snapshot(port, "axis.0.diag?", ["version", "value"])["value"], 23)
        self.assertEqual(port.written, b"axis.0.diag?\n")


if __name__ == "__main__":
    unittest.main()
