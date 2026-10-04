"""Focused standalone conversion/build-contract checks; never start PX4/Gazebo.

Set ARION_OVERLAY_TEST_ROOT to an artifact directory before running. The PX4
source defaults to the sibling checkout; no fixture writes into that checkout.
"""
import os
import importlib.util
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

REPO = Path(__file__).resolve().parents[1]
PX4 = Path(os.environ.get("ARION_PX4_SOURCE", REPO / "PX4-Autopilot"))
RECEIVER = Path("src/modules/mavlink/mavlink_receiver.cpp")
PATCH = REPO / "patches/px4-v1.17.0/0002-sitl-external-battery.patch"


def checked_run(args, **kwargs):
    result = subprocess.run(args, text=True, capture_output=True, **kwargs)
    if result.returncode:
        raise AssertionError(f"{args!r} exited {result.returncode}\n{result.stdout}\n{result.stderr}")
    return result


class OverlayTests(unittest.TestCase):
    def setUp(self):
        root = Path(os.environ["ARION_OVERLAY_TEST_ROOT"]).resolve()
        root.mkdir(parents=True, exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(prefix="conversion-", dir=root)
        self.addCleanup(self.temp.cleanup)
        self.work = Path(self.temp.name)

    def helper(self):
        path = REPO / "tools/build_sitl_overlay.py"
        self.assertTrue(path.is_file(), "project-local overlay builder is required")
        spec = importlib.util.spec_from_file_location("sitl_overlay", path)
        if spec is None or spec.loader is None:
            self.fail("overlay builder cannot be loaded")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_build_command_isolation(self):
        helper = self.helper()
        entry = {"directory": "/base", "file": "/source/main.cpp", "command": 'c++ -DCONFIG_ARCH_BOARD_PX4_SITL -DPX4_BINARY_DIR=\\\"/base\\\" -I/base -MMD -MF old.d -o old.o -c /source/main.cpp'}
        command = helper.rewrite_compile(entry, Path("/out/source/main.cpp"), Path("/out/obj/main.cpp.o"), Path("/out"))
        self.assertIn('/out/source/main.cpp', command)
        self.assertIn('/out/obj/main.cpp.o', command)
        self.assertIn('-DPX4_BINARY_DIR="/out"', command)
        self.assertIn('-I/source', command)
        self.assertNotIn('old.d', command)
        self.assertNotIn('old.o', command)
        self.assertNotIn('/source/main.cpp', command)
        link = helper.extract_link_command(': && c++ old.o lib.a -o bin/px4 && cd /base/bin && cmake -E create_symlink px4 px4-test')
        self.assertEqual(link, ['c++', 'old.o', 'lib.a', '-o', 'bin/px4'])
        rewritten = helper.rewrite_link(link, {'old.o': '/out/obj/main.cpp.o', 'lib.a': '/out/archives/lib.a'}, Path('/out'), Path('/base'))
        self.assertIn('/out/bin/px4', rewritten)
        self.assertIn('/out/archives/lib.a', rewritten)
        self.assertNotIn('cmake', rewritten)

    def test_ipc_relocation_fails_closed(self):
        helper = self.helper()
        for kind, original in [('lock', '"/tmp/px4_lock"'), ('socket', '"/tmp/px4-sock-"')]:
            transformed = helper.relocate_ipc(original, kind, Path('/project/ipc'))
            self.assertNotIn('/tmp/', transformed)
            self.assertIn('/project/ipc/px4', transformed)
            with self.assertRaises(ValueError):
                helper.relocate_ipc(original + original, kind, Path('/project/ipc'))
            with self.assertRaises(ValueError):
                helper.relocate_ipc(original, kind, Path('/' + 'x' * 100))

    def test_runtime_packaging_preserves_aliases_without_old_binary(self):
        helper = self.helper()
        base = self.work / 'base'; out = self.work / 'out'
        (base / 'bin').mkdir(parents=True); (base / 'etc/init.d-posix').mkdir(parents=True)
        (base / 'bin/px4').write_bytes(b'old binary')
        (base / 'bin/px4-alias.sh').write_text("alias listener='px4-listener --instance $px4_instance'\n")
        (base / 'bin/px4-listener').symlink_to('px4')
        (base / 'etc/init.d-posix/rcS').write_text('# test rcS\n')
        (out / 'bin').mkdir(parents=True); (out / 'bin/px4').write_bytes(b'new ELF')
        helper.package_runtime(base, out)
        self.assertEqual((out / 'bin/px4').read_bytes(), b'new ELF')
        self.assertEqual((out / 'bin/px4-listener').readlink(), Path('px4'))
        self.assertEqual((out / 'bin/px4-alias.sh').read_bytes(), (base / 'bin/px4-alias.sh').read_bytes())
        self.assertEqual((out / 'etc/init.d-posix/rcS').read_bytes(), (base / 'etc/init.d-posix/rcS').read_bytes())

    def receiver_text(self):
        # In RED, exercise the uncorrected function, not a missing-file error.
        if not PATCH.exists():
            return (PX4 / RECEIVER).read_text()
        for rel in (RECEIVER, Path("src/modules/mavlink/streams/BATTERY_STATUS.hpp")):
            dest = self.work / rel
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(PX4 / rel, dest)
        checked_run(["patch", "--batch", "--fuzz=0", "-p1", "-i", str(PATCH)], cwd=self.work)
        return (self.work / RECEIVER).read_text()

    def run_receiver_fixture(self, assertions, extra=""):
        source = self.receiver_text()
        start = source.index("void\nMavlinkReceiver::handle_message_battery_status(")
        end = source.index("\nvoid\nMavlinkReceiver::", start + 5)
        function = source[start:end]
        fixture = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#define PX4_ISFINITE std::isfinite
namespace math { template<class T> T max(T a,T b) { return std::max(a,b); } }
constexpr int MAV_BATTERY_FUNCTION_ALL=1, MAV_BATTERY_TYPE_LIPO=3;
struct mavlink_battery_status_t {
    uint8_t id{}; int16_t temperature{}, current_battery{};
    int8_t battery_remaining{}; int32_t current_consumed{}, time_remaining{};
    int32_t energy_consumed{}; uint8_t battery_function{}, type{};
    uint16_t voltages[10]{}, voltages_ext[4]{};
};
struct mavlink_message_t { uint8_t sysid{42}, compid{180}; mavlink_battery_status_t payload{}; };
struct {uint8_t sysid{42}, compid{1};} mavlink_system;
void mavlink_msg_battery_status_decode(mavlink_message_t *m, mavlink_battery_status_t *b) { *b=m->payload; }
uint64_t hrt_absolute_time() { return 12345; }
struct battery_status_s {
    static constexpr uint8_t SOURCE_EXTERNAL=1, WARNING_NONE=0, WARNING_LOW=1, WARNING_CRITICAL=2, WARNING_EMERGENCY=3;
    uint64_t timestamp{}; uint8_t cell_count{}, id{}, source{}, warning{};
    float voltage_cell_v[14]{}, voltage_v{}, current_a{}, remaining{}, discharged_mah{}, temperature{};
    float time_remaining_s{}, current_average_a{}, scale{}; bool connected{};
};
struct Param { float value; float get() { return value; } };
struct Pub { battery_status_s value{}; int count{}; void publish(const battery_status_s &b) { value=b; ++count; } };
class MavlinkReceiver {
public:
    Param _param_bat_emergen_thr{0.05f}, _param_bat_crit_thr{0.10f}, _param_bat_low_thr{0.15f};
    Pub _battery_pub;
    void handle_message_battery_status(mavlink_message_t *msg);
};
bool close(float a, float b) { return std::fabs(a-b)<0.00002f; }
''' + function + extra + r'''
int main() {
    MavlinkReceiver receiver; mavlink_message_t msg;
    auto &m=msg.payload;
    std::fill_n(m.voltages,10,UINT16_MAX);
    m.voltages[0]=4010; m.voltages[1]=4020; m.voltages[2]=4030; m.voltages[3]=4040;
    m.current_battery=725; m.current_consumed=123; m.battery_remaining=73; m.temperature=2850;
''' + assertions + "\n}\n"
        cpp = self.work / "fixture.cpp"
        cpp.write_text(fixture)
        exe = self.work / "fixture"
        checked_run(["c++", "-std=c++14", "-Wall", "-Wextra", "-Werror", "-fsanitize=undefined,float-cast-overflow", "-fno-sanitize-recover=all", str(cpp), "-o", str(exe)])
        checked_run([str(exe)], cwd=self.work)

    def test_known_battery_temperature_units(self):
        self.run_receiver_fixture(r'''
    receiver.handle_message_battery_status(&msg);
    const auto &b=receiver._battery_pub.value;
    assert(receiver._battery_pub.count==1);
    assert(close(b.temperature,28.5f));
    assert(close(b.voltage_v,16.10f)); assert(b.cell_count==4);
    assert(close(b.current_a,7.25f)); assert(close(b.remaining,0.73f));
    assert(close(b.discharged_mah,123.f));
''')

    def test_external_identity_unknowns_and_sender_filter(self):
        self.run_receiver_fixture(r'''
    m.current_battery=-1; m.battery_remaining=-1; m.current_consumed=-1;
    m.temperature=INT16_MAX; m.time_remaining=0;
    receiver.handle_message_battery_status(&msg);
    const auto &b=receiver._battery_pub.value;
    assert(b.id==1 && b.source==battery_status_s::SOURCE_EXTERNAL);
    assert(b.current_a==-1.f && b.remaining==-1.f && b.discharged_mah==-1.f);
    assert(std::isnan(b.temperature) && std::isnan(b.time_remaining_s));
    assert(b.warning==battery_status_s::WARNING_NONE);
    assert(b.current_average_a==-1.f && b.scale==-1.f);
    for (int id=1;id<=255;++id) {
        m.id=id; receiver.handle_message_battery_status(&msg);
    }
    assert(receiver._battery_pub.count==1); // one uORB publisher: battery 0 only
    m.id=0; msg.sysid=43; receiver.handle_message_battery_status(&msg);
    msg.sysid=42; msg.compid=1; receiver.handle_message_battery_status(&msg);
    assert(receiver._battery_pub.count==1);
    msg.compid=180; m.time_remaining=120; m.battery_remaining=4;
    receiver.handle_message_battery_status(&msg);
    assert(close(b.time_remaining_s,120.f)); assert(b.warning==battery_status_s::WARNING_EMERGENCY);
    m.current_battery=-250; m.temperature=-1234; m.battery_remaining=73;
    receiver.handle_message_battery_status(&msg);
    assert(close(b.current_a,-2.5f)); assert(close(b.temperature,-12.34f));
    assert(b.warning==battery_status_s::WARNING_NONE);
''')


    def test_unknown_current_and_soc_restream(self):
        self.receiver_text()
        stream = (self.work / "src/modules/mavlink/streams/BATTERY_STATUS.hpp").read_text()
        assignments = stream[stream.index("\t\t\t\tbat_msg.id ="):stream.index("\n\t\t\t\tswitch (battery_status.warning)")]
        temperature = stream[stream.index("\t\t\t\t// check if temperature valid"):stream.index("\n\t\t\t\t// fill cell voltages")]
        extra = "\nmavlink_battery_status_t output(const battery_status_s &battery_status) { mavlink_battery_status_t bat_msg{};\n" + assignments + temperature + "\nreturn bat_msg; }\n"
        # run_receiver_fixture applies the patch again: remove only this fixture's copies.
        shutil.rmtree(self.work / "src")
        self.run_receiver_fixture(r'''
    m.current_battery=-1; m.battery_remaining=-1; m.temperature=INT16_MAX;
    receiver.handle_message_battery_status(&msg);
    auto wire=output(receiver._battery_pub.value);
    assert(wire.current_battery==-1 && wire.battery_remaining==-1);
    assert(wire.temperature==INT16_MAX && wire.time_remaining==0 && wire.id==0);
    m.current_battery=-250; m.battery_remaining=73; m.temperature=2850;
    receiver.handle_message_battery_status(&msg);
    wire=output(receiver._battery_pub.value);
    assert(wire.current_battery==-250 && wire.battery_remaining==73 && wire.temperature==2850);
''', extra=extra)


    def test_pack_only_voltage_has_no_phantom_cell_count(self):
        self.run_receiver_fixture(r'''
    std::fill_n(m.voltages,10,UINT16_MAX); m.voltages[0]=16100;
    receiver.handle_message_battery_status(&msg);
    const auto &b=receiver._battery_pub.value;
    assert(b.cell_count==0 && close(b.voltage_v,16.1f) && b.connected);
    assert(close(b.voltage_cell_v[0],0.f));
    m.voltages[0]=65534; m.voltages[1]=1000;
    receiver.handle_message_battery_status(&msg);
    assert(b.cell_count==0 && close(b.voltage_v,66.534f));
    std::fill_n(m.voltages,10,UINT16_MAX);
    receiver.handle_message_battery_status(&msg);
    assert(!b.connected && b.cell_count==0 && close(b.voltage_v,0.f));
''')


if __name__ == "__main__":
    unittest.main()
