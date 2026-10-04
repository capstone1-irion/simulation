#include "arion/CoupledPowerModel.hpp"
#include "arion/MotorFrame.hpp"
#include "arion/BatteryFrame.hpp"
#include "arion/BatteryPublishClock.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
void expect(bool condition, const std::string &message)
{
    if (!condition) { throw std::runtime_error(message); }
}
void near(double actual, double expected, double tolerance, const std::string &message)
{
    expect(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
           message + ": actual=" + std::to_string(actual) +
           " expected=" + std::to_string(expected));
}
arion::PowerMotorInput joints(const std::array<double, 4> &rpm)
{
    arion::PowerMotorInput input;
    for (std::size_t i = 0; i < rpm.size(); ++i) {
        const double direction = i < 2 ? 1.0 : -1.0;
        input.joint_rad_s[i] = direction * rpm[i] * (2.0 * std::acos(-1.0)) / 60.0 / 10.0;
    }
    return input;
}
void joint_response()
{
    arion::CoupledPowerModel model;
    expect(model.update(0, 0.0, false, joints({4042.0, 4855.0, 6800.0, 9857.0})),
           "all four joint observations accepted");
    const auto first = model.sample();
    const std::array<double, 4> expected_rpm{4042.0, 4855.0, 6800.0, 9857.0};
    const std::array<double, 4> expected_a{1.44, 2.29, 5.61, 16.37};
    for (std::size_t i = 0; i < 4; ++i) {
        near(first.rpm[i], expected_rpm[i], 1e-8, "signed joint velocity restored to rotor RPM");
        near(first.motor_current_a[i], expected_a[i], 1e-10, "16 V bench knot current");
        expect(first.rpm_source[i] == arion::PowerRpmSource::JointVelocity,
               "joint observation is not command feedback");
    }
    expect(model.update(10000000, 0.01, false, joints({0.0, 2021.0, 4255.5, 11000.0})),
           "changed joint observations accepted");
    const auto &changed = model.sample();
    near(changed.motor_current_a[0], 0.0, 1e-12, "stopped rotor current assumption");
    near(changed.motor_current_a[1], 0.36, 1e-12, "explicit assumed low-RPM quadratic");
    near(changed.motor_current_a[2], (1.44 + 1.87) / 2.0, 1e-10, "bench interpolation");
    near(changed.motor_current_a[3], 16.37, 1e-12, "upper current cap, not extrapolation");
    near(changed.rpm[3], 11000.0, 1e-8, "current cap does not silently clamp observed RPM");
}
void modeled_fallback_response()
{
    arion::CoupledPowerModel model;
    arion::PowerMotorInput input;
    input.command_valid = true;
    input.command_rad_s = {400.0, 500.0, 600.0, 700.0};
    expect(model.update(0, 0.0, false, input), "missing joints use explicitly modeled response");
    near(model.sample().rpm[0], 0.0, 0.0, "command is not echoed as measured RPM");
    expect(model.update(12500000, 0.0125, false, input), "modeled spin-up accepted");
    const double rpm_per_rad_s = 60.0 / (2.0 * std::acos(-1.0));
    for (std::size_t i = 0; i < 4; ++i) {
        near(model.sample().rpm[i], input.command_rad_s[i] * rpm_per_rad_s * (1.0 - std::exp(-1.0)),
             1e-9, "first-order spin-up uses 12.5 ms, not command echo");
        expect(model.sample().rpm_source[i] == arion::PowerRpmSource::ModeledCommand,
               "fallback carries modeled-source identity");
    }
    const auto spun_up = model.sample().rpm;
    input.command_rad_s.fill(0.0);
    expect(model.update(37500000, 0.025, false, input), "modeled spin-down accepted");
    near(model.sample().rpm[0], spun_up[0] * std::exp(-1.0), 1e-9, "25 ms spin-down response");
    input.joint_rad_s[0] = joints({4500.0, 0.0, 0.0, 0.0}).joint_rad_s[0];
    expect(model.update(47500000, 0.01, false, input), "joint plus modeled sources accepted");
    near(model.sample().rpm[0], 4500.0, 1e-9, "observed joint takes priority over command");
    expect(model.sample().rpm_source[0] == arion::PowerRpmSource::JointVelocity,
           "mixed-source frame identifies actual joint slot");
}
void battery_load_and_wire_contract()
{
    arion::CoupledPowerModel model;
    const auto input = joints({4042.0, 4855.0, 6800.0, 9857.0});
    expect(model.update(0, 0.0, false, input), "initial loaded sample");
    const double total_a = 1.44 + 2.29 + 5.61 + 16.37 + 0.5;
    near(model.sample().pack_current_a, total_a, 1e-10, "battery receives four currents plus avionics");
    const double initial_v = model.sample().terminal_voltage_v;
    expect(model.update(1000000000, 1.0, false, input), "one simulated second of load");
    const auto &sample = model.sample();
    near(sample.consumed_mah, total_a / 3.6, 1e-10, "coulomb counting to mAh");
    near(sample.soc, 0.95 - total_a / (5.0 * 3600.0), 1e-12, "4S 5Ah reference SOC decreases");
    near(sample.open_circuit_voltage_v, 4.0 * (3.3 + 0.9 * sample.soc), 1e-10, "uncalibrated linear OCV");
    near(sample.terminal_voltage_v, sample.open_circuit_voltage_v - total_a * 0.04, 1e-10,
         "ohmic pack sag");
    expect(sample.terminal_voltage_v < initial_v, "depletion lowers terminal voltage at same load");
    const auto motor = model.motor_frame();
    const auto battery = model.battery_frame();
    arion::MotorFrame motor_wire;
    arion::BatteryWireValues battery_wire;
    expect(motor.size() == 12 && arion::decode_motor_frame(motor.data(), motor.size(), motor_wire),
           "existing 12-float motor parser accepts model");
    expect(battery.size() == 9 && arion::decode_battery_frame(battery.data(), battery.size(), battery_wire),
           "existing 9-float battery parser accepts model");
    for (std::size_t i = 0; i < 4; ++i) {
        near(motor_wire[i].rpm, sample.rpm[i], 1e-3, "motor RPM slot");
        near(motor_wire[i].current, sample.motor_current_a[i], 1e-5, "motor current slot");
        near(motor_wire[i].voltage, sample.terminal_voltage_v, 1e-5, "motor reports pack terminal voltage");
        near(battery[i + 1] * 4.0, sample.terminal_voltage_v, 1e-5, "balanced reference cell volts");
    }
    near(battery[0], 0.0, 0.0, "one pack ID 0");
    near(battery[5], total_a, 1e-5, "battery amps slot");
    near(battery[6], sample.soc, 1e-6, "battery SOC slot");
    near(battery[7], 25.0, 0.0, "documented constant temperature placeholder");
    near(battery[8], sample.consumed_mah, 1e-5, "battery consumed mAh slot");

    arion::CoupledPowerConfig cfg;
    cfg.capacity_ah = 2.0;
    cfg.pack_resistance_ohm = 0.06;
    cfg.initial_soc = 0.8;
    cfg.auxiliary_current_a = 1.0;
    arion::CoupledPowerModel custom(cfg);
    expect(custom.update(0, 0.0, false, input), "custom reference setup");
    expect(custom.update(2000000000, 2.0, false, input), "custom reference load");
    near(custom.sample().pack_current_a, total_a + 0.5, 1e-10, "configured auxiliary current");
    near(custom.sample().soc, 0.8 - (total_a + 0.5) * 2.0 / (2.0 * 3600.0), 1e-12,
         "configured initial SOC and capacity");
    near(custom.sample().terminal_voltage_v,
         custom.sample().open_circuit_voltage_v - (total_a + 0.5) * 0.06, 1e-10,
         "configured pack resistance");
}
void simulation_time_lifecycle()
{
    arion::CoupledPowerModel model;
    const auto input = joints({4042.0, 4042.0, 4042.0, 4042.0});
    expect(model.update(10000000000LL, 10.0, false, input), "start at arbitrary sim time");
    near(model.sample().consumed_mah, 0.0, 0.0, "first observation does not backfill pre-start load");
    expect(model.update(10010000000LL, 0.01, false, input), "10 ms simulation step");
    const auto before_pause = model.battery_frame();
    const auto rpm_before_pause = model.sample().rpm;
    expect(!model.update(10010000000LL, 600.0, true, joints({0.0, 0.0, 0.0, 0.0})), "paused output suppressed");
    expect(model.battery_frame() == before_pause && model.sample().rpm == rpm_before_pause,
           "pause cannot discharge or change motor state despite wall-time-sized dt");
    expect(!model.update(10010000000LL, 600.0, false, input), "duplicate sim stamp cannot integrate");
    expect(model.battery_frame() == before_pause, "duplicate stamp preserves charge");
    expect(model.update(10020000000LL, 600.0, false, input), "sim clock caps untrusted dt");
    near(model.sample().consumed_mah, (4.0 * 1.44 + 0.5) * 0.02 / 3.6, 1e-12,
         "integrate simulation time, not wall time");
    expect(model.update(5000000, -10.0, false, input), "rewind clears model even with negative Gazebo dt");
    near(model.sample().soc, 0.95, 0.0, "rewind restores configured SOC");
    near(model.sample().consumed_mah, 0.0, 0.0, "rewind clears consumed charge");
    expect(model.update(15000000, 0.01, false, input), "integration resumes after rewind");
    expect(model.sample().consumed_mah > 0.0, "resumed load discharges battery");
    expect(!model.update(0, 0.0, true, input), "paused rewind has no publication");
    near(model.sample().consumed_mah, 0.0, 0.0, "paused rewind still resets internal charge");
    near(model.sample().soc, 0.95, 0.0, "paused rewind resets internal SOC");
}
void validity_and_bounds()
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const auto good = joints({4042.0, 4469.0, 4855.0, 5301.0});
    arion::CoupledPowerModel model;
    expect(model.update(0, 0.0, false, good), "valid baseline before malformed input");
    int64_t now = 0;
    for (double bad : {nan, inf, 201.0}) {
        auto invalid = good;
        invalid.joint_rad_s[3] = bad;
        const auto before = model.battery_frame();
        const auto rpm_before = model.sample().rpm;
        now += 10000000;
        expect(!model.update(now, 0.01, false, invalid), "reject nonfinite/out-of-envelope joint sample");
        expect(!model.sample().valid, "rejected sample must not be published");
        expect(model.battery_frame() == before && model.sample().rpm == rpm_before,
               "malformed fourth motor cannot partially commit load or RPM");
    }
    for (double bad_dt : {nan, inf, -0.01}) {
        now += 10000000;
        expect(!model.update(now, bad_dt, false, good), "reject malformed non-rewind dt");
    }
    expect(!model.update(-1, 0.01, false, good), "reject negative simulation stamp");
    arion::PowerMotorInput fallback;
    fallback.command_valid = true;
    for (double bad : {nan, inf, -1.0}) {
        fallback.command_rad_s[2] = bad;
        now += 10000000;
        expect(!model.update(now, 0.01, false, fallback), "reject invalid fallback command");
    }
    fallback.command_rad_s.fill(2000.0);
    now += 1000000000;
    expect(model.update(now, 1.0, false, fallback), "finite commands saturate at motor-model limit");
    near(model.sample().rpm[2], 1000.0 * 60.0 / (2.0 * std::acos(-1.0)), 1e-8,
         "fallback maxRotVelocity is 1000 rad/s");
    auto measured = good;
    measured.command_valid = true;
    measured.command_rad_s.fill(nan);
    now += 10000000;
    expect(model.update(now, 0.01, false, measured), "invalid unused commands do not hide valid joints");
    const auto consumed = model.sample().consumed_mah;
    now += 10000000;
    expect(!model.update(now, 0.01, false, arion::PowerMotorInput{}), "no source is unavailable, not zero load");
    near(model.sample().consumed_mah, consumed, 0.0, "unavailable interval holds charge");

    using Field = double arion::CoupledPowerConfig::*;
    const std::array<Field, 4> fields{&arion::CoupledPowerConfig::capacity_ah,
        &arion::CoupledPowerConfig::pack_resistance_ohm,
        &arion::CoupledPowerConfig::initial_soc, &arion::CoupledPowerConfig::auxiliary_current_a};
    for (Field field : fields) {
        for (double bad : {nan, inf, -1.0, 1000000.0}) {
            arion::CoupledPowerConfig cfg;
            cfg.*field = bad;
            bool rejected = false;
            try { const arion::CoupledPowerModel invalid(cfg); }
            catch (const std::invalid_argument &) { rejected = true; }
            expect(rejected, "reject invalid reference pack configuration");
        }
    }
    arion::CoupledPowerConfig cfg;
    cfg.capacity_ah = 0.0;
    bool rejected_zero = false;
    try { const arion::CoupledPowerModel invalid(cfg); }
    catch (const std::invalid_argument &) { rejected_zero = true; }
    expect(rejected_zero, "zero capacity cannot divide by zero");
    cfg.capacity_ah = 0.01;
    cfg.initial_soc = 0.2;
    cfg.pack_resistance_ohm = 1.0;
    cfg.auxiliary_current_a = 100.0;
    arion::CoupledPowerModel depleted(cfg);
    expect(depleted.update(0, 0.0, false, good), "bounded extreme reference starts");
    expect(depleted.update(std::numeric_limits<int64_t>::max(),
                          std::numeric_limits<double>::max(), false, good), "large simulated dt remains finite");
    near(depleted.sample().soc, 0.0, 0.0, "SOC cannot become negative");
    near(depleted.sample().consumed_mah, 0.01 * 1000.0 * 0.2, 1e-12, "counted discharge caps at initial charge");
    const auto bounded = depleted.battery_frame();
    arion::BatteryWireValues decoded;
    expect(arion::decode_battery_frame(bounded.data(), bounded.size(), decoded),
           "voltage floor and current/capacity limits remain wire-safe");
    depleted.reset();
    near(depleted.sample().soc, 0.2, 0.0, "explicit system reset preserves configuration");
    near(depleted.sample().consumed_mah, 0.0, 0.0, "explicit system reset clears discharge");
    expect(!depleted.sample().valid, "reset requires fresh rotor observations");
}
void publication_clocks()
{
    arion::PowerPublishClock motor(50000000);
    arion::BatteryPublishClock battery;
    int motor_count = 0;
    int battery_count = 0;
    for (int64_t ns = 0; ns < 1000000000; ns += 1000000) {
        motor_count += motor.due(ns, false) ? 1 : 0;
        battery_count += battery.due(ns, false) ? 1 : 0;
    }
    expect(motor_count == 20 && battery_count == 5, "20 Hz motor / 5 Hz battery in simulation time");
    expect(!motor.due(1000000000, true) && !battery.due(1000000000, true), "pause suppresses both streams");
    expect(motor.due(1000000000, false) && battery.due(1000000000, false), "resume publishes latest, not backlog");
    expect(!motor.due(1000000000, false) && !battery.due(1000000000, false), "same-stamp publication suppressed");
    expect(motor.due(0, false) && battery.due(0, false), "rewind resets both schedules");
    expect(!motor.due(-1, false), "negative publication stamp rejected");
    motor = arion::PowerPublishClock(50000000);
    battery = arion::BatteryPublishClock{};
    expect(motor.due(0, false) && battery.due(0, false), "explicit system reset restores immediate publication");
}
}  // namespace

int main()
{
    try {
        joint_response();
        std::cout << "PASS joint_response\n";
        modeled_fallback_response();
        std::cout << "PASS modeled_fallback_response\n";
        battery_load_and_wire_contract();
        std::cout << "PASS battery_load_and_wire_contract\n";
        simulation_time_lifecycle();
        std::cout << "PASS simulation_time_lifecycle\n";
        validity_and_bounds();
        std::cout << "PASS validity_and_bounds\n";
        publication_clocks();
        std::cout << "PASS publication_clocks\n";
        std::cout << "PASS 6 focused model tests\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
