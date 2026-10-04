#include "arion/CoupledPowerModel.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
// 입력 조건과 메시지를 검사하며 실패 시 시험을 중단한다(제품 상태 변경 없음).
void expect(bool value, const std::string &message)
{
    if (!value) { throw std::runtime_error(message); }
}
// 같은 단위의 두 수치를 허용 오차 내에서 비교한다(제품 상태 변경 없음).
void near(double actual, double expected, double tolerance, const std::string &message)
{
    expect(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}
// 합성 단위시험용 4S 프로파일을 만든다. CMU 측정값을 대신하지 않는다.
arion::CoupledPowerConfig profiles()
{
    arion::CoupledPowerConfig config;
    arion::BatteryProfile first;
    first.id = "test_normal";
    first.capacity_ah = 5.0;
    first.pack_resistance_ohm = 0.04;
    first.initial_soc = 0.95;
    first.ocv_soc = {0.0, 0.5, 1.0};
    first.ocv_cell_v = {3.0, 3.6, 4.2};
    config.profiles.push_back(first);
    first.id = "test_aged";
    first.capacity_ah = 3.0;
    first.pack_resistance_ohm = 0.08;
    first.ocv_cell_v = {3.1, 3.5, 4.1};
    config.profiles.push_back(first);
    config.initial_profile = "test_normal";
    return config;
}
// 0.95 SOC의 셀 OCV 선형 보간과 4S 팩 합산을 검사한다.
void profile_interpolation()
{
    const arion::CoupledPowerConfig legacy{5.0, 0.04, 0.95, 0.5};
    const arion::CoupledPowerModel old_api(legacy);
    expect(old_api.profile_id() == "assumed_linear_4s", "legacy four-field aggregate API");
    arion::CoupledPowerModel model(profiles());
    expect(model.profile_id() == "test_normal", "initial profile selected");
    near(model.sample().open_circuit_voltage_v, 4.0 * 4.14, 1e-12, "4S interpolated OCV");
    near(model.sample().terminal_voltage_v, 4.0 * 4.14 - 0.5 * 0.04, 1e-12, "pack resistance sag");
    expect(model.select_profile("test_aged"), "loaded profile selectable");
    near(model.sample().open_circuit_voltage_v, 4.0 * 4.04, 1e-12, "selected curve interpolation");
    near(model.sample().terminal_voltage_v, 4.0 * 4.04 - 0.5 * 0.08, 1e-12, "selected pack resistance");
    expect(!model.select_profile("missing"), "unknown profile rejected");
    expect(model.profile_id() == "test_aged", "rejected selection preserves profile");
}
// 불완전/비유한 곡선, 잘못된 4S/ID/팩 설정을 초기화 전에 거부하는지 검사한다.
void profile_validation()
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (int mutation = 0; mutation < 14; ++mutation) {
        auto config = profiles();
        auto &p = config.profiles[0];
        switch (mutation) {
        case 0: p.capacity_ah = 0.0; break;
        case 1: p.pack_resistance_ohm = 0.0; break;
        case 2: p.capacity_ah = nan; break;
        case 3: p.pack_resistance_ohm = nan; break;
        case 4: p.ocv_soc = {0.0, 1.0, 0.5}; break;
        case 5: p.ocv_soc = {0.1, 0.5, 1.0}; break;
        case 6: p.ocv_cell_v.pop_back(); break;
        case 7: p.ocv_cell_v[1] = nan; break;
        case 8: p.ocv_cell_v[1] = 0.0; break;
        case 9: p.series_cells = 3; break;
        case 10: p.id = "unsafe\"id"; break;
        case 11: config.profiles[1].id = p.id; break;
        case 12: config.initial_profile = "missing"; break;
        case 13: p.initial_soc = 1.1; break;
        }
        bool rejected = false;
        try { const arion::CoupledPowerModel invalid(config); }
        catch (const std::invalid_argument &) { rejected = true; }
        expect(rejected, "invalid profile mutation " + std::to_string(mutation));
    }
}
}  // namespace

// 입력 RPM을 slowdown=10인 joint rad/s로 바꿔 합성 관측값을 만든다.
arion::PowerMotorInput joint_input(double rpm)
{
    arion::PowerMotorInput input;
    for (auto &joint : input.joint_rad_s) { joint = rpm * (2.0 * std::acos(-1.0)) / 600.0; }
    return input;
}

// 모터 한 슬롯의 전류/보고 RPM만 바꾸고 팩 부하 연동·해제·리셋을 검사한다.
void faults_and_reset()
{
    arion::CoupledPowerModel model(profiles());
    const auto input = joint_input(6800.0);
    expect(model.update(0, 0.0, false, input), "baseline observation");
    expect(model.set_motor_fault(2, 2.0, 0.25), "fault accepted");
    expect(!model.sample().valid, "new fault requires matching fresh sample before publication");
    expect(model.update(1000000000, 1.0, false, input), "faulted observation");
    near(model.sample().rpm[2], 1700.0, 1e-9, "reported RPM scaled");
    near(model.sample().observed_rpm[2], 6800.0, 1e-9, "joint RPM retained");
    near(model.sample().motor_current_a[2], 11.22, 1e-9, "lookup uses actual joint RPM");
    near(model.sample().motor_current_a[1], 5.61, 1e-9, "other slot unaffected");
    near(model.sample().pack_current_a, 5.0 * 5.61 + 0.5, 1e-9, "scaled current enters pack");
    const double charge = model.sample().consumed_mah;
    expect(charge > 0.0, "fault consumes charge");
    expect(!model.set_motor_fault(4, 1.0, 1.0), "index bound");
    expect(!model.set_motor_fault(0, 3.01, 1.0), "current bound");
    expect(!model.set_motor_fault(0, 1.0, -0.1), "rpm bound");
    expect(!model.set_motor_fault(0, std::numeric_limits<double>::quiet_NaN(), 1.0), "finite scale");
    model.clear_faults();
    expect(!model.sample().valid, "clear invalidates old faulted sample without recharging");
    near(model.sample().consumed_mah, charge, 0.0, "clear does not recharge");
    expect(model.update(2000000000, 1.0, false, input), "clear applied");
    near(model.sample().rpm[2], 6800.0, 1e-9, "RPM restored");
    expect(model.select_profile("test_aged"), "profile changed");
    expect(model.set_motor_fault(3, 3.0, 0.0), "boundary scales accepted");
    model.reset();
    expect(model.profile_id() == "test_aged", "reset preserves selected profile");
    near(model.sample().consumed_mah, 0.0, 0.0, "reset charge zero");
    near(model.sample().soc, 0.95, 0.0, "reset initial SOC");
    expect(!model.sample().valid && model.current_scales()[3] == 1.0 && model.rpm_scales()[3] == 1.0,
           "reset clears fault and freshness");
    model.restore();
    expect(model.profile_id() == "test_normal", "restore initial configured profile");
    expect(model.reset_count() == 3, "select/reset/restore tracked");
}

// 좁은 순수 모델 시험을 실행하며 Gazebo/PX4나 네트워크를 실행하지 않는다.
int main()
{
    try {
        profile_interpolation();
        std::cout << "PASS profile_interpolation\n";
        profile_validation();
        std::cout << "PASS profile_validation\n";
        faults_and_reset();
        std::cout << "PASS faults_and_reset\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
