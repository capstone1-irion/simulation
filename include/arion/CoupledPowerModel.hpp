#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace arion {

enum class PowerRpmSource { Unavailable, JointVelocity, ModeledCommand };

struct BatteryProfile {
    std::string id;
    double capacity_ah{5.0};
    double pack_resistance_ohm{0.04};
    double initial_soc{0.95};
    std::vector<double> ocv_soc;
    std::vector<double> ocv_cell_v;
    unsigned series_cells{4};
};

// 프로파일 ID를 길이 1~64의 ASCII 영숫자/밑줄/하이픈/점으로 한정한다(상태 변경 없음).
inline bool valid_power_profile_id(const std::string &id)
{
    if (id.empty() || id.size() > 64) { return false; }
    for (unsigned char c : id) {
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.')) { return false; }
    }
    return true;
}

struct CoupledPowerConfig {
    double capacity_ah{5.0};
    double pack_resistance_ohm{0.04};
    double initial_soc{0.95};
    double auxiliary_current_a{0.5};
    std::vector<BatteryProfile> profiles{};
    std::string initial_profile{};
};

struct PowerMotorInput {
    // Signed, slowed simulation joint rad/s, NOT rotor commands or RPM.
    std::array<std::optional<double>, 4> joint_rad_s{};
    // Unslowed nonnegative rotor rad/s; optional modeled fallback only.
    std::array<double, 4> command_rad_s{};
    bool command_valid{false};
};

struct CoupledPowerSample {
    std::array<double, 4> rpm{};
    std::array<double, 4> observed_rpm{};  // 전류 lookup은 보고용 이상 배율을 적용하기 전 RPM 사용.
    std::array<double, 4> motor_current_a{};
    std::array<PowerRpmSource, 4> rpm_source{};
    double pack_current_a{0.0};
    double consumed_mah{0.0};
    double soc{0.0};
    double open_circuit_voltage_v{0.0};
    double terminal_voltage_v{0.0};
    bool valid{false};
};

// Pure telemetry model: no actuator/force output and no wall-clock dependency.
class CoupledPowerModel {
public:
    // 용량(Ah), 팩 저항(Ω), SOC와 OCV 곡선을 받아 초기 4S 상태를 구성한다.
    explicit CoupledPowerModel(const CoupledPowerConfig &config = {}) : config_(config)
    {
        if (!within(config_.capacity_ah, 0.01, 1000.0) ||
            !within(config_.pack_resistance_ohm, 0.0, 1.0) ||
            !within(config_.initial_soc, 0.0, 1.0) ||
            !within(config_.auxiliary_current_a, 0.0, 100.0)) {
            throw std::invalid_argument("invalid coupled-power reference pack configuration");
        }
        const bool legacy = config_.profiles.empty();
        if (legacy) {
            BatteryProfile assumed;
            assumed.id = "assumed_linear_4s";
            assumed.capacity_ah = config_.capacity_ah;
            assumed.pack_resistance_ohm = config_.pack_resistance_ohm;
            assumed.initial_soc = config_.initial_soc;
            assumed.ocv_soc = {0.0, 1.0};
            assumed.ocv_cell_v = {3.3, 4.2};
            config_.profiles.push_back(assumed);
        }
        config_.initial_profile = config_.initial_profile.empty() ?
            config_.profiles.front().id : config_.initial_profile;
        std::set<std::string> ids;
        for (std::size_t i = 0; i < config_.profiles.size(); ++i) {
            validate_profile(config_.profiles[i], legacy);
            if (!ids.insert(config_.profiles[i].id).second) {
                throw std::invalid_argument("duplicate battery profile ID");
            }
            if (config_.profiles[i].id == config_.initial_profile) { profile_index_ = i; }
        }
        if (!ids.count(config_.initial_profile)) { throw std::invalid_argument("unknown initial profile"); }
        update_battery(sample_, 0.0);
    }

    // 현재 선택된 프로파일 ID를 읽는다. 충전량이나 상태를 변경하지 않는다.
    const std::string &profile_id() const { return config_.profiles[profile_index_].id; }

    // 불변 로드 목록을 읽는다. 콜백은 모델 대신 이 ID 목록의 복사본을 사용한다.
    const std::vector<BatteryProfile> &profiles() const { return config_.profiles; }

    // 초기 설정 프로파일 ID를 읽는다(현재 선택이나 충전량 변경 없음).
    const std::string &initial_profile_id() const { return config_.initial_profile; }

    // 전류 배율(무차원) 네 개를 읽는다. 구동부에는 적용되지 않는다.
    const std::array<double, 4> &current_scales() const { return current_scales_; }

    // 보고 RPM 배율(무차원) 네 개를 읽는다. 실제 joint 관측은 보존된다.
    const std::array<double, 4> &rpm_scales() const { return rpm_scales_; }

    // 초기화 후 발생한 reset 횟수(세대 번호)를 읽는다(상태 변경 없음).
    uint64_t reset_count() const { return reset_count_; }

    // 슬롯 0..3의 전류 0..3/RPM 0..1 배율을 설정한다. 힘·joint·명령은 변경하지 않는다.
    bool set_motor_fault(std::size_t index, double current_scale, double rpm_scale)
    {
        if (index >= 4 || !within(current_scale, 0.0, 3.0) || !within(rpm_scale, 0.0, 1.0)) {
            return false;
        }
        current_scales_[index] = current_scale;
        rpm_scales_[index] = rpm_scale;
        sample_.valid = false;
        return true;
    }

    // 모든 이상 배율을 1로 복구한다. 소비량/SOC는 그대로 유지한다.
    void clear_faults()
    {
        current_scales_.fill(1.0);
        rpm_scales_.fill(1.0);
        sample_.valid = false;
    }

    // 최초 설정 프로파일로 돌아가 초기 SOC/소비량/모터 이상을 재설정한다.
    void restore() { select_profile(config_.initial_profile); }

    // 미리 로드한 ID를 선택하고 초기 SOC/소비량으로 재설정한다. 미등록 ID는 거부한다.
    bool select_profile(const std::string &id)
    {
        for (std::size_t i = 0; i < config_.profiles.size(); ++i) {
            if (config_.profiles[i].id == id) {
                profile_index_ = i;
                reset();
                return true;
            }
        }
        return false;
    }

    // 선택 프로파일의 초기 SOC로 복귀하고 누적 mAh와 관측 시각을 지운다.
    void reset()
    {
        clear_faults();
        ++reset_count_;
        sample_ = CoupledPowerSample{};
        initialized_ = false;
        previous_ns_ = 0;
        update_battery(sample_, 0.0);
    }

    // 시뮬레이션 ns/초와 joint rad/s로 관측·전류·SOC를 갱신한다. pause에는 적분하지 않는다.
    bool update(int64_t now_ns, double dt_s, bool paused,
                const PowerMotorInput &input)
    {
        if (now_ns < 0) { sample_.valid = false; return false; }
        const bool rewound = initialized_ && now_ns < previous_ns_;
        if (rewound) { reset(); }
        const bool repeated = initialized_ && now_ns == previous_ns_;
        const bool invalid_dt = !std::isfinite(dt_s) || (dt_s < 0.0 && !rewound);
        // First observation / reset establishes an origin, never backfills load.
        dt_s = initialized_ ? std::min(dt_s, static_cast<double>(now_ns - previous_ns_) * 1e-9) : 0.0;
        previous_ns_ = now_ns;
        initialized_ = true;
        if (paused || repeated) { return false; }
        if (invalid_dt) { sample_.valid = false; return false; }
        CoupledPowerSample next = sample_;
        for (std::size_t i = 0; i < 4; ++i) {
            if (input.joint_rad_s[i]) {
                // Signed rotation is legitimate; nonfinite/absurd observations are not.
                // Envelope allows twice the 1000 rad/s stock motor command limit.
                if (!within(*input.joint_rad_s[i], -200.0, 200.0)) {
                    sample_.valid = false;
                    return false;
                }
                next.observed_rpm[i] = std::abs(*input.joint_rad_s[i]) * 10.0 * kRpmPerRadS;
                next.rpm_source[i] = PowerRpmSource::JointVelocity;
            } else if (input.command_valid) {
                if (!std::isfinite(input.command_rad_s[i]) || input.command_rad_s[i] < 0.0) {
                    sample_.valid = false;
                    return false;
                }
                const double target = std::clamp(input.command_rad_s[i], 0.0, 1000.0) * kRpmPerRadS;
                const double tau = target > next.observed_rpm[i] ? 0.0125 : 0.025;
                next.observed_rpm[i] += (target - next.observed_rpm[i]) * -std::expm1(-dt_s / tau);
                next.rpm_source[i] = PowerRpmSource::ModeledCommand;
            } else {
                sample_.valid = false;
                return false;
            }
            next.rpm[i] = next.observed_rpm[i] * rpm_scales_[i];
            next.motor_current_a[i] = current_for_rpm(next.observed_rpm[i]) * current_scales_[i];
        }
        update_battery(next, dt_s);
        next.valid = true;
        sample_ = next;
        return true;
    }

    // 마지막 모사 샘플을 읽는다. valid=false인 샘플은 발행하면 안 된다.
    const CoupledPowerSample &sample() const { return sample_; }

    // RPM/V/A 순서의 기존 12-float wire 값을 만든다(상태 변경 없음).
    std::array<float, 12> motor_frame() const
    {
        std::array<float, 12> result{};
        for (std::size_t i = 0; i < 4; ++i) {
            result[i] = static_cast<float>(sample_.rpm[i]);
            result[i + 4] = static_cast<float>(sample_.terminal_voltage_v);
            result[i + 8] = static_cast<float>(sample_.motor_current_a[i]);
        }
        return result;
    }

    // 4개 동일 셀 V, 팩 A, SOC, 고정 25°C, 소비 mAh를 기존 wire 순서로 만든다.
    std::array<float, 9> battery_frame() const
    {
        const float cell_v = static_cast<float>(sample_.terminal_voltage_v / 4.0);
        return {0.0f, cell_v, cell_v, cell_v, cell_v,
                static_cast<float>(sample_.pack_current_a), static_cast<float>(sample_.soc),
                25.0f, static_cast<float>(sample_.consumed_mah)};
    }

private:
    // 4S 프로파일의 Ah/Ω/SOC/셀 V 배열을 검사한다. 레거시 호환 시에만 0Ω를 허용한다.
    static void validate_profile(const BatteryProfile &profile, bool legacy)
    {
        if (!valid_power_profile_id(profile.id) || profile.series_cells != 4 ||
            !std::isfinite(profile.capacity_ah) || profile.capacity_ah <= 0.0 ||
            !std::isfinite(profile.capacity_ah * 1000.0) ||
            !std::isfinite(profile.pack_resistance_ohm) || profile.pack_resistance_ohm < 0.0 ||
            (!legacy && profile.pack_resistance_ohm == 0.0) ||
            !within(profile.initial_soc, 0.0, 1.0) || profile.ocv_soc.size() < 2 ||
            profile.ocv_soc.size() != profile.ocv_cell_v.size() ||
            profile.ocv_soc.front() != 0.0 || profile.ocv_soc.back() != 1.0) {
            throw std::invalid_argument("invalid 4S battery profile configuration");
        }
        for (std::size_t i = 0; i < profile.ocv_soc.size(); ++i) {
            if (!within(profile.ocv_soc[i], 0.0, 1.0) ||
                (i && profile.ocv_soc[i] <= profile.ocv_soc[i - 1]) ||
                !std::isfinite(profile.ocv_cell_v[i]) || profile.ocv_cell_v[i] <= 0.0 ||
                !std::isfinite(profile.ocv_cell_v[i] * 4.0)) {
                throw std::invalid_argument("invalid battery OCV curve");
            }
        }
    }

    // 단위가 동일한 범위와 비교해 유한 수치인지 확인한다(상태 변경 없음).
    static bool within(double value, double minimum, double maximum)
    {
        return std::isfinite(value) && value >= minimum && value <= maximum;
    }

    // 합산 전류(A)와 경과 초로 mAh/SOC를 적분하고 셀 OCV를 4S 팩 전압(V)으로 환산한다.
    void update_battery(CoupledPowerSample &sample, double dt_s) const
    {
        const auto &profile = config_.profiles[profile_index_];
        sample.pack_current_a = config_.auxiliary_current_a;
        for (double amps : sample.motor_current_a) { sample.pack_current_a += amps; }
        const double capacity_mah = profile.capacity_ah * 1000.0;
        sample.consumed_mah = std::min(profile.initial_soc * capacity_mah,
                                      sample.consumed_mah + sample.pack_current_a * dt_s / 3.6);
        sample.soc = std::clamp(profile.initial_soc - sample.consumed_mah / capacity_mah, 0.0, 1.0);
        const auto upper = std::upper_bound(profile.ocv_soc.begin(), profile.ocv_soc.end(), sample.soc);
        const auto high = std::min<std::size_t>(profile.ocv_soc.size() - 1,
            static_cast<std::size_t>(upper - profile.ocv_soc.begin()));
        const auto low = high - 1;
        const double fraction = (sample.soc - profile.ocv_soc[low]) /
                                (profile.ocv_soc[high] - profile.ocv_soc[low]);
        sample.open_circuit_voltage_v = 4.0 * (profile.ocv_cell_v[low] + fraction *
                                              (profile.ocv_cell_v[high] - profile.ocv_cell_v[low]));
        // Only a wire-safe 1 mV/cell floor, NOT an electrochemical cutoff model.
        sample.terminal_voltage_v = std::max(0.004, sample.open_circuit_voltage_v -
                                            sample.pack_current_a * profile.pack_resistance_ohm);
    }

    static constexpr double kRpmPerRadS = 60.0 / 6.28318530717958647692;
    // 실제 관측 RPM으로 16V 벤치 전류(A)를 보간하며 표 밖은 명시된 가정/상한을 쓴다.
    static double current_for_rpm(double rpm)
    {
        // User's AIR2216II-KV920 + T1045II bench table at 16 V.
        constexpr std::array<double, 15> speeds{
            4042, 4469, 4855, 5301, 5780, 6298, 6800, 7281,
            7679, 8096, 8468, 8867, 9257, 9675, 9857};
        constexpr std::array<double, 15> amps{
            1.44, 1.87, 2.29, 2.86, 3.60, 4.53, 5.61, 6.78,
            7.92, 9.20, 10.59, 12.11, 13.81, 15.68, 16.37};
        // Below the table: assumed quadratic, not a calibrated idle curve.
        if (rpm < speeds.front()) {
            const double fraction = rpm / speeds.front();
            return amps.front() * fraction * fraction;
        }
        for (std::size_t i = 1; i < speeds.size(); ++i) {
            if (rpm <= speeds[i]) {
                const double fraction = (rpm - speeds[i - 1]) / (speeds[i] - speeds[i - 1]);
                return amps[i - 1] + fraction * (amps[i] - amps[i - 1]);
            }
        }
        return amps.back();  // No bench evidence for higher-RPM extrapolation.
    }

    CoupledPowerConfig config_;
    std::size_t profile_index_{0};
    std::array<double, 4> current_scales_{1.0, 1.0, 1.0, 1.0};
    std::array<double, 4> rpm_scales_{1.0, 1.0, 1.0, 1.0};
    uint64_t reset_count_{0};
    CoupledPowerSample sample_;
    bool initialized_{false};
    int64_t previous_ns_{0};
};

// Same no-catch-up policy as BatteryPublishClock, with a 20 Hz motor period.
class PowerPublishClock {
public:
    // 양의 ns 주기를 저장한다. 벽시계 작업이나 외부 발행은 시작하지 않는다.
    explicit PowerPublishClock(int64_t period_ns) : period_ns_(period_ns)
    {
        if (period_ns <= 0) { throw std::invalid_argument("publication period must be positive"); }
    }

    // 시뮬레이션 ns 기준 발행 여부를 계산하고 마지막 발행 시각만 갱신한다.
    bool due(int64_t now_ns, bool paused)
    {
        if (paused || now_ns < 0) { return false; }
        const bool rewound = initialized_ && now_ns < previous_ns_;
        previous_ns_ = now_ns;
        if (initialized_ && !rewound && now_ns - published_ns_ < period_ns_) { return false; }
        initialized_ = true;
        published_ns_ = now_ns;
        return true;
    }

private:
    int64_t period_ns_;
    bool initialized_{false};
    int64_t previous_ns_{0};
    int64_t published_ns_{0};
};

}  // namespace arion
