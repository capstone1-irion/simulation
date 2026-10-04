#include "arion/BatteryFrame.hpp"
#include "arion/BatteryPublishClock.hpp"
#include "arion/CoupledPowerModel.hpp"
#include "arion/PowerControl.hpp"

#include <gz/common/Console.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/float_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/transport/Node.hh>

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

namespace arion {
namespace {
struct CommandMailbox {
    std::mutex mutex;
    std::array<double, 4> rad_s{};
    uint64_t revision{0};
    bool valid{false};
    bool active{true};
};

// SDF 값이 있으면 형식 검사를 거쳐 읽고, 없으면 명시 기본값을 반환한다(상태 변경 없음).
template<typename T>
T parameter(const std::shared_ptr<const sdf::Element> &sdf,
            const std::string &name, const T &fallback)
{
    if (!sdf || !sdf->HasElement(name)) { return fallback; }
    const auto value = sdf->Get<T>(name, fallback);
    if (!value.second) { throw std::invalid_argument("invalid SDF parameter: " + name); }
    return value.first;
}

// 발행 메시지 헤더에 단위/출처를 설명할 key/value 항목 하나를 추가한다.
void metadata(gz::msgs::Header &header, const std::string &key, const std::string &value)
{
    auto *entry = header.add_data();
    entry->set_key(key);
    entry->add_value(value);
}
}  // namespace

// SDF의 공백 분리 유한 수치 배열을 읽는다. 필수 필드 누락/비수치/과대 배열은 거부한다.
static std::vector<double> profile_curve(const std::shared_ptr<const sdf::Element> &element,
                                         const std::string &name)
{
    if (!element->HasElement(name)) { throw std::invalid_argument("missing profile curve: " + name); }
    std::istringstream input(parameter(element, name, std::string{}));
    input.imbue(std::locale::classic());
    std::string token;
    std::vector<double> values;
    while (input >> token) {
        double value = 0.0;
        if (!parse_power_number(token, value) || values.size() >= 4096) {
            throw std::invalid_argument("invalid profile curve: " + name);
        }
        values.push_back(value);
    }
    return values;
}

// SDF의 4S 프로파일 Ah/Ω/SOC/셀 V를 읽는다. 명시 catalog가 불완전하면 레거시로 대체하지 않는다.
CoupledPowerConfig power_config_from_sdf(const std::shared_ptr<const sdf::Element> &element)
{
    CoupledPowerConfig config;
    config.capacity_ah = parameter(element, "capacity_ah", config.capacity_ah);
    config.pack_resistance_ohm = parameter(element, "pack_resistance_ohm", config.pack_resistance_ohm);
    config.initial_soc = parameter(element, "initial_soc", config.initial_soc);
    config.auxiliary_current_a = parameter(element, "auxiliary_current_a", config.auxiliary_current_a);
    config.initial_profile = parameter(element, "initial_profile", std::string{});
    if (!element || !element->HasElement("battery_profiles")) { return config; }
    const auto catalog = element->FindElement("battery_profiles");
    if (!catalog->HasElement("profile") || config.initial_profile.empty()) {
        throw std::invalid_argument("battery_profiles requires records and initial_profile");
    }
    for (auto entry = catalog->FindElement("profile"); entry; entry = entry->GetNextElement("profile")) {
        if (config.profiles.size() >= 64 || !entry->HasAttribute("id") ||
            !entry->HasElement("capacity_ah") || !entry->HasElement("pack_resistance_ohm") ||
            !entry->HasElement("initial_soc")) { throw std::invalid_argument("incomplete battery profile"); }
        BatteryProfile profile;
        if (!entry->GetAttribute("id")->Get(profile.id)) { throw std::invalid_argument("invalid profile ID"); }
        profile.capacity_ah = parameter<double>(entry, "capacity_ah", 0.0);
        profile.pack_resistance_ohm = parameter<double>(entry, "pack_resistance_ohm", 0.0);
        profile.initial_soc = parameter<double>(entry, "initial_soc", -1.0);
        profile.series_cells = parameter<unsigned>(entry, "series_cells", 4);
        profile.ocv_soc = profile_curve(entry, "ocv_soc");
        profile.ocv_cell_v = profile_curve(entry, "ocv_cell_v");
        config.profiles.push_back(std::move(profile));
    }
    // 순수 모델의 단일 validator로 ID 중복/4S/곡선/팩 수치를 모두 검사한다.
    const CoupledPowerModel validated(config);
    return config;
}

// StringMsg 요청을 공유 mailbox로 전달하는 콜백을 만든다. this/모델 참조를 캡처하지 않는다.
std::function<bool(const gz::msgs::StringMsg &, gz::msgs::StringMsg &)>
power_control_callback(std::shared_ptr<PowerControlMailbox> mailbox)
{
    // 비동기 요청을 큐에 넣거나 상태 JSON을 읽는다. 시뮬레이션 상태는 변경하지 않는다.
    return [mailbox = std::move(mailbox)](const gz::msgs::StringMsg &request, gz::msgs::StringMsg &reply) {
        try { reply.set_data(mailbox->handle(request.data())); }
        catch (const std::exception &) { reply.set_data(power_json_error("power control internal error")); }
        return true;
    };
}

// Observes propulsion only. Never writes forces, JointVelocityCmd, Actuators,
// motor commands, PX4 parameters, or any part of the flight-control path.
class CoupledPowerState final : public gz::sim::System,
                                public gz::sim::ISystemConfigure,
                                public gz::sim::ISystemPreUpdate,
                                public gz::sim::ISystemPostUpdate,
                                public gz::sim::ISystemReset {
public:
    // 종료 시 공유 mailbox를 닫는다. 이미 복사된 콜백은 owner 해제 후에도 닫힌 객체만 접근한다.
    ~CoupledPowerState() override
    {
        if (controls_) { controls_->close(); }
        std::lock_guard<std::mutex> lock(commands_->mutex);
        commands_->active = false;
    }

    // 모델/SDF를 검증한 뒤 관측 발행원과 독립 StringMsg 제어 서비스를 등록한다.
    void Configure(const gz::sim::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   gz::sim::EntityComponentManager &ecm,
                   gz::sim::EventManager &) override
    {
        const gz::sim::Model model(entity);
        if (!model.Valid(ecm)) {
            gzerr << "CoupledPowerState must be attached to a model\n";
            return;
        }
        try {
            const CoupledPowerConfig config = power_config_from_sdf(sdf);
            if (parameter(sdf, "rotorVelocitySlowdownSim", 10.0) != 10.0) {
                throw std::invalid_argument("reference model requires rotorVelocitySlowdownSim=10");
            }
            allow_command_fallback_ = parameter(sdf, "allow_command_fallback", false);
            power_ = CoupledPowerModel(config);  // Validates before advertising anything.
            controls_ = std::make_shared<PowerControlMailbox>(power_);
        } catch (const std::exception &error) {
            gzerr << "CoupledPowerState disabled: " << error.what() << '\n';
            return;
        }
        model_entity_ = entity;
        const std::string model_name = model.Name(ecm);
        const std::string prefix = "/arion/" + model_name;
        motor_publisher_ = node_.Advertise<gz::msgs::Float_V>(prefix + "/motor_state");
        battery_publisher_ = node_.Advertise<gz::msgs::Float_V>(prefix + "/battery_state");
        if (!motor_publisher_.Valid() || !battery_publisher_.Valid()) {
            gzerr << "CoupledPowerState cannot advertise both state topics\n";
            return;
        }
        if (allow_command_fallback_) {
            const std::string topic = "/" + model_name + "/command/motor_speed";
            // Snapshot dispatch can outlive this plugin: capture ONLY shared mailbox.
            const auto mailbox = commands_;
            if (!node_.Subscribe<gz::msgs::Actuators>(topic,
                // 옵션 명령 rad/s만 공유 mailbox에 복사한다. model/this는 참조하지 않는다.
                [mailbox](const gz::msgs::Actuators &message) {
                    std::lock_guard<std::mutex> lock(mailbox->mutex);
                    if (!mailbox->active) { return; }
                    ++mailbox->revision;
                    mailbox->valid = message.velocity_size() == 4;
                    if (!mailbox->valid) { return; }
                    for (std::size_t i = 0; i < 4; ++i) {
                        mailbox->rad_s[i] = message.velocity(static_cast<int>(i));
                        if (!std::isfinite(mailbox->rad_s[i]) || mailbox->rad_s[i] < 0.0) {
                            mailbox->valid = false;
                        }
                    }
                })) {
                gzerr << "CoupledPowerState cannot subscribe to fallback commands\n";
                return;
            }
        }
        if (!node_.Advertise<gz::msgs::StringMsg, gz::msgs::StringMsg>(
                prefix + "/power_control", power_control_callback(controls_))) {
            controls_->close();
            gzerr << "CoupledPowerState cannot advertise power control service\n";
            return;
        }
        configured_ = true;
        gzmsg << "CoupledPowerState: observing rotor_0_joint..rotor_3_joint; "
              << "4S OCV/ohmic model, one-way load coupling, telemetry-only faults; profile="
              << power_.profile_id() << '\n';
    }

    // joint 관측 component만 요청한다. 구동 속도/힘/명령 및 전력 상태는 변경하지 않는다.
    void PreUpdate(const gz::sim::UpdateInfo &,
                   gz::sim::EntityComponentManager &ecm) override
    {
        if (!configured_) { return; }
        const gz::sim::Model model(model_entity_);
        for (std::size_t i = 0; i < joints_.size(); ++i) {
            if (joints_[i] == gz::sim::kNullEntity || !gz::sim::Joint(joints_[i]).Valid(ecm)) {
                joints_[i] = model.JointByName(ecm, "rotor_" + std::to_string(i) + "_joint");
            }
            if (joints_[i] != gz::sim::kNullEntity) {
                // Requests the observation component only; does not set velocity.
                gz::sim::Joint(joints_[i]).EnableVelocityCheck(ecm);
            }
        }
    }

    // 유일한 전력 상태 변경 경로: 제어 적용→joint 관측→sim-time 적분→상태 복사/발행 순서다.
    void PostUpdate(const gz::sim::UpdateInfo &info,
                    const gz::sim::EntityComponentManager &ecm) override
    {
        if (!configured_) { return; }
        const auto now_ns = sim_ns(info);
        if (previous_post_ns_ && now_ns < *previous_post_ns_ && !controls_->snapshot().world_reset_pending) {
            controls_->request_world_reset();
        }
        if (controls_->apply_pending(power_, now_ns, info.paused)) { reset_runtime_caches(); }
        previous_post_ns_ = now_ns;
        PowerMotorInput input;
        for (std::size_t i = 0; i < joints_.size(); ++i) {
            const auto *velocity = ecm.Component<gz::sim::components::JointVelocity>(joints_[i]);
            if (!velocity || velocity->Data().empty()) { continue; }
            // A rotor is a single-DOF joint. Invalid observations must not silently
            // turn into a command estimate; the pure model rejects this NaN.
            input.joint_rad_s[i] = velocity->Data().size() == 1 ? velocity->Data()[0] :
                                  std::numeric_limits<double>::quiet_NaN();
        }
        if (allow_command_fallback_) {
            std::lock_guard<std::mutex> lock(commands_->mutex);
            if (commands_->revision != command_revision_) {
                command_revision_ = commands_->revision;
                command_seen_ns_ = now_ns;
            }
            // Arrival freshness in SIMULATION time, not a claim of sensor freshness.
            input.command_valid = commands_->valid && command_seen_ns_ && now_ns >= *command_seen_ns_ &&
                                  now_ns - *command_seen_ns_ <= kCommandTimeoutNs;
            input.command_rad_s = commands_->rad_s;
        }
        const double dt_s = std::chrono::duration<double>(info.dt).count();
        const bool updated = power_.update(now_ns, dt_s, info.paused, input);
        controls_->refresh(power_, now_ns, info.paused, updated);
        if (!updated) {
            if (!info.paused && !power_.sample().valid && !warned_unavailable_) {
                gzwarn << "CoupledPowerState: no valid four-rotor sample; state publication held\n";
                warned_unavailable_ = true;
            }
            return;
        }
        warned_unavailable_ = false;
        if (motor_clock_.due(now_ns, info.paused)) {
            publish(motor_publisher_, power_.motor_frame(), "arion.motor_state.v1", now_ns);
        }
        if (battery_clock_.due(now_ns, info.paused)) {
            publish(battery_publisher_, power_.battery_frame(), kBatterySchema, now_ns);
        }
    }

    // world Reset은 mailbox 경계만 기록한다. 선택 프로파일 보존/충전량 초기화는 PostUpdate에서 한다.
    void Reset(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &) override
    {
        if (configured_) { controls_->request_world_reset(); }
    }

private:
    // Gazebo simTime을 정수 ns로 변환한다(상태 변경 없음).
    static int64_t sim_ns(const gz::sim::UpdateInfo &info)
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    }

    // PostUpdate reset 직후 발행 시계와 fallback 신선도를 지운다. 프로파일/물리 구동부는 보존한다.
    void reset_runtime_caches()
    {
        motor_clock_ = PowerPublishClock(50000000);
        battery_clock_ = BatteryPublishClock{};
        command_seen_ns_.reset();
        command_revision_ = 0;
        warned_unavailable_ = false;
        std::lock_guard<std::mutex> lock(commands_->mutex);
        commands_->rad_s.fill(0.0);
        commands_->revision = 0;
        commands_->valid = false;
    }

    // wire 배열과 sim ns를 발행하고 출처/세대 metadata를 붙인다. 수치 이상은 텔레메트리 전용이다.
    template<std::size_t N>
    void publish(gz::transport::Node::Publisher &publisher,
                 const std::array<float, N> &values, const char *schema, int64_t now_ns)
    {
        gz::msgs::Float_V message;
        auto *header = message.mutable_header();
        header->mutable_stamp()->set_sec(now_ns / 1000000000);
        header->mutable_stamp()->set_nsec(static_cast<int32_t>(now_ns % 1000000000));
        metadata(*header, "schema", schema);
        metadata(*header, "power_model", "profile_4s_ocv_ohmic_v1");
        metadata(*header, "profile_id", power_.profile_id());
        metadata(*header, "generation", std::to_string(power_.reset_count()));
        metadata(*header, "reset_count", std::to_string(power_.reset_count()));
        metadata(*header, "applied_revision", std::to_string(controls_->snapshot().applied_revision));
        metadata(*header, "fault_scope", "telemetry_and_electrical_load_only_no_thrust_change");
        metadata(*header, "current_model", "16V_bench_piecewise_low_quadratic_high_capped");
        metadata(*header, "motor_voltage_source", "pack_terminal_model_no_speed_feedback");
        metadata(*header, "temperature_source", "constant_25C_not_thermal_model");
        metadata(*header, "voltage_floor_active", power_.sample().terminal_voltage_v <= 0.004 ? "true" : "false");
        metadata(*header, "charge_exhausted", power_.sample().soc <= 0.0 ? "true" : "false");
        auto *sources = header->add_data();
        sources->set_key("rpm_source");
        auto *regions = header->add_data();
        regions->set_key("current_curve_region");
        for (std::size_t i = 0; i < 4; ++i) {
            sources->add_value(power_.sample().rpm_source[i] == PowerRpmSource::JointVelocity ?
                               "joint_velocity" : "modeled_command_first_order");
            const double rpm = power_.sample().observed_rpm[i];
            regions->add_value(rpm < 4042.0 ? "assumed_low_rpm" :
                               rpm > 9857.0 ? "capped_above_bench" : "bench_interpolation_16V");
        }
        for (float value : values) { message.add_data(value); }
        if (!publisher.Publish(message)) { gzerr << "CoupledPowerState publication failed: " << schema << '\n'; }
    }

    static constexpr int64_t kCommandTimeoutNs = 500000000;
    CoupledPowerModel power_;
    PowerPublishClock motor_clock_{50000000};
    BatteryPublishClock battery_clock_;
    gz::sim::Entity model_entity_{gz::sim::kNullEntity};
    std::array<gz::sim::Entity, 4> joints_{};
    bool configured_{false};
    bool allow_command_fallback_{false};
    bool warned_unavailable_{false};
    std::optional<int64_t> previous_post_ns_;
    std::optional<int64_t> command_seen_ns_;
    uint64_t command_revision_{0};
    std::shared_ptr<CommandMailbox> commands_{std::make_shared<CommandMailbox>()};
    std::shared_ptr<PowerControlMailbox> controls_;
    gz::transport::Node node_;
    gz::transport::Node::Publisher motor_publisher_;
    gz::transport::Node::Publisher battery_publisher_;
};

}  // namespace arion

GZ_ADD_PLUGIN(arion::CoupledPowerState, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate,
              gz::sim::ISystemPostUpdate, gz::sim::ISystemReset)
