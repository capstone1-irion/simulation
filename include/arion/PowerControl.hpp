#pragma once

#include "arion/CoupledPowerModel.hpp"

#include <charconv>
#include <deque>
#include <iomanip>
#include <limits>
#include <locale>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace arion {

enum class PowerControlKind { Status, Fault, Clear, Reset, Profile, Restore };

struct PowerControlCommand {
    PowerControlKind kind{PowerControlKind::Status};
    std::size_t motor_index{0};
    double current_scale{1.0};
    double rpm_scale{1.0};
    std::string profile_id;
    uint64_t revision{0};
};

// 임의 바이트 문자열을 JSON 문자열로 이스케이프한다. 제어/비ASCII 바이트도 안전하게 출력한다.
inline std::string power_json_string(const std::string &value)
{
    constexpr char hex[] = "0123456789abcdef";
    std::string output = "\"";
    for (unsigned char c : value) {
        if (c == '"') { output += "\\\""; }
        else if (c == '\\') { output += "\\\\"; }
        else if (c == '\n') { output += "\\n"; }
        else if (c == '\r') { output += "\\r"; }
        else if (c == '\t') { output += "\\t"; }
        else if (c < 0x20 || c >= 0x7f) {
            output += "\\u00";
            output += hex[c >> 4];
            output += hex[c & 15];
        } else { output += static_cast<char>(c); }
    }
    return output + '"';
}

// 오류 설명을 계약의 JSON 실패 응답으로 만든다(상태 변경 없음).
inline std::string power_json_error(const std::string &error)
{
    return "{\"ok\":false,\"error\":" + power_json_string(error) + '}';
}

// 전체 숫자 토큰을 로캘 독립 유한 double로 읽는다. 단위/범위는 호출자가 검사한다.
inline bool parse_power_number(const std::string &text, double &value)
{
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    return result.ec == std::errc{} && result.ptr == text.data() + text.size() && std::isfinite(value);
}

// 최대 256바이트의 공백 분리 명령을 검증한다. 성공할 때만 출력 명령을 교체한다.
inline bool parse_power_command(const std::string &text, PowerControlCommand &command,
                                std::string &error)
{
    error = "invalid power control command";
    if (text.empty() || text.size() > 256) { return false; }
    for (unsigned char c : text) {
        if ((c < 32 && c != '\t' && c != '\n' && c != '\r' && c != '\f' && c != '\v') || c >= 127) {
            return false;
        }
    }
    std::istringstream input(text);
    input.imbue(std::locale::classic());
    std::vector<std::string> tokens;
    std::string token;
    while (input >> token) { tokens.push_back(token); }
    if (tokens.empty()) { return false; }
    PowerControlCommand next;
    if (tokens.size() == 1) {
        if (tokens[0] == "status") { next.kind = PowerControlKind::Status; }
        else if (tokens[0] == "clear") { next.kind = PowerControlKind::Clear; }
        else if (tokens[0] == "reset") { next.kind = PowerControlKind::Reset; }
        else if (tokens[0] == "restore") { next.kind = PowerControlKind::Restore; }
        else { return false; }
    } else if (tokens.size() == 2 && tokens[0] == "profile" && valid_power_profile_id(tokens[1])) {
        next.kind = PowerControlKind::Profile;
        next.profile_id = tokens[1];
    } else if (tokens.size() == 4 && tokens[0] == "fault") {
        if (tokens[1].size() != 1 || tokens[1][0] < '0' || tokens[1][0] > '3' ||
            !parse_power_number(tokens[2], next.current_scale) ||
            !parse_power_number(tokens[3], next.rpm_scale) ||
            next.current_scale < 0.0 || next.current_scale > 3.0 ||
            next.rpm_scale < 0.0 || next.rpm_scale > 1.0) { return false; }
        next.kind = PowerControlKind::Fault;
        next.motor_index = static_cast<std::size_t>(tokens[1][0] - '0');
    } else { return false; }
    command = std::move(next);
    error.clear();
    return true;
}

struct PowerControlStatus {
    uint64_t accepted_revision{0};
    uint64_t applied_revision{0};
    uint64_t cancelled_through_revision{0};
    uint64_t last_cancelled_from_revision{0};
    std::string profile_id;
    std::string initial_profile_id;
    uint64_t reset_count{0};
    int64_t sim_ns{0};
    int64_t last_sample_ns{-1};
    std::array<double, 4> motor_current_scales{1, 1, 1, 1};
    std::array<double, 4> motor_rpm_scales{1, 1, 1, 1};
    double soc{0.0};
    double consumed_mah{0.0};
    std::size_t pending_commands{0};
    bool paused{true};
    bool sample_valid{false};
    bool world_reset_pending{false};
};

// 읽기 전용 상태 복사본을 로캘 독립 JSON으로 만든다. 모터 배율은 ground truth 제어 정보다.
inline std::string power_status_json(const PowerControlStatus &state)
{
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(17) << std::boolalpha
        << "{\"ok\":true,\"applied_revision\":" << state.applied_revision
        << ",\"accepted_revision\":" << state.accepted_revision
        << ",\"profile_id\":" << power_json_string(state.profile_id)
        << ",\"initial_profile_id\":" << power_json_string(state.initial_profile_id)
        << ",\"reset_count\":" << state.reset_count << ",\"generation\":" << state.reset_count
        << ",\"sim_ns\":" << state.sim_ns << ",\"last_sample_ns\":" << state.last_sample_ns
        << ",\"motor_current_scales\":[";
    for (std::size_t i = 0; i < 4; ++i) { if (i) { out << ','; } out << state.motor_current_scales[i]; }
    out << "],\"motor_rpm_scales\":[";
    for (std::size_t i = 0; i < 4; ++i) { if (i) { out << ','; } out << state.motor_rpm_scales[i]; }
    out << "],\"telemetry_only\":true,\"paused\":" << state.paused
        << ",\"sample_valid\":" << state.sample_valid << ",\"soc\":" << state.soc
        << ",\"consumed_mah\":" << state.consumed_mah
        << ",\"pending_commands\":" << state.pending_commands
        << ",\"world_reset_pending\":" << state.world_reset_pending
        << ",\"cancelled_through_revision\":" << state.cancelled_through_revision
        << ",\"last_cancelled_from_revision\":" << state.last_cancelled_from_revision << '}';
    return out.str();
}

// 콜백은 이 독립 공유 객체만 소유한다. 모델 변경 메서드는 PostUpdate 스레드만 호출한다.
class PowerControlMailbox {
public:
    // 검증된 모델에서 불변 ID 목록과 초기 상태를 복사하며 모델 참조를 보관하지 않는다.
    explicit PowerControlMailbox(const CoupledPowerModel &model)
    {
        for (const auto &profile : model.profiles()) { profile_ids_.insert(profile.id); }
        refresh_locked(model, 0, true, false);
    }

    // 콜백에서 명령을 검증/큐잉하거나 마지막 상태를 읽는다. 모델은 절대 변경하지 않는다.
    std::string handle(const std::string &text)
    {
        PowerControlCommand command;
        std::string error;
        if (!parse_power_command(text, command, error)) { return power_json_error(error); }
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_) { return power_json_error("power control is closed"); }
        if (command.kind == PowerControlKind::Status) { return power_status_json(snapshot_locked()); }
        if (command.kind == PowerControlKind::Profile && !profile_ids_.count(command.profile_id)) {
            return power_json_error("unknown profile ID");
        }
        if (pending_.size() >= kMaxPending || accepted_revision_ == std::numeric_limits<uint64_t>::max()) {
            return power_json_error("power control queue is full");
        }
        command.revision = accepted_revision_ + 1;
        pending_.push_back(command);
        accepted_revision_ = command.revision;
        return "{\"ok\":true,\"accepted_revision\":" + std::to_string(accepted_revision_) + '}';
    }

    // Gazebo Reset 경계에서 이전 대기 명령을 취소한다. 모델 reset은 다음 PostUpdate까지 미룬다.
    void request_world_reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!pending_.empty()) {
            state_.last_cancelled_from_revision = pending_.front().revision;
            state_.cancelled_through_revision = pending_.back().revision;
            pending_.clear();
        }
        world_reset_pending_ = true;
    }

    // PostUpdate에서만 FIFO 명령을 적용한다. pause에는 world reset 외 일반 명령을 보류한다.
    bool apply_pending(CoupledPowerModel &model, int64_t now_ns, bool paused)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto previous_resets = model.reset_count();
        if (world_reset_pending_) {
            model.reset();
            world_reset_pending_ = false;
        }
        if (!paused && active_) {
            for (const auto &command : pending_) {
                switch (command.kind) {
                case PowerControlKind::Fault:
                    model.set_motor_fault(command.motor_index, command.current_scale, command.rpm_scale);
                    break;
                case PowerControlKind::Clear: model.clear_faults(); break;
                case PowerControlKind::Reset: model.reset(); break;
                case PowerControlKind::Profile: model.select_profile(command.profile_id); break;
                case PowerControlKind::Restore: model.restore(); break;
                case PowerControlKind::Status: break;
                }
                state_.applied_revision = command.revision;
            }
            pending_.clear();
        }
        refresh_locked(model, now_ns, paused, false);
        return model.reset_count() != previous_resets;
    }

    // PostUpdate가 모델 관측을 마친 뒤 sim ns/새 샘플 여부와 함께 상태 복사본을 갱신한다.
    void refresh(const CoupledPowerModel &model, int64_t now_ns, bool paused, bool sample_updated)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        refresh_locked(model, now_ns, paused, sample_updated);
    }

    // 다른 스레드에 일관된 상태 복사본을 반환한다. 모델 자체는 참조하지 않는다.
    PowerControlStatus snapshot() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshot_locked();
    }

    // 소유자 종료 시 새 명령을 거부하고 큐를 지운다. 지연 콜백은 안전하게 오류를 받는다.
    void close()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_ = false;
        pending_.clear();
    }

private:
    // mutex 보유 중 마지막 적용 상태와 현재 큐 정보를 하나의 읽기 복사본으로 합친다.
    PowerControlStatus snapshot_locked() const
    {
        auto copy = state_;
        copy.accepted_revision = accepted_revision_;
        copy.pending_commands = pending_.size();
        copy.world_reset_pending = world_reset_pending_;
        return copy;
    }

    // mutex 보유 및 sim-thread 소유 모델에서 ns/SOC/mAh/배율을 복사하고 세대별 신선도를 관리한다.
    void refresh_locked(const CoupledPowerModel &model, int64_t now_ns, bool paused, bool sample_updated)
    {
        if (state_.reset_count != model.reset_count()) { state_.last_sample_ns = -1; }
        state_.reset_count = model.reset_count();
        state_.profile_id = model.profile_id();
        state_.initial_profile_id = model.initial_profile_id();
        state_.motor_current_scales = model.current_scales();
        state_.motor_rpm_scales = model.rpm_scales();
        state_.sim_ns = now_ns;
        state_.paused = paused;
        state_.soc = model.sample().soc;
        state_.consumed_mah = model.sample().consumed_mah;
        state_.sample_valid = model.sample().valid;
        if (sample_updated) { state_.last_sample_ns = now_ns; }
    }

    static constexpr std::size_t kMaxPending = 128;
    mutable std::mutex mutex_;
    std::set<std::string> profile_ids_;
    std::deque<PowerControlCommand> pending_;
    PowerControlStatus state_;
    uint64_t accepted_revision_{0};
    bool world_reset_pending_{false};
    bool active_{true};
};

}  // namespace arion
