#include "arion/PowerControl.hpp"

#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <google/protobuf/struct.pb.h>
#include <google/protobuf/util/json_util.h>

#include <cstdlib>
#include <iostream>
#include <set>
#include <string>

namespace {
struct Options {
    std::string service;
    std::string command;
    unsigned timeout_ms{2000};
    bool status{false};
};

// 명시 서비스/명령과 1..120000ms 제한시간을 검사한다. 실패 시 외부 요청을 시작하지 않는다.
bool parse_options(int argc, char **argv, Options &options, std::string &error)
{
    Options next;
    std::set<std::string> seen;
    error = "usage: --service /arion/MODEL/power_control --command COMMAND [--timeout-ms 2000]";
    for (int i = 1; i < argc; i += 2) {
        const std::string key(argv[i]);
        if (i + 1 >= argc || !seen.insert(key).second) { return false; }
        const std::string value(argv[i + 1]);
        if (key == "--service") { next.service = value; }
        else if (key == "--command") { next.command = value; }
        else if (key == "--timeout-ms") {
            const auto parsed = std::from_chars(value.data(), value.data() + value.size(), next.timeout_ms);
            if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() ||
                next.timeout_ms < 1 || next.timeout_ms > 120000) { return false; }
        } else { return false; }
    }
    const std::string prefix = "/arion/";
    const std::string suffix = "/power_control";
    if (next.service.size() <= prefix.size() + suffix.size() || next.service.compare(0, prefix.size(), prefix) ||
        next.service.compare(next.service.size() - suffix.size(), suffix.size(), suffix) ||
        !arion::valid_power_profile_id(next.service.substr(prefix.size(),
            next.service.size() - prefix.size() - suffix.size()))) { return false; }
    arion::PowerControlCommand command;
    if (!arion::parse_power_command(next.command, command, error)) { return false; }
    next.status = command.kind == arion::PowerControlKind::Status;
    options = std::move(next);
    error.clear();
    return true;
}

// Protobuf Struct의 필드 하나를 찾는다. 없는 필드를 기본값으로 오인하지 않는다.
const google::protobuf::Value *field(const google::protobuf::Struct &object, const std::string &name)
{
    const auto entry = object.fields().find(name);
    return entry == object.fields().end() ? nullptr : &entry->second;
}

// JSON 숫자가 유한한 비음수 정수인지 검사한다(원문 정밀도나 문자열은 변경하지 않는다).
bool unsigned_number(const google::protobuf::Value *value)
{
    return value && value->kind_case() == google::protobuf::Value::kNumberValue &&
           std::isfinite(value->number_value()) && value->number_value() >= 0.0 &&
           std::floor(value->number_value()) == value->number_value();
}

// 모터별 무차원 배율 네 개가 유한한 0..maximum 범위인지 검사한다.
bool scale_array(const google::protobuf::Value *value, double maximum)
{
    if (!value || value->kind_case() != google::protobuf::Value::kListValue ||
        value->list_value().values_size() != 4) { return false; }
    for (const auto &item : value->list_value().values()) {
        if (item.kind_case() != google::protobuf::Value::kNumberValue || !std::isfinite(item.number_value()) ||
            item.number_value() < 0.0 || item.number_value() > maximum) { return false; }
    }
    return true;
}

// 64KiB 이하 응답 JSON을 검사해 성공0/서버오류1/잘못된 응답4를 구분한다. 원문은 그대로 둔다.
int reply_exit_code(const std::string &json, bool status)
{
    if (json.empty() || json.size() > 65536) { return 4; }
    google::protobuf::Struct response;
    if (!google::protobuf::util::JsonStringToMessage(json, &response).ok()) { return 4; }
    const auto *ok = field(response, "ok");
    if (!ok || ok->kind_case() != google::protobuf::Value::kBoolValue) { return 4; }
    if (!ok->bool_value()) {
        const auto *error = field(response, "error");
        return error && error->kind_case() == google::protobuf::Value::kStringValue &&
               !error->string_value().empty() ? 1 : 4;
    }
    if (!status) {
        const auto *revision = field(response, "accepted_revision");
        return unsigned_number(revision) && revision->number_value() > 0.0 ? 0 : 4;
    }
    const auto *profile = field(response, "profile_id");
    const auto *scope = field(response, "telemetry_only");
    if (!unsigned_number(field(response, "applied_revision")) ||
        !unsigned_number(field(response, "reset_count")) || !unsigned_number(field(response, "sim_ns")) ||
        !profile || profile->kind_case() != google::protobuf::Value::kStringValue ||
        !arion::valid_power_profile_id(profile->string_value()) ||
        !scope || scope->kind_case() != google::protobuf::Value::kBoolValue || !scope->bool_value() ||
        !scale_array(field(response, "motor_current_scales"), 3.0) ||
        !scale_array(field(response, "motor_rpm_scales"), 1.0)) { return 4; }
    return 0;
}

// 로컬 오류를 JSON으로 출력하고 지정한 실패 코드를 반환한다(오류 응답 위조 성공 없음).
int fail(int code, const std::string &message)
{
    std::cout << arion::power_json_error(message) << '\n';
    return code;
}
}  // namespace

// GZ_PARTITION의 실제 StringMsg 서비스에 한 번 요청하고 검증된 JSON 원문을 stdout에 전달한다.
int main(int argc, char **argv)
{
    const char *partition = std::getenv("GZ_PARTITION");
    if (!partition || !*partition) { return fail(2, "nonempty GZ_PARTITION is required"); }
    Options options;
    std::string error;
    if (!parse_options(argc, argv, options, error)) { return fail(2, error); }
    try {
        gz::transport::Node node;
        gz::msgs::StringMsg request;
        gz::msgs::StringMsg reply;
        request.set_data(options.command);
        bool result = false;
        if (!node.Request(options.service, request, options.timeout_ms, reply, result)) {
            return fail(3, "power control request timed out (application unknown; do not blindly retry)");
        }
        if (!result) { return fail(3, "power control transport reported failure"); }
        const int code = reply_exit_code(reply.data(), options.status);
        if (code == 4) { return fail(4, "malformed power control JSON reply"); }
        std::cout << reply.data() << '\n';
        return code;
    } catch (const std::exception &) {
        return fail(3, "power control transport exception");
    }
}
