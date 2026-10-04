// 제품 CLI의 순수 옵션/응답 검증을 포함해 테스트한다. 이 main은 transport를 호출하지 않는다.
#define main arion_power_control_cli_main
#include "../tools/power_control.cpp"
#undef main

#include <stdexcept>
#include <vector>

namespace {
// 옵션 문자열을 argv로 전달해 제품 파서를 호출한다(외부 상태 변경 없음).
bool options(std::vector<std::string> args, Options &out)
{
    std::vector<char *> argv;
    for (auto &arg : args) { argv.push_back(arg.data()); }
    std::string error;
    return parse_options(static_cast<int>(argv.size()), argv.data(), out, error);
}
// CLI 인수의 필수 항목, 중복, 범위 및 공백 명령 전달을 확인한다.
void cli_options()
{
    Options out;
    if (!options({"control", "--service", "/arion/unit/power_control", "--command", "fault 2 2 0.5"}, out) ||
        out.timeout_ms != 2000 || out.command != "fault 2 2 0.5") { throw std::runtime_error("valid options"); }
    for (const auto &args : std::vector<std::vector<std::string>>{
             {"control"}, {"control", "--service", "/arion/unit/power_control"},
             {"control", "--service", "/wrong/service", "--command", "status"},
             {"control", "--service", "/arion/unit/power_control", "--command", "status extra"},
             {"control", "--service", "/arion/unit/power_control", "--command", "status", "--timeout-ms", "0"},
             {"control", "--service", "/arion/unit/power_control", "--command", "status", "--timeout-ms", "3x"},
             {"control", "--service", "/arion/unit/power_control", "--command", "status", "--command", "reset"}}) {
        if (options(args, out)) { throw std::runtime_error("invalid options accepted"); }
    }
}
// 실제 서버 JSON 생성기를 검증하며 잘못된/오류 응답을 성공으로 처리하지 않는지 검사한다.
void cli_replies()
{
    arion::CoupledPowerModel model;
    arion::PowerControlMailbox mailbox(model);
    if (reply_exit_code(mailbox.handle("status"), true) != 0 ||
        reply_exit_code(mailbox.handle("clear"), false) != 0 ||
        reply_exit_code(mailbox.handle("profile missing"), false) != 1) {
        throw std::runtime_error("valid service reply classification");
    }
    for (const std::string value : {"not json", "{}", "{\"ok\":true}", "{\"ok\":1}",
            "{\"ok\":false}", "{\"ok\":true,\"accepted_revision\":-1}",
            "{\"ok\":true,\"accepted_revision\":0.5}", "{\"ok\":true,\"accepted_revision\":\"1\"}"}) {
        if (reply_exit_code(value, false) != 4) { throw std::runtime_error("malformed reply accepted: " + value); }
    }
    if (reply_exit_code(mailbox.handle("clear"), true) != 4 ||
        reply_exit_code(std::string(65537, ' '), false) != 4) {
        throw std::runtime_error("wrong shape/oversize reply accepted");
    }
}
}  // namespace

// 순수 CLI 경계 시험만 실행한다. 실제 서비스 전달 검증은 부모의 Gazebo 실행에 남긴다.
int main()
{
    try {
        cli_options();
        std::cout << "PASS cli_options\n";
        cli_replies();
        std::cout << "PASS cli_replies\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
