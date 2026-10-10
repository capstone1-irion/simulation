// 역할: 중계의 입력 검사·명령 문법·수락 대기·오류 보존을 합성 값으로 검사한다.
// 입력: 명시적인 Actuators/StringMsg fixture. 출력: 성공한 검사 수 또는 첫 실패.
// 호출자: 최종 통합 검증의 native 단계. 실제 joint/비행 검증은 integration.py가 맡는다.
// 제품을 시험용으로 바꾸지 않고 같은 번역 단위의 내부 순수 함수를 검사한다.
#include "../native/motor_fault.cpp"
#include <limits>
#include <stdexcept>

namespace
{
int checks = 0;
void Require(bool condition, const char *description)
{
  if (!condition)
    throw std::runtime_error(description);
  ++checks;
}
}

int main()
{
  try
  {
    gz::msgs::Actuators input;
    Require(!capstone::ValidInput(input), "empty velocity must be rejected");
    for (double value : {100.0, 200.0, 300.0, 400.0, 1200.0})
      input.add_velocity(value);
    input.add_position(-0.5);
    input.add_normalized(0.8);
    Require(capstone::ValidInput(input), "additional valid fields must be preserved");
    input.set_velocity(1, -1.0);
    Require(!capstone::ValidInput(input), "negative velocity must be rejected");
    input.set_velocity(1, std::numeric_limits<double>::quiet_NaN());
    Require(!capstone::ValidInput(input), "NaN velocity must be rejected");
    input.set_velocity(1, 200.0);
    input.set_position(0, std::numeric_limits<double>::infinity());
    Require(!capstone::ValidInput(input), "nonfinite additional field must be rejected");

    for (int motor = 1; motor <= 4; ++motor)
    {
      capstone::Command command;
      bool query = false;
      Require(capstone::Parse("set " + std::to_string(motor) + " 0.9 3", command, query) == nullptr,
              "each declared motor must be accepted");
      Require(command.motor == motor && command.scale == 0.9 && command.duration == 3 && !query,
              "parsed values must match literal request");
    }
    for (const auto *text : {"", "set 0 0.9 1", "set 01 0.9 1", "set 5 0.9 1",
                            "set 1 0 1", "set 1 1.1 1", "set 1 0.9 0",
                            "set 1 0.9 121", "set 1 nan 1", "clear extra", "status extra"})
    {
      capstone::Command command;
      bool query = false;
      Require(capstone::Parse(text, command, query) != nullptr, "invalid request must be rejected");
    }
    capstone::Mailbox box;
    gz::msgs::StringMsg request, reply;
    request.set_data("status");
    capstone::Request(box, request, reply);
    Require(reply.data().find("clock_not_ready") != std::string::npos,
            "no clock must not create a synthetic timestamp");
    box.model = "synthetic_test_only";
    box.simTime = std::chrono::seconds(10);
    Require(capstone::Status(box, capstone::Clock::now()).find("\"command_rad_s\":null") != std::string::npos,
            "no input must remain null");
    box.inputSeq = 1;
    box.inputAge = 0.0;
    box.paused = true;
    request.set_data("set 1 0.9 3");
    capstone::Request(box, request, reply);
    Require(box.pending.has_value() && box.acceptedRevision == 1 && box.appliedRevision == 0,
            "paused acceptance must not claim physical application");
    request.set_data("clear");
    capstone::Request(box, request, reply);
    Require(reply.data().find("pending_command") != std::string::npos && box.acceptedRevision == 1,
            "pending request must not be overwritten");
    capstone::Fail(box, "synthetic_first_failure");
    capstone::Fail(box, "synthetic_later_failure");
    Require(box.fatal == "synthetic_first_failure", "first failure must remain latched");
    box.pending.reset();
    capstone::Request(box, request, reply);
    Require(reply.data().find("relay_failed") != std::string::npos && !box.pending,
            "clear must not reset failed run");
    std::cout << "native synthetic checks passed: " << checks << '\n';
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "native check failed: " << error.what() << '\n';
    return 1;
  }
}
