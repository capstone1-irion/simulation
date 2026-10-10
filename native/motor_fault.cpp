// 역할: PX4 Actuators를 상시 중계하고 승인된 한 모터의 명령만 약화한다.
// 입력: 실제 모델 이름의 motor_speed topic, StringMsg 고장 서비스, Gazebo SIM 시각.
// 출력: motor_speed_applied 원문 복사본, 상태 JSON, 관측 전용 joint 정답.
// 호출자: capstone_x500 SDF / Python 실행 감독자. 물리 힘·PX4·실기체는 수정하지 않는다.

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Actuators.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <iostream>
#include <locale>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

namespace capstone
{
namespace
{
using Clock = std::chrono::steady_clock;
using SimTime = Clock::duration;
using Values = std::array<std::optional<double>, 4>;
constexpr double kInputTimeout = 0.5;  // 진행 중인 SIM 초. 최초 입력 전에는 감독자가 제한한다.
constexpr double kWallTimeout = 2.0;   // 진행 중 입력 단절의 보조 벽시계 제한.
constexpr double kStatusPeriod = 0.05;
constexpr double kHeartbeatPeriod = 1.0;
constexpr double kSlowdown = 10.0;     // stock x500의 물리 joint 속도 축소율.

struct Command
{
  bool clear{false};
  int motor{0};                       // 외부 인터페이스는 1..4.
  double scale{1.0};
  double duration{0.0};               // SIM 초.
  std::uint64_t revision{0};
};

struct Fault
{
  int motor{0};                       // 0이면 패스스루.
  double scale{1.0};
  std::optional<double> started;
  std::optional<double> until;
};

// 모든 transport callback은 이것만 값으로 캡처한다. plugin this/ECM을 참조하지 않는다.
struct Mailbox
{
  std::mutex mutex;
  bool stopped{false};
  bool dirty{true};
  std::string model;
  std::string fatal;
  std::optional<gz::msgs::Actuators> input;
  std::uint64_t receivedSeq{0};
  std::optional<SimTime> receivedSim;
  std::optional<Clock::time_point> receivedWall;
  std::optional<Command> pending;      // 수동 명령 대기 슬롯은 한 개뿐이다.
  std::uint64_t acceptedRevision{0};
  std::uint64_t appliedRevision{0};
  Fault fault;

  // 아래는 마지막 Pre/PostUpdate에서 만든 관측 snapshot이다.
  std::optional<SimTime> simTime;
  bool paused{true};
  std::optional<Clock::time_point> lastAdvanceWall;
  std::optional<Clock::time_point> activeSince;
  std::uint64_t inputSeq{0};
  std::optional<double> inputAge;
  std::optional<Clock::time_point> sampledInputWall;
  Values command{};
  Values output{};
  Values joints{};
  std::uint64_t outputInputSeq{0};
  std::optional<double> outputSim;
  std::optional<double> lastTransition;
};

// chrono의 실제 관측 시각만 초로 바꾼다. 생산자 timestamp를 만들어 넣지 않는다.
double Seconds(SimTime value)
{
  return std::chrono::duration<double>(value).count();
}

void Fail(Mailbox &box, const char *reason)
{
  if (box.fatal.empty())
  {
    box.fatal = reason;
    box.dirty = true;
    std::cerr << "MotorFault: " << reason << " (new run required)\n";
  }
}

// 4개 이상을 허용하여 추가 velocity 및 다른 protobuf 필드를 그대로 보존한다.
bool ValidInput(const gz::msgs::Actuators &msg)
{
  if (msg.velocity_size() < 4)
    return false;
  for (double value : msg.velocity())
    if (!std::isfinite(value) || value < 0.0)
      return false;
  for (double value : msg.position())
    if (!std::isfinite(value))
      return false;
  for (double value : msg.normalized())
    if (!std::isfinite(value))
      return false;
  return true;
}

Values FirstFour(const gz::msgs::Actuators &msg)
{
  Values result;
  for (int i = 0; i < 4; ++i)
    result[i] = msg.velocity(i);
  return result;
}

void Number(std::ostream &out, std::optional<double> value)
{
  if (value && std::isfinite(*value))
    out << *value;
  else
    out << "null";
}

void Array(std::ostream &out, const Values &values)
{
  out << '[';
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    if (i)
      out << ',';
    Number(out, values[i]);
  }
  out << ']';
}

// 문자열은 코드 내부의 고정 오류 코드/검사한 모델 이름뿐이다. 호출자는 mutex를 가진다.
std::string Status(const Mailbox &box, Clock::time_point now,
                   std::optional<bool> ok = std::nullopt,
                   const char *requestError = nullptr)
{
  const char *error = !box.fatal.empty() ? box.fatal.c_str() :
                      (box.inputSeq == 0 ? "no_input" : nullptr);
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(17) << std::showpoint << std::boolalpha;
  out << "{\"schema\":\"capstone.motor.v1\",\"model\":\"" << box.model
      << "\",\"sim_time_s\":";
  Number(out, box.simTime ? std::optional<double>(Seconds(*box.simTime)) :
                          std::nullopt);
  out << ",\"paused\":" << box.paused << ",\"input_seq\":" << box.inputSeq
      << ",\"input_age_s\":";
  Number(out, box.inputAge);
  out << ",\"input_wall_age_s\":";
  Number(out, box.sampledInputWall ?
      std::optional<double>(Seconds(now - *box.sampledInputWall)) : std::nullopt);
  out << ",\"clock_wall_age_s\":";
  Number(out, box.lastAdvanceWall ?
      std::optional<double>(Seconds(now - *box.lastAdvanceWall)) : std::nullopt);
  out << ",\"command_rad_s\":";
  if (box.inputSeq == 0)
    out << "null";
  else
    Array(out, box.command);
  out << ",\"output_rad_s\":";
  if (!box.outputSim)
    out << "null";
  else
    Array(out, box.output);
  out << ",\"joint_rad_s\":";
  Array(out, box.joints);
  out << ",\"output_input_seq\":" << box.outputInputSeq
      << ",\"output_sim_time_s\":";
  Number(out, box.outputSim);
  out << ",\"fault_active\":" << (box.fault.motor != 0)
      << ",\"motor\":" << box.fault.motor << ",\"scale\":" << box.fault.scale
      << ",\"fault_started_s\":";
  Number(out, box.fault.started);
  out << ",\"fault_until_s\":";
  Number(out, box.fault.until);
  out << ",\"last_transition_s\":";
  Number(out, box.lastTransition);
  out << ",\"accepted_revision\":" << box.acceptedRevision
      << ",\"applied_revision\":" << box.appliedRevision
      << ",\"pending\":" << box.pending.has_value()
      << ",\"ready\":" << (error == nullptr && box.outputSim.has_value())
      << ",\"fatal\":" << !box.fatal.empty() << ",\"error\":";
  if (error)
    out << '"' << error << '"';
  else
    out << "null";
  if (ok)
  {
    out << ",\"ok\":" << *ok << ",\"request_error\":";
    if (requestError)
      out << '"' << requestError << '"';
    else
      out << "null";
  }
  out << '}';
  return out.str();
}

// 정확히 세 문법만 받는다. 잘못된 요청은 진행 중인 실험/기존 대기 명령을 바꾸지 않는다.
const char *Parse(const std::string &text, Command &command, bool &query)
{
  if (text.size() > 160)
    return "invalid_request";
  std::istringstream in(text);
  in.imbue(std::locale::classic());
  std::string verb, motor, extra;
  if (!(in >> verb))
    return "invalid_request";
  query = verb == "status";
  if (query || verb == "clear")
  {
    command.clear = !query;
    return (in >> extra) ? "invalid_request" : nullptr;
  }
  if (verb != "set" || !(in >> motor >> command.scale >> command.duration) ||
      (in >> extra))
    return "invalid_request";
  if (motor.size() != 1 || motor[0] < '1' || motor[0] > '4')
    return "invalid_motor";
  command.motor = motor[0] - '0';
  if (!std::isfinite(command.scale) || command.scale <= 0.0 || command.scale > 1.0)
    return "invalid_scale";
  if (!std::isfinite(command.duration) || command.duration <= 0.0 ||
      command.duration > 120.0)
    return "invalid_duration";
  return nullptr;
}

// status는 pause 중에도 응답한다. true는 전송 처리 성공이며 요청 수락은 JSON ok로 구분한다.
bool Request(Mailbox &box, const gz::msgs::StringMsg &request,
             gz::msgs::StringMsg &reply)
{
  std::lock_guard<std::mutex> lock(box.mutex);
  Command command;
  bool query{false};
  const char *error = Parse(request.data(), command, query);
  const auto now = Clock::now();
  if (!box.simTime)
  {
    // 아직 실제 clock을 관측하지 않았으므로 0초나 wall 시각을 대신 넣지 않는다.
    reply.set_data("{\"ok\":false,\"request_error\":\"clock_not_ready\"}");
    return true;
  }
  if (!error && !query)
  {
    if (box.stopped || !box.fatal.empty())
      error = "relay_failed";
    else if (!box.simTime || box.inputSeq == 0)
      error = "not_ready";
    else if (box.inputAge && *box.inputAge > kInputTimeout)
      error = "input_stale";
    else if (!box.paused && (!box.lastAdvanceWall ||
             Seconds(now - *box.lastAdvanceWall) > kWallTimeout))
      error = "clock_not_fresh";
    else if (box.pending)
      error = "pending_command";
    else if (!command.clear && box.fault.motor != 0)
      error = "fault_already_active";
    else
    {
      command.revision = ++box.acceptedRevision;
      box.pending = command;
      box.dirty = true;
    }
  }
  reply.set_data(Status(box, now, error == nullptr, error));
  return true;
}

bool SafeName(const std::string &name)
{
  return !name.empty() && std::all_of(name.begin(), name.end(), [](char c)
  {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
           (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
  });
}
}  // namespace

class MotorFault final : public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPreUpdate,
                         public gz::sim::ISystemPostUpdate,
                         public gz::sim::ISystemReset
{
public:
  ~MotorFault() override
  {
    {
      std::lock_guard<std::mutex> lock(mailbox_->mutex);
      mailbox_->stopped = true;
    }
    // 이미 dispatch에 복사된 callback도 shared mailbox를 소유하므로 this 해제와 무관하다.
    if (node_)
    {
      if (subscribed_)
        node_->Unsubscribe(inputTopic_);
      if (serving_)
        node_->UnadvertiseSrv(service_);
    }
  }

  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override
  {
    std::lock_guard<std::mutex> lock(mailbox_->mutex);
    auto &box = *mailbox_;
    model_ = gz::sim::Model(entity);
    if (!model_.Valid(ecm))
    {
      Fail(box, "not_a_model");
      return;
    }
    const auto name = model_.Name(ecm);
    if (!SafeName(name))
    {
      Fail(box, "invalid_model_name");
      return;
    }
    box.model = name;  // SDF 기본 이름이 아닌 런타임 이름: 예 capstone_x500_31.
    const char *partition = std::getenv("GZ_PARTITION");
    if (!partition || !*partition ||
        !gz::transport::TopicUtils::IsValidPartition(partition))
    {
      Fail(box, "isolated_partition_required");
      return;
    }
    node_ = std::make_unique<gz::transport::Node>();
    inputTopic_ = "/" + name + "/command/motor_speed";
    const auto outputTopic = "/" + name + "/command/motor_speed_applied";
    service_ = "/capstone/" + name + "/fault";
    statusPub_ = node_->Advertise<gz::msgs::StringMsg>("/capstone/" + name + "/status");
    outputPub_ = node_->Advertise<gz::msgs::Actuators>(outputTopic);
    if (!statusPub_.Valid() || !outputPub_.Valid())
      Fail(box, "advertise_failed");

    // std::function으로 overload를 명확히 하고 오직 shared_ptr 값만 캡처한다.
    const auto shared = mailbox_;
    std::function<void(const gz::msgs::Actuators &)> inputCallback =
        [shared](const gz::msgs::Actuators &msg)
    {
      std::lock_guard<std::mutex> guard(shared->mutex);
      if (shared->stopped)
        return;
      if (!ValidInput(msg))
      {
        Fail(*shared, "invalid_input");
        return;
      }
      shared->input = msg;
      ++shared->receivedSeq;
      shared->receivedSim = shared->simTime;
      shared->receivedWall = Clock::now();
    };
    subscribed_ = node_->Subscribe<gz::msgs::Actuators>(inputTopic_, inputCallback);
    if (!subscribed_)
      Fail(box, "subscribe_failed");
    std::function<bool(const gz::msgs::StringMsg &, gz::msgs::StringMsg &)> callback =
        [shared](const gz::msgs::StringMsg &request, gz::msgs::StringMsg &reply)
    {
      return Request(*shared, request, reply);
    };
    serving_ = node_->Advertise<gz::msgs::StringMsg, gz::msgs::StringMsg>(service_, callback);
    if (!serving_)
      Fail(box, "service_failed");

    for (int i = 0; i < 4; ++i)
    {
      joints_[i] = model_.JointByName(ecm, "rotor_" + std::to_string(i) + "_joint");
      if (joints_[i] == gz::sim::kNullEntity)
        Fail(box, "missing_rotor_joint");
      else if (!ecm.Component<gz::sim::components::JointVelocity>(joints_[i]))
        ecm.CreateComponent(joints_[i], gz::sim::components::JointVelocity());
    }
    CheckBypass(ecm);
    // 첫 Pre/PostUpdate의 실제 SIM 시각부터 no_input 상태를 공고한다.
  }

  // 명령 적용은 진행하는 SIM tick에서만 수행한다. pause/무시계 callback에서는 적용하지 않는다.
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    std::lock_guard<std::mutex> lock(mailbox_->mutex);
    auto &box = *mailbox_;
    const auto now = Clock::now();
    const auto previous = box.simTime;
    const bool wasPaused = box.paused;
    if (info.dt < SimTime::zero() || (previous && info.simTime < *previous))
      Fail(box, "sim_time_rewind");
    else if (previous && info.simTime > *previous &&
             (info.simTime - *previous != info.dt ||
              (lastIteration_ && info.iterations != *lastIteration_ + 1)))
      Fail(box, "sim_time_discontinuity");
    else if (lastIteration_ && info.iterations < *lastIteration_)
      Fail(box, "sim_iteration_reset");
    lastIteration_ = info.iterations;
    box.simTime = info.simTime;
    box.paused = info.paused;
    if (wasPaused != box.paused)
      box.dirty = true;
    CheckBypass(ecm);

    if (box.input)
    {
      if (box.inputSeq == 0)
        box.dirty = true;
      // 첫 clock 이전 수신은 첫 실제 관측 tick에 연결한다. Actuators header는 손대지 않는다.
      if (!box.receivedSim)
        box.receivedSim = info.simTime;
      box.inputAge = Seconds(info.simTime - *box.receivedSim);
      box.sampledInputWall = box.receivedWall;
      box.inputSeq = box.receivedSeq;
      box.command = FirstFour(*box.input);
    }
    const bool advancing = !info.paused && info.dt > SimTime::zero() &&
                           (!previous || info.simTime > *previous);
    if (!advancing)
      return;
    if (wasPaused || !box.activeSince)
      box.activeSince = now;  // pause에 머문 벽시계 시간은 단절 watchdog에 적분하지 않는다.
    box.lastAdvanceWall = now;
    if (!box.input || !box.fatal.empty())
      return;
    if (box.inputAge && *box.inputAge > kInputTimeout)
      Fail(box, "input_stale_sim");
    const auto wallOrigin = std::max(*box.receivedWall, *box.activeSince);
    if (Seconds(now - wallOrigin) > kWallTimeout)
      Fail(box, "input_stale_wall");
    if (!box.fatal.empty())
      return;

    const double sim = Seconds(info.simTime);
    Fault next = box.fault;
    bool transition = false;
    if (next.until && sim >= *next.until)
    {
      next = Fault{};
      transition = true;
    }
    if (box.pending)
    {
      const auto &command = *box.pending;
      next = Fault{};
      if (!command.clear)
      {
        next.motor = command.motor;
        next.scale = command.scale;
        next.started = sim;
        next.until = sim + command.duration;
      }
      transition = true;
    }
    auto output = *box.input;  // header, position, normalized, 추가 velocity, unknown fields 보존.
    if (next.motor != 0)
      output.set_velocity(next.motor - 1, output.velocity(next.motor - 1) * next.scale);
    if (!outputPub_.Valid() || !outputPub_.Publish(output))
    {
      Fail(box, "output_publish_failed");
      return;  // 성공하지 않은 출력/revision을 적용했다고 기록하지 않는다.
    }
    box.output = FirstFour(output);
    box.outputInputSeq = box.inputSeq;
    box.outputSim = sim;
    box.fault = next;
    if (box.pending)
    {
      box.appliedRevision = box.pending->revision;
      box.pending.reset();
    }
    if (transition)
    {
      box.lastTransition = sim;
      box.dirty = true;
    }
  }

  // 물리 후 관측만 읽는다. force/JointVelocityCmd/Actuators component는 만들거나 쓰지 않는다.
  void PostUpdate(const gz::sim::UpdateInfo &,
                  const gz::sim::EntityComponentManager &ecm) override
  {
    std::lock_guard<std::mutex> lock(mailbox_->mutex);
    CheckBypass(ecm);
    for (int i = 0; i < 4; ++i)
    {
      mailbox_->joints[i].reset();
      const auto *velocity = ecm.Component<gz::sim::components::JointVelocity>(joints_[i]);
      if (velocity && !velocity->Data().empty())
      {
        const double value = velocity->Data()[0] * kSlowdown;
        if (std::isfinite(value))
          mailbox_->joints[i] = value;  // signed rad/s: +ccw, -cw. 절댓값/명령 대체 없음.
      }
    }
    PublishStatus(Clock::now(), false);
  }

  // Gazebo의 reset 자동 재구성으로 오류가 사라지지 않도록 명시적으로 reset을 거부한다.
  void Reset(const gz::sim::UpdateInfo &info,
             gz::sim::EntityComponentManager &) override
  {
    std::lock_guard<std::mutex> lock(mailbox_->mutex);
    mailbox_->simTime = info.simTime;
    mailbox_->paused = info.paused;
    Fail(*mailbox_, "world_reset_requires_new_run");
    PublishStatus(Clock::now(), true);
  }

private:
  void CheckBypass(const gz::sim::EntityComponentManager &ecm)
  {
    if (model_.Entity() != gz::sim::kNullEntity &&
        ecm.Component<gz::sim::components::Actuators>(model_.Entity()))
      Fail(*mailbox_, "competing_actuators_component");
  }

  // 진행 중 ~20 Hz SIM + 상태 전이. 정지/시작 때도 1 Hz wall heartbeat로 발견 가능하다.
  // heartbeat의 SIM 시각은 그대로이며 새 입력이나 물리 진행을 뜻하지 않는다.
  void PublishStatus(Clock::time_point now, bool force)
  {
    auto &box = *mailbox_;
    if (!box.simTime)
      return;
    const bool simDue = box.simTime && (!lastStatusSim_ ||
        Seconds(*box.simTime - *lastStatusSim_) >= kStatusPeriod);
    const bool wallDue = !lastStatusWall_ ||
        Seconds(now - *lastStatusWall_) >= kHeartbeatPeriod;
    if (!force && !box.dirty && !simDue && !wallDue)
      return;
    lastStatusSim_ = box.simTime;
    lastStatusWall_ = now;
    box.dirty = false;
    if (!statusPub_.Valid())
      return;  // Configure가 이미 오류를 남겼다. 감독자는 첫 상태 수신 제한으로 실패한다.
    gz::msgs::StringMsg message;
    message.set_data(Status(box, now));
    if (!statusPub_.Publish(message))
      Fail(box, "status_publish_failed");
  }

  std::shared_ptr<Mailbox> mailbox_{std::make_shared<Mailbox>()};
  gz::sim::Model model_{gz::sim::kNullEntity};
  std::array<gz::sim::Entity, 4> joints_{
      gz::sim::kNullEntity, gz::sim::kNullEntity, gz::sim::kNullEntity, gz::sim::kNullEntity};
  std::unique_ptr<gz::transport::Node> node_;
  gz::transport::Node::Publisher outputPub_;
  gz::transport::Node::Publisher statusPub_;
  std::string inputTopic_;
  std::string service_;
  bool subscribed_{false};
  bool serving_{false};
  std::optional<std::uint64_t> lastIteration_;
  std::optional<SimTime> lastStatusSim_;
  std::optional<Clock::time_point> lastStatusWall_;
};
}  // namespace capstone

GZ_ADD_PLUGIN(capstone::MotorFault, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate,
              gz::sim::ISystemPostUpdate, gz::sim::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(capstone::MotorFault, "capstone::MotorFault")
