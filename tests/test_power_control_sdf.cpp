// SDF 파서만 호출한다. Gazebo server, 물리 엔진, transport Node 객체는 시작하지 않는다.
#include "arion/CoupledPowerModel.hpp"
#include "arion/PowerControl.hpp"
#include <gz/msgs/stringmsg.pb.h>
#include <functional>
#include <sdf/Element.hh>
#include <sdf/parser.hh>

#include <iostream>
#include <stdexcept>

namespace arion {
// 제품 SDF 설정 파서의 외부 연결점을 선언한다(실제 구현은 plugin 번역단위에 있다).
CoupledPowerConfig power_config_from_sdf(const std::shared_ptr<const sdf::Element> &element);
// 실제 StringMsg 서비스 콜백 생성 함수를 선언한다(모의 transport 응답으로 대체하지 않는다).
std::function<bool(const gz::msgs::StringMsg &, gz::msgs::StringMsg &)>
power_control_callback(std::shared_ptr<PowerControlMailbox> mailbox);
}

namespace {
// 합성 plugin XML을 SDFormat으로 읽어 파서 단위시험 입력을 만든다(네트워크 없음).
std::shared_ptr<const sdf::Element> plugin_sdf(const std::string &body)
{
    auto document = std::make_shared<sdf::SDF>();
    sdf::init(document);
    if (!sdf::readString("<sdf version='1.9'><model name='unit'><link name='base'/><plugin "
                         "name='unit' filename='unused'>" + body + "</plugin></model></sdf>", document)) {
        throw std::runtime_error("fixture XML parsing failed");
    }
    return document->Root()->GetElement("model")->GetElement("plugin");
}
// 필수 프로파일 필드와 셀 곡선을 실 SDFormat 파서에서 모델 설정으로 전달하는지 검사한다.
void profile_sdf()
{
    const std::string record = "<profile id='unit_normal'><capacity_ah>5</capacity_ah>"
        "<pack_resistance_ohm>0.04</pack_resistance_ohm><initial_soc>0.95</initial_soc>"
        "<ocv_soc>0 0.5 1</ocv_soc><ocv_cell_v>3 3.6 4.2</ocv_cell_v></profile>";
    std::string catalog;
    for (int i = 0; i < 4; ++i) {
        auto entry = record;
        entry.replace(entry.find("unit_normal"), std::string("unit_normal").size(), "unit_" + std::to_string(i));
        catalog += entry;
    }
    const auto config = arion::power_config_from_sdf(plugin_sdf(
        "<battery_profiles>" + catalog + "</battery_profiles><initial_profile>unit_3</initial_profile>"));
    arion::CoupledPowerModel model(config);
    if (config.profiles.size() != 4 || model.profile_id() != "unit_3") {
        throw std::runtime_error("SDF profile not loaded");
    }
    for (int i = 0; i < 4; ++i) {
        if (!model.select_profile("unit_" + std::to_string(i))) { throw std::runtime_error("four profile selection"); }
    }
    for (const std::string &body : std::vector<std::string>{"<battery_profiles/>", "<battery_profiles>" + record + "</battery_profiles>",
             "<battery_profiles><profile id='incomplete'/></battery_profiles><initial_profile>incomplete</initial_profile>"}) {
        bool rejected = false;
        try { const auto ignored = arion::power_config_from_sdf(plugin_sdf(body)); (void)ignored; }
        catch (const std::invalid_argument &) { rejected = true; }
        if (!rejected) { throw std::runtime_error("incomplete profile SDF must fail closed"); }
    }
}
}  // namespace

// 제품 콜백의 StringMsg 응답/승인과 독립 공유 수명을 sim 실행 없이 확인한다.
void control_callback_lifetime()
{
    arion::CoupledPowerModel model;
    auto mailbox = std::make_shared<arion::PowerControlMailbox>(model);
    auto callback = arion::power_control_callback(mailbox);
    gz::msgs::StringMsg request;
    gz::msgs::StringMsg reply;
    request.set_data("fault 1 2 0.5");
    if (!callback(request, reply) || reply.data() != "{\"ok\":true,\"accepted_revision\":1}" ||
        model.current_scales()[1] != 1.0) { throw std::runtime_error("callback must enqueue, not apply"); }
    mailbox->close();
    mailbox.reset();
    request.set_data("status");
    if (!callback(request, reply) || reply.data().find("\"ok\":false") == std::string::npos) {
        throw std::runtime_error("delayed callback after owner release must be safe");
    }
}

// 외부 프로세스를 실행하지 않고 SDF 초기화 경계만 검증한다.
int main()
{
    try {
        profile_sdf();
        std::cout << "PASS profile_sdf\n";
        control_callback_lifetime();
        std::cout << "PASS control_callback_lifetime\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
