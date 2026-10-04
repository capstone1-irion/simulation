#include <gz/msgs/float_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <array>
#include <chrono>
#include <string>

namespace arion {

// Transport fixture only. Never writes joint forces, speeds, or motor commands.
class FixedMotorState final : public gz::sim::System,
                              public gz::sim::ISystemConfigure,
                              public gz::sim::ISystemPreUpdate {
public:
    void Configure(const gz::sim::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &,
                   gz::sim::EntityComponentManager &ecm,
                   gz::sim::EventManager &) override
    {
        const gz::sim::Model model(entity);
        if (!model.Valid(ecm)) { return; }
        const std::string topic = "/arion/" + model.Name(ecm) + "/motor_state";
        publisher_ = node_.Advertise<gz::msgs::Float_V>(topic);
    }

    void PreUpdate(const gz::sim::UpdateInfo &info,
                   gz::sim::EntityComponentManager &) override
    {
        if (info.paused || !publisher_.Valid()) { return; }
        if (info.simTime < previous_time_) { next_publish_ = info.simTime; }
        previous_time_ = info.simTime;
        if (info.simTime < next_publish_) { return; }
        next_publish_ = info.simTime + std::chrono::milliseconds(50);
        const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
        if (ns < 0) { return; }
        gz::msgs::Float_V message;
        auto *header = message.mutable_header();
        header->mutable_stamp()->set_sec(ns / 1000000000);
        header->mutable_stamp()->set_nsec(static_cast<int32_t>(ns % 1000000000));
        auto *schema = header->add_data();
        schema->set_key("schema");
        schema->add_value("arion.motor_state.v1");
        // RPM[1..4], V[1..4], A[1..4]. Deliberately distinct test values.
        const std::array<float, 12> fixture{{1101, 2202, 3303, 4404,
                                          15.1f, 15.2f, 15.3f, 15.4f,
                                          1.1f, 2.2f, 3.3f, 4.4f}};
        for (float value : fixture) { message.add_data(value); }
        publisher_.Publish(message);
    }

private:
    std::chrono::steady_clock::duration previous_time_{0};
    std::chrono::steady_clock::duration next_publish_{0};
    gz::transport::Node node_;
    gz::transport::Node::Publisher publisher_;
};

}  // namespace arion

GZ_ADD_PLUGIN(arion::FixedMotorState, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
