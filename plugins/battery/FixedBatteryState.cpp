#include "arion/BatteryFrame.hpp"
#include "arion/BatteryPublishClock.hpp"

#include <gz/common/Console.hh>
#include <gz/msgs/float_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <chrono>
#include <memory>
#include <string>

namespace arion {

// A telemetry fixture only: never changes forces, commands or battery physics.
class FixedBatteryState final : public gz::sim::System,
                                public gz::sim::ISystemConfigure,
                                public gz::sim::ISystemPreUpdate {
public:
    void Configure(const gz::sim::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &,
                   gz::sim::EntityComponentManager &ecm,
                   gz::sim::EventManager &) override
    {
        const gz::sim::Model model(entity);
        if (!model.Valid(ecm)) {
            gzerr << "FixedBatteryState must be attached to a model\n";
            return;
        }
        const std::string topic = "/arion/" + model.Name(ecm) + "/battery_state";
        publisher_ = node_.Advertise<gz::msgs::Float_V>(topic);
        if (!publisher_.Valid()) { gzerr << "Cannot advertise battery topic: " << topic << '\n'; }
    }

    void PreUpdate(const gz::sim::UpdateInfo &info,
                   gz::sim::EntityComponentManager &) override
    {
        const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
        if (!publisher_.Valid() || !clock_.due(ns, info.paused)) { return; }
        gz::msgs::Float_V message;
        auto *header = message.mutable_header();
        header->mutable_stamp()->set_sec(ns / 1000000000);
        header->mutable_stamp()->set_nsec(static_cast<int32_t>(ns % 1000000000));
        auto *schema = header->add_data();
        schema->set_key("schema");
        schema->add_value(kBatterySchema);
        for (const float value : kFixedBatteryValues) { message.add_data(value); }
        if (!publisher_.Publish(message)) { gzerr << "Battery frame publication failed\n"; }
    }

private:
    BatteryPublishClock clock_;
    gz::transport::Node node_;
    gz::transport::Node::Publisher publisher_;
};

}  // namespace arion

GZ_ADD_PLUGIN(arion::FixedBatteryState, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
