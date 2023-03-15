#include <gz/sim/System.hh>

namespace proto_gazebo {

class PosePlugin : public gz::sim::System, public gz::sim::ISystemUpdate {
public:
  void Update(const gz::sim::UpdateInfo &info_,
              const gz::sim::EntityComponentManager &ecm_) override;
};

} // namespace proto_gazebo

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(proto_gazebo::PosePlugin,
              gz::sim::System,
              proto_gazebo::PosePlugin::ISystemPostUpdate)
