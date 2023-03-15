#include <string>
#include <gz/sim/System.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

namespace proto_gazebo {

class HelloWorldPlugin : public gz::sim::System,
                         public gz::sim::ISystemPostUpdate {
public:
  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager & /*_ecm*/) override {
    std::string msg = "Hello World! Simulation is ";
    if (!_info.paused)
      msg += "not ";
    msg += "paused.";

    // Messages printed with gzmsg only show when running with verbosity 3 or
    // higher (i.e. gz sim -v 3)
    gzmsg << msg << std::endl;
  }
};

} // namespace proto_gazebo

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(proto_gazebo::HelloWorldPlugin,
              gz::sim::System,
              proto_gazebo::HelloWorldPlugin::ISystemPostUpdate)
