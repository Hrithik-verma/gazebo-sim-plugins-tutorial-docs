# Light Control Service Way System Plugin

<iframe
  width="960"
  height="540"
  src="https://www.youtube.com/embed/4A5l1Uwbsw4?si=70ksm7EQxMw6BtMW"
  title="Gazebo Sim Structure Plugin Perspective"
  frameborder="0"
  allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
  allowfullscreen>
</iframe>


In the last plugin of [Light Control Plugin](light-control-system-plugin.md) will added ```gz service``` to turn light on/off. Same way as earlier we are using ```gz-transport``` for service/client.


![gz-service-light](assets/images/gz-service_light.png)


here we are going to send service call request on type empty to gz-transport server.


<br>
<br>

gazebo transport <br>
- [Services c++ example](https://gazebosim.org/api/transport/15/services.html){target=_blank}<br>
- [Services python example](https://gazebosim.org/api/transport/13/services.html){target=_blank}<br>


## Plugin


<details>
   <summary>.sdf plugin part</summary>

```xml
<plugin
    filename="LightControlServiceWay"
    name="gz::sim::LightControlServiceWay">

    <on_off_service_name>lights_on_srv</on_off_service_name>
</plugin>

```
</details>


<details>
   <summary>.hh file</summary>

```c++
#ifndef LIGHT_CONTROL_PLUGIN_HH_
#define LIGHT_CONTROL_PLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Light.hh>
#include <gz/math/Color.hh>
#include <gz/sim/Light.hh>

#include <gz/msgs.hh>
#include <gz/transport.hh>


#include <vector>

namespace gz
{
namespace sim
{
class LightControlServiceWay : public System, public ISystemPreUpdate, public ISystemConfigure
{
public:
  // Constructor
  LightControlServiceWay() = default;

  void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override;

  // ISystemPreUpdate method
  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;

private:
  // Function to find the light entity by name
  void FindLightEntities(EntityComponentManager &_ecm);

  //empty service call to turn on/off light
  bool srvLightToggle(const gz::msgs::Empty &_req,
  gz::msgs::Empty &_rep);

  // Time accumulator for color cycling
  double time = 0.0;

  std::vector<Entity> lightEntities; //list of light entity

  gz::transport::Node node; //gz node
  std::string serviceName; //service name
  bool isLightOn; //val for light on/off
  bool pendingApply{false};   // apply state change once (e.g., when toggled OFF)
};
}  // namespace sim
}  // namespace gz

#endif

```

</details>




<details>
   <summary>.cc file</summary>

```c++
#include "tutorial_gazebo_plugins/LightControlServiceWay.hh"

#include <chrono>
#include <cmath>
#include <iostream>

#include <gz/common/Console.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/light.pb.h>

#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/LightCmd.hh>
#include <gz/sim/Conversions.hh>
#include <gz/math/Color.hh>
#include <gz/plugin/Register.hh> //for GZ_ADD_PLUGIN_ALIAS()


using namespace gz;
using namespace gz::sim;

//////////////////////////////////////////////////
void LightControlServiceWay::Configure(const Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
  if (!_sdf || !_sdf->HasElement("on_off_service_name"))
  {
    gzerr << "LightControlServiceWay plugin requires <on_off_service_name>."
          << std::endl;
    return;
  }

  this->serviceName = _sdf->Get<std::string>("on_off_service_name");
 

  // Initial state
  this->isLightOn = true;
  this->pendingApply = true;  // push initial state to the world once

  
  // Advertise service: Empty -> Empty, callback returns bool
  bool ok = this->node.Advertise(
      this->serviceName,
      &LightControlServiceWay::srvLightToggle,
      this);

  if (!ok)
  {
    std::cerr << "Error advertising service [" << this->serviceName << "]"
              << std::endl;
  }
  
   gzmsg << "[LightControlServiceWay] Service Name: "
        << this->serviceName << std::endl;


}

//////////////////////////////////////////////////
bool LightControlServiceWay::srvLightToggle(
    const gz::msgs::Empty &/*_req*/, gz::msgs::Empty &/*_rep*/)
{
  // Flip state and mark that we must push a single command if turning OFF
  this->isLightOn = !this->isLightOn;
  this->pendingApply = true;

  gzmsg << "[LightControlServiceWay] Toggled. isLightOn = " << std::boolalpha << this->isLightOn << std::endl;

  std::cerr << "[LightControlServiceWay] Toggled. isLightOn = "
        << std::boolalpha << this->isLightOn << std::endl;
  return true;
}

//////////////////////////////////////////////////
void LightControlServiceWay::FindLightEntities(EntityComponentManager &_ecm)
{
  this->lightEntities.clear();
  _ecm.Each<components::Light>(
      [&](const Entity &_e, const components::Light *) -> bool
      {
        this->lightEntities.push_back(_e);
        return true;
      });
}

//////////////////////////////////////////////////
void LightControlServiceWay::PreUpdate(const UpdateInfo &_info,
                                     EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  // Only advance animation time while ON
  if (this->isLightOn)
  {
    this->time += std::chrono::duration_cast<std::chrono::duration<double>>(
                         _info.dt).count();
  }

  this->FindLightEntities(_ecm);
  if (this->lightEntities.empty())
    return;

  // Color for ON animation (computed only if ON)
  gz::math::Color animColor(1, 1, 1, 1);
  if (this->isLightOn)
  {
    const double r = 0.5 * (1.0 + std::sin(this->time * 0.5));
    const double g = 0.5 * (1.0 + std::sin(this->time * 0.5 + 2.0));
    const double b = 0.5 * (1.0 + std::sin(this->time * 0.5 + 4.0));
    animColor = gz::math::Color(r, g, b, 1.0);
  }

  for (const Entity e : this->lightEntities)
  {
    // When OFF and no state change to apply, do nothing at all
    if (!this->isLightOn && !this->pendingApply)
      continue;

    auto lightComp = _ecm.Component<components::Light>(e);
    if (!lightComp)
      continue;

    // Start from current SDF, then apply our command
    const sdf::v14::Light &sdfLight = lightComp->Data();
    gz::msgs::Light msg = gz::sim::convert<gz::msgs::Light>(sdfLight);

    if (this->isLightOn)
    {
      // While ON: animate color every tick and ensure light is enabled
      gz::msgs::Set(msg.mutable_diffuse(),  animColor);
      gz::msgs::Set(msg.mutable_specular(), animColor);
      msg.set_is_light_off(false);
      // Intensity left as configured by SDF; can be animated if desired
    }
    else
    {
      // Transitioning to OFF: send ONCE, then stop publishing
      msg.set_is_light_off(true);
      msg.set_intensity(0.0);   // defensive: ensure no residual emission
    }

    _ecm.SetComponentData<components::LightCmd>(e, msg);
    _ecm.SetChanged(e, components::LightCmd::typeId,
                    ComponentState::PeriodicChange);
  }

  if (!this->isLightOn && this->pendingApply)
    this->pendingApply = false;

}

// --- Plugin registration ---
GZ_ADD_PLUGIN(
  gz::sim::LightControlServiceWay,
  gz::sim::System,
  gz::sim::LightControlServiceWay::ISystemConfigure,
  gz::sim::LightControlServiceWay::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::LightControlServiceWay,
                    "gz::sim::LightControlServiceWay")

```

</details>



## Gazebo Messages

[gz-msgs github](https://github.com/gazebosim/gz-msgs/tree/gz-msgs12/proto/gz/msgs){target=_blank}

gz-msgs use [google Protobuf messages](https://github.com/protocolbuffers/protobuf){target=_blank}

![gz empty msg](assets/images/empty_gz_msg.png)


[gz msgs Empty Class Reference](https://gazebosim.org/api/msgs/9/classgz_1_1msgs_1_1Empty.html){target=_blank}


![empty gazebo msg class](assets/images/empty_class_gz.png)


<br>
<br>


## Gazebo Transport Node

[transport class api](https://gazebosim.org/api/transport/14/classgz_1_1transport_1_1Node.html){target=_blank}


![gz transport](assets/images/gz-transport.png)


```c++
 gz::transport::Node node; // GZ Transport node
```

<br>



![gz advertise service](assets/images/gz_advertise_srv.png)


```c++
// Advertise service: Empty -> Empty, callback returns bool
bool ok = this->node.Advertise(
    this->serviceName,
    &LightControlServiceWay::srvLightToggle,
    this);

if (!ok)
{
std::cerr << "Error advertising service [" << this->serviceName << "]"
            << std::endl;
}
```

<br>
<br>

service callback

```c++
bool LightControlServiceWay::srvLightToggle(
    const gz::msgs::Empty &/*_req*/, gz::msgs::Empty &/*_rep*/)
{
  // Flip state and mark that we must push a single command if turning OFF
  this->isLightOn = !this->isLightOn;
  this->pendingApply = true;

  gzmsg << "[LightControlServiceWay] Toggled. isLightOn = " << std::boolalpha << this->isLightOn << std::endl;

  std::cerr << "[LightControlServiceWay] Toggled. isLightOn = "
        << std::boolalpha << this->isLightOn << std::endl;
  return true;
}

```


## Variable

```isLightOn:``` toggle base on when light is on or off <br>
```pendingApply:``` until light off is pending its true once done it becomes false <br>



## Gz Service

gazebo service list
```bash
gz service -l
```

To check topic
```bash
gz service -i -s /lights_on_srv
```

<br>

To send comand to light Empty service
```bash
gz service -s /lights_on_srv   --reqtype gz.msgs.Empty   --reptype gz.msgs.Empty   --req ''
```