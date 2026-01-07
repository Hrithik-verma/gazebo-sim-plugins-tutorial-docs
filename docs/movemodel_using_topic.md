# Move Model Using Topic System Plugin


<iframe
  width="960"
  height="540"
  src="https://www.youtube.com/embed/xeQoCLLQVaU?si=Jg9ZwF6qU2KpY39g"
  title="Gazebo Sim Structure Plugin Perspective"
  frameborder="0"
  allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
  allowfullscreen>
</iframe>


In these plugin logic & system api are same as [Move Model System Plugin](move-model.md) & only addition is we are going to use gz sim topic to control the velocity. We will write a [gazebo transport](https://gazebosim.org/api/transport/15/tutorials.html){target=_blank} node which will subscribe to gazebo sim topic and base on the value will move the model.


![move_model_topic_way_plugin](assets/images/gz_subscriber.png)


<br>
<br>

gazebo transport <br>
- [publisher/subscriber c++ example](https://gazebosim.org/api/transport/15/messages.html){target=_blank}<br>
- [publisher/subscriber python example](https://gazebosim.org/api/transport/13/python.html){target=_blank}<br>


<br>
<br>

## Plugin

<details>
   <summary>.sdf plugin part</summary>
```xml
<plugin
    filename="MoveModelTopicWay"
    name="gz::sim::systems::MoveModelTopicWay">
  <model_name>TestCube</model_name>
  <topic_name>cmd_vel_z</topic_name>
</plugin>
```

</details>



<details>
   <summary>.hh file</summary>

```c++
#ifndef SYSTEM_PLUGIN_MODEL_HH_
#define SYSTEM_PLUGIN_MODEL_HH_

//! [header]
#include <gz/sim/System.hh> // to inherit system
#include "gz/sim/Model.hh"  // for Model component
#include "gz/sim/components/LinearVelocity.hh" // for linear velocity 
#include "gz/sim/components/LinearVelocityCmd.hh" // for LinearVelocityCmd component
#include "gz/sim/components/Name.hh"  // for Name component
#include <gz/plugin/Register.hh> //for GZ_ADD_PLUGIN_ALIAS()
#include <gz/msgs/Utility.hh>    //for msg

#include <gz/transport/Node.hh> //for gz transport node
#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h> // Include the Double message type

namespace gz
{
namespace sim
{
namespace systems
{
  /// \brief plugin to move a model
  /// plugin interface.
  class MoveModelTopicWay :
    // This class is a system.
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    // This class also implements the ISystemPreUpdate interface.
    public gz::sim::ISystemUpdate
  {
   public:
    MoveModelTopicWay();

    ~MoveModelTopicWay() override;

    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override;

    void Update(const UpdateInfo &_info,
                  EntityComponentManager &_ecm) override;

   private:
    std::string modelName;
    std::string topicName;
    double zVelocity{0.0};
    Entity targetEntity{kNullEntity};
    gz::transport::Node node; // GZ Transport node

    /* transport msg callback */
    void OnTransportMsg(const gz::msgs::Double & _msg);
  };
}
}
}

//! [header]

#endif
```

</details>


<details>
   <summary>.cc file</summary>

```c++
#include "MoveModelTopicWay.hh"



using namespace gz;
using namespace sim;
using namespace systems;

MoveModelTopicWay::MoveModelTopicWay() {
  std::cout<<"MoveModelTopicWay Plugin Started!!"<<std::endl;

}
MoveModelTopicWay::~MoveModelTopicWay(){
  std::cout<<"MoveModelTopicWay Plugin stopped!!"<<std::endl;

}

void MoveModelTopicWay::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  // 1. Read the target model name from SDF
  if (!_sdf->HasElement("model_name"))
  {
    gzerr << "MoveModel plugin requires a <model_name> element." << std::endl;
    return;
  }
  this->modelName = _sdf->Get<std::string>("model_name");
  gzmsg << "Target Model Name: " << this->modelName << std::endl;

  if (!_sdf->HasElement("topic_name"))
  {
    gzerr << "MoveModel plugin requires a <topic_name> element." << std::endl;
    return;
  }

  this->topicName = _sdf->Get<std::string>("topic_name");
  gzmsg << "Cmd vel z Topic Name: " << this->topicName << std::endl;

  this->node.Subscribe(this->topicName, &MoveModelTopicWay::OnTransportMsg, this);

  if (_sdf->HasElement("z_velocity"))
  {
    this->zVelocity = _sdf->Get<double>("z_velocity");
  }
  gzmsg << "Initial Z Velocity: " << this->zVelocity << " m/s" << std::endl;


}

void MoveModelTopicWay::OnTransportMsg(const gz::msgs::Double & _msg){
  // Directly extract the double value from the message
  this->zVelocity = _msg.data();
  gzdbg << "Received new Z-velocity command: " << this->zVelocity << std::endl;
}

void MoveModelTopicWay::Update(const UpdateInfo &_info,
                          EntityComponentManager &_ecm)
{
  // Only run if the simulation is not paused
  if (_info.paused)
    return;

  // 1. Find the target model entity by name (if not found yet)
  if (this->targetEntity == kNullEntity)
  {
    auto entityOpt = _ecm.EntityByName(this->modelName);
    if (!entityOpt.has_value())
    {
      gzdbg << "Model [" << this->modelName
            << "] not found yet. Skipping velocity application." << std::endl;
      return;
    }

    this->targetEntity = entityOpt.value();
    gzmsg << "Found target model entity: " << this->targetEntity << std::endl;
  }

  //method 1
  // 2.Get / create LinearVelocity component
  // auto velComp =
  //     _ecm.Component<components::LinearVelocityCmd>(this->targetEntity);

  // if (!velComp)
  // {
  //   velComp = _ecm.CreateComponent(this->targetEntity,
  //                                  components::LinearVelocityCmd());
  //   gzmsg << "Added LinearVelocity component to model: "
  //         << this->modelName << std::endl;
  // }

  // if (!velComp)
  // {
  //   gzerr << "Failed to create/get LinearVelocity component for model ["
  //         << this->modelName << "]." << std::endl;
  //   return;
  // }

  // // 3.Set the Z-axis linear velocity: (0, 0, zVelocity)
  // const gz::math::Vector3d vel(0.0, 0.0, this->zVelocity);
  // velComp->Data() = vel;

                  /// or
                  
  // method 2
  //instate of step 2 & 3 directly run     
  const gz::math::Vector3d vel(0.0, 0.0, this->zVelocity);     
  _ecm.SetComponentData<components::LinearVelocityCmd>(this->targetEntity,{vel});

}

// Register the plugin with Gazebo Sim
GZ_ADD_PLUGIN(gz::sim::systems::MoveModelTopicWay,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::MoveModelTopicWay,
                    "gz::sim::systems::MoveModelTopicWay")
```

</details>


## Gazebo Messages
[gz-msgs github](https://github.com/gazebosim/gz-msgs/tree/gz-msgs12/proto/gz/msgs){target=_blank}

gz-msgs use [google Protobuf messages](https://github.com/protocolbuffers/protobuf){target=_blank}


![gz double msg](assets/images/gz-double.png)


[gz msgs Double Class Reference](https://gazebosim.org/api/msgs/9/classgz_1_1msgs_1_1Double.html){target=_blank}

![double gazebo msg class](assets/images/double_class.png)



## Gazebo Transport Node

[transport class api](https://gazebosim.org/api/transport/14/classgz_1_1transport_1_1Node.html){target=_blank}

![gz transport](assets/images/gz-transport.png)


```c++
 gz::transport::Node node; // GZ Transport node
```




![gz-subscriber](assets/images/gz-subscriber.png)


```c++
this->node.Subscribe(this->topicName, &MoveModelTopicWay::OnTransportMsg, this);
```

topic callback function

```c++
void MoveModelTopicWay::OnTransportMsg(const gz::msgs::Double & _msg){
  // Directly extract the double value from the message
  this->zVelocity = _msg.data();
  gzdbg << "Received new Z-velocity command: " << this->zVelocity << std::endl;
}
```

## Gazebo tools (command line)

list topic
```
gz topic -l
```

check topic msg type
```
gz topic -i -t /cmd_vel_z
```

send msg
```
gz topic -t /cmd_vel_z -m gz.msgs.Double -p 'data: -1.5'
```

