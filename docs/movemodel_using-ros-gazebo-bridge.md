# Move Model Using ROS GZ Bridge

<iframe
  width="960"
  height="540"
  src="https://www.youtube.com/embed/TU77ek6_bA0?si=XCMtr3D7qakm01Cs"
  title="Gazebo Sim Structure Plugin Perspective"
  frameborder="0"
  allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
  allowfullscreen>
</iframe>



From Now on we are using ros_ws for all of our plugin.
This plugin is just copy paste of [MoveModelTopicWay](movemodel_using_topic.md) into ros_ws & addition is we are using [ros_gz_bridge](https://docs.ros.org/en/jazzy/p/ros_gz_bridge/){target=_blank} to map ros2 topic with gazebo topic & control the movel model zVelocity. So the same system plugin on addition is ros2 topic.


![ros2 gz bridge roscon img](assets/images/ros-gz-bridge-roscon.png)

<br>
<br>

![ros2 gz bridge our case](assets/images/ros_gz_our_case.png)


<br>

![total plugin ros2 gz bridge](assets/images/totol_plugin_ros_gz.png)



## ros gz bridge sytax
```
<topic>@<ros_msg_type>@<gz_msg_type>
```

```
/cmd_vel_z@std_msgs/msg/Float64@gz.msgs.Double[

```

The ROS message type is followed by an ```@, [, or ]``` symbol where:

- @ is a bidirectional bridge.

- [ is a bridge from Gazebo to ROS.

- ] is a bridge from ROS to Gazebo.


it sometime confusing better to use yaml file way

```yaml
- ros_topic_name: /cmd_vel_z
  gz_topic_name: /cmd_vel_z
  ros_type_name: std_msgs/msg/Float64
  gz_type_name: gz.msgs.Double
  direction: BIDIRECTIONAL
```

#### Important:
there are only limited set of msg supported for ros gz brige

![supported_msg](assets/images/supported_ros_gz_msg.png)

more here details here in the doc [list of supported msg](https://docs.ros.org/en/jazzy/p/ros_gz_bridge/){target=_blank}



## Plugin

<details>
   <summary>ros2 launch python file</summary>

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    plugin_pkg_path = FindPackageShare('tutorial_gazebo_plugins')

    set_plugin_path = SetEnvironmentVariable(
    'GZ_SIM_SYSTEM_PLUGIN_PATH',
    PathJoinSubstitution([plugin_pkg_path, 'plugins'])
    )

    world_path = os.path.join(
        get_package_share_directory('yt_tutorial_gazebo_ros'),
        'worlds',
        'move_model_topic_way.sdf',
    )

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            # see next section for -v
            'gz_args': f'-r -v4 {world_path}',
        }.items(),
    )


    # gz_to_ros_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='z_velocity_bridge',
    #     output='screen',
    #     arguments=[
    #         'cmd_vel_z@gz.msgs.Double@std_msgs/msg/Float64'
    #     ],
    # )

    config_file_path = os.path.join(
    get_package_share_directory('yt_tutorial_gazebo_ros'),
    'configs',
    'model_topic_way_bridge.yaml'
    )

    gz_to_ros_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='z_velocity_bridge',
    output='screen',
    parameters=[{
        'config_file': config_file_path
    }],
)

    return LaunchDescription([set_plugin_path, gz_sim, gz_to_ros_bridge])

```

</details>


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


## Build ros2 pkg

```
cd ros2_ws
colcon build --symlink-install
```

## Run ros2 pkg

```
source install/setup.bash
ros2 launch yt_tutorial_gazebo_ros move_model_topic_way.launch.py
```