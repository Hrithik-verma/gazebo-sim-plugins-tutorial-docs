# MoveModelDirectRos Plugin

<iframe
  width="960"
  height="540"
  src="https://www.youtube.com/embed/LD-NH1-37zk?si=DSS_htDGrThdXJ1S"
  title="Gazebo Sim Structure Plugin Perspective"
  frameborder="0"
  allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
  allowfullscreen>
</iframe>


As we understood in the last method that ros-gz-brige has limited supported msgs. So we are going undertand to add direct ros2 rclcpp into gazebo sim system plugin.

rclcpp we can direct use standard ros2 ```publisher/subscriber```,```service/client``` rclcpp everything of a ros2 node syntex is same.





## Need A Different thread for ros2 in plugin why?


ros executor code snap

```c++
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
executor.spin();  // blocks this thread, one callback at a time
```

<br>

![ros thread](assets/images/ros_executor_from_doc.png)

image beautifully display how ros executors work. For example 2 msg are received from topic than ros executors send one method to ros subscriber callback waits for it to process that is the time when it block main thread & once processed sends next msg.


<br>


```executor.spin()``` blocking loop until the node shuts down, centralizing event handling for subscriptions, timers, and services within a thread. 

that means using executor.spin() on gazebo sim will also ***get blocked(stop for some time)*** or ***become unsmooth***

![block main thread](assets/images/block_main_thread.png)


<br>
<br>
<br>

### Add New Thread For ros_executor

so simple solution is make another thread for ros2 using ```std::thread()```

![new thread](assets/images/new_thread.png)

ros spin work with blocking but not harming gz main thread as it a different thread

everthing of ros run on the new thread (name: cpp_thread) means ```OnRosMsg(...)``` subscriber callback is also part of new thread


<br>
<br>

## Code

create the ros2 node & create singe threaded executor. More on [ros2 executors](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Executors.html)
```c++

  // Create the ROS 2 node
  this->ros_node_ = std::make_shared<rclcpp::Node>("gz_sim_move_model_plugin");
  //   this->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  this->ros_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  this->ros_executor_->add_node(this->ros_node_);
```


<br>


lamda function contain ```ros_executor->spin()```<br>
[std::thread()](https://www.geeksforgeeks.org/cpp/multithreading-in-cpp/) used for creating the new thread

```c++


  //lamda function having ros spin
  auto ros_thread_spin = [this]() 
  { 
    this->ros_executor_->spin(); 
  };

  //make a new thread for ros spin
  this->cpp_thread = std::thread(ros_thread_spin);

```


<details>
   <summary>.hh file</summary>

```c++
#ifndef SYSTEM_MOVE_MODEL_DIRECTROS_PLUGIN_MODEL_HH_
#define SYSTEM_MOVE_MODEL_DIRECTROS_PLUGIN_MODEL_HH_

//! [header]
#include <gz/sim/System.hh>
#include <gz/math/Vector3.hh>

// ROS 2 Includes
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gz
{
namespace sim
{
namespace systems
{
  /// \brief plugin to move a model
  /// plugin interface.
  class MoveModelDirectRos :
    // This class is a system.
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    // This class also implements the ISystemPreUpdate interface.
    public gz::sim::ISystemUpdate
  {
   public:
    MoveModelDirectRos();

    ~MoveModelDirectRos() override;

    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override;

    void Update(const UpdateInfo &_info,
                  EntityComponentManager &_ecm) override;

   private:
    std::string modelName;
    std::string rosTopicName;
    double zVelocity{0.0};
    Entity targetEntity{kNullEntity};
    // ROS 2 Members
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
    // rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_
    rclcpp::executors::SingleThreadedExecutor::SharedPtr ros_executor_; //Executor to spin the controller
    std::thread cpp_thread; //std thread to make new thread


    /* ROS 2 message callback */
    /**
     * @brief ROS 2 callback function for received velocity messages.
     * @param _msg The shared pointer to the incoming std_msgs/Float64 message.
    */
    void OnRosMsg(const std_msgs::msg::Float64::SharedPtr _msg);
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
#include "tutorial_gazebo_plugins/MoveModelDirectRos.hh"

#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"
#include "gz/sim/components/Name.hh"
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

#include <rclcpp/executors.hpp> // New: ROS 2 includes


using namespace gz;
using namespace sim;
using namespace systems;

MoveModelDirectRos::MoveModelDirectRos(){
  std::cout<<"MoveModelDirectRos Plugin Started!!"<<std::endl;

}
MoveModelDirectRos::~MoveModelDirectRos() {
  std::cout<<"MoveModelDirectRos Plugin Stopped!!"<<std::endl;

}

void MoveModelDirectRos::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  // --- Gazebo Sim Configuration ---
  if (!_sdf->HasElement("model_name"))
  {
    gzerr << "MoveModel plugin requires a <model_name> element." << std::endl;
    return;
  }
  this->modelName = _sdf->Get<std::string>("model_name");
  gzmsg << "Target Model Name: " << this->modelName << std::endl;

  if (!_sdf->HasElement("ros_topic_name"))
  {
    gzerr << "MoveModel plugin requires a <topic_name> element." << std::endl;
    return;
  }

  this->rosTopicName = _sdf->Get<std::string>("ros_topic_name");
  gzmsg << "Cmd vel Z ROS 2 Topic Name: " << this->rosTopicName << std::endl;
  
  if (_sdf->HasElement("z_velocity"))
  {
    this->zVelocity = _sdf->Get<double>("z_velocity");
  }
  gzmsg << "Initial Z Velocity: " << this->zVelocity << " m/s" << std::endl;


  // --- ROS 2 Initialization ---
  // Initialize ROS 2 (safe to call multiple times)
  if (!rclcpp::ok())
  {
    // Pass in dummy arguments.
    rclcpp::init(0, nullptr); 
  }

  // ### Explanation ###
  /*

  ros executor code snap:
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();  // blocks this thread, one callback at a time

  gz main thread:
  ------

  in loop:
   PreUpdate()     -> using ros spin (executor.spin()) in main thread  will block gazebo sim
   Update()       
   PostUpdate()

   
   solution:

   main gz thread                  --->(make another thread for ros)  using std::thread 
   ------                           |
                                    |    -> ros spin work with blocking but not harm gz main thread 
  in loop:                          |        as it a different thread
   PreUpdate()    -> no blocking    |    
   Update()                         |    -> std::thread -> make new thread cpp_thread having ros_executor->spin()
   PostUpdate()                     |  

  */

  // Create the ROS 2 node
  this->ros_node_ = std::make_shared<rclcpp::Node>("gz_sim_move_model_plugin");
//   this->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  this->ros_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  this->ros_executor_->add_node(this->ros_node_);

  //lamda function having ros spin
  auto ros_thread_spin = [this]() 
  { 
    this->ros_executor_->spin(); 
  };

  //make a new thread for ros spin
  this->cpp_thread = std::thread(ros_thread_spin);


  // Create the ROS 2 subscriber
  this->sub_ = this->ros_node_->create_subscription<std_msgs::msg::Float64>(
      this->rosTopicName, 
      10, // QoS history depth
      std::bind(&MoveModelDirectRos::OnRosMsg, this, std::placeholders::_1)
  );
  
  gzmsg << "ROS 2 Subscriber created for topic: " << this->rosTopicName << std::endl;
}


void MoveModelDirectRos::OnRosMsg(const std_msgs::msg::Float64::SharedPtr _msg){
  // Extract the double value from the ROS 2 Float64 message
  this->zVelocity = _msg->data;
  gzdbg << "Received new Z-velocity command from ROS 2: " << this->zVelocity << std::endl;
}

void MoveModelDirectRos::Update(const UpdateInfo &_info,
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

  // 2. Set the Z-axis linear velocity: (0, 0, zVelocity)
  const gz::math::Vector3d vel(0.0, 0.0, this->zVelocity);

  // 3. Apply the velocity command component
  _ecm.SetComponentData<components::LinearVelocityCmd>(this->targetEntity,{vel});
}

// Register the plugin with Gazebo Sim
GZ_ADD_PLUGIN(gz::sim::systems::MoveModelDirectRos,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::MoveModelDirectRos,
                    "gz::sim::systems::MoveModelDirectRos")
```

</details>

every other thing is same as previous [MoveModel Using ros-gz-birdge](movemodel_using-ros-gazebo-bridge.md)

## Build ros2 pkg

```
cd ros2_ws
colcon build --symlink-install
```

## Run ros2 pkg

```
source install/setup.bash
ros2 launch yt_tutorial_gazebo_ros move_model_direct_ros.launch.py
```

