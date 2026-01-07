# Joint Control System Plugin

<iframe
  width="960"
  height="540"
  src="https://www.youtube.com/embed/AA70Fp-3KhY?si=DZoSSBzeI9WY6cFG"
  title="Gazebo Sim Structure Plugin Perspective"
  frameborder="0"
  allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
  allowfullscreen>
</iframe>


We are goining to write a joint control system plugin to control one joint using 3 methods.


![joint_1](assets/images/joint_1.png)


we will control ```joint_1``` 

using 3 methods:
1. Velocity<br>
2. Force<br>
3. Position<br>


## 1. Find the Joint Entity

uing ```EntityByComponents()```

![entity by compoent](assets/images/entity_by_component.png)

[EntityComponentManager doc](https://gazebosim.org/api/sim/9/classgz_1_1sim_1_1EntityComponentManager.html)

find such a joint entity which has all of the followings components: <br>
&nbsp;  &nbsp; &nbsp;     parent entity is ***model component*** to which plugin is attached, <br>
&nbsp; &nbsp; &nbsp;      has a ***name component*** of value ```joint_1``` <br>
&nbsp; &nbsp;  &nbsp;     ***joint compenent*** is attach on it <br>

[reference code](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/Model.cc#L132C1-L133C1)

```c++
  this->jointEntity = _ecm.EntityByComponents(
      components::ParentEntity(_entity),
      components::Name(jointName),
      components::Joint());
```


## 2. Make Sure Plugin Attached To Model Only

```xml
<model>

....
  <plugin>  .... </plugin>

</model>
```

![entity has component](assets/images/entityhascomponent.png)

```c++

if(!_ecm.EntityHasComponentType(_entity, components::Model::typeId)){
    gzerr << "JointControl plugin must be added on <model>...</model> scope only" << std::endl;
    return;
}
```

## 3. Components To Control Joint

List Of Component To Control Joint

[components doc](https://gazebosim.org/api/sim/9/namespacegz_1_1sim_1_1components.html)

![joint](assets/images/joint_componets.png)


...``Cmd``: means component to command or write on it


but position do have any direct ```PositionCmd```


using 3 methods:

### 1. Velocity

using ```components::JointVelocityCmd``` <br>

unit: ```radian/sec``` for revolute joint <br>

data type: ```vector<double>``` <br>


```c++

if (this->mode == Mode::Velocity)
{
    //if component is not preset than create it
    if (!_ecm.EntityHasComponentType(this->jointEntity, components::JointVelocityCmd::typeId))
        _ecm.CreateComponent(this->jointEntity, components::JointVelocityCmd({0.0}));

    _ecm.SetComponentData<components::JointVelocityCmd>(this->jointEntity,
                                                            {this->cmdVelocity});
}

```



### 2. Force: 
using ```components::JointForceCmd```


unit: ```Nm``` for revolute joint <br>

data type: ```vector<double>``` <br>

```c++
else if (this->mode == Mode::Force){

    //if component is not preset than create it
    if (!_ecm.EntityHasComponentType(this->jointEntity, components::JointForceCmd::typeId))
        _ecm.CreateComponent(this->jointEntity, components::JointForceCmd({0.0}));

    _ecm.SetComponentData<components::JointForceCmd>(this->jointEntity,
                                                            {this->cmdTorque});
}

```


### 3. Position

as there is no direct position cmd so will use ```PD control```. Its a feedback loop control system very commandely used.<br>
good [PID Simulator](https://www.luisllamas.es/en/pid-controller-simulator/) for understand PID.

![pd control](assets/images/pd_control.png)


will be using ```components::JointForceCmd``` to control torque using PD control




```c++
double error = this->targetPos - currentPos;
double torque = this->pGain * error - this->dGain * currentVel;
```

<br>

To Make Sure we are torque is within limit will set a min/max limit
also stop applying torque once target is reached 

```c++
double maxT = this->maxTorque;        
double clamp_torque = std::clamp(torque, -maxT, maxT);

//once reached within the tolerance will stop applying torque
if(error >= -pTolerance && error <= pTolerance){
    clamp_torque = 0.0;
}
```

<br>



On time a fixed interval we want to switch between pos ```A <-> B``` <br>
will use change target from A to B & vise verse base on time

```c++
// on give time interval will switch target position
auto elapsed = _info.simTime - this->lastSwitchTime;
if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >= this->waitTime)
{
    this->targetPos = (this->targetPos == this->posA) ? this->posB : this->posA;
    this->lastSwitchTime = _info.simTime;
    }
```



## Full Plugin

<details>
   <summary>.sdf plugin part</summary>

```xml
<model>
....

  <plugin filename="JointControl" name="gz::sim::JointControl">
    <joint_name>joint_1</joint_name>

    <!-- posssible mode: 
          velocity
          force
          position
    -->
    <mode>velocity</mode>
    
    <!-- velocity mode (rad/sec)-->
    <cmd_vel>1.0</cmd_vel>

    <!-- force mode (Nm)-->
    <cmd_torque>15.0</cmd_torque>

    <!--position mode (radian)-->
    <pos_a>0.523599</pos_a> <!-- 30 deg-->
    <pos_b>2.0944</pos_b> <!-- 120 deg-->
    <wait_time>6</wait_time>
    <!-- PD control-->
    <p_gain>10.0</p_gain>
    <d_gain>1.0</d_gain>
    <max_torque>10.0</max_torque>
    <pos_tolerence>0.1</pos_tolerence>
  </plugin>

</model>


```

</details>



<details>
   <summary>  .hh file</summary>


```c++
#ifndef JOINT_CONTROL_PLUGIN_HH_
#define JOINT_CONTROL_PLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/ParentEntity.hh>

#include <vector>
#include <string>


enum class Mode
{
  Force,
  Velocity,
  Position
};

namespace gz
{
namespace sim
{
class JointControl : public System, public ISystemConfigure,public ISystemPreUpdate
{
public:
  // Constructor
  JointControl() = default;

  void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override;

  // ISystemPreUpdate method
  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;

private:
  // Function to find the light entity by name
  std::string jointName; //joint name
  Entity jointEntity; //joint entity

  Mode mode{Mode::Velocity};
  // Velocity mode
  double cmdVelocity{0.0}; // for Velcoity Mode
  double cmdTorque{0.0}; // for Force Mode
  // Position mode: oscillate between 2 targets // Position Mode
  double posA{0.0};
  double posB{0.0};
  int waitTime{1}; //time to wait from pos A->B
  double targetPos{0.0}; // current target pose
  bool hasInitTime{false}; // for init time
  double pGain{50.0};  // P 
  double dGain{1.0};   // D
  double maxTorque{10.0}; // max torque
  double pTolerance{0.01}; // position tolerance 
  std::chrono::steady_clock::duration lastSwitchTime{0}; //last time when switching happended

};
}  // namespace sim
}  // namespace gz

#endif

```

</details>



<details>
   <summary>  .cc file</summary>

```c++
#include "tutorial_gazebo_plugins/JointControl.hh"

#include <gz/plugin/Register.hh>


#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Name.hh>

#include <algorithm>  // for std::clamp


using namespace gz;
using namespace gz::sim;

void JointControl::Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr)
{
  if (!_sdf->HasElement("joint_name"))
  {
    gzerr << "JointControl plugin requires <joint_name>" << std::endl;
    return;
  }

  this->jointName = _sdf->Get<std::string>("joint_name");

  if(!_ecm.EntityHasComponentType(_entity, components::Model::typeId)){
    gzerr << "JointControl plugin must be added on <model>...</model> scope only" << std::endl;
    return;
  }


  if (_sdf->HasElement("mode"))
  {
    const auto modeStr = _sdf->Get<std::string>("mode");
    if (modeStr == "velocity" || modeStr == "vel"){
      this->mode = Mode::Velocity;
      gzmsg << "Mode: Velocity" << std::endl;
    }
    
    else if(modeStr == "force" || modeStr == "f"){
      this->mode = Mode::Force;
      gzmsg << "Mode: Force" << std::endl;
    }
    else if (modeStr == "position" || modeStr == "pos"){
      this->mode = Mode::Position;
      gzmsg << "Mode: Position" << std::endl;
    }
    else
      gzerr << "JointControl: unknown <mode> '" << modeStr
            << "'. Using 'velocity'." << std::endl;
  }

  if (_sdf->HasElement("cmd_vel"))
    this->cmdVelocity = _sdf->Get<double>("cmd_vel");

  if (_sdf->HasElement("cmd_torque"))
    this->cmdTorque = _sdf->Get<double>("cmd_torque");

  if (_sdf->HasElement("pos_a"))
    this->posA = _sdf->Get<double>("pos_a");
  if (_sdf->HasElement("pos_b"))
    this->posB = _sdf->Get<double>("pos_b");

  if (_sdf->HasElement("p_gain"))
    this->pGain = _sdf->Get<double>("p_gain");

  if (_sdf->HasElement("d_gain"))
    this->dGain = _sdf->Get<double>("d_gain");

  if (_sdf->HasElement("max_torque"))
    this->maxTorque = _sdf->Get<double>("max_torque");
  
  if (_sdf->HasElement("pos_tolerence"))
    this->pTolerance = _sdf->Get<double>("pos_tolerence");

  if (_sdf->HasElement("wait_time"))
    this->waitTime = _sdf->Get<int>("wait_time");


  /* find such a joint entity which has all of the followings components:
        parent entity is model entity which plugin is attached to,
        has a name component of joint name 
        joint compenent is attach on it
        reference code: https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/Model.cc#L132C1-L133C1
  */
  this->jointEntity = _ecm.EntityByComponents(
      components::ParentEntity(_entity),
      components::Name(jointName),
      components::Joint());

  this->targetPos = this->posA;   // start by going to posA
  this->lastSwitchTime = std::chrono::steady_clock::duration::zero(); // or set on first PreUpdate
}

// ---------------------------------------------------------------------
void JointControl::PreUpdate(const UpdateInfo &_info,
                             EntityComponentManager &_ecm)
{
   if (_info.paused)
    return;

   if(!this->jointEntity)
     return;


  if (this->mode == Mode::Velocity)
  {
    if (!_ecm.EntityHasComponentType(this->jointEntity, components::JointVelocityCmd::typeId))
      _ecm.CreateComponent(this->jointEntity, components::JointVelocityCmd({0.0}));

    _ecm.SetComponentData<components::JointVelocityCmd>(this->jointEntity,
                                                          {this->cmdVelocity});
  }

  else if (this->mode == Mode::Force){
    
    //if component is not preset than create it
    if (!_ecm.EntityHasComponentType(this->jointEntity, components::JointForceCmd::typeId))
      _ecm.CreateComponent(this->jointEntity, components::JointForceCmd({0.0}));

    _ecm.SetComponentData<components::JointForceCmd>(this->jointEntity,
                                                          {this->cmdTorque});
  }

    
  else if (this->mode == Mode::Position){

    if (!this->hasInitTime)
    {
      this->lastSwitchTime = _info.simTime;
      this->hasInitTime = true;
    }

    
    // on give time interval will switch target position
    auto elapsed = _info.simTime - this->lastSwitchTime;
    if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >= this->waitTime)
    {
      this->targetPos = (this->targetPos == this->posA) ? this->posB : this->posA;
      this->lastSwitchTime = _info.simTime;
    }
    
    //read current position
    auto posComp = _ecm.Component<components::JointPosition>(this->jointEntity);
    if (!posComp)
    {
      _ecm.CreateComponent(this->jointEntity, components::JointPosition());
      gzdbg << "JointPosition component missing\n";
      return;
    }
    if (posComp->Data().empty())
    {
      gzdbg << "JointPosition data empty\n";
      return;
    }
    double currentPos = posComp->Data()[0];

    // Current velocity
    auto velComp = _ecm.Component<components::JointVelocity>(this->jointEntity);
    if (!velComp)
    {
      _ecm.CreateComponent(this->jointEntity, components::JointVelocity());
      gzdbg << "JointPosition component missing\n";
      return;
    }
    double currentVel = velComp->Data()[0];

    // PD control
    double error = this->targetPos - currentPos;
    double torque = this->pGain * error - this->dGain * currentVel;

    double maxT = this->maxTorque;        
    double clamp_torque = std::clamp(torque, -maxT, maxT);

    //once reached within the tolerance will stop applying torque
    if(error >= -pTolerance && error <= pTolerance){
      clamp_torque = 0.0;
    }

    // ensure force cmd exists (if you didn’t create in Configure)
    if (!_ecm.EntityHasComponentType(this->jointEntity, components::JointForceCmd::typeId))
      _ecm.CreateComponent(this->jointEntity, components::JointForceCmd({0.0}));

    _ecm.SetComponentData<components::JointForceCmd>(this->jointEntity, {clamp_torque});
    gzmsg << "pos target=" << this->targetPos << " cur=" << currentPos
          << " torque=" << clamp_torque << std::endl;

  }
  
}

// Register the plugin
GZ_ADD_PLUGIN(
    JointControl,
    gz::sim::System,
    JointControl::ISystemConfigure,
    JointControl::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(JointControl, "gz::sim::JointControl")

```

</details>
