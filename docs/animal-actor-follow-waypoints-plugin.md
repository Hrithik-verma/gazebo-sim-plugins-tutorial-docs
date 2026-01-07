# Animal Actor Follow WayPoints Plugin


<iframe
  width="960"
  height="540"
  src="https://www.youtube.com/embed/sRh92U6co9U?si=hXFPTbJOfC5yOn87"
  title="Gazebo Sim Structure Plugin Perspective"
  frameborder="0"
  allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
  allowfullscreen>
</iframe>



In these tutoiral we are going to used a animated dog as a [gazebo actor](https://gazebosim.org/docs/latest/actors/){target=_blank} & make it follow waypoints & animate walking dog.

![dog moving in place](assets/images/dog_moving_in_place.gif)


[Download Blender Models](https://drive.google.com/drive/folders/16JOQmX5YZCFksrcwPnlo7EStDOKoxt0w?usp=sharing){target=_blank}
<br>

Good Youtube Videos on Modelling & Animal Animation:
[link1](https://www.youtube.com/watch?v=CNd7nqUgnj0){target=_blank}, [link2](https://www.youtube.com/watch?v=zB6Z-_k5ZsI&list=LL&index=3){target=_blank}

## Code Explain

-----
<iframe
  src="https://docs.google.com/presentation/d/1YtqX-2POK3Z39q8GK21g0kbrMlOyHcRIcYPCjFwBEJ4/edit?usp=sharing"
  width="960"
  height="569"
  frameborder="0"
  allowfullscreen
></iframe>



reakted api: [gazebo components](https://gazebosim.org/api/sim/9/namespacegz_1_1sim_1_1components.html), [Actor Wrapper Class](https://gazebosim.org/api/sim/9/classgz_1_1sim_1_1Actor.html){target=_blank}, [Actor.cc code in github](https://github.com/gazebosim/gz-sim/blob/gz-sim10/src/Actor.cc){target=_blank}

![code map](assets/images/code_mapping_actor.png)


## 1) How the actor’s world pose is computed


```c++
  auto originOpt = actor.Pose(_ecm);
  if (!originOpt)
  {
    if (doPrint)
      gzdbg << "[ActorWaypointSystem] Waiting for actor origin Pose component...\n";
    return;
  }
  const math::Pose3d origin = *originOpt;

  // TrajectoryPose
  math::Pose3d traj = math::Pose3d::Zero;
  if (auto trajComp = _ecm.Component<components::TrajectoryPose>(this->actorEntity))
    traj = trajComp->Data();

  // Derive current actor world pose from origin and trajectory
  math::Pose3d world = origin * traj;
```



Instead of:

world = origin ∘ traj

Read it as:

**“Start at the origin pose, then apply the trajectory pose on top of it.”**

In words:

- The actor has an **origin pose** that already lives in the world.
- The actor also has a **trajectory pose** that describes motion relative to that origin.
- To know where the actor actually is in the world:
  1. Take the origin pose
  2. Apply the trajectory pose’s translation and rotation relative to that origin

Result:
- You get the **final world position and orientation** of the actor.

Equivalent procedural explanation:

- Rotate the trajectory position by the origin rotation
- Add that rotated position to the origin position
- Multiply the origin rotation by the trajectory rotation
---



## 2) Direction toward a waypoint

```c++
//current waypoint target
const math::Pose3d targetWorld = this->waypoints[this->currentWaypoint];

// Direction in world frame
math::Vector3d dir = targetWorld.Pos() - world.Pos();
dir.Z(0);                       // ignore Z for motion 
const double dist = dir.Length();
```

![directional vector](assets/images/direction_vector.png)

Instead of vector subtraction notation, read it as:

**“Direction = target position minus current position.”**

In words:

- Look at where the actor is right now
- Look at where the waypoint is
- Draw an arrow from the actor to the waypoint
- That arrow is the direction vector

Then:

- Set the vertical (Z) part of that arrow to zero  
  → movement is only on the ground plane

---



## 4) Distance To WayPoint
```c++

// Advance waypoint
if (dist < 0.05)
{
this->currentWaypoint = (this->currentWaypoint + 1) % this->waypoints.size();
if (doPrint)
    gzdbg << "[ActorWaypointSystem] Reached waypoint, advancing to wp="
        << this->currentWaypoint << "\n";
return;
}

if (dist > 1e-9)
dir.Normalize();

```


Instead of:

dist = ||dir||

Read it as:

**“Distance is the length of the direction arrow.”**

In words:

- Measure how long the arrow from actor to waypoint is
- That number tells you how far away the waypoint is

If the distance is very small (less than 5 cm):

- Consider the waypoint reached
- Switch to the next waypoint

---

Instead of:

dir̂ = dir / ||dir||

Read it as:

**“Scale the direction arrow so its length becomes exactly 1.”**

In words:

- Keep the direction the same
- Change the arrow length to one unit
- This makes it usable for movement at a constant speed

---

## 6) How far the actor moves this update

```c++
// Step in world
  world.Pos() += dir * step;

  // Keep a constant height (or waypoint height) + offset
  world.Pos().Z() = targetWorld.Pos().Z() + this->zOffset;

  // Face direction
  const double yaw = atan2(dir.Y(), dir.X());
  world.Rot() = math::Quaterniond(0, 0, yaw + this->yawOffset);

```

![step](assets/images/step.png)

Instead of:

step = speed × dt

Read it as:

**“Distance moved this frame equals speed multiplied by elapsed time.”**

In words:

- Speed is meters per second
- `dt` is seconds since the last update
- Multiply them to get meters moved this update

---

Instead of writing a Z equation, read it as:

**“Ignore vertical movement and snap the actor’s height to the waypoint height plus an offset.”**

In words:

- Z is not integrated over time
- Every update:
  - Actor Z = waypoint Z + configured offset

This guarantees stable vertical placement.

---

Instead of:

yaw = atan2(dy, dx)

Read it as:

**“Compute the angle that points in the same direction as the movement arrow.”**

In words:

- Look at the direction arrow in the X–Y plane
- Find the angle it makes relative to the X axis
- That angle becomes the actor’s yaw

Then:

- Add a yaw offset if configured
- Set roll and pitch to zero

---


## 7) Converting a world pose back into trajectory space

```c++

  // Convert desired world pose -> trajectory (relative to origin)
  // world = origin * traj  =>  traj = origin^-1 * world
  const math::Pose3d trajCmd = origin.Inverse() * world;

  // Set manual trajectory pose
  actor.SetTrajectoryPose(_ecm, trajCmd);

  // Force notify that this component changed (this is the key)
  _ecm.SetChanged(this->actorEntity,
                  gz::sim::components::TrajectoryPose::typeId,
                  gz::sim::ComponentState::OneTimeChange);
```


![world to local](assets/images/world_2_local.png)

Instead of:

trajCmd = origin⁻¹ ∘ world

Read it as:

**“Remove the origin pose from the world pose to get a trajectory-relative pose.”**

In words:

- You already know where the actor should be in the world.
- But the system only accepts trajectory poses (relative to origin).
- So you:
  1. Undo the origin’s rotation and translation
  2. Express the world pose relative to the origin frame

This answers the question:
> “What trajectory pose would produce this world pose if applied on top of the origin?”

---

## 8) Full update logic in plain English

Each simulation update does the following:

1. Read the actor’s origin pose
2. Read the actor’s trajectory pose
3. Combine them to get the current world pose
4. Compute a flat (X–Y) direction toward the current waypoint
5. Measure distance to the waypoint
6. If close enough:
   - Advance to the next waypoint
7. Otherwise:
   - Move forward by `speed × dt`
   - Set height to waypoint Z + offset
   - Rotate the actor to face the movement direction
8. Convert the resulting world pose back into a trajectory pose
9. Write that trajectory pose back to the ECS

---


## Plugin


<details>
   <summary>.sdf file plugin path</summary>
  
```xml
<include>
  <name>dog_walk</name>
  <pose>0 0 0.01 0 0 0</pose>
  <uri>model://dog_walk</uri>
  
  <plugin
    filename="AnimalActorFollowWaypoints"
    name="gz::sim::AnimalActorFollowWaypoints">

    <waypoint>0 0 0 0 0 0</waypoint>
    <waypoint>0 8 0 0 0 0</waypoint>
    <waypoint>8 8 0 0 0 1.57</waypoint>
    <waypoint>0 8 0 0 0 3.14</waypoint>
    
    <speed>0.1</speed>
    <yaw_offset>0.0</yaw_offset>   
    <z_offset>0.01</z_offset> 

  </plugin>
</include>

```

</details>


<details>
   <summary>.hh file</summary>

```c++
#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components.hh>
#include <gz/sim/Actor.hh>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh> //for GZ_ADD_PLUGIN_ALIAS()

#include <chrono>
#include <cmath>
#include <vector>

namespace gz::sim
{

class AnimalActorFollowWaypoints :
  public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
public:
  void Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &) override;

  void PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm) override;

private:
  Entity actorEntity{kNullEntity};

  std::vector<math::Pose3d> waypoints;
  size_t currentWaypoint{0};
  std::chrono::steady_clock::time_point lastWallPrint{};
  double yawOffset;
  double zOffset{0.0};


  double speed{1.0}; // meters / second
};

}
```

</details>



<details>
   <summary>.cc file</summary>

```c++

#include "tutorial_gazebo_plugins/AnimalActorFollowWaypoints.hh"

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
void AnimalActorFollowWaypoints::Configure(
  const Entity &,
  const std::shared_ptr<const sdf::Element> &_sdf,
  EntityComponentManager &_ecm,
  EventManager &)
{
  // Find actor
  _ecm.Each<components::Actor>(
    [&](const Entity &_ent, const components::Actor *) -> bool
    {
      this->actorEntity = _ent;
      return false;
    });

  if (this->actorEntity == kNullEntity)
  {
    gzerr << "[AnimalActorFollowWaypoints] No Actor found\n";
    return;
  }

   if (!_ecm.Component<components::Actor>(this->actorEntity))
  {
    gzerr << "[AnimalActorFollowWaypoints] Plugin attached to a non-actor entity\n";
    return;
  }

  // Read waypoints
  for (auto elem = _sdf->FindElement("waypoint");
       elem; elem = elem->GetNextElement("waypoint"))
  {
    this->waypoints.push_back(elem->Get<math::Pose3d>());
  }

  if (_sdf->HasElement("speed"))
    this->speed = _sdf->Get<double>("speed");

  if (this->waypoints.empty())
    gzerr << "[AnimalActorFollowWaypoints] No waypoints provided\n";
  
  if (_sdf->HasElement("yaw_offset"))
    this->yawOffset = _sdf->Get<double>("yaw_offset");

  if (_sdf->HasElement("z_offset"))
    this->zOffset = _sdf->Get<double>("z_offset");

  gzdbg << "[AnimalActorFollowWaypoints] Configure: actorEntity=" << this->actorEntity
      << " waypoints=" << this->waypoints.size()
      << " speed=" << this->speed << "\n";
}

void AnimalActorFollowWaypoints::PreUpdate(
  const UpdateInfo &_info,
  EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (this->waypoints.empty())
    return;

  // Throttle debug prints (~1 Hz)
  const auto now = std::chrono::steady_clock::now();
  const bool doPrint =
    (this->lastWallPrint.time_since_epoch().count() == 0) ||
    (now - this->lastWallPrint > std::chrono::seconds(1));
  if (doPrint)
    this->lastWallPrint = now;


  //
  gz::sim::Actor actor(this->actorEntity);
  if (!actor.Valid(_ecm))
  {
    if (doPrint)
      gzerr << "[AnimalActorFollowWaypoints] PreUpdate: entity " << this->actorEntity
            << " not a valid Actor\n";
    return;
  }

  // Origin pose (trajectory reference)
  auto originOpt = actor.Pose(_ecm);
  if (!originOpt)
  {
    if (doPrint)
      gzdbg << "[AnimalActorFollowWaypoints] Waiting for actor origin Pose component...\n";
    return;
  }
  const math::Pose3d origin = *originOpt;

  // TrajectoryPose
  math::Pose3d traj = math::Pose3d::Zero;
  if (auto trajComp = _ecm.Component<components::TrajectoryPose>(this->actorEntity))
    traj = trajComp->Data();

  // Derive current actor world pose from origin and trajectory
  math::Pose3d world = origin * traj;

  //current waypoint target
  const math::Pose3d targetWorld = this->waypoints[this->currentWaypoint];

  // Direction in world frame
  math::Vector3d dir = targetWorld.Pos() - world.Pos();
  dir.Z(0);                       // ignore Z for motion 
  const double dist = dir.Length();

  if (doPrint)
  {
    gzdbg << "[AnimalActorFollowWaypoints] Tick entity=" << this->actorEntity
          << " wp=" << this->currentWaypoint << "/" << this->waypoints.size()
          << " dt=" << std::chrono::duration<double>(_info.dt).count()
          << " origin.pos=" << origin.Pos()
          << " world.pos=" << world.Pos()
          << " traj.pos=" << traj.Pos()
          << " target.pos=" << targetWorld.Pos()
          << " dist=" << dist
          << "\n";
  }

  // Advance waypoint
  if (dist < 0.05)
  {
    this->currentWaypoint = (this->currentWaypoint + 1) % this->waypoints.size();
    if (doPrint)
      gzdbg << "[AnimalActorFollowWaypoints] Reached waypoint, advancing to wp="
            << this->currentWaypoint << "\n";
    return;
  }

  if (dist > 1e-9)
    dir.Normalize();

  //this->speed * dt means  Distance covered in one physics frame
  const double dt = std::chrono::duration<double>(_info.dt).count();
  const double step = this->speed * dt;

  // Step in world
  world.Pos() += dir * step;

  // Keep a constant height (or waypoint height) + offset
  world.Pos().Z() = targetWorld.Pos().Z() + this->zOffset;

  // Face direction
  const double yaw = atan2(dir.Y(), dir.X());
  world.Rot() = math::Quaterniond(0, 0, yaw + this->yawOffset);

  // Convert desired world pose -> trajectory (relative to origin)
  // world = origin * traj  =>  traj = origin^-1 * world
  const math::Pose3d trajCmd = origin.Inverse() * world;

  // Set manual trajectory pose
  actor.SetTrajectoryPose(_ecm, trajCmd);

  // Force notify that this component changed (this is the key)
  _ecm.SetChanged(this->actorEntity,
                  gz::sim::components::TrajectoryPose::typeId,
                  gz::sim::ComponentState::OneTimeChange);


  if (doPrint)
  {
    gzdbg << "[AnimalActorFollowWaypoints] SetTrajectoryPose traj(before)=" << traj.Pos()
          << " trajCmd(after)=" << trajCmd.Pos()
          << " world(after)=" << world.Pos()
          << " yaw=" << yaw
          << "\n";
  }
}
// Register the plugin with Gazebo Sim
GZ_ADD_PLUGIN(gz::sim::AnimalActorFollowWaypoints,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::AnimalActorFollowWaypoints,
                    "gz::sim::systems::AnimalActorFollowWaypoints")
```
</details>


```GZ_SIM_RESOURCE_PATH``` added so that gazebo sim can find model path

<details>
   <summary>launch file</summary>


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

    ros_pkg_path = FindPackageShare('yt_tutorial_gazebo_ros')

    set_plugin_path = SetEnvironmentVariable(
    'GZ_SIM_SYSTEM_PLUGIN_PATH',
    PathJoinSubstitution([plugin_pkg_path, 'plugins'])
    )

    # to load models
    set_model_path = SetEnvironmentVariable(
    'GZ_SIM_RESOURCE_PATH',
    PathJoinSubstitution([ros_pkg_path, 'models'])
    )

    world_path = os.path.join(
        get_package_share_directory('yt_tutorial_gazebo_ros'),
        'worlds',
        'animal_actor_follow_waypoints.sdf',
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




    return LaunchDescription([set_plugin_path,set_model_path, gz_sim])

```

<details>
