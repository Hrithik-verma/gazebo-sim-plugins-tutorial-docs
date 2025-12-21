# Print Entity System Plugin



[PrintEntitySystemPlugin.hh](https://github.com/Hrithik-verma/gazebo-sim-plugins-tutorial/blob/main/standalone_gz_sim_plugins/print_entity_system_plugin/PrintEntitySystemPlugin.hh)

<details>
   <summary>.hh file</summary>

```c++
#ifndef SYSTEM_PLUGIN_MODEL_HH_
#define SYSTEM_PLUGIN_MODEL_HH_

//! [header]
#include <gz/sim/System.hh>  // to inherit system
#include "gz/sim/Model.hh"  // for Model component
#include "gz/sim/components/LinearVelocity.hh" // for linear velocity 
#include "gz/sim/components/LinearVelocityCmd.hh" // for LinearVelocityCmd component
#include "gz/sim/components/Name.hh"  // for Name component
#include <gz/plugin/Register.hh> //for GZ_ADD_PLUGIN_ALIAS()



namespace gz
{
namespace sim
{
namespace systems
{
  /// \brief plugin to move a model
  /// plugin interface.
  class MoveModel :
    // This class is a system.
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    // This class also implements the ISystemPreUpdate interface.
    public gz::sim::ISystemPreUpdate
  {
   public:
    MoveModel();

    ~MoveModel() override;

    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override;

    void PreUpdate(const UpdateInfo &_info,
                  EntityComponentManager &_ecm) override;

   private:
    std::string modelName;
    double zVelocity{0.0};
    Entity targetEntity{kNullEntity};
  };
}
}
}

//! [header]

#endif

```
</details>

[PrintEntitySystemPlugin.cc](https://github.com/Hrithik-verma/gazebo-sim-plugins-tutorial/blob/main/standalone_gz_sim_plugins/print_entity_system_plugin/PrintEntitySystemPlugin.cc)


<details>
   <summary>.cc file</summary>

```c++
#include "PrintEntitySystemPlugin.hh"


using namespace gz;
using namespace sim;
using namespace systems;

PrintEntitySystemPlugin::PrintEntitySystemPlugin(){
  std::cout<<"PrintEntitySystemPlugin Plugin Started!!"<<std::endl;
}
PrintEntitySystemPlugin::~PrintEntitySystemPlugin(){
  std::cout<<"PrintEntitySystemPlugin Plugin stopped!!"<<std::endl;
}
void PrintEntitySystemPlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->targetEntity = _entity;
  auto Name = _ecm.Component<components::Name>(this->targetEntity);
  this->modelName = Name->Data();

  gzmsg << "Target Entity value: " << this->targetEntity << std::endl;
  gzmsg << "Target Entity Name: " << this->modelName << std::endl;


}

void PrintEntitySystemPlugin::PreUpdate(const UpdateInfo &_info,
                          EntityComponentManager &_ecm)
{
  // Only run if the simulation is not paused
  if (_info.paused)
    return;

  
}

// Register the plugin with Gazebo Sim
GZ_ADD_PLUGIN(gz::sim::systems::PrintEntitySystemPlugin,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::PrintEntitySystemPlugin,
                    "gz::sim::systems::PrintEntitySystemPlugin")

```

</details>


lets understand code line by line


## .hh file Explanation

include header files 

``` c++
#ifndef WORLD_PLUGIN_HH
#define WORLD_PLUGIN_HH

//! [header]
#include <gz/sim/System.hh>   //to inherit system
#include "gz/sim/components/Name.hh" // for name component
#include <gz/plugin/Register.hh> //for GZ_ADD_PLUGIN_ALIAS()
```


<br>
all plugin are in gz::sim:::systems namespace
```c++
namespace gz
{
namespace sim
{
namespace systems
{
/// \brief plugin to move a model
/// plugin interface.
class PrintEntitySystemPlugin :
    // This class is a system.
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
    PrintEntitySystemPlugin();

    ~PrintEntitySystemPlugin() override;

    void Configure(const Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                EntityComponentManager &_ecm,
                EventManager &_eventMgr) override;

    void PreUpdate(const UpdateInfo &_info,
                EntityComponentManager &_ecm) override;

private:
    std::string modelName;
    Entity targetEntity;
};
}
}
}
```


PrintEntitySystemPlugin <- inherits <br>
 - ```gz::sim::System``` :  needed for all system plugins <br>
 - ````gz::sim::ISystemConfigure```: needed for accesing sdf,entity <br>
 - ```gz::sim::ISystemPreUpdate```: we are just reading data so even PostUpdate can be used but it not doing anything here. <br>

 as we know from previous understanding as we inherit ```ISystemConfigure``` we need to write defination of ```Configure()``` function <br>
 & same way ```ISystemPreUpdate``` we nee to write defination of ```PreUpdate(..)``` function


<br>
<br>

## .cc file Explanation


same gz::sim:::systems namespace are added 

```c++
#include "PrintEntitySystemPlugin.hh"


using namespace gz;
using namespace sim;
using namespace systems;

```


prints added on constructor & destructor 

```c++

PrintEntitySystemPlugin::PrintEntitySystemPlugin(){
  std::cout<<"PrintEntitySystemPlugin Plugin Started!!"<<std::endl;
}
PrintEntitySystemPlugin::~PrintEntitySystemPlugin(){
  std::cout<<"PrintEntitySystemPlugin Plugin stopped!!"<<std::endl;
}

```
<br>

main code is

```_entity``` is the entity it attached to ```world,model,light``` etc 


```c++
void PrintEntitySystemPlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
```


![enity_world](assets/images/entity_world_point.png)

```xml
<world>

    <plugin>  </<plugin>>

</world>
```
so here ```_entity``` is world entity

<br>
![model_entity](assets/images/entity_model_point.png)

```xml
<world>

  <model>

    <plugin>  </<plugin>
  </model>

</world>
```

here it ```_entity``` is model entity



<br>
means find the value of the **Name Component** attached to **targetEntity** 

```c++
auto Name = _ecm.Component<components::Name>(this->targetEntity);
this->modelName = Name->Data();
```

![Component](assets/images/component_fn.png)


![name_component](assets/images/name_component.png)

```Name->Data()```  why?  ```->Data()``` because its comes from ```component class``` <br>
return type of ```_ecm.Component<..>``` is ```component```

[gz::sim::components::component in gz doc](https://gazebosim.org/api/sim/9/classgz_1_1sim_1_1components_1_1Component.html)

![data](assets/images/Data.png)





need register the plugin with all the class it inhertited 
```c++
// Register the plugin with Gazebo Sim
GZ_ADD_PLUGIN(gz::sim::systems::PrintEntitySystemPlugin,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)
```

alias
```c++
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::PrintEntitySystemPlugin,
                    "gz::sim::systems::PrintEntitySystemPlugin")
```

Important:
only name=```gz::sim::systems::PrintEntitySystemPlugin``` allowed for plugin you can't put any other name because it how we define on the alis ```GZ_ADD_PLUGIN_ALIAS(...)```

so 

```
<!-- custom plugin attach to world-->
<plugin
    filename="PrintEntitySystemPlugin"
    name="gz::sim::systems::PrintEntitySystemPlugin">
</plugin>

```

```filename= PrintEntitySystemPlugin``` 

 or 

```filename= libPrintEntitySystemPlugin.so``` 

so no other name is allowed

comes from CMakeList.txt

```cmake
add_library(PrintEntitySystemPlugin SHARED PrintEntitySystemPlugin.cc)
set_property(TARGET PrintEntitySystemPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(PrintEntitySystemPlugin
  PUBLIC gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PUBLIC gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})
```