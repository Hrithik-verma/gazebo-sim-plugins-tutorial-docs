## Common In All System Plugin

<iframe
  width="960"
  height="540"
  src="https://www.youtube.com/embed/AZ9E101Ven4?si=pOsolaYBxKR_Hyqe"
  title="Gazebo Sim Structure Plugin Perspective"
  frameborder="0"
  allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
  allowfullscreen>
</iframe>

1) Go to a standalone plugin folder:
```bash
cd <path-to>/gazebo-sim-plugins-tutorial/standalone_gz_sim_plugins/<plugin_name>
```

2) Build:
```bash
mkdir build && cd build
cmake ..
make
cd ..
```

3) Export plugin path (so Gazebo Sim can find it):
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd)/build
```

4) Launch Gazebo Sim with an SDF:
```bash
gazebo sim -v 4 <sdf_file_path>.sdf
```



Notes:<br>
- "-v 4" prints debug logs (useful while developing)<br>
- If Gazebo can’t find the plugin, re-check GZ_SIM_SYSTEM_PLUGIN_PATH and that build succeeded.<br>

<br>


## Code related

###  ```_entity``` of Configure()

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


![model_entity](assets/images/entity_model_point.png)

here it ```_entity``` is model entity


### Plugin in XML

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

