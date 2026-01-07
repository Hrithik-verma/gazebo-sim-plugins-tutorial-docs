# Common Things In All ros2_ws Plugins

<iframe
  width="960"
  height="540"
  src="https://www.youtube.com/embed/TU77ek6_bA0?si=XCMtr3D7qakm01Cs"
  title="Gazebo Sim Structure Plugin Perspective"
  frameborder="0"
  allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
  allowfullscreen>
</iframe>


## Structure

```text
├── ros2_ws/
│   ├── src/
│   │   ├── yt_tutorial_gazebo_ros/      # ROS 2 Gazebo launch & integration
|   |   |      └──config
|   |   |      └──launch
|   |   |      └──worlds
|   |   |      └──CmakeLists.txt
|   |   |      └──package.xml
│   │   └── tutorial_gazebo_plugins/     # ROS 2 Gazebo plugins
|   |          └──include                # Plugin .hh file
|   |          └──src                    # Plugin .cc file
│   └── ...                               # ROS 2 build, install, log folders
│
```


## ROS2 Gazebo Sim Plugin 
   There are 2 ways:

   1. Using ros-gz-bridge
   2. Using Direct rclcpp in gazebo sim plugin

<br>

## Naming Convention In The Tutorial 

```<Name>``` - pure gazebo sim plugin<br>
```<Name>TopicWay```  - using ros2 gz bridge to communicate<br>
```<Name>DirectRos``` - using ros2 rclcpp directly in gazebo sim plugin<br>


<br>
<br>

## launch file

- ```GZ_SIM_SYSTEM_PLUGIN_PATH``` variable is added in the launch file
- gz sim launch using ```ros_gz_sim``` pkg & ```'gz_sim.launch.py``` launch file

```python
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

```


## Build & Run ros2 pkg

```
cd ros2_ws
colcon build --symlink-install
```

```
source install/setup.bash
ros2 launch yt_tutorial_gazebo_ros <name>.launch.py
```