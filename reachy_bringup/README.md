# About LifeCycle Node

Here is the [state machine](https://design.ros2.org/articles/node_lifecycle.html
)

ROS launch file [example](https://github.com/ros2/launch_ros/blob/6370c127868a5056a8a02c9412c59bebdaefcf81/launch_ros/examples/lifecycle_pub_sub_launch.py#L59)

Python node file [example](https://github.com/ros2/demos/blob/rolling/lifecycle_py/lifecycle_py/talker.py)


## launch parameteres

parameter | default | values | description
--- | --- | --- | ---
`fake` | `false` | `true`, `false` | Start on fake_reachy mode with this launch file.
`gazebo` | `false` | `true`, `false` | Start a fake_hardware with gazebo as simulation tool.
`start_sdk_server` | `false` | `true`, `false` | Start sdk_server along with reachy nodes with this launch file.
`start_rviz` | `false` | `true`, `false`, `*get_rviz_conf_choices()` | Start RViz2 automatically with this launch file.
`foxglove` | `false` | `true`, `false` | Start FoxGlove bridge with this launch file.
`controllers` | `default` | `default`, `trajectory` | Controller Mode
`ethercat_master` | `true` | `true`, `false` | Start EtherCAT server.

You can set any of these parameters in the launch file or in the command line. For example:
```bash
ros2 launch reachy_bringup reachy.launch.py fake:=true gazebo:=true
```