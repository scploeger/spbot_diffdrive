# spbot_diffdrive

Custom ROS 2 hardware interface for differential drive robots, based on the [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos).

## Repository Structure

- **bringup/**: Launch files and runtime configurations.
- **description/**: Robot URDF, RViz configs, and meshes.
- **doc/**: Documentation and resources.
- **hardware/**: Custom hardware interface implementation.
- **test/**: Testing scripts and configs.
- **CMakeLists.txt**: Package build instructions.
- **package.xml**: Dependencies and package details.

## Quick Start

### Clone and Build

Clone into your ROS 2 workspace:

```bash
cd ~/your_ros2_ws/src
git clone https://github.com/scploeger/spbot_diffdrive.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

### Include in your Robot Description

Add the hardware plugin to your `ros2_control.xacro`:

```xml
<plugin>spbot_diffdrive/SPBotDiffDriveHardware</plugin>
```

### Launch

Source workspace and launch:

```bash
source install/setup.bash
ros2 launch spbot_diffdrive bringup.launch.py
```

## To-Do

- [ ] Tune PID parameters.
- [ ] Improve hardware interface documentation.
- [ ] Add more robust testing.

## Acknowledgments

Built on [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos).

