# airaflot_waterdrone_ros2
ROS2 packages for Airaflot Water Drone

## Requirements
* ROS2

## install
Clone packages to your ROS2 workspase:
```bash
cd ros2_ws/src
git clone https://github.com/LoSk-p/airaflot_waterdrone_ros2.git
mv airaflot_waterdrone_ros2/airaflot_waterdrone airaflot_waterdrone
mv airaflot_waterdrone_ros2/airaflot_msgs airaflot_msgs
rm -r airaflot_waterdrone_ros2
```

Build packages
```bash
cd ros2_ws
colcon build
source install/setup.bash
```