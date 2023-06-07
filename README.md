# Base robot controller for ROS2 and Flatland 

Robot controller with commented code examples to serve as a base to Flatland projects using ROS2.

Clone the repository inside `workspace/src`:
```
git clone https://github.com/FilipeAlmeidaFEUP/ros2_flatland_robot_controller.git
```

Build inside the `workspace` folder:
```
rosdep install -i --from-path src --rosdistro humble -y
colcon build
source install/setup.bash
```

Run the package:
```
ros2 launch serp_teleop serp_teleop.launch.py
```