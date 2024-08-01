# dual_panda_demo
dual_panda_demo ros2


# prepare
```
wget http://fishros.com/install -O fishros && . fishros
install ros-humble-desktop
install rosdepc

git clone https://github.com/wawatt/dual_panda_demo.git
cd src
sudo apt update && rosdepc install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

```
colcon build
. install/setup.bash

ros2 launch dual_panda_description demo.launch.py
ros2 launch dual_panda_moveit_config demo_my.launch.py

ros2 launch moveit_setup_assistant setup_assistant.launch.py
```