# dual_panda_demo
dual_panda_demo ros2


# prepare
```
wget http://fishros.com/install -O fishros && . fishros
install ros-humble-desktop
install rosdepc
sudo apt install -y ros-humble-moveit*
sudo apt update && rosdepc install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

```
colcon build
. install/setup.bash
ros2 launch panda_description demo.launch.py
```