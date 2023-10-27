# Twist Ros2_to_GZ_SIM v1.0
---
### Copy code into some directory (probably in your ros2 package)
```bash
cp ~/your_Github_Dir/gazeboMovePlugin ~/your_Ros2_pkg/gazeboMovePlugin
```

### export directory to GZ_SIM_PLUGIN Path
```bash
(add command)
```
### add plugin to your models sdf file
```
<plugin>
  (add code)
</plugin>
```

### change model and topic name
```c++
movePlugin.cpp:

std::string topic = "/box/cmd_vel", modelName = "box";
->
std::string topic = "/your_name/cmd_vel", modelName = "your_name";
```

### create build folder and compile
```bash
mkdir build
cd build
cmake ..
make
```

### run GZ_ROS2 Topic Bridge:
```bash
(add command)
```

Now you can publish Twist Messages on your ros2 topic, which will move the gazebo model!

