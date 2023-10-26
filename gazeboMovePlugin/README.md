# Twist Ros2_to_GZ_SIM v1.0
---
### Copy code into some directory
```
(add command)
```

### export directory to GZ_SIM_PLUGIN Path
```
(add command)
```

### change model and topic name
```
movePlugin.cpp:

std::string topic = "/box/cmd_vel", modelName = "box";
->
std::string topic = "/your_name/cmd_vel", modelName = "your_name";
```

### run GZ_ROS2 Topic Bridge:
```
(add command)
```

now you can publish Twist Messages on your ros2 topic and it will move the gz model

