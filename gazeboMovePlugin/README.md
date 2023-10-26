# Twist Ros2_to_GZ_SIM v1.0
---
Copy code into some directory

export directory to GZ_SIM_PLUGIN Path
(add command)

change model name to look for in movePlugin.cpp
(default value: "box"

change topic name in movePlugin.cpp
(default value: "/box/cmd_vel")

run GZ_ROS2 Topic Bridge:
(add command)

now you can publish Twist Messages on your ros2 topic and it will move the gz model 

