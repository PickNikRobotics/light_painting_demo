# UR5 Light Painting Demo

### Installation

Clone this repo. Clone the UR2 package and follow the full installation instructions there.

https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver

### Running A Simulation

`ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=true`

`ros2 launch light_painting_demo ur_moveit_cpp.launch.py ur_type:=ur5e robot_ip:="xxx.xxx" use_fake_hardware:=true launch_rviz:=false`

### Adding a new image

Save a 1000-pixel by 1000-pixel image to /resources for future reference. The waypoints you want should be in the middle-ish.

You can easily edit the image and get X/Y coordinates at this website:  https://jspaint.app

Save the key point X/Y coordinates from the image to /resources/whatever_name.yaml. The upper left corner of the image is (0, 0)
