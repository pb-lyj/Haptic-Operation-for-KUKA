# haptic  ros2 package

This ROS package is designed for tactile robotics experiments on a KUKA robot. 
# Deoendence
Its main dependencies include:
- ubuntu 24.04
- ros2-jazzy
- python 3.12

- FRI
- Gazebo
	```
	sudo apt update
	sudo apt install ros-jazzy-gz-gazebo
	sudo apt install ros-jazzy-ros-gz-sim
	sudo apt install ros-jazzy-ros-gz
	```
- LBR
	https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_demos/lbr_demos_advanced_py/doc/lbr_demos_advanced_py.html
	https://github.com/lbr-stack/lbr_fri_ros2_stack

# Function
## Robotic arm movement
- Connect KUKA
	- https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_demos/lbr_demos_advanced_py/doc/lbr_demos_advanced_py.html
```
	ros2 launch lbr_bringup hardware.launch.py \
    ctrl:=lbr_joint_position_command_controller \
    model:=iiwa14 # [iiwa7, iiwa14, med7, med14]
```
```
	ros2 run lbr_demos_advanced_py admittance_control --ros-args \
    -r __ns:=/lbr \
	    --params-file ros2 pkg prefix
lbr_demos_advanced_py/share/lbr_demos_advanced_py/config/admittance_control.yaml
```

This will start the robotic arm information broadcast at the same time

- Moving the robotic arm to its original position

```
	ros2 run haptic reset
```

## Tac3D Sensor

- Sensor information broadcasting Node
	```
	ros2 run haptic tac3d_l
	```

	```
	ros2 run haptic tac3d_r
	```

## Human Teach and Data record

After the robotic arm is connected and the tactile sensor is activated, you can run this file to record all the data they broadcast.

```
ros2 run haptic dataset_recorder_launch
```

## Launch all of the above procedures

```
ros2 launch haptic teach_record.py
```