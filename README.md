# -Arduino_Robot_Arm
Designing the Robotic Arm
I built a simple 5-DOF robot arm. The robot only has revolute joints powered by SG90 low torque and inexpensive hobby servo motors. These motors are position-controlled and need PWM signals for movement. For low-level control of the servos, an Arduino Uno is used.

The hardware components used here are:

1.SG90 Servo Motors (5)
2.Arduino Uno R3 Development Board
3.Cardboard to fabricate robot arm links

Robot initial positions
![image](https://user-images.githubusercontent.com/80271396/182416324-410b82a8-372d-4e77-a432-a7c7d78c8498.png)


The robot arm URDF in the previous article on Robotic Arm Simulation in Robot Operating System (ROS) was a very rudimentary design. For this project, I created a URDF using coarse approximations for the servos and the joints. Readers can try creating the URDF using STL files provided in the repository. The virtual model of the robot has the same visual and collision links. The robot joints are named joint_0, joint_1, joint_2, joint_3, and joint_4 with joint_0 at the robot base.

Note: The first servo for joint_0 sits inside the circular disc platform. It is not visible in the URDF but can be seen in the exploded view of the CAD model.

![image](https://user-images.githubusercontent.com/80271396/182418228-c28beffd-2657-40a8-971f-b9928120bd16.png)

## Building the Robot Hardware
For the ease of fabrication, I used easily available cardboard. There were only a few parts that I cut out and put together using super glue. You can refer to the CAD files for measurements.

The SG90 servo motors used here have an angular range of 0-200°. The joint angle limits in the URDF file are mentioned as ±90° centered at home. The homing offset is done in the Arduino code.

Note: When mounting the links to the servos, make sure you drive the servos to the home position at 90° such that the links have a ±90° motion in either direction of rotation.

### Interfacing the Robot Arm With Arduino
Servo motors need PWM signals for motion and several digital pins on the Arduino Uno are capable of generating PWM signals. These digital pins are used to connect the servos. The connections are shown in the diagram below.

![image](https://user-images.githubusercontent.com/80271396/182419915-2b5881ba-4cb4-4535-a3b4-f8d00aff059c.png)

## Setting Up the Infrastructure
* To get your Arduino ready, follow the steps in the previous article How to use Arduino with Robot Operating System (ROS) to set up the rosserial ROS package, the ros_lib Arduino library - ROS support for the Arduino IDE.
* To set up the ROS package on your host machine, follow the steps in the previous article Robotic Arm Simulation in Robot Operating System (ROS) and setup ROS, create a ROS workspace and install the ROS package provided here.
* The robot-arm-control-ros ROS package has this file structure:
![image](https://user-images.githubusercontent.com/80271396/182420024-4514f0a2-054e-49fe-9f3b-b51b9c26c29a.png)



* cad - All the STL files for the robot model.
* CMakeLists.txt - Script for CMake meta build system
* robot-arm-control-arduino - Arduino code that talks to the ROS nodes
* launch - Contains launch files for the Rviz simulator
* src - Contains the node that publishes the joint instructions to Arduino
* urdf - Contains the URDF model of the robot
* rviz - Contains the default configurations for the Rvix simulator
* Code and Explanation

## Code and Explanation
![image](https://user-images.githubusercontent.com/80271396/182420380-9169a70f-47ed-423a-bd97-1d8880a3ce85.png)

## Launch file
The launch file is similar to the one used in the previous article. It launches the robot_state_publisher node and visualizes the URDF model on Rviz. It also launches the joint_state_publisher node which creates a GUI with sliders to control each joint of the servo. These joint instructions are published to the joint_states topic used by the robot_state_publisher as well as the Arduino (to know the use desired joint angles for the robot) via the serial_node that facilitates the communication to control the real robot.



Arduino Code
The Arduino Code is self-explanatory and well commented, but here are a few suggestions for readers who want to extend the code:

 Arduino Code
The Arduino Code is self-explanatory and well commented, but here are a few suggestions for readers who want to extend the code:

* Inside the loop() function, the call to node_handle.spinOnce() is made to ensure that ROS processes all the messages, subscriber callbacks, and other buffers. Readers can use node_hanlde.spin() for an infinite loop outside the Arduino loop() method.
* In the setup() method, it is important to set the baud rate prior to calling init() on the node_handle() else the baud rate is not correctly set and causes communication issues.
* The call made to writeServos() to update the servo positions directly sets the servo positions to use desired values resulting in a jerky motion. Readers can implement interpolated or profiled motion for smoother motion.
* The Arduino Uno used here has low buffer and memory which causes performance issues when multiple memory consuming messages are used. Switch to an Arduino Mega to avoid such situations.

***

Program Execution
Connect the Arduino Uno to the laptop via the USB cable and confirm that the board appears in the list of available serial ports in the Arduino IDE. Compile the code, select the board type, and upload it to the board.

Once the code is uploaded, execute the following commands in multiple terminals on the machine simultaneously.

1. Start the ROS Master - roscore
1. Run rosserial client on the machine - rosrun rosserial_python serial_node.py _port:=/dev/tty<USB# or ACM#> _baud:=115200
1. The serial port is determined at run time for either ttyUSB or ttyACM. The exact port number can be found
1. from the Arduino IDE or using dmesg | grep tty.
1. Run the launch file to simulate and control the robot. - roslaunch robot-arm-control-ros simulate.launch
1. The Arduino can now see the joint_states topic and the data being published to it by the GUI thereby controlling the servos on the robot.
