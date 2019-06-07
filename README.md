# air_hockey

* Author: Carlos Prados <carlos.prados@hotmail.com>

Package for controlling the robot to play air hockey game.


## DEPENDENCIES

- To comunicate throughout the serial port with the Arduino.

https://github.com/ros-drivers/rosserial.git


# Include

- Necessary items to comunicate ROS and Arduino to the control of different motors DC.
- Necessary messages to comunicate nodes.
- Programs of diverses nodes to execute the defined task.


# Quick Start

- To generate all the necessary libraries for the comunication with Arduino, in the desired folder:

rm -rf ros_lib

rosrun rosserial_arduino make_libraries.py .


## Executables

- To generate the executable for python codes

chmod +x vision.py

- To execute the master and link the serial port (not recommendable)

roslaunch air_hockey arduino.launch


## Execute the executables (Accurate order)

- Run the master

roscore

- Charge the program in Arduino.

- Link the serial port (if not launch):

rosrun rosserial_python serial_node.py _baud:=57600 _port:=/dev/ttyUSB0

- To execute the nodes separately in the computer (in order)

rosrun air_hockey vision.py
rosrun air_hockey prediccion
rosrun air_hockey planificacion
rosrun air_hockey gestor_es
rosrun air_hockey gestor


## To move motors manually

- Observate the robot position:

rostopic echo /pos_robot

- Send to one position (i.e. pos [10,30]):

rostopic pub --once /control air_hockey/hockey_pose "{tiempo: 0.0, x: 10.0, y: 30.0, cal_x: false, cal_y: false}"

- Search the sincronism position (i.e. the motor 1)

rosservice call /moveHome "id: 1"



