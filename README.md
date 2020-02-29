# FormationControl

## Pre-requisites
Add folder Eigen (https://gitlab.com/libeigen/eigen/-/tree/master/Eigen) in the same directory
Add folder include (https://github.com/headmyshoulder/odeint-v2/tree/master/include/boost/numeric) in the same directory
Add contens of git https://github.com/mavlink/c_uart_interface_example under PX4 folder

## Compiling and Building
Use terimanl command: g++ -I. -I include/ -I PX4/mavlink/include/mavlink/v2.0 formationcontrol.cpp PX4/serial_port.cpp PX4/autopilot_interface.cpp -o mavlink_control -lpthread -std=c++11 
