#ifndef GRIPPER_CONTROL_HPP
#define GRIPPER_CONTROL_HPP

// File gripper_control.hpp
// Function definitions for gripper_control.cpp

int gripper_open(unsigned int gpio);
// open the GPIO port for the solenoid magnet
// input : GPIO port number
// output : file descriptor of GPIO port

int gripper_on(unsigned int fd_gpio); 
// turn on the solenoid magnet
// input : GPIO port number

int gripper_off(unsigned int fd_gpio);
// turn off the solenoid magnet
// input : GPIO port number

int gripper_close(unsigned int gpio, unsigned int fd_gpio);
// close the GPIO port for the solenoid magnet
// input : GPIO port number, file descriptor for GPIO

#endif