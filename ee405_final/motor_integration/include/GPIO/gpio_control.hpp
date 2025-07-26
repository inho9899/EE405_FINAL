#ifndef GPIO_CONTROL_HPP
#define GPIO_CONTROL_HPP

// File gpio_control.hpp
// Function definitions for gpio_control.cpp

#define MAX_BUF 64 /* For the max length of string */

int gpio_export(unsigned int gpio); // gpio means gpio number (0 to 127)
/*

  - input : GPIO port number to export
  - function that exports GPIO port

*/
int gpio_unexport(unsigned int gpio);
/*

  - input : GPIO port number to unexport
  - function that unexports GPIO port

*/
int gpio_set_dir(unsigned int gpio, unsigned int out); // out = 0: in. out = 1: out.
/* 

  - input : GPIO port number to set direction / desired direction(out : 1, in : 0)
  - function sets the direction of the exported GPIO port

*/

int gpio_fd_open(unsigned int gpio); //Returns file descriptor fd
/*

  - input : GPIO port number to get fd
  - output : fd(file descriptor; information used to write values to the configured GPIO)
  - function which opens the GPIO port and get the file descriptor info

*/

int gpio_fd_close(int fd);
/*
  - function which closes the GPIO port using file descriptor information
*/

int gpio_fd_set_value(int fd, unsigned int value); // value can be 0 or 1
/* 

  - input : GPIO port file descriptor / desired value(1: ON, 0: OFF)
  - Sample code to turn on/off the GPIO
    (IN CASE OF out direction)

*/

#endif
