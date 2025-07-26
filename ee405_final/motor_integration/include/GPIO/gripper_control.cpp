#include <stdio.h>
#include <string.h>
#include <iostream>
#include "../include/GPIO/gpio_control.hpp"

int gripper_open(unsigned int gpio)
{
  /*
    open the GPIO port for the solenoid magnet
    - input : GPIO port number
    - output : file descriptor of GPIO port
    
    !! FILL WITH YOUR CODE !!
  */
  gpio_export(gpio);
  gpio_set_dir(gpio, 1);
  
  return gpio_fd_open(gpio);
}

int gripper_on(unsigned int fd_gpio)
{
  /*
    turn on the solenoid magnet
    - input : GPIO port number

    !! FILL WITH YOUR CODE !!
  */
  gpio_fd_set_value(fd_gpio, 1);

}

int gripper_off(unsigned int fd_gpio)
{
  /*
    turn off the solenoid magnet
    - input : GPIO port number

    !! FILL WITH YOUR CODE !!  
  */
 gpio_fd_set_value(fd_gpio, 0);
}

int gripper_close(unsigned int gpio, unsigned int fd_gpio)
{
  /*
    close the GPIO port for the solenoid magnet
    - input : GPIO port number, file descriptor for GPIO

    !! FILL WITH YOUR CODE !!
  */
  gpio_set_dir(gpio, 0);
  gpio_unexport(gpio);
  gpio_fd_close(fd_gpio);
}