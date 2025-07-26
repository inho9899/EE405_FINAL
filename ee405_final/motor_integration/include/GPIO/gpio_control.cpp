#include "../include/GPIO/gpio_control.hpp"

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

int gpio_export(unsigned int gpio)
{

/*

  - input : GPIO port number to export
  - function that exports GPIO port

  !! MODIFY FOLLOWING SAMPLE CODE PROPERLY !!

  int fd;
  char buf[MAX_BUF];
  int gpio = XX;
  fd = open("/sys/class/gpio/export", O_WRONLY);
  sprintf(buf, "%d", gpio);
  write(fd, buf, strlen(buf));
  close(fd);

*/

char buf[MAX_BUF];
int fd = open("/sys/class/gpio/export", O_WRONLY);
sprintf(buf, "%d", gpio);
write(fd, buf, strlen(buf));
close(fd);
}


int gpio_unexport(unsigned int gpio)
{

/*

  - input : GPIO port number to unexport
  - function that unexports GPIO port

  !! MODIFY FOLLOWING SAMPLE CODE PROPERLY !!

  fd = open("/sys/class/gpio/unexport", O_WRONLY);
  sprintf(buf, "%d", gpio);
  write(fd, buf, strlen(buf));
  close(fd);

*/

int fd = open("/sys/class/gpio/unexport", O_WRONLY);
char buf[MAX_BUF];
sprintf(buf, "%d", gpio);
write(fd, buf, strlen(buf));
close(fd);
}


int gpio_set_dir(unsigned int gpio, unsigned int direction)
{

/* 

  - input : GPIO port number to set direction / desired direction(out : 1, in : 0)
  - function sets the direction of the exported GPIO port

  !! MODIFY FOLLOWING SAMPLE CODE PROPERLY !!

  sprintf(buf, "/sys/class/gpio/gpio%d/direction", gpio);
  fd = open(buf, O_WRONLY); // 
  write(fd, "out", 3); // Set out direction
  write(fd, "in", 2); // Set in direction
  close(fd);

*/
char buf[MAX_BUF];
sprintf(buf, "/sys/class/gpio/gpio%d/direction", gpio);
int fd = open(buf, O_WRONLY); // 
if (direction == 1){
  write(fd, "out", 3); // Set out direction
}else if (direction == 0){
  write(fd, "in", 2); // Set in direction
}
close(fd);
}


int gpio_fd_open(unsigned int gpio)
{

/*

  - input : GPIO port number to get fd
  - output : fd(file descriptor; information used to write values to the configured GPIO)
  - function which opens the GPIO port and get the file descriptor info

  !! MODIFY FOLLOWING SAMPLE CODE PROPERLY !!

  sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio);
  fd = open(buf, O_RDWR | O_NONBLOCK);

*/
char buf[MAX_BUF];
sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio);
int fd = open(buf, O_RDWR | O_NONBLOCK);
return fd;
}


int gpio_fd_close(int fd)
{

/*
  - function which closes the GPIO port using file descriptor information
  - input : fd to close corresponding GPIO port
*/

  close(fd);
}


int gpio_fd_set_value(int fd, unsigned int value)
{

/* 

  - input : GPIO port file descriptor / desired value(1: ON, 0: OFF)
  - Sample code to turn on/off the GPIO
    (IN CASE OF out direction)

  !! MODIFY FOLLOWING SAMPLE CODE PROPERLY !!

  sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio);
  fd = open(buf, O_WRONLY);
  write(fd, "1", 1);// Set GPIO ON
  write(fd, "0", 1);// Set GPIO OFF
  close(fd);

*/

if (value == 1){
  write(fd, "1", 1);// Set GPIO ON
}else if (value == 0){
  write(fd, "0", 1);// Set GPIO OFF
}
}