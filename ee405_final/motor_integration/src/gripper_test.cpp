#include <stdio.h>
#include <string.h>
#include <chrono>
#include "../include//GPIO/gpio_control.hpp"
#include "../include/GPIO/gripper_control.hpp"

int main(int argc, char *argv[])
{
  int gpio_fd_30;
  int pickup_time = 5;
  /*
    1. Define variables for GPIO port number, GPIO file descriptor and length of time duration for holding an object
    !! FILL WITH YOUR CODE !!
  */

  printf("gripper_control\n");

  /*
    2. Open the gripper GPIO port
    !! FILL WITH YOUR CODE !!
  */

  
  gpio_fd_30 = gripper_open(30);

  

  /* Save start time of the loop */
  std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();
  std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
  std::chrono::microseconds loop_elasped_time_microsec = std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time);

  /* Turn on the gripper while pickup_time */
  while(loop_elasped_time_microsec.count()/1e6 < pickup_time)
  {
    current_time = std::chrono::system_clock::now();
    loop_elasped_time_microsec = std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time);
    /*
      3. Turn on the gripper
      !! FILL WITH YOUR CODE !!
    */
    gripper_on(gpio_fd_30);
  }

  gripper_off(gpio_fd_30);

  /*
    4. Close the gripper GPIO port
    !! FILL WITH YOUR CODE !!
  */

  gripper_close(30, gpio_fd_30);
  return 0;
}