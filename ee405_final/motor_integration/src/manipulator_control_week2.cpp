/**
 * @file manipulator_control_week12.cpp
 * @author Jiwan Han (jw.han@kaist.ac.kr), Jinyeong Jeong (jinyeong.jeong@kaist.ac.kr)
 * @brief 
 * @version 0.1
 * @date 2023-03-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <unistd.h>
#include <chrono>
#include <Eigen/Dense>
#include <csignal>
#include <thread>

#include "Controller/ArticulatedSystem.hpp"
#include "KeyInput/getche.h"
#include "Common/Kinematics.hpp"
#include "Common/DirectTeaching.hpp"
#include "../include//GPIO/gpio_control.hpp"
#include "../include/GPIO/gripper_control.hpp"
using namespace Eigen;
using namespace std;

//////////////// IMPORTANT BELOW COMMAND //////////////////
// cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
// echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
// https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#
//////////////// IMPORTANT BELOW COMMAND //////////////////

// GetKeyboardInput() function : Compute the task command from the keyboard input for teleopeartion.
// The following keyboard input determines the task command. The position_resolution variable represents the position resolution for teleoperation
// r : move 1cm in the world z-axis 
// f : move -1cm in the world z-axis 
// w : move 1cm in the world y-axis 
// s : move -1cm in the world y-axis 
// d : move 1cm in the world x-axis 
// a : move -1cm in the world x-axis 
// i : move to the initial position
// e : end the teleoperation
bool is_key_updated = false; 
Eigen::Vector3d task_command;
char key='i';
void GetKeyboardInput()
{
    double position_resolution=0.01; // The position resolution for teleoperation is 1cm
    while(1)
    {
        std::cout << "Press 'r', 'f', 'w', 's', 'd', or 'a' to move the end-effector. Press 'i' to move to the initial position. Press 'e' to end this mode" << std::endl;
        key = getch(); // Get keyboard input from the user
        is_key_updated=true; // When a new keyboard input is received, is_key_upated is set to true
        if(key == 'r') { 
            task_command<<0, 0, position_resolution; 
        } 
        else if(key == 'f') { 
            task_command<<0, 0, -position_resolution; 
        } 
        else if(key == 'w') { 
            task_command<< 0, position_resolution, 0; /* implement here */
        } 
        else if(key == 's') { 
            task_command<< 0, -position_resolution, 0; /* implement here */
        } 
        else if(key == 'd') { 
            task_command<< position_resolution, 0, 0; /* implement here */
        } 
        else if(key == 'a') { 
            task_command<< -position_resolution, 0, 0; /* implement here */
        }
        else if(key == 'e') { 
            break;
        } 
    }
}
int main(int argc, char *argv[])
{       
    std::thread t1 = std::thread(GetKeyboardInput); // Create thread t1 to get the keyboard input from the user

    // The manipulator has 4 DOF
    int dof = 4;

    // The control frequency is 100Hz
    double control_freq = 100;
    double dt = 1/control_freq;

    // Threshold setting
    const double threshold = 0.01;

    // Dynamixel setup, ID, position resolution, motor torque constant, etc.
    std::vector<uint8_t> dynamixel_id_set;
    std::vector<int32_t> position_resolution;
    std::vector<double>  motor_torque_constants;
    for (size_t i = 0; i < dof; i++)
    {
        dynamixel_id_set.emplace_back(i); 
        position_resolution.emplace_back(POSITION_RESOLUTION);
        motor_torque_constants.emplace_back(MOTOR_TORQUE_CONSTANT);
    }
    
    // Initialize the Articulated_system, 
    // We need a dof of the system, Operation Mode(Position, Velocity, Current),
    // USB Port information (for serical communication),
    // and dynamixel setup
    ArticulatedSystem articulated_system(dof, ArticulatedSystem::Mode::VELOCITY, "/dev/ttyUSB0", 
                                        dynamixel_id_set, position_resolution, motor_torque_constants); //4 DOF manipulator

    /**
     * The member function articulated_system.GetJointAngle() returns the joint angles of the robot system, which in this context refer to the motor angle.
     */
    Eigen::VectorXd currentQ = articulated_system.GetJointAngle();
    Eigen::VectorXd initialQ = articulated_system.GetJointAngle();

    // Declare targetQ(motor angle), targetQdot(motor velocity)
    Eigen::VectorXd targetQ(dof); targetQ.setZero();
    Eigen::VectorXd targetQdot(dof); targetQdot.setZero();

    bool is_ready_for_teleoperation=false;
    Eigen::VectorXd prev_targetQ(dof); // Declare prev_targetQ(the previous targetQ value)
    
    // The initial joint position for teleoperation is [0, 0, PI/2, -PI/2]. This configuraiton is far from singularities.
    Eigen::VectorXd initial_position(dof); initial_position << 0, 0, M_PI/2, -M_PI/2; 
    prev_targetQ = initial_position;

    /**
     * @brief time instant is initialized.
     * loop_start : represents the moment when the 'while' loop starts.
     * loop_end : represents the moment when the 'while' loop ends.
     * current_time : current time instant. 
     * initial_time : The time instant when the program(=main function) starts.
     */
    std::chrono::system_clock::time_point loop_start= std::chrono::system_clock::now();
    std::chrono::system_clock::time_point loop_end= std::chrono::system_clock::now();
    std::chrono::system_clock::time_point current_time= std::chrono::system_clock::now();
    std::chrono::system_clock::time_point initial_time= std::chrono::system_clock::now();
    
    // P gain.
    Eigen::MatrixXd Kp(dof, dof); Kp.setZero();

    // ///////////////////////////////////// Direct teaching mode /////////////////////////////////////
    DirectTeaching direct_teaching(dof);
    direct_teaching.initialization(currentQ); // initialization for direct teaching
    direct_teaching.setDirectTeachingMode(false);
    /////////////////////////////////////// Direct teaching mode END /////////////////////////////////////
    
    // The following variables are for trajectory generation before starting teleoperation
    int NodeIndex=0;
    int NodeNum=500;
    

    // Mode setting
    bool implementMotion = false;
    int rockIndex = 0;
    int destinationIndex = 8;
    int sequenceIndex = 0;

    // GPIO setting
    int gpio_fd_30;
    gpio_fd_30 = gripper_open(30);

    
    while(1){
        current_time= std::chrono::system_clock::now();
        auto loop_elasped_time_microsec = std::chrono::duration_cast<std::chrono::microseconds>(current_time - loop_start);
        auto total_elasped_time_microsec = std::chrono::duration_cast<std::chrono::microseconds>(current_time - initial_time);

        // For 100 Hz control loop.
        if(loop_elasped_time_microsec.count()>=dt*1e6)
        {
            double total_elasped_time_sec = static_cast<double>(total_elasped_time_microsec.count())/1e6;
            loop_start = std::chrono::system_clock::now();

            currentQ = articulated_system.GetJointAngle();

            if(!is_ready_for_teleoperation){ // Move the end-effector to initial_position before starting teleoperation
                targetQ = (initial_position - initialQ)/NodeNum*NodeIndex+initialQ; // trajectory generation using the first-order polynomial
                NodeIndex++;
                if(NodeIndex>NodeNum) {
                    is_ready_for_teleoperation=true; // Now, it is ready for teleopeartion
                    std::cout<<"Start teleoperation!"<<std::endl;
                }
            }else{
                ///////////////////////////////////// Teleoperation mode Start /////////////////////////////////////
                if(is_key_updated){ // Perform teleoperation only when a new keyboard input is received (is_key_upated=true)                
                    if(key=='e'){ // End teleoperation
                        break;
                    }else if (key=='q'){ // Start seq 1: i-> rock
                        implementMotion = true;

                        rockIndex ++;
                        FILE *file = fopen("hello.txt", "rt");
                        char buf[256];
                        if(fgets(buf, sizeof(buf), file) != nullptr){
                            destinationIndex = buf[0]-'0';
                        }

                        sequenceIndex = 1;

                        direct_teaching.setEndPointIndex(rockIndex);
                        direct_teaching.setDirectTeachingMode(true);
                        std::cout<< "here" << std::endl;
                        direct_teaching.initialization(currentQ);
                        
                    }else // When the wrong keyboard input is received, the targetQ is the previous targetQ value
                    {
                        targetQ=prev_targetQ;
                    }

                    is_key_updated=false; // Set is_key_updated to false in order to check if the new keyboard input is received
                }            
                else{ // Before receiving the new keyboard input, the targetQ is the previous targetQ value
                    targetQ=prev_targetQ;
                }        
                ///////////////////////////////////// Teleoperation mode END /////////////////////////////////////
                

                ///////////////////////////////////// Direct Teaching mode Start /////////////////////////////////////
                if (direct_teaching.getDirectTeachingMode()){
                    targetQ = direct_teaching.TrajectoryGeneration();
                    //std::cout<< targetQ(0) << ' ' << targetQ(1) << ' ' << ' ' << targetQ(2)<< ' ' << targetQ(3) << '\n';
                }
                ///////////////////////////////////// Direct Teaching mode END /////////////////////////////////////
            
                // Change Mode
                if (sequenceIndex == 1){
                    Eigen::VectorXd error = currentQ - direct_teaching.getDestinationPoint(rockIndex);
                    if(error.norm() < threshold){
                        gripper_on(gpio_fd_30);
                        std::cout << "mode change from 1 to 2" << std::endl;
                        //sleep(1);
                        sequenceIndex = 2;
                        direct_teaching.setEndPointIndex(0);
                        direct_teaching.setDirectTeachingMode(true);
                        direct_teaching.initialization(currentQ);
                    }
                }

                if (sequenceIndex == 2){
                    Eigen::VectorXd error = currentQ - direct_teaching.getDestinationPoint(0);
                    if(error.norm() < threshold){
                        std::cout << "mode change from 2 to 3" << std::endl;
                        sequenceIndex = 3;
                        direct_teaching.setEndPointIndex(destinationIndex);
                        direct_teaching.setDirectTeachingMode(true);
                        direct_teaching.initialization(currentQ);
                    }
                }

                if (sequenceIndex == 3){
                    Eigen::VectorXd error = currentQ - direct_teaching.getDestinationPoint(destinationIndex);
                    if(error.norm() < threshold){
                        gripper_off(gpio_fd_30);
                        //sleep(1000);
                        std::cout << "mode change from 3 to 4" << std::endl;
                        sequenceIndex = 4;
                        direct_teaching.setEndPointIndex(0);
                        direct_teaching.setDirectTeachingMode(true);
                        direct_teaching.initialization(currentQ);
                    }
                }

                if (sequenceIndex == 4){
                    Eigen::VectorXd error = currentQ - direct_teaching.getDestinationPoint(0);
                    if(error.norm() < threshold){
                        std::cout << "mode change from 4 to 1" << std::endl;
                        sequenceIndex = 0;
                        
                    }
                }
            }


            

            ///////////////////////////////////// P Position control USING Velocity Mode /////////////////////////////////////
            for (size_t i = 0; i < dof; i++)
            {
                Kp(i,i) = 4;
            }           
            targetQdot = -Kp * (currentQ - targetQ); /* Implement here. You have to implement joint space P position Controller */
            articulated_system.SetGoalVelocity(targetQdot); // velocity control mode
            ///////////////////////////////////// P Position control USING Velocity Mode END /////////////////////////////////////
            loop_end= std::chrono::system_clock::now();
            auto one_step_calculation_time = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start);
        }      
    }

    t1.join(); //  allows the main function to wait until t1 completes its execution
    
    gripper_close(30, gpio_fd_30);
    return 0;
}
