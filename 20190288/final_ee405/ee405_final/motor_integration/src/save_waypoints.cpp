/**
 * @file manipulator_control.cpp
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

#include "Controller/ArticulatedSystem.hpp"
#include "FileIO/MatrixFileIO.hpp"
#include "KeyInput/getche.h"

using namespace Eigen;
using namespace std;

//////////////// IMPORTANT BELOW COMMAND //////////////////
// cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
// echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
// https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#
//////////////// IMPORTANT BELOW COMMAND //////////////////


int main(int argc, char *argv[])
{       

    // The manipulator has 4 DOF
    int dof = 4;

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
                                        dynamixel_id_set, position_resolution, motor_torque_constants);
    
    Eigen::VectorXd waypoint1(dof); waypoint1.setZero(); // the first user-defined waypoint for direct teaching
    Eigen::VectorXd waypoint2(dof); waypoint2.setZero(); //  the second user-defined waypoint for direct teaching
    Eigen::VectorXd waypoint3(dof); waypoint3.setZero();
    Eigen::VectorXd waypoint4(dof); waypoint4.setZero();
    Eigen::VectorXd waypoint5(dof); waypoint5.setZero();
    Eigen::VectorXd waypoint6(dof); waypoint6.setZero();
    Eigen::VectorXd waypoint7(dof); waypoint7.setZero();
    Eigen::VectorXd waypoint8(dof); waypoint8.setZero();
    Eigen::VectorXd waypoint9(dof); waypoint9.setZero();
    Eigen::VectorXd waypoint10(dof); waypoint10.setZero();
    Eigen::VectorXd waypoint11(dof); waypoint11.setZero();
    Eigen::VectorXd waypoint12(dof); waypoint12.setZero();
    Eigen::VectorXd waypoint13(dof); waypoint13.setZero();
    Eigen::VectorXd waypoint14(dof); waypoint14.setZero();
    char key; // the keyboard input


    std::vector<Eigen::VectorXd> stack_UserDefinedWaypoints;  // We will save two user-defined waypoints in csv files

    articulated_system.DisableTorque();
    while(1){ 
        cout << "To save (i)-th way point, press i (i=1 or 2). To end this process, press 'e'" << endl;
        key = getch(); // Get keyboard input from the user

        if(key == '1') { // The keyboard input is 1
            waypoint1 =  articulated_system.GetJointAngle();/* Implement here using GetJointAngle() */
            cout << "Save the first user-defined waypoint" << endl;
            cout << "waypoint1 :" <<  endl;
            cout << waypoint1 << endl;
        }
        else if(key=='2'){ //  The keyboard input is 2
            waypoint2 =  articulated_system.GetJointAngle();/* Implement here using GetJointAngle() */
            cout << "Save the second user-defined waypoint" << endl;
            cout << "waypoint2 :" <<  endl;
            cout << waypoint2 << endl;
        }
        else if(key=='3'){ 
            waypoint3 =  articulated_system.GetJointAngle();
            cout << "Save the third user-defined waypoint" << endl;
            cout << "waypoint3 :" <<  endl;
            cout << waypoint3 << endl;
        }
        else if(key=='4'){ 
            waypoint4 =  articulated_system.GetJointAngle();
            cout << "Save the fourth user-defined waypoint" << endl;
            cout << "waypoint4 :" <<  endl;
            cout << waypoint4 << endl;
        }
        else if(key=='5'){ 
            waypoint5 =  articulated_system.GetJointAngle();
            cout << "Save the fifth user-defined waypoint" << endl;
            cout << "waypoint5 :" <<  endl;
            cout << waypoint5 << endl;
        }
        else if(key=='6'){ 
            waypoint6 =  articulated_system.GetJointAngle();
            cout << "Save the sixth user-defined waypoint" << endl;
            cout << "waypoint6 :" <<  endl;
            cout << waypoint6 << endl;
        }
        else if(key=='7'){ 
            waypoint7 =  articulated_system.GetJointAngle();
            cout << "Save the seventh user-defined waypoint" << endl;
            cout << "waypoint7 :" <<  endl;
            cout << waypoint7 << endl;
        }
        else if(key=='8'){ 
            waypoint8 =  articulated_system.GetJointAngle();
            cout << "Save the eith user-defined waypoint" << endl;
            cout << "waypoint8 :" <<  endl;
            cout << waypoint8 << endl;
        }
        else if(key=='9'){ 
            waypoint9 =  articulated_system.GetJointAngle();
            cout << "Save the nineth user-defined waypoint" << endl;
            cout << "waypoint9 :" <<  endl;
            cout << waypoint9 << endl;
        }
        else if(key=='a'){ 
            waypoint10 =  articulated_system.GetJointAngle();
            cout << "Save the tenth user-defined waypoint" << endl;
            cout << "waypoint10 :" <<  endl;
            cout << waypoint10 << endl;
        }
        else if(key=='b'){ 
            waypoint11 =  articulated_system.GetJointAngle();
            cout << "Save the eleventh user-defined waypoint" << endl;
            cout << "waypoint11 :" <<  endl;
            cout << waypoint11 << endl;
        }
        else if(key=='c'){ 
            waypoint12 =  articulated_system.GetJointAngle();
            cout << "Save the twelveth user-defined waypoint" << endl;
            cout << "waypoint12 :" <<  endl;
            cout << waypoint12 << endl;
        }
        else if(key=='d'){ 
            waypoint13 =  articulated_system.GetJointAngle();
            cout << "Save the thirteenth user-defined waypoint" << endl;
            cout << "waypoint13 :" <<  endl;
            cout << waypoint13 << endl;
        }
        else if(key=='f'){ 
            waypoint14 =  articulated_system.GetJointAngle();
            cout << "Save the fourteenth user-defined waypoint" << endl;
            cout << "waypoint14 :" <<  endl;
            cout << waypoint14 << endl;
        }
        else if(key=='e') break;
        else{ // wrong keyboard input
            cout << "Wrong keyboard input" << endl;
        }
    }
    stack_UserDefinedWaypoints.emplace_back(waypoint1); // stack waypoint1
    stack_UserDefinedWaypoints.emplace_back(waypoint2); // stack waypoint2
    stack_UserDefinedWaypoints.emplace_back(waypoint3);
    stack_UserDefinedWaypoints.emplace_back(waypoint4);
    stack_UserDefinedWaypoints.emplace_back(waypoint5);
    stack_UserDefinedWaypoints.emplace_back(waypoint6);
    stack_UserDefinedWaypoints.emplace_back(waypoint7);
    stack_UserDefinedWaypoints.emplace_back(waypoint8);
    stack_UserDefinedWaypoints.emplace_back(waypoint9);
    stack_UserDefinedWaypoints.emplace_back(waypoint10);
    stack_UserDefinedWaypoints.emplace_back(waypoint11);
    stack_UserDefinedWaypoints.emplace_back(waypoint12);
    stack_UserDefinedWaypoints.emplace_back(waypoint13);
    stack_UserDefinedWaypoints.emplace_back(waypoint14);


    // Save the two waypoints to a CSV file named "waypoints.csv" in the same directory as the executable file.
    MatrixXd waypoints_matrix = MatrixFileIO::ConvertStdEigenVectorToEigenMatrix(stack_UserDefinedWaypoints);
    MatrixFileIO::saveData("waypoints.csv", waypoints_matrix);
    

    return 0;
}
