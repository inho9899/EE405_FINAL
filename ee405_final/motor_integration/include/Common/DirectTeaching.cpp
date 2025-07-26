/**
 * @file DirectTeaching.cpp
 * @author Jinyeong Jeong (jinyeong.jeong@kaist.ac.kr)
 * @brief 
 * @version 0.1
 * @date 2023-04-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "DirectTeaching.hpp"

DirectTeaching::DirectTeaching(int dof_args)
{
    NumWaypoints=15; // the number of waypoints
    dof=dof_args; // DOF of the manipulator
    stack_waypoints.resize(NumWaypoints, dof); // Each row in the stack_waypoints matrix represents a waypoint for direct teaching

    NumNodes= 400; // the number of nodes 
    NodeIndex=0; // node index. The range of NodeIndex is 0 ~ (NumNodes-1)
    StartPointIndex=0; // Waypoint index of start point in trajectory generation. The range of StartPointIndex is 0 ~ (NumWaypoints-1) 
    EndPointIndex=1; // Waypoint index of end point in trajectory generation. The range of EndPointIndex is 0 ~ (Numwaypoints-1)

}

DirectTeaching::~DirectTeaching()
{
}

void DirectTeaching::setEndPointIndex(int idx) {
    EndPointIndex = idx;
    NodeIndex=0;
}

void DirectTeaching::setDirectTeachingMode(bool input){
    DirectTeachingMode = input;
}

bool DirectTeaching::getDirectTeachingMode(){
    return DirectTeachingMode;
}

// Define stack_waypoints where each row represents a waypoint for direct teaching in joint space
// The first waypoint for direct teaching is the initial joint angle when the program runs
void DirectTeaching::initialization(const Eigen::VectorXd &current_q)
{
    // Save two user-defined waypoints to the stack_waypoints using the "waypoints.csv" file
    // Note that the first waypoint for direct teaching is the initial joint angle when the program runs
    // std::cout << current_q(0) << current_q(1) << current_q(2) << current_q(3) << "\n";
    stack_waypoints <<current_q(0), current_q(1), current_q(2), current_q(3), MatrixFileIO::openData("waypoints.csv");
    
    std::cout << "Each row represents a waypoint for direct teaching in joint space: \n" << stack_waypoints<< std::endl;
    
}


// Generate trajectory between waypoints by using the 1st order polynomial 
Eigen::VectorXd DirectTeaching::TrajectoryGeneration()
{
    Eigen::VectorXd initial_position(dof); initial_position << 0, 0, M_PI/2, -M_PI/2; 
    Eigen::VectorXd targetQ(dof); targetQ.setZero();
    if (EndPointIndex != 0){
    for(int j=0; j<dof; j++){
        targetQ(j)= stack_waypoints(StartPointIndex, j) + (stack_waypoints(EndPointIndex, j) - stack_waypoints(StartPointIndex, j)) / NumNodes * NodeIndex;/* Implement here. targetQ(j) is the result of the first order polynomial interpolation 
        between stack_waypoints(StartPointIndex, j) and stack_waypoints(EndPointIndex, j) */
    }
    } else {
    for(int j=0; j<dof; j++){
        targetQ(j)= stack_waypoints(StartPointIndex, j) + (initial_position(j) - stack_waypoints(StartPointIndex, j)) / NumNodes * NodeIndex;/* Implement here. targetQ(j) is the result of the first order polynomial interpolation 
        between stack_waypoints(StartPointIndex, j) and stack_waypoints(EndPointIndex, j) */
    }
    }

    NodeIndex++; 
 
    if(NodeIndex>=NumNodes){ // NodeIndex is out of range. The next targetQ reaches the end point.
        if (EndPointIndex != 0){
            targetQ = stack_waypoints.row(EndPointIndex);
        } else {
            targetQ = initial_position;
        }
        // StartPointIndex++; // Move the start point to the next waypoint
        // EndPointIndex++; // Mote the end point to the next waypoint

        // NodeIndex= 0;/* Implement here.*/

        // if(StartPointIndex>=NumWaypoints){ // StartPointIndex is out of range
        //     StartPointIndex= 0;/* Implement here.*/
        // } 
        // if(EndPointIndex>=NumWaypoints){ // EndPointIndex is out of range
        //     EndPointIndex= 0;/* Implement here.*/
        // }  
    }

    return targetQ;
}

Eigen::VectorXd DirectTeaching::getDestinationPoint(int destinationIndex){
    Eigen::VectorXd initial_position(dof); initial_position << 0, 0, M_PI/2, -M_PI/2; 
    if (destinationIndex != 0){
        return stack_waypoints.row(destinationIndex);
    } else{
        return initial_position;
    }
    
}