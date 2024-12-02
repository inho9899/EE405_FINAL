/**
 * @file Kinematics.cpp
 * @author Jinyeong Jeong (jinyeong.jeong@kaist.ac.kr)
 * @brief 
 * @version 0.1
 * @date 2023-04-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "Kinematics.hpp"

Kinematics::Kinematics()
{
}

Kinematics::~Kinematics()
{
}


// Return the following successive rotation
// rotation of y (rad) along the body z-axis ->
// rotation of p (rad) along the body y-axis ->
// rotation of r (rad) along the body x-axis 
Eigen::Matrix3d Kinematics::GetRotationMatrix(double r, double p, double y)
{    
    // Rx : Rotation of r (rad) along the body x-axis
    // Ry : Rotation of p (rad) along the body y-axis
    // Rz : Rotation of y (rad) along the body z-axis
    Eigen::Matrix3d Rx, Ry, Rz;
    Rx.setZero(); Ry.setZero(); Rz.setZero();
 
    Rx << 1, 0, 0,
        0, cos(r), -sin(r),
        0, sin(r), cos(r);
    Ry << cos(p), 0, sin(p),
        0, 1, 0,
        -sin(p), 0, cos(p);
    Rz << cos(y), -sin(y), 0,
        sin(y), cos(y), 0,
        0, 0, 1;
    return Rz*Ry*Rx;
}

Eigen::MatrixXd Kinematics::GetJacobianMatrix(const Eigen::VectorXd &current_q)
{
    // Rotation of the ID 0 motor frame with respect to the world frame
    Eigen::Matrix3d Rw0_;
    Rw0_<< 0, 0, -1,
          -1, 0, 0,
           0, 1, 0;

    // Rotation by the ID 0 motor revolution                         
    Eigen::Matrix3d R0_0=GetRotationMatrix(0, current_q(0), 0);

    // Rotation of the ID 1 motor frame with respect to the ID 0 motor frame
    Eigen::Matrix3d R01_;
    R01_<< 0, -1, 0,
           0, 0, 1,
           -1, 0, 0;

    // Rotation by the ID 1 motor revolution                         
    Eigen::Matrix3d R1_1=GetRotationMatrix(0, current_q(1), 0);

    // Rotation of the ID 2 motor frame with respect to the ID 1 motor frame
    Eigen::Matrix3d R12_=GetRotationMatrix(0, 0, M_PI);
    
    // Rotation by the ID 2 motor revolution                          
    Eigen::Matrix3d R2_2=GetRotationMatrix(0, current_q(2), 0);

    // Rotation of the ID 3 motor frame with respect to the ID 2 motor frame
    Eigen::Matrix3d R23_=GetRotationMatrix(0, 0, M_PI);

    // Rotation by the ID 3 motor revolution                         
    Eigen::Matrix3d R3_3=GetRotationMatrix(0, current_q(3), 0);

    // Rotation of the ID 0 motor frame with respect to the world frame 
    Eigen::Matrix3d Rw0=Rw0_*R0_0;
    // Rotation of the ID 1 motor frame with respect to the world frame
    Eigen::Matrix3d Rw1=Rw0*R01_*R1_1;
    // Rotation of the ID 2 motor frame with respect to the world frame
    Eigen::Matrix3d Rw2=Rw1*R12_*R2_2;
    // Rotation of the ID 3 motor frame with respect to the world frame
    Eigen::Matrix3d Rw3=Rw2*R23_*R3_3;

    Eigen::Vector3d motor_axis={0, 1, 0}; // This is the motor axis represented in the body frame (= the motor frame)

    Eigen::Vector3d Pw0=Rw0*motor_axis; // The ID 0 motor axis represented in the world frame
    Eigen::Vector3d Pw1=Rw1*motor_axis; // The ID 1 motor axis represented in the world frame
    Eigen::Vector3d Pw2=Rw2*motor_axis; // The ID 2 motor axis represented in the world frame
    Eigen::Vector3d Pw3=Rw3*motor_axis; // The ID 3 motor axis represented in the world frame

    Eigen::Vector3d r01_0={0, 0.0345, 0}; // A vector pointing the ID 1 motor frame from the ID 0 motor frame represented in the ID 0 motor frame
    Eigen::Vector3d r12_1={0, 0, 0.1165}; // A vector pointing the ID 2 motor frame from the ID 1 motor frame represented in the ID 1 motor frame
    Eigen::Vector3d r23_2={0, 0, 0.0965}; // A vector pointing the ID 3 motor frame from the ID 2 motor frame represented in the ID 2 motor frame
    Eigen::Vector3d r3e_3={0, 0, 0.08}; // A vector pointing the end-effector frame from the ID 3 motor frame represented in the ID 3 motor frame

    Eigen::Vector3d r3e_w= Rw3 * r3e_3; /* implement a vector pointing the end-effector frame from the ID 3 motor frame represented in the world frame */
    Eigen::Vector3d r2e_w= r3e_w + Rw2 * r23_2; /* implement a vector pointing the end-effector frame from the ID 2 motor frame represented in the world frame */
    Eigen::Vector3d r1e_w= r2e_w + Rw1 * r12_1; /* implement a vector pointing the end-effector frame from the ID 1 motor frame represented in the world frame */
    Eigen::Vector3d r0e_w= r1e_w + Rw0 * r01_0; /* implement a vector pointing the end-effector frame from the ID 0 motor frame represented in the world frame */

    Eigen::MatrixXd JacobianTranslationEE(3, 4); JacobianTranslationEE.setZero();
    
    JacobianTranslationEE << Pw0.cross(r0e_w), Pw1.cross(r1e_w), Pw2.cross(r2e_w), Pw3.cross(r3e_w); /* implement the Jacobian matrix for world linear velocity */
    
    return JacobianTranslationEE;
}

