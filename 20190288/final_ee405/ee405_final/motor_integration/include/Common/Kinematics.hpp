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
#pragma once

#include <Eigen/Dense>

class Kinematics
{
private:

public:
    Kinematics();
    ~Kinematics();
    static Eigen::Matrix3d GetRotationMatrix(double r, double p, double y);
    static Eigen::MatrixXd GetJacobianMatrix(const Eigen::VectorXd &current_q);
};

