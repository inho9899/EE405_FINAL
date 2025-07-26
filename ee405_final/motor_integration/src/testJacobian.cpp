#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <unistd.h>
#include <chrono>
#include <Eigen/Dense>
#include <csignal>

#include "Common/Kinematics.hpp"
using namespace Eigen;
using namespace std;

int main(int argc, char *argv[])
{       

    std::cout << "Enter the joint angles" << std::endl;
    Eigen::Vector4d current_q;
    current_q.setZero();
    double a, b, c, d;
    scanf("%lf %lf %lf %lf", &a, &b, &c, &d);

    current_q<<a, b, c, d;


    Eigen::MatrixXd J=Kinematics::GetJacobianMatrix(current_q); // J is the Jacobian matrix for linear velocity represented in the world frame

    std::cout << "Jacobian matrix for the world linear velocity" << std::endl;
    std::cout << J << std::endl;

    return 0;
}
