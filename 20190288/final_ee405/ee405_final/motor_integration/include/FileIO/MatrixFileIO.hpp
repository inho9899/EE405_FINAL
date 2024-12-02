/**
 * @file MatrixFileIO.h
 * @author Jiwan Han (jw.han@kaist.ac.kr)
 * @brief Reference: https://github.com/AleksandarHaber/Save-and-Load-Eigen-Cpp-Matrices-Arrays-to-and-from-CSV-files/blob/master/source_file.cpp
 * @version 0.1
 * @date 2022-09-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <vector>

class MatrixFileIO {
public:
    enum M_TYPE {
        INT,
        DOUBLE
    };
    MatrixFileIO();
    ~MatrixFileIO();
    static void saveData(std::string fileName, const Eigen::MatrixXd& matrix);
    static Eigen::MatrixXd openData(std::string fileToOpen);
    static Eigen::VectorXd ConvertStdVectorToEigenVector(const std::vector<double>& input);
    static Eigen::MatrixXd ConvertStdEigenVectorToEigenMatrix(const std::vector<Eigen::VectorXd>& input);
};


