/**
 * @file MatrixFileIO.cpp
 * @author Jiwan Han (jw.han@kaist.ac.kr)
 * @brief Reference: https://github.com/AleksandarHaber/Save-and-Load-Eigen-Cpp-Matrices-Arrays-to-and-from-CSV-files/blob/master/source_file.cpp
 * @version 0.1
 * @date 2023-02-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "FileIO/MatrixFileIO.hpp"

MatrixFileIO::MatrixFileIO()
{

}
MatrixFileIO::~MatrixFileIO()
{

}
void MatrixFileIO::saveData(std::string fileName, const Eigen::MatrixXd& matrix)
{
    //https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
    const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");

    std::ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(CSVFormat);
        file.close();
    }
}

Eigen::MatrixXd MatrixFileIO::openData(std::string fileToOpen)
{

    // the inspiration for creating this function was drawn from here (I did NOT copy and paste the code)
    // https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix

    // the input is the file: "fileToOpen.csv":
    // a,b,c
    // d,e,f
    // This function converts input file data into the Eigen matrix format



    // the matrix entries are stored in this variable row-wise. For example if we have the matrix:
    // M=[a b c
    //	  d e f]
    // the entries are stored as matrixEntries=[a,b,c,d,e,f], that is the variable "matrixEntries" is a row vector
    // later on, this vector is mapped into the Eigen matrix format
    std::vector<double> matrixEntries;

    // in this object we store the data from the matrix
    std::ifstream matrixDataFile(fileToOpen);

    // this variable is used to store the row of the matrix that contains commas
    std::string matrixRowString;

    // this variable is used to store the matrix entry;
    std::string matrixEntry;

    // this variable is used to track the number of rows
    int matrixRowNumber = 0;


    while (getline(matrixDataFile, matrixRowString)) // here we read a row by row of matrixDataFile and store every line into the string variable matrixRowString
    {
        std::stringstream matrixRowStringStream(matrixRowString); //convert matrixRowString that is a string to a stream variable.

        while (getline(matrixRowStringStream, matrixEntry, ',')) // here we read pieces of the stream matrixRowStringStream until every comma, and store the resulting character into the matrixEntry
        {
            matrixEntries.push_back(stod(matrixEntry));   //here we convert the string to double and fill in the row vector storing all the matrix entries
        }

        matrixRowNumber++; //update the column numbers
    }

    // here we convet the vector variable into the matrix and return the resulting object,
    // note that matrixEntries.data() is the pointer to the first memory location at which the entries of the vector matrixEntries are stored;
    return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);

}

Eigen::VectorXd MatrixFileIO::ConvertStdVectorToEigenVector(const std::vector<double>& input)
{
    Eigen::VectorXd output(input.size());
    for (size_t i = 0; i < input.size(); i++)
    {
        output(i) = input.at(i);
    }
    
    return output;
}

/**
 * @brief  Vector Stack  {Eigen::Vector1; 
 *                        Eigen::Vector2; 
 *                        Eigen::Vector3; 
 *                        Eigen::Vector4} 
 *                     =>
 *       Eigen::Matrix (Eigen::Vector1; 
 *                      Eigen::Vector2;
 *                      Eigen::Vector3;
 *                      Eigen::Vector4;)
 * 
 * 
 * @param input 
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd MatrixFileIO::ConvertStdEigenVectorToEigenMatrix(const std::vector<Eigen::VectorXd>& input)
{
    if (input.size() == 0)
    {
        printf("%s, %d [ERROR] input's size is zero. please check the parameter.\n", __FILE__, __LINE__);
        return Eigen::Matrix<double,1,1>::Zero();
    }

    Eigen::MatrixXd output(input.size(), input.at(0).rows());
    for (size_t i = 0; i < input.size(); i++)
    {
        for (size_t j = 0; j < input.at(0).rows(); j++)
        {
            output(i,j) = input.at(i)[j];
        }
        
    }
    
    return output;
}