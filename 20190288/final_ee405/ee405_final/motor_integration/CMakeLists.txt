cmake_minimum_required(VERSION 3.10)
project(EE405_ManipulatorControl LANGUAGES CXX)

add_compile_options(-std=c++17)

### find package: pre-installed library The CMake can automatically find following installed package.
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

### include_directories: We can set include directories, The CMake can look for the header file now in this path.
include_directories (${EIGEN3_INCLUDE_DIRS}
                    include
                    include/dynamixel_sdk
                    src)
### link_directories: We can set link directories, The CMake can look for the library file now in this path in order to link library and executable file.
link_directories(
    lib
    lib/dynamixel_sdk
)
### add_library: How to make a static library file using the following code. 
### If you want to create a shared library, change 'STATIC' to 'SHARED'. 
### However, for cross-compiling, it is not recommended to change to 'SHARED'.
add_library(EE405_manipulator STATIC include/Controller/ArticulatedSystem.cpp
                                            include/FileIO/MatrixFileIO.cpp
                                            include/Common/DirectTeaching.cpp
                                            include/Common/Kinematics.cpp
                                            include/KeyInput/getche.cpp
                                            include/GPIO/gpio_control.cpp
                                            include/GPIO/gripper_control.cpp
                                            )
### week1 direct teaching

### add_executable: Make executable file using following save_waypoints.cpp code.
add_executable(save_waypoints src/save_waypoints.cpp )

### target_link_libraries: link between exe file and library file.
### EE405_manipulator: The static library which we make. please refer to add_library().
### dxl_sbc_cpp: The dxl_sbc_cpp shared library is from the Dynamixel SDK and enables motor rotation in the program.
target_link_libraries(save_waypoints EE405_manipulator dxl_sbc_cpp)

### add_executable: Make executable file using following manipulator_control_week1.cpp code.
add_executable(manipulator_control_week1 src/manipulator_control_week1.cpp )

target_link_libraries(manipulator_control_week1 EE405_manipulator dxl_sbc_cpp)


# ### week2 teleoperation

### add_executable: Make executable file using following manipulator_control_week2.cpp code.
add_executable(manipulator_control_week2 src/manipulator_control_week2.cpp)

### target_link_libraries: link between exe file and library file.
### EE405_manipulator: The static library which we make. please refer to add_library().
### dxl_sbc_cpp: The dxl_sbc_cpp shared library is from the Dynamixel SDK and enables motor rotation in the program.
### Threads::Threads: Thread module of CMake generates the Threads::Threads imported target
target_link_libraries(manipulator_control_week2 EE405_manipulator dxl_sbc_cpp Threads::Threads)

add_executable(testJacobian src/testJacobian.cpp )
target_link_libraries(testJacobian EE405_manipulator dxl_sbc_cpp)
