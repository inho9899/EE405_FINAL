/**
 * @file ArticulatedSystem.hpp
 * @author Jiwan Han (jw.han@kaist.ac.kr), Jinyeong Jung (jinyeong.jeong@kaist.ac.kr)
 * @brief 
 * @version 0.1
 * @date 2023-03-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

// #include "dynamixel_sdk/DynamixelInterface.hpp"
#include <Eigen/Dense>

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <unistd.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"  // Uses DYNAMIXEL SDK library

// #define PRINT_VERBOSE

/********* DYNAMIXEL Model definition *********
***** (Use only one definition at a time) *****/
#define X_SERIES // X330, X430, X540, 2X430
// #define PRO_SERIES // H54, H42, M54, M42, L54, L42
// #define PRO_A_SERIES // PRO series with (A) firmware update.
// #define P_SERIES  // PH54, PH42, PM54
// #define XL320  // [WARNING] Operating Voltage : 7.4V
// #define MX_SERIES // MX series with 2.0 firmware update.

// Control table address
#if defined(X_SERIES) || defined(MX_SERIES)
    #define ADDR_TORQUE_ENABLE          64
    #define ADDR_BUS_WATCHDOG           98
    #define ADDR_GOAL_POSITION          116
    #define ADDR_GOAL_VELOCITY          104
    #define ADDR_GOAL_CURRENT           102
    #define ADDR_PRESENT_POSITION       132
    #define ADDR_PRESENT_VELOCITY       128
    #define ADDR_PRESENT_CURRENT        126
    #define ADDR_OPERATING_MODE         11
    #define MINIMUM_POSITION_LIMIT      0  // Refer to the Minimum Position Limit of product eManual // (Not used)
    #define MAXIMUM_POSITION_LIMIT      4095  // Refer to the Maximum Position Limit of product eManual // (Not used)
    #define POSITION_RESOLUTION         4096  // 12 bits encoder
    #define VELOCITY_LIMIT              350  // 0.229 rpm per counts, (Not used)
    #define CURRENT_LIMIT               910  // 1 mA per counts, (Not used)
    #define BAUDRATE                    4000000
    #define RPM_PER_COUNT               (0.229)
    #define mA_PER_COUNT                (1.0) // xc330 (1.0), xh540: 2.69, please refer to https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/
    // #define mA_PER_COUNT                (2.69) // xc330 (1.0), xh540: 2.69, please refer to https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/
    // #define MOTOR_TORQUE_CONSTANT       (1.17742175857) // Nm/A (please refer to line 157 in this ArticulatedSystem.hpp file.)
    // #define MOTOR_TORQUE_CONSTANT       (0.99951243296) // Nm/A (please refer to line 157 in this ArticulatedSystem.hpp file.)
    #define MOTOR_TORQUE_CONSTANT       (1.17742175857) // Nm/A (please refer to line 157 in this ArticulatedSystem.hpp file.)
    #define LEN_PRESENT_POSITION        4
    #define LEN_PRESENT_VELOCITY        4
    #define LEN_PRESENT_CURRENT         2
    #define LEN_GOAL_CURRENT            2
    #define LEN_GOAL_VELOCITY           4
    #define LEN_GOAL_POSITION           4
    #define CURRENT_MODE                0 // (Not used)
#elif defined(PRO_SERIES)
    #define ADDR_TORQUE_ENABLE          562  // Control table address is different in DYNAMIXEL model
    #define ADDR_GOAL_POSITION          596
    #define ADDR_PRESENT_POSITION       611
    #define MINIMUM_POSITION_LIMIT      -150000  // Refer to the Minimum Position Limit of product eManual
    #define MAXIMUM_POSITION_LIMIT      150000  // Refer to the Maximum Position Limit of product eManual
    #define BAUDRATE                    57600
#elif defined(P_SERIES) ||defined(PRO_A_SERIES)
    #define ADDR_TORQUE_ENABLE          512  // Control table address is different in DYNAMIXEL model
    #define ADDR_BUS_WATCHDOG           546
    #define ADDR_GOAL_POSITION          564
    #define ADDR_GOAL_VELOCITY          552
    #define ADDR_GOAL_CURRENT           550
    #define ADDR_PRESENT_POSITION       580
    #define ADDR_PRESENT_VELOCITY       576
    #define ADDR_OPERATING_MODE         11
    #define CURRENT_MODE                0
    #define MINIMUM_POSITION_LIMIT      -150000  // Refer to the Minimum Position Limit of product eManual
    #define MAXIMUM_POSITION_LIMIT      150000  // Refer to the Maximum Position Limit of product eManual
    // #define POSITION_RESOLUTION         1003846
    #define VELOCITY_LIMIT              350
    #define CURRENT_LIMIT               22740
    #define BAUDRATE                    4000000
    #define RPM_PER_COUNT               (0.01)
    #define mA_PER_COUNT                1
    #define LEN_PRO_PRESENT_POSITION    4
    #define LEN_PRO_GOAL_CURRENT        2
#elif defined(XL320)
    #define ADDR_TORQUE_ENABLE          24
    #define ADDR_GOAL_POSITION          30
    #define ADDR_PRESENT_POSITION       37
    #define MINIMUM_POSITION_LIMIT      0  // Refer to the CW Angle Limit of product eManual
    #define MAXIMUM_POSITION_LIMIT      1023  // Refer to the CCW Angle Limit of product eManual
    #define BAUDRATE                    1000000  // Default Baudrate of XL-320 is 1Mbps
#endif

// DYNAMIXEL Protocol Version (1.0 / 2.0)
// https://emanual.robotis.com/docs/en/dxl/protocol2/
#define PROTOCOL_VERSION  2.0

// Factory default ID of all DYNAMIXEL is 1
// #define DXL_ID  1

// Use the actual port assigned to the U2D2.
// ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
// #define DEVICENAME  "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0
#define DXL_MOVING_STATUS_THRESHOLD     20  // DYNAMIXEL moving status threshold
#define ESC_ASCII_VALUE                 0x1b

#define FAIL_ -1
#define SUCCESS_ 0

//4DOF dynamixel robot arm interface
class ArticulatedSystem 
{     
public:
    enum Mode
    {
        CURRENT  = 0, 
        VELOCITY = 1, 
        POSITION = 3
    }; 
private:
    //Robot Dgree of Freedom
    const int dof;
    uint8_t mode;
    //joint state
    Eigen::VectorXd qpos;
    Eigen::VectorXd qvel;
    Eigen::VectorXd qtorque;
    

    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler *portHandler;

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler *packetHandler;

    uint8_t dxl_error = 0;             // DYNAMIXEL error
    
    bool dxl_addparam_result = false;                 // addParam result
    bool dxl_getdata_result = false;                  // GetParam result
    std::vector<dynamixel::GroupSyncRead> groupSyncReads;
    std::vector<dynamixel::GroupSyncWrite> groupSyncWrites; 

    int dxl_comm_result;

    const std::string dyx_device_name = "/dev/ttyUSB0";
    std::vector<uint8_t> dynamixel_id_set; 
    std::vector<int32_t> position_resolution; 
    std::vector<double> motor_torque_constants; 


    ///////////////////////////// The contents related to the motor torque constant //////////////////////////////////
    // const double XH430-W350 motor_torque_constant = 2.66409266409; 
    // XH430-W350 : (2.82-0.06)/(1.12-0.028*3)=2.66409266409 [Nm/A] from performance graph
    // Average the stall torque constant (3.1/1.2 + 3.4/1.3 + 4.2/1.5)/3 = 2.66623931624 [Nm/A] ()
    // Stall torque: 3.4 [N.m] (at 12.0 [V], 1.3 [A])
    // Gear ratio(N): 353.5
    // please refer to performance graph (https://emanual.robotis.com/docs/en/dxl/x/xh430-w350/#performance-graph)

    // const double XM430-W350 motor_torque_constant = 1.79320987654
    // XM430-W350 : (2.975-0.07)/(1.8-0.18)=1.79320987654 [Nm/A] from performance graph
    // Average the stall torque constant (3.8/2.1 + 4.1/2.3 + 4.8/2.7)/3 = 1.78997009432 [Nm/A]
    // Stall torque: 4.1 [N.m] (at 12.0 [V], 2.3 [A])
    // Gear ratio(N): 353.5
    // please refer to performance graph (https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#performance-graph)

    // const double XH430-W350 motor_torque_constant = 1.86147186147; 
    // XH540-W270 : (8.1-0.36)/(4.34-0.182)=1.86147186147 [Nm/A] from performance graph
    // Average the stall torque constant (9.2/4.5 + 9.9/4.9 + 11.7/5.9)/3 = 2.01596781839 [Nm/A]
    // Stall torque: 9.9 [N.m] (at 12.0 [V], 4.9 [A])
    // Gear ratio(N): 272.5
    // please refer to performance graph (https://emanual.robotis.com/docs/en/dxl/x/xh540-w270/#performance-graph)

    // const double XH540-W150 motor_torque_constant = 0.99951243296; 
    // XH540-W150 : (4.5-0.4)/(4.662-0.56)=0.99951243296 [Nm/A] from performance graph
    // Average the stall torque constant (6.6/4.5 + 7.1/4.9 + 8.5/5.9)/3 = 1.45210807487 [Nm/A]
    // Stall torque: 7.1 [N.m] (at 12.0 [V], 4.9 [A])
    // Gear ratio(N): 152.3
    // please refer to performance graph (https://emanual.robotis.com/docs/en/dxl/x/xh540-w270/#performance-graph)

    /******* XC330-T288-T datasheet, please refer to https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/
    const double XC330-T288-T motor_torque_constant = Unknown; 
    XC330-T288-T : Unknown [Nm/A] from performance graph
    Average the stall torque constant (0.76/0.61 + 0.92/0.8 + 1/0.88)/3 = (maybe) 1.17742175857 [Nm/A] 
    Stall torque: 0.76 [N.m] (at 9.0 [V], 0.61 [A])
                    0.92 [N.m] (at 11.1 [V], 0.80 [A])
                    1.00 [N.m] (at 12.0 [V], 0.88 [A])
    Gear ratio(N): 288.35
    */

    

public:

    ArticulatedSystem(int dof_args, uint8_t mode_args, const std::string dyx_device_name, 
                                    const std::vector<uint8_t>& dynamixel_id_set_args, 
                                    const std::vector<int32_t>& position_resolution_args, 
                                    const std::vector<double>& motor_torque_constants_args);
    ~ArticulatedSystem();

    double NormalizeAngle(double angle, double minimum_angle=-M_PI, double supremum_angle=M_PI);
    void DisableTorque();

    Eigen::VectorXd GetJointAngle();
    Eigen::VectorXd GetJointVelocity();
    Eigen::VectorXd GetTorqueStates();

    void SetGoalTorque(const Eigen::VectorXd &target_torque);

    void SetGoalPosition(const Eigen::VectorXd &target_position);

    void SetGoalVelocity(const Eigen::VectorXd &target_velocity);


};