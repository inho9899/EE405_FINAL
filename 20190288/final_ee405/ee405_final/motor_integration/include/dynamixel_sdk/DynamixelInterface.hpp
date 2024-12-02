#pragma once

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
// #include <termios.h> // because of multiple declaration.
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
    #define ADDR_OPERATING_MODE         11
    #define MINIMUM_POSITION_LIMIT      0  // Refer to the Minimum Position Limit of product eManual
    #define MAXIMUM_POSITION_LIMIT      4095  // Refer to the Maximum Position Limit of product eManual
    #define POSITION_RESOLUTION         4096  // 12 bits encoder
    #define VELOCITY_LIMIT              350  // 0.229 rpm per counts
    #define BAUDRATE                    4000000
    #define RPM_PER_COUNT               (0.229)
    #define mA_PER_COUNT                2.69
    #define LEN_PRO_PRESENT_POSITION    4
    #define LEN_PRO_GOAL_CURRENT        2
    #define CURRENT_MODE                0
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
    #define MINIMUM_POSITION_LIMIT      -150000  // Refer to the Minimum Position Limit of product eManual
    #define MAXIMUM_POSITION_LIMIT      150000  // Refer to the Maximum Position Limit of product eManual
    // #define POSITION_RESOLUTION         1003846
    #define VELOCITY_LIMIT              350
    #define CURRENT_LIMIT               22740
    #define BAUDRATE                    4000000
    #define RPM_PER_COUNT               (0.01)
    #define mA_PER_COUNT                1
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

class DynamixelInterface
{
private:
    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler *portHandler;

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler *packetHandler;

    uint8_t dxl_error = 0;                          // DYNAMIXEL error
    uint8_t dynamixel_id = 1;                          // DYNAMIXEL error
    #if defined(XL320)
    int16_t dxl_present_position = 0;  // XL-320 uses 2 byte Position data
    #else
    int32_t dxl_present_position = 0;  // Read 4 byte Position data
    int32_t dxl_present_velocity = 0;  // Read 4 byte Position data
    #endif

    int dxl_comm_result;

    double qpos;
    double qvel;
    double qpos_offset;
    

public:
    DynamixelInterface(const std::string dyx_device_name, const uint8_t dynamixel_id_args, const double qpos_offset_args = 0, int32_t position_resolution = 4096)
    {
        dynamixel_id = dynamixel_id_args;
        qpos_offset = qpos_offset_args;

        portHandler = dynamixel::PortHandler::getPortHandler(dyx_device_name.c_str());
        packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
        // int index = 0;
        dxl_comm_result = COMM_TX_FAIL;             // Communication result

        // Open port
        if (portHandler->openPort()) {
            printf("Succeeded to open the port!\n");
        }
        else {
            printf("Failed to open the port!\n");
            return;
        }

        // Set port baudrate
        if (portHandler->setBaudRate(BAUDRATE)) {
            printf("Succeeded to change the baudrate!\n");
        }
        else {
            printf("Failed to change the baudrate!\n");
            return;
        }

        // // Enable DYNAMIXEL Torque
        // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dynamixel_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
 
        // if (dxl_comm_result != COMM_SUCCESS) {
        //     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        // }
        // else if (dxl_error != 0) {
        //     printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        // }
        // else {
        //     printf("Succeeded enabling DYNAMIXEL Torque.\n");
        // }
        

        // Enable BusWatch dog
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dynamixel_id, ADDR_BUS_WATCHDOG, 0, &dxl_error); // disable
        // Enable BusWatch dog
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dynamixel_id, ADDR_BUS_WATCHDOG, 3, &dxl_error); // 3 = 3 * 20 ms.
        sleep(0.5);
    }
    ~DynamixelInterface() 
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dynamixel_id, ADDR_BUS_WATCHDOG, 0, &dxl_error); // disable WATCHDOG
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dynamixel_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else {
            printf("Succeeded disabling DYNAMIXEL Torque.\n");
        }
    
        // Close port
        portHandler->closePort();
    }

    double GetQpos()
    {
        return qpos;
    }

    double GetQvel()
    {
        return qvel;
    }

    // /**
    //  * @brief normalize angle to [0, 2*pi]
    //  * 
    //  * @param angle 
    //  */
    // double NormalizeAngle(double angle, double minimum_angle = 0, double supremum_angle = 2*M_PI) 
    // {
    //     while(angle < minimum_angle)
    //     {
    //         angle += 2*M_PI;
    //     }
    //     while(angle >= supremum_angle)
    //     {
    //         angle -= 2*M_PI;
    //     }
    //     return angle;
    // }

    int ObtainToQposAndQvelFromDyanmixel()
    {
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dynamixel_id, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            return -1;
        }
        else if (dxl_error != 0) 
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            return -1;
        }

        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dynamixel_id, ADDR_PRESENT_VELOCITY, (uint32_t*)&dxl_present_velocity, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            return -1;
        }
        else if (dxl_error != 0) 
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            return -1;
        }

        qpos = 2 * M_PI * static_cast<double>(dxl_present_position) / static_cast<double>(POSITION_RESOLUTION) + qpos_offset; // TODO : utilizing HOMING OFFSET of the Dynamixel.
        qvel = static_cast<double>(dxl_present_velocity) * RPM_PER_COUNT * 2*M_PI / (60.0);

        #ifdef PRINT_VERBOSE
        printf("[ID:%03d] Present Position:%f\n", dynamixel_id, qpos);
        printf("[ID:%03d] Present Velocity:%f\n", dynamixel_id, qvel);
        #endif

        return 0;

    }

    int SetGoalCurrentToDynamixel(double goal_current_A)
    {
        // goal_velocity_rads
        double goal_current_mA =  goal_current_A * 1e3;
        int16_t dxl_goal_current = ( goal_current_mA ) / mA_PER_COUNT;
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dynamixel_id, ADDR_GOAL_CURRENT, dxl_goal_current, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            return -1;
        }
        else if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            return -1;
        }
        return 0;
        
    }

    int SetGoalVelocityToDynamixel(double goal_velocity_rads)
    {
        // goal_velocity_rads
        double goal_velocity_rpm =  goal_velocity_rads * 60.0 / (2*M_PI);
        int32_t dxl_goal_velocity = ( goal_velocity_rpm ) / RPM_PER_COUNT;
        #ifdef PRINT_VERBOSE
        printf("dxl_goal_velocity : %d\n", dxl_goal_velocity);
        #endif
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dynamixel_id, ADDR_GOAL_VELOCITY, dxl_goal_velocity, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            return -1;
        }
        else if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            return -1;
        }
        return 0;
    }

    
    int SetGoalPositionToDynamixel(double goal_position_rad_reflected_offset)
    {
        // goal_velocity_rads
        double goal_position_rad = goal_position_rad_reflected_offset - qpos_offset; // TODO : utilizing HOMING OFFSET of the Dynamixel.
        goal_position_rad = NormalizeAngle(goal_position_rad, -M_PI, M_PI);
        double goal_position_count =  (goal_position_rad / (2*M_PI)) * POSITION_RESOLUTION;
        int32_t dxl_goal_position = goal_position_count;
        #ifdef PRINT_VERBOSE
        printf("dxl_goal_position : %d\n", dxl_goal_position);
        #endif
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dynamixel_id, ADDR_GOAL_POSITION, dxl_goal_position, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            return -1;
        }
        else if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            return -1;
        }
        return 0;
    }

};