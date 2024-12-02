/**
 * @file ArticulatedSystem.cpp
 * @author Jiwan Han (jw.han@kaist.ac.kr), Jinyeong Jung (jinyeong.jeong@kaist.ac.kr)
 * @brief 
 * @version 0.1
 * @date 2023-03-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "Controller/ArticulatedSystem.hpp"

ArticulatedSystem::ArticulatedSystem(int dof_args, uint8_t mode_args, const std::string dyx_device_name, 
                                     const std::vector<uint8_t>& dynamixel_id_set_args, 
                                     const std::vector<int32_t>& position_resolution_args, 
                                     const std::vector<double>& motor_torque_constants_args) 
                                    : dof(dof_args), mode(mode_args), qpos(dof), qvel(dof), qtorque(dof)
{   
    

    dynamixel_id_set = dynamixel_id_set_args;
    position_resolution = position_resolution_args;
    motor_torque_constants = motor_torque_constants_args;
    portHandler = dynamixel::PortHandler::getPortHandler(dyx_device_name.c_str());
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    // int index = 0;
    dxl_comm_result = COMM_TX_FAIL;             // Communication result

    // POSITION_RESOLUTION = position_resolution;
    groupSyncReads.push_back(dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION));
    groupSyncReads.push_back(dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY));
    groupSyncReads.push_back(dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_CURRENT,  LEN_PRESENT_CURRENT));

    groupSyncWrites.push_back(dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION));
    groupSyncWrites.push_back(dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY));
    groupSyncWrites.push_back(dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CURRENT,  LEN_GOAL_CURRENT));

    
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

    // Set the mode
    for (size_t i = 0; i < dof; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dynamixel_id_set.at(i), ADDR_OPERATING_MODE, static_cast<uint8_t>(mode), &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            printf("(%s, %d) %s\n", __FILE__, __LINE__, packetHandler->getTxRxResult(dxl_comm_result));

        }
        else if (dxl_error != 0) {
            printf("(%s, %d) %s\n", __FILE__, __LINE__, packetHandler->getRxPacketError(dxl_error));

        }
        else {

            switch (mode)
            {
            case Mode::CURRENT:
                printf("Succeeded operating mode to current mode.\n");
                break;
            case Mode::VELOCITY:
                printf("Succeeded operating mode to velocity mode.\n");
                break;
            case Mode::POSITION:                
                printf("Succeeded operating mode to position mode.\n");
                break;
            default:
                printf("\033[0;31m"); // set printf's color to the red.
                printf("%s, %d, [ERROR] \n", __FILE__, __LINE__);
                printf("\033[0m");
                break;
            }
        }
    }
    

    // Enable DYNAMIXEL Torque
    for (size_t i = 0; i < dof; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dynamixel_id_set.at(i), ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            printf("(%s, %d) %s\n", __FILE__, __LINE__, packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0) {
            printf("(%s, %d) %s\n", __FILE__, __LINE__, packetHandler->getRxPacketError(dxl_error));
        }
        else {
            printf("Succeeded enabling DYNAMIXEL Torque.\n");
        }
     
        // Disable BusWatch dog
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dynamixel_id_set.at(i), ADDR_BUS_WATCHDOG, 0, &dxl_error); // disable
        // Enable BusWatch dog (Search this very useful function.)
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dynamixel_id_set.at(i), ADDR_BUS_WATCHDOG, 3, &dxl_error); // 3 = 3 * 20 ms.

        // Add parameter storage for Dynamixel#1 present position value
        dxl_addparam_result = groupSyncReads.at(0).addParam(dynamixel_id_set.at(i));
        dxl_addparam_result = groupSyncReads.at(1).addParam(dynamixel_id_set.at(i));
        dxl_addparam_result = groupSyncReads.at(2).addParam(dynamixel_id_set.at(i));

        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", dynamixel_id_set.at(i));
            return ;
        }
        sleep(0.1);

    }
}

ArticulatedSystem::~ArticulatedSystem()
{   
    //clear the articulated system
    // articulated_system.clear();
    for (size_t i = 0; i < dof; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dynamixel_id_set.at(i), ADDR_BUS_WATCHDOG, 0, &dxl_error); // disable WATCHDOG
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dynamixel_id_set.at(i), ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else {
            printf("Succeeded disabling DYNAMIXEL Torque.\n");
        }
    }
    // Close port
    portHandler->closePort();
}

void ArticulatedSystem::DisableTorque()
{
    for (size_t i = 0; i < dof; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dynamixel_id_set.at(i), ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else {
            printf("Succeeded disabling DYNAMIXEL Torque.\n");
        }
    } 
}

double ArticulatedSystem::NormalizeAngle(double angle, double minimum_angle, double supremum_angle) 
{
    while(angle < minimum_angle)
    {
        angle += 2*M_PI;
    }
    while(angle >= supremum_angle)
    {
        angle -= 2*M_PI;
    }
    return angle;
}

Eigen::VectorXd ArticulatedSystem::GetJointAngle()
{   

    // Syncread present position
    dxl_comm_result = groupSyncReads.at(0).txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else 
    {
        for (size_t i = 0; i < dof; i++)
        {
            if (groupSyncReads.at(0).getError(dynamixel_id_set.at(i), &dxl_error))
            {
                printf("[ID:%03d] %s\n", dynamixel_id_set.at(i), packetHandler->getRxPacketError(dxl_error));
            }
        }
    }

    for (size_t i = 0; i < dof; i++)
    {
        // Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncReads.at(0).isAvailable(dynamixel_id_set.at(i), ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[%s, %d][ID:%03d] groupSyncRead getdata failed\n", __FILE__, __LINE__, dynamixel_id_set.at(i));
            return qpos;
        }
    }
    
    for (size_t i = 0; i < dof; i++)
    {
        int32_t qpos_counts = groupSyncReads.at(0).getData(dynamixel_id_set.at(i), ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        qpos(i) = (2 * M_PI / position_resolution.at(i)) * qpos_counts;
        qpos(i)=NormalizeAngle(qpos(i));
    }
    return qpos;
}

Eigen::VectorXd ArticulatedSystem::GetJointVelocity()
{   
    dxl_comm_result = groupSyncReads.at(1).txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else 
    {
        for (size_t i = 0; i < dof; i++)
        {
            if (groupSyncReads.at(1).getError(dynamixel_id_set.at(i), &dxl_error))
            {
                printf("[ID:%03d] %s\n", dynamixel_id_set.at(i), packetHandler->getRxPacketError(dxl_error));
            }
        }
    }

    for (size_t i = 0; i < dof; i++)
    {
        // Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncReads.at(1).isAvailable(dynamixel_id_set.at(i), ADDR_PRESENT_VELOCITY, LEN_PRESENT_POSITION);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[%s, %d][ID:%03d] groupSyncRead getdata failed\n", __FILE__, __LINE__, dynamixel_id_set.at(i));
            return qvel;
        }
    }
    
    for (size_t i = 0; i < dof; i++)
    {
        int32_t qvel_counts = groupSyncReads.at(1).getData(dynamixel_id_set.at(i), ADDR_PRESENT_VELOCITY, LEN_PRESENT_POSITION);
        qvel(i) = qvel_counts * RPM_PER_COUNT * 2*M_PI / (60.0);
    }
    return qvel;
}

Eigen::VectorXd ArticulatedSystem::GetTorqueStates()
{   

    // Syncread present position
    dxl_comm_result = groupSyncReads.at(2).txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else 
    {
        for (size_t i = 0; i < dof; i++)
        {
            if (groupSyncReads.at(2).getError(dynamixel_id_set.at(i), &dxl_error))
            {
                printf("[ID:%03d] %s\n", dynamixel_id_set.at(i), packetHandler->getRxPacketError(dxl_error));
            }
        }
    }

    for (size_t i = 0; i < dof; i++)
    {
        // Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncReads.at(2).isAvailable(dynamixel_id_set.at(i), ADDR_PRESENT_CURRENT, LEN_GOAL_CURRENT);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[%s, %d][ID:%03d] groupSyncRead getdata failed\n", __FILE__, __LINE__, dynamixel_id_set.at(i));
        }
    }
    
    for (size_t i = 0; i < dof; i++)
    {
        int16_t qcurrent_counts = groupSyncReads.at(2).getData(dynamixel_id_set.at(i), ADDR_PRESENT_CURRENT, LEN_GOAL_CURRENT);
        if (i==0) qtorque(i) = (mA_PER_COUNT/1000.0)*motor_torque_constants[i]*qcurrent_counts;
        else qtorque(i) = (1/1000.0)*motor_torque_constants[i]*qcurrent_counts;
    }
    return qtorque;
}

void ArticulatedSystem::SetGoalTorque(const Eigen::VectorXd &target_torque)
{

    // Eigen::VectorXd target_current(dof);

    for(int i=0; i<dof; i++)
    {
        uint8_t param_goal_current[LEN_GOAL_CURRENT];
        double target_current = target_torque(i)/motor_torque_constants[i];
        double target_current_mA = target_current * 1e3;
        int16_t dxl_goal_current = static_cast<int16_t> (( target_current_mA ) / mA_PER_COUNT);
        if (dxl_goal_current > CURRENT_LIMIT)
        {
            dxl_goal_current = CURRENT_LIMIT;
        }
        else if (dxl_goal_current < -CURRENT_LIMIT)
        {
            dxl_goal_current = -CURRENT_LIMIT;
        }
        // printf("dxl_goal_current: %d\n", dxl_goal_current);
        // uint16_t 
        param_goal_current[0] = DXL_LOBYTE(dxl_goal_current);
        param_goal_current[1] = DXL_HIBYTE(dxl_goal_current);

        dxl_addparam_result = groupSyncWrites.at(2).addParam(dynamixel_id_set.at(i), param_goal_current);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", dynamixel_id_set.at(i));
            return ;
        }
    }
    // Syncwrite goal position
    dxl_comm_result = groupSyncWrites.at(2).txPacket();
    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWrites.at(2).clearParam();
}


void ArticulatedSystem::SetGoalPosition(const Eigen::VectorXd &target_position)
{
    for (int i = 0; i < dof; i++)
    {

        uint8_t param_goal_position[LEN_GOAL_POSITION];
        double goal_position_count =  (NormalizeAngle(target_position(i)) / (2*M_PI)) * position_resolution.at(i);
        // double goal_position_count =  (target_position(i) / (2*M_PI)) * position_resolution.at(i);
        int32_t dxl_goal_position = goal_position_count;

        // uint16_t 
        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position));
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position));
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position));
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position));

        dxl_addparam_result = groupSyncWrites.at(0).addParam(dynamixel_id_set.at(i), param_goal_position);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", dynamixel_id_set.at(i));
            return ;
        }
    }
    // Syncwrite goal position
    dxl_comm_result = groupSyncWrites.at(0).txPacket();
    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWrites.at(0).clearParam();

}

void ArticulatedSystem::SetGoalVelocity(const Eigen::VectorXd &target_velocity)
{
    for(int i=0; i<dof; i++)
    {
        uint8_t param_goal_velocity[LEN_GOAL_VELOCITY];
        double goal_velocity_rpm =  target_velocity(i) * 60.0 / (2*M_PI);
        int32_t dxl_goal_velocity = ( goal_velocity_rpm ) / RPM_PER_COUNT;

        // uint16_t 
        param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_velocity));
        param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_velocity));
        param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_velocity));
        param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_velocity));

        dxl_addparam_result = groupSyncWrites.at(1).addParam(dynamixel_id_set.at(i), param_goal_velocity);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", dynamixel_id_set.at(i));
            return ;
        }
    }
    // Syncwrite goal position
    dxl_comm_result = groupSyncWrites.at(1).txPacket();
    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWrites.at(1).clearParam();
}