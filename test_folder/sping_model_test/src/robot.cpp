/*****************************************************************************
* Project: AgnathaX robot
* Copyright 2021-2023 Laura Paez Coy and Kamilo Melo
* This code is under MIT licence: https://opensource.org/licenses/MIT
* Laura.Paez@KM-RoBota.com, 04-2023
* Kamilo.Melo@KM-RoBota.com, 05-2023
*****************************************************************************/

#include "robot.h"
#include <bitset>
#include <cstring>

using namespace std;

// Current units and values defined for Dynamixel MX-64R (2.0)
// Ref : https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/#current-limit
#define CURRENT_UNIT                        3.36 // [mA]
#define CURRENT_LIMIT                       1941 // [unit]  -->  Dynamixel units


/* 
    Robot constructor 
    Parameters:
    [in]	num_Motors	Number of motors in the robot
    [in]	baudrate	Baudrate of the port handling communication with motors
    [in]	ID_numbers	List of IDs of the motors
    [in]	portName	Name of the port handling communication with motors
    */
Robot::Robot(int num_Motors,int baudrate, int *ID_numbers, const char *portName)
{   

    const char *log;
    bool result = false;
    uint16_t model_number = 0;
    scanned_model= new int[num_Motors];

    for (uint8_t i = 0; i < num_Motors; i++)
    {
        /* code */
        cout << "ID= " << ID_numbers[i] << endl;
    }
    

    //connect U2D2
    init(portName, baudrate, 2.0);

    //add handler #1 (MAX ALLOWED = 5)
    groupSyncWrite_position = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_,
                                (uint16_t)P_GOAL_POSITION, (uint16_t)LEN_GOAL_POSITION);
    groupSyncWrite_current = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_,
                                (uint16_t)P_GOAL_CURRENT, (uint16_t)LEN_GOAL_CURRENT);

    //add handler #1 (MAX ALLOWED = 5)

    // groupBulkRead_position = new dynamixel::GroupBulkRead(portHandler_, packetHandler_);
    // groupBulkRead_current = new dynamixel::GroupBulkRead(portHandler_, packetHandler_);
    // groupBulkRead_temperature = new dynamixel::GroupBulkRead(portHandler_, packetHandler_);
                                
    groupSyncRead_position = new dynamixel::GroupSyncRead(portHandler_, packetHandler_,
                                (uint16_t)P_PRESENT_POSITION, (uint16_t)LEN_PRESENT_POSITION);
    groupSyncRead_current = new dynamixel::GroupSyncRead(portHandler_, packetHandler_,
                                (uint16_t)P_PRESENT_CURRENT, (uint16_t)LEN_PRESENT_CURRENT);
    groupSyncRead_temperature = new dynamixel::GroupSyncRead(portHandler_, packetHandler_,
                                (uint16_t)P_PRESENT_TEMPERATURE, (uint16_t)LEN_PRESENT_TEMPERATURE);
    // groupSyncRead_position_current_voltage = new dynamixel::GroupBulkRead(portHandler_, packetHandler_,
    //                             (uint16_t)ADDR_INDIRECT_DATA_FOR_READ, (uint16_t)LEN_INDIRECTDATA_FOR_READ);

    //ping each motor to validate the communication is working
    uint8_t dxl_error = 0;                          // Dynamixel error
    for (int i=0; i<num_Motors; i++) {
        result = packetHandler_->ping(portHandler_, ID_numbers[i], &model_number, &dxl_error);
        if (result != COMM_SUCCESS) {
            cout << "Failed to ping, check config file and motor ID: " << ID_numbers[i] << endl;
            cout << packetHandler_->getTxRxResult(result) << endl;
            return;
        }
        else {
            cout << "id: " << ID_numbers[i] << ", model number : " << model_number << endl;
            scanned_model[i]=model_number;
        }
    }

    // Add parameter storage for SYNC reading in this part to save time by only doing it only once 
    // as we're not going to actively change the ID to read, it's better to do it in this way
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    bool dxl_addparam_result = false;                 // addParam result

    groupSyncRead_position->clearParam();
    for (int i=0; i<NumMotors; i++) {
        dxl_addparam_result =groupSyncRead_position->addParam(ID_numbers[i]);
        if (dxl_addparam_result != true) {
            cout << "groupSyncRead addparam failed. ID = " << ID_numbers[i] << endl;
            return;
        }  
    }
    // Add parameter storage for SYNC indirect reading in this part to save time by only doing it only once 
    // as we're not going to actively change the ID to read, it's better to do it in this way
    //reading variables: position, current and voltage
    // Assign_read_indirect_adress(ADDR_INDIRECT_ADDRESS_FOR_READ, ID_numbers);

    //INITIALIZE POSTURE:
    /* Initialize center values of servos -> used for the initial posture of the robot*/
    pos_center = new double[num_Motors];
    for (int i = 0; i < num_Motors; i++)
        pos_center[i] = 3048;  
    SetInitialPosture(ID_numbers); 
}

Robot::~Robot()
{
}

void Robot::init(const char *device_name, int baud_rate, float protocol_version)
{
    portHandler_ = dynamixel::PortHandler::getPortHandler(device_name);
    if (!portHandler_->openPort()) {
        cout<< "Failed to open the motors port!" <<endl;
        return ;
    }
    else
        cout<< "Succeed to open the motors port" <<endl;

    if (!portHandler_->setBaudRate(baud_rate)) {
        cout<< "Failed to set baudrate!" <<endl;
        return ;
    }
    else
        cout<< "Succeeded to change the baudrate!" <<endl;

    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);
}

void Robot::torque_enable(int on_off,int id)
{ //1->enable
        writeRegister(id, P_TORQUE_ENABLE, LEN_TORQUE_ENABLE, on_off);//on_off=1 [ON]
}


void Robot::set_all_torque_enable(int on_off, int *ids)
{ //1->enable
    for (int i=0; i<NumMotors; i++){
        writeRegister(ids[i], P_TORQUE_ENABLE, LEN_TORQUE_ENABLE, on_off);//on_off=1 [ON]
    }
}

/* interface with the motors functions */
void Robot::getParameter(int32_t data, uint8_t *param, int param_array_size)
{
    if (param_array_size == 4) {
        param[0] = DXL_LOBYTE(DXL_LOWORD(data));
        param[1] = DXL_HIBYTE(DXL_LOWORD(data));
        param[2] = DXL_LOBYTE(DXL_HIWORD(data));
        param[3] = DXL_HIBYTE(DXL_HIWORD(data));
    }
    else if (param_array_size == 2) {
        param[0] = DXL_LOBYTE(DXL_LOWORD(data));
        param[1] = DXL_HIBYTE(DXL_LOWORD(data));
    }
    else if (param_array_size == 1) {
        param[0] = DXL_LOBYTE(DXL_LOWORD(data));
    }
    else
        cout<< "wrong number of parameters" <<endl;
}

bool Robot::writeRegister(uint8_t id, uint16_t address, uint16_t length, int32_t data)
{
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    uint8_t dxl_error = 0;                            // Dynamixel error
    int a = id;

    if (length == 1) {
        uint8_t data_byte = (uint8_t)data;
        dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_,
                            id, address, data_byte, &dxl_error);
    }
    else if (length == 2) {
        uint16_t data_byte = (uint16_t)data;
        dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_,
                            id, address, data_byte, &dxl_error);
    }
    else if (length == 4) {
        uint32_t data_byte = (uint32_t)data;
        dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_,
                            id, address, data_byte, &dxl_error);
    }
    else
        cout<<"Check data length"<<endl;


    if (dxl_comm_result != COMM_SUCCESS) {
        cout<< packetHandler_->getTxRxResult(dxl_comm_result);
        return false;
    }
    else if (dxl_error != 0) {
        cout<< packetHandler_->getTxRxResult(dxl_error);
        return false;
    }
    else
        return true;
}

bool Robot::readRegister(uint8_t id, uint16_t address, uint16_t length, int32_t *data)
{
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    uint8_t dxl_error = 0;                            // Dynamixel error

    uint8_t data_1_byte  = 0;
    uint16_t data_2_byte = 0;
    uint32_t data_4_byte = 0;


    if (length == 1) {
        dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_,
                            id, address, &data_1_byte, &dxl_error);
    }
    else if (length == 2) {
        dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_,
                            id, address, &data_2_byte, &dxl_error);
    }
    else if (length == 4) {
        dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_,
                            id, address, &data_4_byte, &dxl_error);
    }
    else
        cout << "Check data length" << endl;

    if (dxl_comm_result != COMM_SUCCESS) {
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;
        return false;
    }
    else if (dxl_error != 0) {
        cout << packetHandler_->getTxRxResult(dxl_error) << endl;
        return false;
    }
    else {
        if (length == 1)
            *data = data_1_byte;
        else if (length == 2)
            *data = data_2_byte;
        else if (length == 4)
            *data = data_4_byte;
        else
            *data = data_1_byte;
        return true;
    }
    return false;
}

// /* joint mode functions*/

// set the motors in comprehensive joint mode
void Robot::jointMode(int *ids, int32_t velocity, int32_t acceleration)
{
    for (int i = 0; i < NumMotors; i++) {
        torque_enable(0, ids[i]); //torque OFF
        //set position control mode
        writeRegister(ids[i], P_OPERATING_MODE, LEN_OPERATING_MODE, POSITION_CONTROL_MODE);
        //set velocity
        writeRegister(ids[i], P_PROFILE_VELOCITY, LEN_PROFILE, velocity);
        //set acceleration
        writeRegister(ids[i], P_PROFILE_ACCELERATION, LEN_PROFILE, acceleration);

        torque_enable(1, ids[i]); //torque ON
    }
}

// set all motors in position control mode
void Robot::setAllJointPositionControlMode(int *ids)
{
    for (int i = 0; i < NumMotors; i++) {
        torque_enable(0, ids[i]); //torque OFF
        //set position control mode
        writeRegister(ids[i], P_OPERATING_MODE, LEN_OPERATING_MODE, POSITION_CONTROL_MODE);
        torque_enable(1, ids[i]); //torque ON
    }
}

// set all motors in torque control mode
void Robot::setAllJointCurrentControlMode(int *ids)
{
    for (int i = 0; i < NumMotors; i++) {
        torque_enable(0, ids[i]); //torque OFF
        //set current control mode
        writeRegister(ids[i], P_OPERATING_MODE, LEN_OPERATING_MODE, TORQUE_CONTROL_MODE);
        cout << "set successful\n";
        torque_enable(1, ids[i]); //torque ON
    }
}

// /* set functions */

// Sets the position of all servos given the angles in radians
void Robot::setAllPositions(double *angles, int *ids)
{
    int goalPosition = 2048;
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    bool dxl_addparam_result = false;                 // addParam result
    uint8_t param_goal_position[LEN_GOAL_POSITION] = {0};

    for (int i=0; i<NumMotors; i++) {
        //convert from Radians to motor value
        // goalPosition = (angle2Position(angles[i], scanned_model[i], true));//true->angle in radians
        goalPosition = angles[i];//true->angle in radians
        getParameter(goalPosition, param_goal_position, LEN_GOAL_POSITION);
        // cout << "ID = " << ids[i] << endl;
        // cout << "position = " << angles[i] << endl;
        dxl_addparam_result = groupSyncWrite_position->addParam(ids[i], param_goal_position);
        if (dxl_addparam_result != true) {
            cout << "groupSyncWrite addparam failed. ID = " << ids[i] << endl;
            return;
        }
    }
    dxl_comm_result = groupSyncWrite_position->txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;
    // Clear syncwrite parameter storage
    groupSyncWrite_position->clearParam();
}

// set the current (torque) to the motors
void Robot::setAllCurrents(double *currents, int *ids) //current unit [ticks]
{
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    bool dxl_addparam_result = false;                 // addParam result
    uint8_t param_goal_current[LEN_GOAL_CURRENT] = {0};
    double temp_data=0;

    for (int i=0; i<NumMotors; i++) {

        // temp_data = currents[i];
        // if(temp_data < 0)
        // {
        //     temp_data *= -1;
        //     temp_data += 1023;
        // }
        getParameter(currents[i], param_goal_current, LEN_GOAL_CURRENT);
        dxl_addparam_result = groupSyncWrite_current->addParam(ids[i], param_goal_current);
        if (dxl_addparam_result != true)
        {
            cout << "groupSyncWrite addparam failed. ID = " << ids[i] << endl;
            return;
        }
    }
    dxl_comm_result = groupSyncWrite_current->txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;
    // Clear syncwrite parameter storage
    groupSyncWrite_current->clearParam();
}

// set the position of all servos to center
void Robot::SetInitialPosture( int *ids)
{
    //changes goal velocity. 2nd argument in jointMode is speed 0->fast 50->example of slow value
    jointMode(ids, 50, 0);

    //takes the robot to the initial position, using previous set speed
    setAllPositions(pos_center, ids);
    sleep(1);

    // //comes back to fast speed
    jointMode(ids, 0, 0);
}

/* 
    Close the communication port to the motors.
    */
void Robot::Close_port(int *ids)
{
    portHandler_->closePort();
}


/* 
    set the limit to the angles [degree] of all motors 
Parameters:
    [in]	CW	Maximum allowed clockwise angle [deg]
    [in]	CCW	Maximum allowed counter-clockwise angle [deg]
    [in]	ids	ID numbers of all motors
Return values
    void	
    */
void Robot::Anglelimits(int CW, int CCW, int *ids) //CW minimum CCW maximum [in degrees!]
{
    //from deg to motor value
    // CW = angle2Position(CW, scanned_model[1], false);
    // CCW = angle2Position(CCW, scanned_model[1], false);

    for (int i = 0; i < NumMotors; i++){
        torque_enable(0, ids[i]); //torque OFF
        writeRegister(ids[i], P_POSITION_LOW_LIMIT, LEN_POSITION_LIMIT, CW);
        writeRegister(ids[i], P_POSITION_HIGH_LIMIT, LEN_POSITION_LIMIT, CCW);
        torque_enable(1, ids[i]); //torque ON
    }
}   

/* Sets the current maximum limit of all servos to a specific value in % of the current limit */
void Robot::CurrentLimit(float current_percentage,int *ids)
{
    //from percentage of current limit to Dynamixel units [unit]
    // int current_value=(int) CURRENT_LIMIT*(current_percentage/100) - 1;
    int current_value = (int) CURRENT_LIMIT*(current_percentage/100);

    for (int i = 0; i < NumMotors; i++) {
        torque_enable(0, ids[i]); //torque OFF
        writeRegister(ids[i], P_CURRENT_HIGH_LIMIT, LEN_CURRENT_LIMIT, current_value);
        // writeRegister(ids[i], P_MAX_TORQUE, LEN_CURRENT_LIMIT, current_value);
        // writeRegister(ids[i], P_TORQUE_LIMIT, LEN_CURRENT_LIMIT, current_value);
        torque_enable(1, ids[i]); //torque ON
    }
}

/* Set return delay time*/
void Robot::SetReturnDelayTime(int DelayTime,int *ids)
{
    for (int i = 0; i < NumMotors; i++) {
        torque_enable(0, ids[i]); //torque OFF
        writeRegister(ids[i], P_RETURN_DELAY_TIME, LEN_RETURN_DELAY_TIME, DelayTime);
        torque_enable(1, ids[i]); //torque ON
    }
}


/* get functions */
// get the position of all servos to angles in radians
void Robot::getAllPositions(double *angles, int *ids)
{   
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    bool dxl_addparam_result = false;                 // addParam result

    // Syncread present position
    dxl_comm_result = groupSyncRead_position->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;

    for (int i = 0; i < NumMotors; i++)
            angles[i]=groupSyncRead_position->getData(ids[i],
                        P_PRESENT_POSITION, LEN_PRESENT_POSITION);
}

// get the current of all servos
void Robot::getAllCurrents(double *currents, int *ids)
{
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    bool dxl_addparam_result = false;                 // addParam result
    uint16_t temp_data=0;

    // Add parameter storage for reading -> this step can be done only once
    groupSyncRead_current->clearParam();
    for (int i = 0; i < NumMotors; i++) {
        dxl_addparam_result = groupSyncRead_current->addParam(ids[i]);
        if (dxl_addparam_result != true) {
            cout << "groupSyncRead_current addparam failed. ID = " << ids[i] << endl;
            return;
        }  
    }

    dxl_comm_result = groupSyncRead_current->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;

    for(int i = 0; i < NumMotors; i++)
    {
        temp_data = groupSyncRead_current->getData(ids[i],
                        (uint16_t)P_PRESENT_CURRENT, (uint16_t)LEN_PRESENT_CURRENT);
        if(temp_data > 0x0800)
        {
            currents[i] = temp_data - 0x0800;
            currents[i] *= -1; 
        }
        else if(temp_data < 0x0800)
        {
            currents[i] = 0x0800 - temp_data;
        }
        else
        {
            currents[i] = 0;
        }
        // currents[i] = groupBulkRead_current->getData(ids[i],
        //                 (uint16_t)P_PRESENT_CURRENT, (uint16_t)LEN_PRESENT_CURRENT);
    }
}

// get the temperature of all servos
void Robot::getAllTemperatures(double *temperatures, int *ids)
{
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    bool dxl_addparam_result = false;                 // addParam result

    // Add parameter storage for reading -> this step can be done only once
    groupSyncRead_temperature->clearParam();
    for (int i = 0; i < NumMotors; i++) {
        dxl_addparam_result = groupSyncRead_temperature->addParam(ids[i]);
        if (dxl_addparam_result != true) {
            cout << "groupSyncRead_current addparam failed. ID = " << ids[i] << endl;
            return;
        }
    }

    dxl_comm_result = groupSyncRead_temperature->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;

    for (int i=0; i<NumMotors; i++)
        temperatures[i] = groupSyncRead_temperature->getData(ids[i],
                            (uint16_t)P_PRESENT_TEMPERATURE, (uint16_t)LEN_PRESENT_TEMPERATURE);
}

// // // get multiple variables simultaneously
// // void Robot::Assign_read_indirect_adress(int start_ind_adress_write, int *ids)
// // {
// //     bool dxl_addparam_result = false;    

// //     for (int i = 0; i < NumMotors; i++) {
// //         torque_enable(0, ids[i]); //torque OFF
// //         //read position
// //         writeRegister(ids[i], start_ind_adress_write + 0 + 0, 2, P_PRESENT_POSITION + 0);
// //         writeRegister(ids[i], start_ind_adress_write + 0 + 2, 2, P_PRESENT_POSITION + 1);
// //         writeRegister(ids[i], start_ind_adress_write + 0 + 4, 2, P_PRESENT_POSITION + 2);
// //         writeRegister(ids[i], start_ind_adress_write + 0 + 6, 2, P_PRESENT_POSITION + 3);
// //         //read current
// //         writeRegister(ids[i], start_ind_adress_write + 8 + 0, 2, P_PRESENT_CURRENT + 0);
// //         writeRegister(ids[i], start_ind_adress_write + 8 + 2, 2, P_PRESENT_CURRENT + 1);
// //         //read voltage
// //         writeRegister(ids[i], start_ind_adress_write + 12 + 0, 2, P_PRESENT_INPUT_VOLTAGE + 0);
// //         writeRegister(ids[i], start_ind_adress_write + 12 + 2, 2, P_PRESENT_INPUT_VOLTAGE + 1);

// //         torque_enable(1, ids[i]); //torque ON
// //         dxl_addparam_result = groupSyncRead_position_current_voltage->addParam(ids[i]);
// //         if (dxl_addparam_result != true) {
// //             cout << "groupBulkRead_current addparam failed ID = " << ids[i] << endl;
// //             return;
// //         }  
// //     }
// // }

// // void Robot::IndirectAddressReading(int *ids, double *angles, double *current, double *voltage)
// // {
// //     int dxl_comm_result = COMM_TX_FAIL;               // Communication result
// //     bool dxl_addparam_result = false;                 // addParam result
// //     int current_temp = 0;

// //     dxl_comm_result = groupSyncRead_position_current_voltage->txRxPacket();
// //     if (dxl_comm_result != COMM_SUCCESS)
// //         cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;

// //     for (int i = 0; i < NumMotors; i++) {
// //         // Get present position value [radians]
// //         angles[i] = groupSyncRead_position_current_voltage->getData(ids[i],
// //                    ADDR_INDIRECT_DATA_FOR_READ,LEN_PRESENT_POSITION);
// //         // Get present current value in mA
// //         current[i] = (short)groupSyncRead_position_current_voltage->getData(ids[i],
// //                     ADDR_INDIRECT_DATA_FOR_READ + 4,LEN_PRESENT_CURRENT);
// //         // Get present voltage value in volts
// //         voltage[i] = groupSyncRead_position_current_voltage->getData(ids[i],
// //                     ADDR_INDIRECT_DATA_FOR_READ+4+2,LEN_PRESENT_INPUT_VOLTAGE);
// //     }

// //     // formating
// //     for (int i = 0; i < NumMotors; i++) {
// //         angles[i] = position2Angle(angles[i],scanned_model[i],true);  //in radians
// //         current[i] = current[i]*CURRENT_UNIT;  // in mA
// //         voltage[i] = voltage[i]/10;  // in v
// //     }
// // }
