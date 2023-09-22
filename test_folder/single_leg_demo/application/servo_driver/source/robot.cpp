/*****************************************************************************
* Project: Krock robot
* chen chen
*****************************************************************************/

#include "robot.h"
#include <bitset>
#include <cstring>
#include <math.h>
#include "utils.hpp"

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
    motor_nums = num_Motors;
    scanned_model= new int[motor_nums];

    for (uint8_t i = 0; i < motor_nums; i++)
    {
        /* code */
        cout << "ID= " << ID_numbers[i] << endl;
        motor_id[i] =  ID_numbers[i]; 
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
    groupSyncRead_voltage = new dynamixel::GroupSyncRead(portHandler_, packetHandler_,
                                (uint16_t)P_PRESENT_INPUT_VOLTAGE, (uint16_t)LEN_PRESENT_INPUT_VOLTAGE);
    groupSyncRead_velocity = new dynamixel::GroupSyncRead(portHandler_, packetHandler_,
                                (uint16_t)P_PRESENT_VELOCITY, (uint16_t)LEN_PRESENT_VELOCITY);
    groupSyncRead_temperature = new dynamixel::GroupSyncRead(portHandler_, packetHandler_,
                                (uint16_t)P_PRESENT_TEMPERATURE, (uint16_t)LEN_PRESENT_TEMPERATURE);
                                
    groupSyncRead_current_velocity_position = new dynamixel::GroupSyncRead(portHandler_, packetHandler_,
                                (uint16_t)ADDR_INDIRECT_DATA_FOR_READ, (uint16_t)LEN_INDIRECTDATA_FOR_READ);

    //ping each motor to validate the communication is working
    uint8_t dxl_error = 0;                          // Dynamixel error
    for (int i=0; i<motor_nums; i++) {
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
    for (int i=0; i<motor_nums; i++) {
        dxl_addparam_result =groupSyncRead_position->addParam(ID_numbers[i]);
        if (dxl_addparam_result != true) {
            cout << "groupSyncRead addparam failed. ID = " << ID_numbers[i] << endl;
            return;
        }  
    }
    // Add parameter storage for SYNC indirect reading in this part to save time by only doing it only once 
    // as we're not going to actively change the ID to read, it's better to do it in this way
    //reading variables: position, current and voltage
    Assign_read_indirect_adress(ADDR_INDIRECT_ADDRESS_FOR_READ, ID_numbers);

    //INITIALIZE POSTURE:
    /* Initialize center values of servos -> used for the initial posture of the robot*/
    pos_center = new double[motor_nums];
    for (int i = 0; i < motor_nums; i++)
        pos_center[i] = 0;  
    SetInitialPosture(ID_numbers); 
}

Robot::~Robot()
{  
    set_all_torque_enable(0, motor_id);
    printf("88\n");
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
    for (int i=0; i<motor_nums; i++){
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


// get multiple variables simultaneously
void Robot::Assign_read_indirect_adress(int start_ind_adress_write, int *ids)
{
    bool dxl_addparam_result = false;    

    for (int i = 0; i < motor_nums; i++) {
        torque_enable(0, ids[i]); //torque OFF

        //read current
        writeRegister(ids[i], start_ind_adress_write + 0 + 0, 2, P_PRESENT_CURRENT + 0);
        writeRegister(ids[i], start_ind_adress_write + 0 + 2, 2, P_PRESENT_CURRENT + 1);

        //read voltage
        writeRegister(ids[i], start_ind_adress_write + 4 + 0, 2, P_PRESENT_VELOCITY + 0);
        writeRegister(ids[i], start_ind_adress_write + 4 + 2, 2, P_PRESENT_VELOCITY + 1);
        writeRegister(ids[i], start_ind_adress_write + 4 + 4, 2, P_PRESENT_VELOCITY + 2);
        writeRegister(ids[i], start_ind_adress_write + 4 + 6, 2, P_PRESENT_VELOCITY + 3);
        
        //read position
        writeRegister(ids[i], start_ind_adress_write + 12 + 0, 2, P_PRESENT_POSITION + 0);
        writeRegister(ids[i], start_ind_adress_write + 12 + 2, 2, P_PRESENT_POSITION + 1);
        writeRegister(ids[i], start_ind_adress_write + 12 + 4, 2, P_PRESENT_POSITION + 2);
        writeRegister(ids[i], start_ind_adress_write + 12 + 6, 2, P_PRESENT_POSITION + 3);

        torque_enable(1, ids[i]); //torque ON
        dxl_addparam_result = groupSyncRead_current_velocity_position->addParam(ids[i]);
        if (dxl_addparam_result != true) {
            cout << "groupSyncRead_current_velocity_position addparam failed ID = " << ids[i] << endl;
            return;
        }  
    }
}


// /* joint mode functions*/

// set the motors in comprehensive joint mode
void Robot::jointMode(int *ids, int32_t velocity, int32_t acceleration)
{
    for (int i = 0; i < motor_nums; i++) {
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
    for (int i = 0; i < motor_nums; i++) {
        torque_enable(0, ids[i]); //torque OFF
        //set position control mode
        writeRegister(ids[i], P_OPERATING_MODE, LEN_OPERATING_MODE, POSITION_CONTROL_MODE);
        torque_enable(1, ids[i]); //torque ON
    }
}

// set all motors in torque control mode
void Robot::setAllJointCurrentControlMode(int *ids)
{
    for (int i = 0; i < motor_nums; i++) {
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

    for (int i=0; i<motor_nums; i++) {
        //convert from Radians to motor value
        goalPosition = (angle2Position(angles[i], scanned_model[i], true));//true->angle in radians
        // goalPosition = angles[i];//true->angle in radians
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

    for (int i=0; i<motor_nums; i++) {

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
    jointMode(ids, 50, 80);

    //takes the robot to the initial position, using previous set speed
    setAllPositions(pos_center, ids);
    sleep(2);

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

    for (int i = 0; i < motor_nums; i++){
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

    for (int i = 0; i < motor_nums; i++) {
        torque_enable(0, ids[i]); //torque OFF
        writeRegister(ids[i], P_CURRENT_HIGH_LIMIT, LEN_CURRENT_LIMIT, current_value);
        torque_enable(1, ids[i]); //torque ON
    }
}

/* Set return delay time*/
void Robot::SetReturnDelayTime(int DelayTime,int *ids)
{
    for (int i = 0; i < motor_nums; i++) {
        torque_enable(0, ids[i]); //torque OFF
        writeRegister(ids[i], P_RETURN_DELAY_TIME, LEN_RETURN_DELAY_TIME, DelayTime);
        torque_enable(1, ids[i]); //torque ON
    }
}


/* get functions */
// get the position of all servos to angles in radians ------- unit: (2*Pi)/4096 radian
void Robot::getAllPositions(double *angles, int *ids)
{   
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    bool dxl_addparam_result = false;                 // addParam result

    // Syncread present position
    dxl_comm_result = groupSyncRead_position->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;

    for (int i = 0; i < motor_nums; i++)
    {
            angles[i]=groupSyncRead_position->getData(ids[i],
                        P_PRESENT_POSITION, LEN_PRESENT_POSITION);
            angles[i] = position2Angle(angles[i], scanned_model[i], true);
            // cout << " data :" << angles[i] << endl;
    }
}

// get the current of all servos    unit: 3.36mA
void Robot::getAllCurrents(double *currents, int *ids)
{
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    bool dxl_addparam_result = false;                 // addParam result
    int16_t temp_data=0;

    // Add parameter storage for reading -> this step can be done only once
    groupSyncRead_current->clearParam();
    for (int i = 0; i < motor_nums; i++) {
        dxl_addparam_result = groupSyncRead_current->addParam(ids[i]);
        if (dxl_addparam_result != true) {
            cout << "groupSyncRead_current addparam failed. ID = " << ids[i] << endl;
            return;
        }  
    }

    dxl_comm_result = groupSyncRead_current->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;

    for(int i = 0; i < motor_nums; i++)
    {
        temp_data = groupSyncRead_current->getData(ids[i],
                        (uint16_t)P_PRESENT_CURRENT, (uint16_t)LEN_PRESENT_CURRENT);
        currents[i] = double(temp_data);
    }
}




// get the input voltage of all servos  -- unit: 0.1V
void Robot::getAllVoltage(double *input_voltage, int *ids)
{
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    bool dxl_addparam_result = false;                 // addParam result

    // Add parameter storage for reading -> this step can be done only once
    groupSyncRead_voltage->clearParam();
    for (int i = 0; i < motor_nums; i++) {
        dxl_addparam_result = groupSyncRead_voltage->addParam(ids[i]);
        if (dxl_addparam_result != true) {
            cout << "groupSyncRead_voltage addparam failed. ID = " << ids[i] << endl;
            return;
        }  
    }

    dxl_comm_result = groupSyncRead_voltage->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;

    for(int i = 0; i < motor_nums; i++)
    {
        input_voltage[i] = groupSyncRead_voltage->getData(ids[i],
                        (uint16_t)P_PRESENT_INPUT_VOLTAGE, (uint16_t)LEN_PRESENT_INPUT_VOLTAGE);
    }
}



// get the velocity of all servos  ---- unit: 1r/min
void Robot::getAllVelocity(double *velocity, int *ids)
{
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    bool dxl_addparam_result = false;                 // addParam result
    int16_t temp_data=0;

    // Add parameter storage for reading -> this step can be done only once
    groupSyncRead_velocity->clearParam();
    for (int i = 0; i < motor_nums; i++) {
        dxl_addparam_result = groupSyncRead_velocity->addParam(ids[i]);
        if (dxl_addparam_result != true) {
            cout << "groupSyncRead_velocity addparam failed. ID = " << ids[i] << endl;
            return;
        }  
    }

    dxl_comm_result = groupSyncRead_velocity->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;

    for(int i = 0; i < motor_nums; i++)
    {

        temp_data = groupSyncRead_velocity->getData(ids[i],
                        (uint16_t)P_PRESENT_VELOCITY, (uint16_t)LEN_PRESENT_VELOCITY);
        velocity[i] = (double)temp_data/1000;
    }
}



// get the temperature of all servos ----- unit:­°C
void Robot::getAllTemperatures(double *temperatures, int *ids)
{
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    bool dxl_addparam_result = false;                 // addParam result

    // Add parameter storage for reading -> this step can be done only once
    groupSyncRead_temperature->clearParam();
    for (int i = 0; i < motor_nums; i++) {
        dxl_addparam_result = groupSyncRead_temperature->addParam(ids[i]);
        if (dxl_addparam_result != true) {
            cout << "groupSyncRead_current addparam failed. ID = " << ids[i] << endl;
            return;
        }
    }

    dxl_comm_result = groupSyncRead_temperature->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;

    for (int i=0; i<motor_nums; i++)
        temperatures[i] = groupSyncRead_temperature->getData(ids[i],
                            (uint16_t)P_PRESENT_TEMPERATURE, (uint16_t)LEN_PRESENT_TEMPERATURE);
}


// get the current,velocity and position of all servos   
void Robot::getAllCurrentsVelocityPosition(double *currents, double *velocity, double *position, int *ids)
{
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    bool dxl_addparam_result = false;                 // addParam result
    bool dxl_getdata_result = false;                  // getParam result
    int16_t temp_data=0;

    // Syncread present current,velocity, position
    dxl_comm_result = groupSyncRead_current_velocity_position->txRxPacket();

    if (dxl_comm_result != COMM_SUCCESS)
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;

    for (int i = 0; i < motor_nums; i++)
    {
        dxl_getdata_result = groupSyncRead_current_velocity_position->isAvailable(ids[i], ADDR_INDIRECT_DATA_FOR_READ + 0, LEN_PRESENT_CURRENT);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", ids[i]);
        }

        dxl_getdata_result = groupSyncRead_current_velocity_position->isAvailable(ids[i], ADDR_INDIRECT_DATA_FOR_READ + LEN_PRESENT_CURRENT, LEN_PRESENT_VELOCITY);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", ids[i]);
        }

        dxl_getdata_result = groupSyncRead_current_velocity_position->isAvailable(ids[i], ADDR_INDIRECT_DATA_FOR_READ + LEN_PRESENT_CURRENT + LEN_PRESENT_VELOCITY, LEN_PRESENT_POSITION);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", ids[i]);
        }

        temp_data = groupSyncRead_current_velocity_position->getData(ids[i], ADDR_INDIRECT_DATA_FOR_READ + 0, LEN_PRESENT_CURRENT);
        currents[i] = (double)temp_data;

        temp_data = groupSyncRead_current_velocity_position->getData(ids[i], ADDR_INDIRECT_DATA_FOR_READ + LEN_PRESENT_CURRENT, LEN_PRESENT_VELOCITY);
        velocity[i] = (double)temp_data/1000;

        position[i] = groupSyncRead_current_velocity_position->getData(ids[i], ADDR_INDIRECT_DATA_FOR_READ + LEN_PRESENT_CURRENT + LEN_PRESENT_VELOCITY, LEN_PRESENT_POSITION);
        position[i] = position2Angle(position[i], scanned_model[i], true);
        // cout << " current data :" << temp_data << endl;
        // cout << " velocity data :" << temp_data << endl;
        // cout << " position data :" << position[i] << endl;
        
    }
}
