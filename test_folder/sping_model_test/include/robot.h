#ifndef __ROBOT_HPP__
#define __ROBOT_HPP__


#include "dynamixel_sdk.h"    
#include "control_table_constants.hpp"
#include <iostream>
#include <unistd.h>


using namespace std;

typedef struct {
    dynamixel::GroupSyncWrite *groupSyncWrite;
} SyncWriteHandler;

typedef struct {
    dynamixel::GroupBulkRead  *groupSyncRead;
} SyncReadHandler;


class Robot {

private:
    dynamixel::PortHandler   *portHandler_;
    dynamixel::PacketHandler *packetHandler_;

    dynamixel::GroupSyncWrite *groupSyncWrite_position;
    dynamixel::GroupSyncWrite *groupSyncWrite_current;


    dynamixel::GroupSyncRead *groupSyncRead_position;
    dynamixel::GroupSyncRead *groupSyncRead_current;
    dynamixel::GroupSyncRead *groupSyncRead_voltage;
    dynamixel::GroupSyncRead *groupSyncRead_velocity;
    dynamixel::GroupSyncRead *groupSyncRead_temperature;

    // SyncWriteHandler syncWriteHandler_[MAX_HANDLER_NUM];
    // SyncReadHandler  syncReadHandler_[MAX_HANDLER_NUM];

    dynamixel::GroupBulkRead  *groupBulkRead_;
    dynamixel::GroupBulkWrite *groupBulkWrite_;

    int motor_nums;
    int motor_id[P_MAX_ID];

public:
    double *pos_center;
    int *scanned_model;
    double presentPosition[P_MAX_ID];

    double feedback_position[P_MAX_ID];         //unit  
    double feedback_velocity[P_MAX_ID];
    double feedback_current[P_MAX_ID];
    double feedback_voltage[P_MAX_ID];
    double feedback_temperature[P_MAX_ID];

    double goal_current[P_MAX_ID];
    double goal_position[P_MAX_ID];

    double L;
    double radius;
    double K;
    double D;

    Robot(int num_Motors, int baudrate, int *ID_numbers, const char *portName);
    ~Robot();
    void init(const char *device_name, int baud_rate, float protocol_version);
    void torque_enable(int on_off, int id); //1->enable
    void set_all_torque_enable(int on_off, int *ids);
    void getParameter(int32_t data, uint8_t *param, int param_array_size);
    // void get2Parameter(int32_t data, uint8_t *param);
    bool writeRegister(uint8_t id, uint16_t address, uint16_t length, int32_t data);
    bool readRegister(uint8_t id, uint16_t address, uint16_t length, int32_t *data);
    void jointMode(int *ids, int32_t velocity, int32_t acceleration);
    void setAllJointPositionControlMode(int *ids);
    void setAllJointCurrentControlMode(int *ids);
    void setAllPositions(double *angles, int *ids);
    void setAllCurrents(double *currents, int *ids);
    void getAllPositions(double *angles, int *ids);
    void getAllCurrents(double *currents, int *ids);
    void getAllVoltage(double *input_voltage, int *ids);
    void getAllVelocity(double *velocity, int *ids);
    void getAllTemperatures(double *temperatures, int *ids);
    void SetInitialPosture( int *ids);
    void Close_port(int *ids);
    void Anglelimits(int CW, int CCW, int *ids);
    void CurrentLimit(float current, int *ids);
    void SetReturnDelayTime(int DelayTime, int *ids);

    void unitConversion(double goal_position, double *position, double *current, double *velocity);
    void vaamModel(double L, double r, double K, double D, double *theta, double *dtheta, double *Fext, double *F);


};

#endif

