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

    // dynamixel::GroupBulkRead *groupBulkRead_position;
    // dynamixel::GroupBulkRead *groupBulkRead_current;
    // dynamixel::GroupBulkRead *groupBulkRead_temperature;
    dynamixel::GroupSyncRead *groupSyncRead_position;
    dynamixel::GroupSyncRead *groupSyncRead_current;
    dynamixel::GroupSyncRead *groupSyncRead_temperature;
    // dynamixel::GroupBulkRead *groupBulkRead_position_current_voltage;

    // SyncWriteHandler syncWriteHandler_[MAX_HANDLER_NUM];
    // SyncReadHandler  syncReadHandler_[MAX_HANDLER_NUM];

    dynamixel::GroupBulkRead  *groupBulkRead_;
    dynamixel::GroupBulkWrite *groupBulkWrite_;

public:
    double *pos_center;
    int *scanned_model;
    double presentPosition[P_MAX_ID];

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
    void getAllTemperatures(double *temperatures, int *ids);
    // void Assign_read_indirect_adress(int start_ind_adress_write, int *ids);
    // void IndirectAddressReading(int *ids, double *angles, double *current, double *voltage);
    void SetInitialPosture( int *ids);
    void Close_port(int *ids);
    void Anglelimits(int CW, int CCW, int *ids);
    void CurrentLimit(float current, int *ids);
    void SetReturnDelayTime(int DelayTime, int *ids);
};

#endif

