/*****************************************************************************
* Project: AgnathaX robot
* Copyright 2021-2023 Laura Paez Coy and Kamilo Melo
* This code is under MIT licence: https://opensource.org/licenses/MIT
* Laura.Paez@KM-RoBota.com, 04-2023
* Kamilo.Melo@KM-RoBota.com, 05-2023
*****************************************************************************/

// Control table address
// Encodes EEPROM and RAM control table for MX dynamixel motors

#define PROTOCOL_VERSION                1
#define POSITION_CONTROL_MODE           3
#define TORQUE_CONTROL_MODE             0
// #define CURRENT_LIMIT                   1024


#define M_PI                        3.1415926
#define P_MAX_ID                    1


#if PROTOCOL_VERSION == 2
    // Read
    // EEPROM
    #define P_MODEL_NO                  0
    #define P_FIRMWARE_VERSION          2
    // Read Write
    // EEPROM
    #define P_DXL_ID                    3
    #define P_BAUD_RATE                 4
    #define P_RETURN_DELAY_TIME         5
    #define P_CW_ANGLE_LIMIT            6
    #define P_CCW_ANGLE_LIMIT           8

    #define P_TEMP_HIGH_LIMIT           11
    #define P_VOLT_LOW_LIMIT            12
    #define P_VOLT_HIGH_LIMIT           13

    #define P_MAX_TORQUE                14

    #define P_STATUS_RETURN_LEVEL       16
    #define P_ALARM_LED                 17
    #define P_ALARM_SHUTDOWN            18
    #define P_MULTI_TURN_OFFSET         20
    #define P_RESOLUTION_DIVIDER        22


    // RAM
    #define P_TORQUE_ENABLE             24
    #define P_LED                       25
    #define P_GOAL_POSITION             30
    #define P_MOVING_SPEED              32
    #define P_TORQUE_LIMIT              34
    #define P_PRESENT_POSITION          36
    #define P_PRESENT_SPEED             38
    #define P_PRESENT_LOAD              40
    #define P_PRESENT_INPUT_VOLTAGE     42
    #define P_PRESENT_TEMPERATURE       43

    #define P_PRESENT_CURRENT           68
    #define P_TORQUE_CONTROL_ENABLE     70
    #define P_GOAL_TORQUE               71
    #define P_GOAL_ACCELERATION         73


    // Data Byte Length
    #define LEN_GOAL_POSITION           2
    #define LEN_PRESENT_POSITION        2
    #define LEN_PRESENT_CURRENT         2
    #define LEN_PRESENT_INPUT_VOLTAGE   1
    #define LEN_PRESENT_TEMPERATURE     1
    #define LEN_LED                     1
    #define LEN_TORQUE_ENABLE           1
    #define LEN_POSITION_LIMIT          2
    #define LEN_CURRENT_LIMIT           2
    #define LEN_RETURN_DELAY_TIME       1
    #define LEN_GOAL_TORQUE             2
    #define LEN_GOAL_CURRENT            2
    #define LEN_TORQUE_CONTROL_ENABLE   1
#else
    // Read
    // EEPROM
    #define P_MODEL_NO                  0
    #define P_MODEL_INFORMATION         2
    #define P_FIRMWARE_VERSION          6
    // Read Write
    // EEPROM
    #define P_DXL_ID                    7
    #define P_BAUD_RATE                 8
    #define P_RETURN_DELAY_TIME         9
    #define P_DRIVE_MODE                10
    #define P_OPERATING_MODE            11
    #define P_TEMP_HIGH_LIMIT           31
    #define P_VOLT_HIGH_LIMIT           32
    #define P_VOLT_LOW_LIMIT            34
    #define P_CURRENT_HIGH_LIMIT        38
    #define P_VELOCITY_HIGH_LIMIT       44
    #define P_POSITION_HIGH_LIMIT       48
    #define P_POSITION_LOW_LIMIT        52

    // RAM
    #define P_TORQUE_ENABLE             64
    #define P_LED                       65
    #define P_GOAL_CURRENT              102
    #define P_GOAL_VELOCITY             104
    #define P_PROFILE_ACCELERATION      108
    #define P_PROFILE_VELOCITY          112
    #define P_GOAL_POSITION             116
    #define P_PRESENT_CURRENT           126
    #define P_PRESENT_VELOCITY          128
    #define P_PRESENT_POSITION          132
    #define P_PRESENT_INPUT_VOLTAGE     144
    #define P_PRESENT_TEMPERATURE       146

        // Data Byte Length
    #define LEN_GOAL_POSITION           4
    #define LEN_GOAL_CURRENT            2
    #define LEN_PRESENT_POSITION        4
    #define LEN_PRESENT_CURRENT         2
    #define LEN_PRESENT_VELOCITY        4
    #define LEN_PRESENT_INPUT_VOLTAGE   2
    #define LEN_PRESENT_TEMPERATURE     1
    #define LEN_LED                     1
    #define LEN_TORQUE_ENABLE           1
    #define LEN_OPERATING_MODE          1
    #define LEN_PROFILE                 4
    #define LEN_POSITION_LIMIT          4
    #define LEN_CURRENT_LIMIT           2
    #define LEN_RETURN_DELAY_TIME       1
#endif


