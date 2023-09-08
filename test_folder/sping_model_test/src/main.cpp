#include <iostream>
#include "yaml-cpp/yaml.h"
#include <vector>
#include "robot.h"

using namespace std;

typedef struct
{
    /* data */
    int execution_time;
    int num_motors;
    std::vector<int> id_number;
    int control_mode;
    int amplitude_head;
    int amplitude_tail;
    int offset;
    float frequency;
    int lambda;
    int angle_limits;
    bool log_robot_status;
    bool log_sensor_data;
    bool is_teleop;
    double spring_coefficient;
    double damping_coefficient;
}
YamlDataTPDF;



YamlDataTPDF yamlData;

int id[P_MAX_ID];

// loads global.yaml file
int ReadGlobalConfig()
{
    YAML::Node config = YAML::LoadFile("../config/global.yaml");

    yamlData.execution_time = config["execution_time"].as<int>();
    yamlData.num_motors = config["num_motors"].as<int>();
    if (yamlData.num_motors != config["id_numbers"].size() )
    {
        cout << "Check # of motors and ID's in global config " << endl;
        return 0;
    }
    yamlData.id_number.resize(yamlData.num_motors);
    cout << "size : " << yamlData.id_number.size() << endl;
    int temp=0;
    for (int i = 0; i < yamlData.num_motors; i++)
    {
        yamlData.id_number[i] = config["id_numbers"][i].as<int>();
        id[i] = config["id_numbers"][i].as<int>();
    }
    yamlData.control_mode = config["control_mode"].as<int>();
    yamlData.amplitude_head = config["amplitude_head"].as<int>();
    yamlData.amplitude_tail = config["amplitude_tail"].as<int>();
    yamlData.offset = config["offset"].as<int>();
    yamlData.frequency = config["frequency"].as<float>();
    yamlData.lambda = config["lambda"].as<int>();
    yamlData.angle_limits = config["angle_limits"].as<int>();
    yamlData.log_robot_status = config["log_robot_status"].as<bool>();
    yamlData.log_sensor_data = config["log_sensor_data"].as<bool>();
    yamlData.is_teleop = config["is_teleop"].as<bool>();
    yamlData.spring_coefficient = config["spring_coefficient"].as<double>();
    yamlData.damping_coefficient = config["damping_coefficient"].as<double>();

#ifdef MY_DEBUG
    cout << " execution_time : "<< yamlData.execution_time  << endl;
    cout << " num_motors : "<< yamlData.num_motors  << endl;
    cout << " id_number[0] : "<< yamlData.id_number[0]  << endl;
    // cout << " id_number[1] : "<< yamlData.id_number[1]  << endl;
    // cout << " id_number[2] : "<< yamlData.id_number[2]  << endl;
    // cout << " id_number[3] : "<< yamlData.id_number[3]  << endl;
    cout << " control_mode : "<< yamlData.control_mode  << endl;
    cout << " amplitude_head : "<< yamlData.amplitude_head  << endl;
    cout << " amplitude_tail : "<< yamlData.amplitude_tail  << endl;
    cout << " offset : "<< yamlData.offset  << endl;
    cout << " frequency : "<< yamlData.frequency  << endl;
    cout << " lambda : "<< yamlData.lambda  << endl;
    cout << " angle_limits : "<< yamlData.angle_limits  << endl;
    cout << " log_robot_status : "<< yamlData.log_robot_status  << endl;
    cout << " log_sensor_data : "<< yamlData.log_sensor_data  << endl;
    cout << " is_teleop : "<< yamlData.is_teleop  << endl;
#endif
    
    return 1;
}


double present_position[P_MAX_ID]={0};
double present_velocity[P_MAX_ID]={0};
double present_voltage[P_MAX_ID]={0};
double present_temperature[P_MAX_ID]={0};
double present_current[P_MAX_ID]={0};

double Ftorque[P_MAX_ID]={10,-10};

int main()
{
    ReadGlobalConfig();
    Robot robot(yamlData.num_motors, 57600, id, "/dev/ttyUSB0");

    double goal_position[P_MAX_ID]={0};

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    bool dxl_getdata_result = false; 

    robot.Anglelimits(0, 4095, id);
    robot.setAllJointCurrentControlMode(id);
    robot.CurrentLimit((float)100, id);
    for(uint8_t i=0; i<P_MAX_ID; i++)
    {
        goal_position[i] = Ftorque[i];
    }

    robot.setAllCurrents(goal_position, id);
    sleep(1);

    // while(1)
    // {
    //     for(uint8_t i=0; i<P_MAX_ID; i++)
    //     {
    //         goal_position[i] = Ftorque[i];
    //         // cout << "goal_position = " << goal_position[i-1] << endl;
    //         // cout << "a = " << a << endl;
    //     }

    //     robot.setAllCurrents(goal_position, id);

    //     robot.getAllPositions(present_position, id);
    //     robot.getAllVelocity(present_velocity, id);

    //     for (uint8_t i = 0; i < P_MAX_ID; i++)
    //     {
    //         /* code */
    //         // cout << "old_position = " << (int)((int)present_position[i-1] + 4096) % 4096 << endl;

    //         present_position[i] =  2048 - (int)((int)present_position[i] + 4096) % 4096; 
    //         present_velocity[i] *= -1;
    //         // cout << "present_position = " << present_position[i-1] << endl;
    //         // present_velocity[i-1] = present_position[i-1] - previous_position[i-1];
            
    //         // Ftorque[i] = yamlData.spring_coefficient * present_position[i];
    //         Ftorque[i] = yamlData.spring_coefficient * present_position[i] - yamlData.damping_coefficient * present_velocity[i];
    //         // Ftorque[i-1] = u * present_velocity[i-1] - k * present_position[i-1];
    //         // previous_position[i-1] = present_position[i-1];
    //         cout << "spring value : " << yamlData.spring_coefficient * present_position[i] << endl;
    //         printf("[ID:%03d] Present Position : %.3f   Present Velocity : %.3f    Torque : %.3f \t\n", i, present_position[i], present_velocity[i], Ftorque[i]);

    //     }
    //     printf("----------------------------------------------\n");
    //     usleep(10000);
    // }
    while(1)
    {
        robot.getAllPositions(present_position, id);
        robot.getAllCurrents(present_current, id);
        robot.getAllVelocity(present_velocity, id);
        robot.getAllVoltage(present_voltage, id);
        robot.getAllTemperatures(present_temperature, id);

        for(uint8_t i=0; i<P_MAX_ID; i++)
        {
            present_position[i] = double((int)((int)present_position[i] + 4096) % 4096)*360/4096;
            printf("ID[%d]    position:%.3f, current:%.3f,  velocity:%.3f    voltage:%.3f,   temperature:%.3f \n", i, present_position[i],
                    present_current[i]*3.36, present_velocity[i], present_voltage[i]/10, present_temperature[i]);
        }

        sleep(1);
        cout << "-------------------------------------"<< endl;
    }
}

