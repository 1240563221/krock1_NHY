#include <iostream>
#include "yaml-cpp/yaml.h"
#include <vector>
#include "robot.h"
#include <signal.h>

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
    double vaam_L;
    double vaam_radius;
    double vaam_K;
    double vaam_D;
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
    yamlData.vaam_L = config["vaam_L"].as<double>();
    yamlData.vaam_radius = config["vaam_radius"].as<double>();
    yamlData.vaam_K = config["vaam_K"].as<double>();
    yamlData.vaam_D = config["vaam_D"].as<double>();

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

typedef struct
{
    /* data */
    double radius;
    // double i1;
    // double i2;
    double L;
    double K;
    double D;
    double Nm;
}VAAMTPDF;

VAAMTPDF vaamData;

void vaamInit(Robot &robot, YamlDataTPDF &yamlData)
{
    robot.L = yamlData.vaam_L;
    robot.radius = yamlData.vaam_radius;
    robot.K = yamlData.vaam_K;
    robot.D = yamlData.vaam_D; 
}




double Ftorque[P_MAX_ID]={0};
double theta=0;

int main()
{
    ReadGlobalConfig();
    Robot robot(yamlData.num_motors, 57600, id, "/dev/ttyUSB1");
    vaamInit(robot, yamlData);

    robot.Anglelimits(0, 4095, id);
    robot.setAllJointCurrentControlMode(id);
    robot.CurrentLimit((float)100, id);

    cout << "radius : " << robot.radius << endl;
    cout << "K : " << robot.K << endl;

    sleep(1);

    while(1)
    {
        robot.getAllPositions(robot.feedback_position, id);
        robot.getAllCurrents(robot.feedback_current, id);
        // robot.getAllVelocity(robot.feedback_velocity, id);

        robot.unitConversion(180, robot.feedback_position, robot.feedback_current, robot.feedback_velocity);
        robot.vaamModel(robot.L, robot.radius, robot.K, robot.D, robot.feedback_position, robot.feedback_velocity, robot.feedback_current, robot.goal_current);

        robot.setAllCurrents(robot.goal_current, id);
        usleep(5000);
        cout << "-------------------------------------"<< endl;
    }
    robot.set_all_torque_enable(0, id);
    return 0;
}

