#include "yaml.hpp"

YamlDataTPDF yamlData;
using namespace std;


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

    cout << " execution_time : "<< yamlData.execution_time  << endl;
    cout << " num_motors : "<< yamlData.num_motors  << endl;
    cout << " id_number[0] : "<< yamlData.id_number[0]  << endl;
    cout << " id_number[1] : "<< yamlData.id_number[1]  << endl;
    cout << " id_number[2] : "<< yamlData.id_number[2]  << endl;
    cout << " id_number[3] : "<< yamlData.id_number[3]  << endl;
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
    
    return 1;
}
