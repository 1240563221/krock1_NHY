#include <iostream>
#include "yaml-cpp/yaml.h"
#include <vector>

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
}
YamlDataTPDF;

YamlDataTPDF yamlData;

int main(int argc,char** argv)
{
    YAML::Node config = YAML::LoadFile("../config/global.yaml");

    yamlData.execution_time = config["execution_time"].as<int>();
    yamlData.num_motors = config["num_motors"].as<int>();
    if (yamlData.num_motors != config["id_numbers"].size() )
    {
        cout << "Check # of motors and ID's in global config " << endl;
    }
    yamlData.id_number.resize(yamlData.num_motors);
    cout << "size : " << yamlData.id_number.size() << endl;
    int temp=0;
    for (int i = 0; i < yamlData.num_motors; i++)
    {
        // cout << " id_number["<< i << "] = " <<  config["id_numbers"][i].as<int>()  << endl;
        // temp = config["id_number"][i].as<int>();
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
    
    
    
    // // YAML::Node config1 = YAML::LoadFile("../config/config.yaml");

    // cout << "name:" << config["name"].as<string>() << endl;
    // cout << "sex:" << config["sex"].as<string>() << endl;
    // cout << "age:" << config["age"].as<int>() << endl;
    // age = config["age"].as<int>();
    // cout << " age " << age << endl;
    // cout << "skills-c++ : " << config["skills"]["c++"].as<int>() << endl;     
    
    // cout << "buffer size = " << config["id"].size() << endl;
    // cout << "id[0] = " << config["id"][0].as<int>() << endl;
    // cout << "id[1] = " << config["id"][1].as<int>() << endl;
    // cout << "id[2] = " << config["id"][2].as<int>() << endl;
    // cout << "id[3] = " << config["id"][3].as<int>() << endl;
    return 0;
}
