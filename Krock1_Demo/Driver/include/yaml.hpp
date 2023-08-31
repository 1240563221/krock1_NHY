#ifndef __YAML_HPP__
#define __YAML_HPP__

#include "yaml-cpp/yaml.h"
#include <iostream>

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

extern YamlDataTPDF yamlData;

int ReadGlobalConfig();

#endif