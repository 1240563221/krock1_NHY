#ifndef __YAML_READ_H__
#define __YAML_READ_H__

#include <iostream>
#include "musclemodel.h"
#include "yaml-cpp/yaml.h"
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

void readMuscleParameters(YamlDataTPDF *tempData, muscleModelTPDF* tempParameters, int *id);

#endif