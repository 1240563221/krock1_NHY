/** 
* @file [yaml_read.h]
* @brief 
* @author <chen chen>
* @date <2023.9.21>
* @version <V1.0>
*
* detailed description for the file.
*/

#ifndef __YAML_READ_H__
#define __YAML_READ_H__

#include <iostream>
#include "musclemodel.h"
#include "yaml-cpp/yaml.h"
using namespace std;


/**
 * @brief   the struct of the yaml_read
*/
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