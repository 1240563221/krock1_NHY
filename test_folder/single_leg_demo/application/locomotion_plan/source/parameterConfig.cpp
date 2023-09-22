#include "gait_parameters.hpp"
#include "single_leg_controller.hpp"
#include <iostream>

using namespace Eigen;

void Locomotion_controller::parameterConfig()
{
    // 读取 YAML 文件
    YAML::Node parameterconfig = YAML::LoadFile("../config/parametercofig.yaml");
    // gaitParams GP1;

    // 创建结构体实例
    GP1.Duty = parameterconfig["Duty"].as<double>();
    GP1.freq_walk = parameterconfig["freq_walk"].as<double>();
    GP1.ellipse_a = parameterconfig["ellipse_a"].as<double>();
    GP1.ellipse_b = parameterconfig["ellipse_b"].as<double>();
    GP1.spineCPGscaling = parameterconfig["spineCPGscaling"].as<double>();
    for (int i = 0; i < 3; i++)
    {
    for (int j = 0; j < 4; j++)
     {
        GP1.midstance(i, j) = parameterconfig["midstance"]["data"][i * 4 + j].as<double>();
     }
    }
    for (int i = 0; i < 4; i++)
    {
    for (int j = 0; j < 4; j++)
    {
        GP1.qNULL(i, j) = parameterconfig["qNULL"]["data"][i * 4 + j].as<double>();
    }
    }
    for (int i = 0; i < 4; i++)
    {
        GP1.phShifts(i) = parameterconfig["phShifts"][i].as<double>();
        GP1.swing_height(i) = parameterconfig["swing_height"][i].as<double>();
        GP1.swing_width(i) = parameterconfig["swing_width"][i].as<double>();
    }
}

