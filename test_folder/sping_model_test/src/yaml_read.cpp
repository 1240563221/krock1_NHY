#include "yaml_read.h"


void readMuscleParameters(YamlDataTPDF *tempData, muscleModelTPDF* tempParameters, int *id)
{
    YAML::Node muscleConfig = YAML::LoadFile("../config/muscle.yaml");

    // YAML::Node config = YAML::LoadFile("../config/global.yaml");

    tempData->num_motors = muscleConfig["num_motors"].as<int>();
    if (tempData->num_motors != muscleConfig["id_numbers"].size() )
    {
        cout << "Check # of motors and ID's in global config " << endl;
        return;
    }
    tempData->id_number.resize(tempData->num_motors);
    cout << "size : " << tempData->id_number.size() << endl;
    for (int i = 0; i < tempData->num_motors; i++)
    {
        tempData->id_number[i] = muscleConfig["id_numbers"][i].as<int>();
        id[i] = muscleConfig["id_numbers"][i].as<int>();
    }


    // YAML::Node muscleConfig = YAML::LoadFile("../config/muscle.yaml");
    
    for (int i = 0; i < P_MAX_ID; i++)
    {
        tempParameters[i].K = muscleConfig["muscle_K"][i].as<double>();
        tempParameters[i].D = muscleConfig["muscle_D"][i].as<double>();
        tempParameters[i].m = muscleConfig["muscle_M"][i].as<double>();
        tempParameters[i].L = muscleConfig["muscle_L"][i].as<double>();
        tempParameters[i].G = muscleConfig["muscle_G"][i].as<double>();
        tempParameters[i].refLocation = muscleConfig["muscle_refLocation"][i].as<double>();
    }
}

