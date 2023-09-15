#include <iostream>
#include "yaml-cpp/yaml.h"
#include <vector>
#include "robot.h"
#include <sys/time.h>
#include <signal.h>
#include "musclemodel.h"

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
muscleModelTPDF muscleRead[P_MAX_ID];


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

// void readMuscleParameters(void)
// {
//     YAML::Node muscleConfig = YAML::LoadFile("../config/muscle.yaml");
//     for (int i = 0; i < P_MAX_ID; i++)
//     {
//         muscleRead[i].K = muscleConfig["muscle_K"][i].as<double>();
//         muscleRead[i].D = muscleConfig["muscle_D"][i].as<double>();
//         muscleRead[i].m = muscleConfig["muscle_M"][i].as<double>();
//         muscleRead[i].L = muscleConfig["muscle_L"][i].as<double>();
//         muscleRead[i].G = muscleConfig["muscle_G"][i].as<double>();
//         muscleRead[i].refLocation = muscleConfig["muscle_refLocation"][i].as<double>();
//     }
// }

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


double current_p[P_MAX_ID];
double current_v[P_MAX_ID];
double current_c[P_MAX_ID];
double goal_p[P_MAX_ID] = {2048};

double Ftorque[P_MAX_ID]={0};
double theta=0;

int main()
{
    ReadGlobalConfig();
    // readMuscleParameters();
    Robot robot(yamlData.num_motors, 57600, id, "/dev/ttyUSB0");
    vaamInit(robot, yamlData);

    robot.Anglelimits(0, 4095, id);
    robot.setAllJointPositionControlMode(id);
    // robot.setAllPositions(goal_p, id);
    // robot.CurrentLimit((float)100, id);

    cout << "radius : " << robot.radius << endl;
    cout << "K : " << robot.K << endl;

    struct timeval time_old;
    struct timeval time_new;

	Fourth_RK_CLASS admittanceControl(1, 0.043, muscleRead, P_MAX_ID);
    cout << "n : " << admittanceControl.getNumber() << endl;
    cout << "K : " << admittanceControl.muscleParameters[0].K << endl;
    cout << "D : " << admittanceControl.muscleParameters[0].D << endl;
    cout << "m : " << admittanceControl.muscleParameters[0].m << endl;
    cout << "L : " << admittanceControl.muscleParameters[0].L << endl;
    cout << "I : " << admittanceControl.muscleParameters[0].I << endl;
    cout << "refLocation : " << admittanceControl.muscleParameters[0].refLocation << endl;

    sleep(1);

    for (int i = 0; i < 5000; i++)
	{   
        gettimeofday(&time_old, NULL);
		// /* code */
        robot.getAllCurrents(robot.feedback_current, id);       // 11ms? kidding me
        robot.getAllVelocity(robot.feedback_velocity, id);      //get feedback need more time 
        robot.getAllPositions(robot.feedback_position, id);
        gettimeofday(&time_new, NULL);
        cout << "run times : " << time_new.tv_usec - time_old.tv_usec << " us " << endl;

        for (int j = 0; j < P_MAX_ID; j++)
        {
            // cout << "raw_position : " << robot.feedback_position[j] << endl;
            // cout << "raw_velocity : " << robot.feedback_velocity[j] << endl;
            // current_p[j] = 2*M_PI*((int)(robot.feedback_position[j]+4096)%4096)/4096 - M_PI;
            // current_v[j] = -robot.feedback_velocity[i]/1000.;
            // cout << "current_position : " << robot.feedback_position[j] << endl;
            // cout << "current_velocity : " << robot.feedback_velocity[j] << endl;
            cout << "raw_position : " << robot.feedback_position[j] << endl;
            cout << "raw_velocity : " << robot.feedback_velocity[j] << endl;
            current_p[j] = 2*M_PI*((int)(robot.feedback_position[j]+4096)%4096)/4096 - M_PI;
            current_v[j] = -robot.feedback_velocity[i]/1000.;
            cout << "current_position : " << current_p[j] << endl;
            cout << "current_velocity : " << current_v[j] << endl;
        }
        
        admittanceControl.calculateMuscleOutput(current_p, current_v, goal_p , robot.feedback_current, P_MAX_ID);

        for (int j = 0; j < P_MAX_ID; j++)
        {
            cout << "theta = " << robot.goal_position[j] << endl;
            robot.goal_position[j] = (robot.goal_position[j] + M_PI)*4096/(2*M_PI);
            /* code */
        }
        cout << "---------------------"  << endl;
        // robot.setAllPositions(robot.goal_position, id);


	}

    robot.set_all_torque_enable(0, id);
    return 0;
}

