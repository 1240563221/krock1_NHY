#include <iostream>
#include "yaml-cpp/yaml.h"
#include <vector>
#include "robot.h"
#include <sys/time.h>
#include <signal.h>
#include "musclemodel.h"
#include "yaml_read.h"
#include "Log.h"
#include "single_leg_controller.hpp"
#include "gait_parameters.hpp"

using namespace std;

YamlDataTPDF yamlData;
muscleModelTPDF muscleRead[P_MAX_ID];

int id[P_MAX_ID];

enum LogDataLength
{   
    INDEX_REF_FOOT_TRAJECTORY_X=0,
    INDEX_REF_FOOT_TRAJECTORY_Y,
    INDEX_REF_FOOT_TRAJECTORY_Z,
    INDEX_JOINT_1,
    INDEX_JOINT_2,
    INDEX_JOINT_3,
    INDEX_JOINT_4,
    INDEX_ACT_FOOT_TRAJECTORY_X,
    INDEX_ACT_FOOT_TRAJECTORY_Y,
    INDEX_ACT_FOOT_TRAJECTORY_Z,
    LOG_DATA_LENGTH,
};


int main()
{
    // ReadGlobalConfig();
    vector<string>  fileNames = {"log_spring"};
    vector<string>  logDataTitle = {"ref_f_traj_x","ref_f_traj_y","ref_f_traj_z","jA_1", "jA_2", "jA_3", 
                                    "jA_4", "act_f_traj_x", "act_f_traj_y", "act_f_traj_z"};
    vector<float>   logData;
    logData.resize(LOG_DATA_LENGTH);

    double t = 0;
    //以下三个需删掉，替换成实际输入输出量
    double* log;

    stcontroller::Log logSystem(fileNames, "/home/cc/Desktop/Git_Folder/krock1_NHY/test_folder/single_leg_demo/logData/");
    logSystem.saveData(fileNames[0], logData, logDataTitle);
    logSystem.step();

    readMuscleParameters(&yamlData, muscleRead, id);
    Robot robot(yamlData.num_motors, P_BAUDRATE, id, "/dev/ttyUSB0");

    robot.Anglelimits(MAX_LIMIT_POSITION_SERVO/4, 3*MAX_LIMIT_POSITION_SERVO/4, id);
    robot.setAllJointPositionControlMode(id);

    struct timeval time_old;
    struct timeval time_new;

    VAAM_Model_CLASS *jointController[P_MAX_ID];
    for (int i = 0; i < P_MAX_ID; i++)
    {
        jointController[i] = new VAAM_Model_CLASS(TIME_INTERVAL, muscleRead[i]);
        
    }

    Locomotion_controller controller;
    // Initialize
    controller.variable_initial();
    controller.constant_initial();
    // parameter config
    controller.parameterConfig();

    while(1)
	{   
        gettimeofday(&time_old, NULL);
		// /* code */

        robot.getAllCurrentsVelocityPosition(robot.feedback_current, robot.feedback_velocity, robot.feedback_position, id);
        gettimeofday(&time_new, NULL);

        controller.run_step(robot.feedback_position, robot.goal_position, t);

        //traj_log为10*1向量，组成为3+4+3，即参考位置xyz，输出角度q0-q3，实际位置xyz
        for(int i=0;i<10;i++)
        {
            logData[i] = controller.traj_log(i);
            cout << "logData[" << i << "]:" << controller.traj_log(i) << endl;
        }

        t += (double)TIME_STEP / 1000.0;


        cout << "run times : " << time_new.tv_usec - time_old.tv_usec << " us " << endl;

        for (int j = 0; j < P_MAX_ID; j++)
        {
            jointController[j]->setGoalPosition(robot.goal_position[j]);
            cout << "current_position : " << robot.feedback_position[j] << endl;
            cout << "current_velocity : " << robot.feedback_velocity[j] << endl;
            
        }
        
        for (int j = 0; j < P_MAX_ID; j++)
        {
            jointController[j]->RKfun(robot.feedback_position[j], robot.feedback_velocity[j], &robot.goal_position[j], robot.feedback_current[j]);
            cout << "theta = " << robot.goal_position[j] << endl;
            // robot.goal_position[j] = (robot.goal_position[j] + M_PI)*MAX_LIMIT_POSITION_SERVO/(2*M_PI);
            /* code */
        }

        for (int j = 0; j < P_MAX_ID; j++)
        {
            if (robot.goal_position[j] < -M_PI || robot.goal_position[j] > M_PI)
            {
                cout << "ID[" << id[j] << "]  calculate error!  goal position is " << robot.goal_position[j] << endl;
            }
        }
        cout << "---------------------"  << endl;
        robot.setAllPositions(robot.goal_position, id);

        // for (int  j = 0; j < P_MAX_ID; j++)
        // {
        //     logData[j] = robot.goal_position[j];
        // }
        
        logSystem.saveData(fileNames[0], logData, logDataTitle);
        logSystem.step();

	}

    for (int i = 0; i < P_MAX_ID; i++)
    {
        delete jointController[i];
        /* code */
    }
    
    robot.set_all_torque_enable(0, id);
    return 0;
}

