#include <iostream>
#include "yaml-cpp/yaml.h"
#include <vector>
#include "robot.h"
#include <sys/time.h>
#include <signal.h>
#include "musclemodel.h"
#include "yaml_read.h"
#include "Log.h"

using namespace std;

YamlDataTPDF yamlData;
muscleModelTPDF muscleRead[P_MAX_ID];

int id[P_MAX_ID];

enum LogDataLength
{
    INDEX_JOINT_1=0,
    INDEX_JOINT_2,
    INDEX_JOINT_3,
    INDEX_JOINT_4,
    INDEX_FOOT_TRAJECTORY_X,
    INDEX_FOOT_TRAJECTORY_Y,
    INDEX_FOOT_TRAJECTORY_Z,
    LOG_DATA_LENGTH,
};


int main()
{
    // ReadGlobalConfig();
    vector<string>  fileNames = {"log_spring"};
    vector<string>  logDataTitle = {"jA_1", "jA_2", "jA_3", "jA_4", "f_tra_x", "f_tra_y", "f_tra_z"};
    vector<float>   logData;
    logData.resize(LOG_DATA_LENGTH);

    stcontroller::Log logSystem(fileNames, "/home/cc/Desktop/Git_Folder/krock1_NHY/test_folder/sping_model_test/logData/");
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

    while(1)
	{   
        gettimeofday(&time_old, NULL);
		// /* code */

        robot.getAllCurrentsVelocityPosition(robot.feedback_current, robot.feedback_velocity, robot.feedback_position, id);
        gettimeofday(&time_new, NULL);
        cout << "run times : " << time_new.tv_usec - time_old.tv_usec << " us " << endl;

        for (int j = 0; j < P_MAX_ID; j++)
        {
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

        for (int  j = 0; j < P_MAX_ID; j++)
        {
            logData[j] = robot.goal_position[j];
        }
        
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

