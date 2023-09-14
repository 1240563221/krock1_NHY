#include <iostream>
#include "yaml-cpp/yaml.h"
#include <vector>
#include "robot.h"
#include <signal.h>

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


#define _USE_MATH_DEFINES 
#include <cmath>
 

class Fourth_RK_CLASS
{
public:
	double m_h;
	int m_n;
	double m_x;
	double m_y;
	double m_z;
	double m_r;
	double m_K;
	double m_D;
	double m_ref1;
	double m_ref2;
	double m_I;
	double m_m;
	double m_L;
	double m_G;
	double m_torque;
    double m_Fext[P_MAX_ID];
	Fourth_RK_CLASS(double x0, double y0, double y1, int n, double h)
	{
		m_x = x0;
		m_y = y0;
		m_z = y1;
		m_n = n;
		m_h = h;
		
		m_ref1 = -0.0;
		m_ref2 = -0.0;
		m_r = 1;
		m_K = 20;
		m_D = 3;
		m_L = 1;
		m_m = 1;
		m_G = 9.81;
		m_I = m_m*m_L*m_L;
		m_torque = 0;
	}

	//y' = z	y'' = 2*r**2*(Ky+Dy')  ---->  z' = 2*r**2*(Ky + Dz)
	double fun_x1(double t, double y, double z)
	{
		return z;
	}

	double fun_x2(double t, double y, double z)
	{
		double T_s1=0, T_s2=0, T_spring=0,D1=0,D2=0,D=0;
		if (m_ref1 < y)
            T_s1 = (m_ref1-y)* m_K;
            
        if (m_ref2 > y)
            T_s2 = (m_ref1-y)* m_K;
        
        T_spring = (T_s1+T_s2)/m_I;
        
        if (z > 0)
            D1 = -z*m_D;
        if (z < 0)
            D2 = -z*m_D;
            
        D = D1+D2/m_I;
		// return   sin(2*M_PI*t/20) - 2 * m_r*m_r*(m_K*y+m_D*z);
		return -m_G * (1. / m_L) * sin(y) - m_Fext[0]*0.03  + m_torque/m_I + T_spring + D;
		// return -9.8*sin(y) - m_D*z;
	}

	// double fun_x2(double t, double y, double z)
	// {
	// 	// cout << "t:" << pow( M_E, 2*t)*sin(2*t) << endl;
	// 	// cout << "y:" << -2*y << endl;
	// 	// cout << "z:" << 2*z << endl;
	// 	return  pow( M_E, 2*t)*sin(t)-2*y+2*z;
	// }

	void RKfun(double *input, double *torque)
	{
        for (int i = 0; i < P_MAX_ID; i++)
        {
            m_Fext[i] = torque[i];
            cout << "m_Fext : " << m_Fext[i] << endl;
        }

		double K1y, K2y, K3y, K4y, K1z, K2z, K3z, K4z;

		K1y = K2y = K3y = K4y = K1z = K2z = K3z = K4z = 0.0;

		double yn_next = 0.0;
		double z_next = 0.0;
		double x,y,z;
		int n = m_n;
		while (n)
		{
			x = m_x;
			y=input[0];
			z=input[1];

			K1z = fun_x1(x, y,  z);
			K1y = fun_x2(x, y,  z);

			K2z = fun_x1(x + m_h/2., y + K1z * m_h/2., z + K1y * m_h/2.);
			K2y = fun_x2(x + m_h/2., y + K1z * m_h/2., z + K1y * m_h/2.);

			K3z = fun_x1(x + m_h/2., y + K2z * m_h/2., z + K2y * m_h/2.);
			K3y = fun_x2(x + m_h/2., y + K2z * m_h/2., z + K2y * m_h/2.);

			K4z = fun_x1(x + m_h, y + K3z * m_h, z + K3y * m_h);
			K4y = fun_x2(x + m_h, y + K3z * m_h, z + K3y * m_h);

			yn_next = y + m_h/6.0 * (K1z + 2*K2z + 2*K3z + K4z);
			z_next = z + m_h/6.0 * (K1y + 2*K2y + 2*K3y + K4y);

			// cout << "n = " << this->m_n - n + 1 << "时：  " << "yn = " << setprecision(5)<<yn_next<< 
			// 		"   yn' = "<< z_next << endl;
			m_x += m_h;
			m_y = yn_next;
			m_z = z_next;
			input[0] = m_y;
			input[1] = m_z;
			n--;
 
		}
	}

};


double Ftorque[P_MAX_ID]={0};
double theta=0;

int main()
{
    ReadGlobalConfig();
    Robot robot(yamlData.num_motors, 57600, id, "/dev/ttyUSB0");
    vaamInit(robot, yamlData);

    robot.Anglelimits(0, 4095, id);
    robot.setAllJointPositionControlMode(id);
    // robot.CurrentLimit((float)100, id);

    cout << "radius : " << robot.radius << endl;
    cout << "K : " << robot.K << endl;

    double phi=0;
    double deltat = 0.02;

    double state[2]={0.5, 0};
	Fourth_RK_CLASS frk(0, state[0], state[1], 1, deltat);

    sleep(1);

    for (int i = 0; i < 5000; i++)
	{
		// /* code */
        robot.getAllCurrents(robot.feedback_current, id);
        // for (int j = 0; j < P_MAX_ID; j++)
        // {
        //     cout << "feedback current: " << robot.feedback_current[j] << endl;
        //     /* code */
        // }
        
        // cout << 
		frk.RKfun(state, robot.feedback_current);
		cout << "theta = " << state[0] << ";     theta'=" << state[1] << endl;

        for (int j = 0; j < P_MAX_ID; j++)
        {
            robot.goal_position[j] = (state[0] + M_PI)*4096/(2*M_PI);
            /* code */
        }
        robot.setAllPositions(robot.goal_position, id);
        usleep(deltat*1000000);

	}

    // while(1)
    // {
    //     robot.getAllPositions(robot.feedback_position, id);
    //     robot.getAllCurrents(robot.feedback_current, id);

    //     for (int i = 0; i < P_MAX_ID; i++)
    //     {
    //         cout << "position ["<< i << "] = "<< robot.feedback_position[i] << endl;
    //         // cout << "position ["<< i << "] = "<< robot.feedback_position[i]/4096*2*M_PI << endl;
    //         cout << "current ["<< i << "] = "<< robot.feedback_current[i] << endl;
    //         /* code */
    //     }

    //     phi += 2*M_PI*deltat;
    //     if(phi > 2*M_PI)
    //         phi -= 2*M_PI;

    //     for (int i = 0; i < P_MAX_ID; i++)
    //     {
    //         robot.goal_position[i] = (M_PI / 4.) * sin(phi) + M_PI;
    //         cout << "rad_raw_goal_position[" << i << "] = " << robot.goal_position[i] << endl; 
    //         if(robot.goal_position[i] < 0)          //rad
    //         {
    //             robot.goal_position[i] += 2*M_PI;
    //         } 
    //         cout << "raw_goal_position[" << i << "] = " << robot.goal_position[i] << endl; 
    //         //now convert rad to servo coder
    //         robot.goal_position[i] *= 4096/(2*M_PI); 
    //         cout << "goal_position[" << i << "] = " << robot.goal_position[i] << endl; 
    //         /* code */
    //     }
    //     robot.setAllPositions(robot.goal_position, id);
    //     usleep(deltat*1000000);
    //     cout << "-------------------------------------"<< endl;
    // }
    robot.set_all_torque_enable(0, id);
    return 0;
}

