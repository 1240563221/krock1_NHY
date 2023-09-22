// File:            main_toflobot_controller.cpp
// Date:		    2023/09/18
// Description:	    single_leg locomotion controller
// Author:	        songtao
// Modifications:	none

#define OPTIMIZATION2
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <iostream>
#include <cmath>
#include <ctime>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdlib.h>
#include <chrono>
#include <ctime>  
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/SVD"
// #include "/usr/include/eigen3/Eigen/Eigen"
// #include "/usr/include/eigen3/Eigen/Dense"
// #include "/usr/include/eigen3/Eigen/Core"
#include <algorithm>
#include <pthread.h>
#include <vector>
#include <thread>
#include <qpOASES.hpp>

#include <iostream>
#include "yaml-cpp/yaml.h"
#include <vector>
#include "robot.h"
#include <sys/time.h>
#include <signal.h>
#include "musclemodel.h"
#include "yaml_read.h"

using namespace std;

YamlDataTPDF yamlData;
muscleModelTPDF muscleRead[P_MAX_ID];

int id[P_MAX_ID];


using namespace std;
using namespace Eigen;

#define LEG_NUM 4
#define TIME_STEP 16
#define my_pi     3.141592653589793
#define number_motor 4
#define number_trunkmotor 3

double t =0.0;
double dt;
double Tf1;
double velocity_x, velocity_y, inputTurningCurvature;
double walkingDirection, walking_forward_velocity;
double freq_walk, turning_curvature, walking_angular_velocity;
Vector4d l_trunk;

// gait parameters
double Duty;
double IG;
double l0, l1, l2, l3, l4;
double ellipse_a, ellipse_b;
double front_angle, hind_angle;
double maxSpeed;
Matrix<double, 3, 4> midstance;
Matrix<double, 3, 4> stanceStart, stanceEstEnd;
Matrix<double, 4, 1> swing_height;
Vector4d phShifts;
double spineCPGscaling;
double constrS;

Matrix<double, 3, 4> feetLocationsLocal, feetReference, feetReference_G;    
Matrix<double, 19, 1> q_output, q_input;
Matrix<double, 4, 4> calcLegAngles;
Matrix<double, 3, 1> q0_trunk_from_spline;
Matrix<double, 5, 3> length_FL, length_FR, length_HL, length_HR;
Matrix4d Rtr0, Rtr1, Rtr2, Rtr3, Rtr_g;
Matrix<double, 4, 4> Fgird, Hgird, Fgird0, Hgird0, Fgird_mean, Hgird_mean, Hhead;
double hgirdlePhi;
Vector4d legs_stance_old, legs_stance;
Vector4d phaseTransitionStanceSwing, phaseTransitionSwingStance;
Matrix<double,4,1> stancePhase, swingPhase;
Vector4d legPhase;
Matrix<double, 3, 2> forVelocity, girdleVelocity;
Vector2d forAngularVelocity, girdleAngularVelocity;
Vector2d forVelocity_filtered, forAngularVelocity_filtered;
Vector2d girdleVelocity_filtered, girdleAngularVelocity_filtered;
Matrix<double, 6, 2> forTraj, girdleTraj;
Matrix<double, 3, 300> forTrajHist;
Vector2d girdleCpgOutput;
Matrix<double,4,4> qNULL;
Vector2d ikin_lam;
Matrix4d ikin_M;
double ikin_max_dist, ikin_tol;
int ikin_maxIter;
Matrix<double,2,4> constrFL, constrHL, constrFR, constrHR;
Vector3d ikin_constr_penalty;
vector<qpOASES::QProblem> ikinQpSolver;

//functions:
void parameterConfig();
void forward_kinematics();
std::vector<Matrix<double, 4, 4>> legKinematics(Vector4d q, int leg);
Matrix<double, 3, 4> Jacob(Vector4d q, int leg);
Vector4d iKinQpOases(int leg, Vector3d pref, Vector4d qref, Vector4d q0, Vector2d lam, 
Matrix4d M, double max_dist, int maxIter, double tol, MatrixXd constr, 
Vector3d constr_penalty);
void legPhaseDynamics();
void leg_spine_generator();
void girdleTrajectories(double spineCPGscaling);
void girdleOscillations(double spineCPGscaling);
void trunkInverseKinematics();
Vector3d trunkForwardKinematics(Vector3d fgird_pos, MatrixXd q_trunk_in);
std::vector<Matrix<double,3,4>> predictTrajectories(int N, double time_step, MatrixXd *predLegPhase);
bool moveSwingLeg(int leg);
Vector3d getSwingTrajectory(Vector3d initPoint, Vector3d middlePoint, Vector3d finalPoint, double phase);
bool moveStanceLeg(int leg);
Matrix<double, 4,4> leginversekinematics();
void calcMotorPosition();



int main(int argc, char **argv) {

    // USING_NAMESPACE_QPOASES
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 舵机初始化：
//   
    // 参数设置
    parameterConfig();
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

    sleep(1);
 //主程序
 //反馈环：直到收到停止命令，停止仿真
  while (1) 
  {
    gettimeofday(&time_old, NULL);
    // /* code */  
    robot.getAllCurrentsVelocityPosition(robot.feedback_current, robot.feedback_velocity, robot.feedback_position, id);
    // gettimeofday(&time_new, NULL);
    // cout << "run times : " << time_new.tv_usec - time_old.tv_usec << " us " << endl;


    // get motor position:
    for (int i = 0; i < P_MAX_ID; i++) {
        // robot.feedback_position[i] = 2*M_PI*((int)(robot.feedback_position[i]+MAX_LIMIT_POSITION_SERVO)%MAX_LIMIT_POSITION_SERVO)/MAX_LIMIT_POSITION_SERVO - M_PI;
        // robot.feedback_velocity[i] = -robot.feedback_velocity[i]/1000.;
        q_input(i) = robot.feedback_position[i];
    }
    forward_kinematics();
    legPhaseDynamics();
    leg_spine_generator();
    //确认电机位置q_input
    calcMotorPosition();

    // set MotorPosition:
    for(int i=0; i< P_MAX_ID; i++){
        if(i == 0 )
        {
            // jointController[i]->muscleParameters.refLocation = -q_output(i);
            jointController[i]->setGoalPosition(-q_output(i));
            cout << "refLocation[" << i << "]: " << -q_output(i)<< endl;
        }
        else
        {
            // jointController[i]->muscleParameters.refLocation = q_output(i);
            jointController[i]->setGoalPosition(q_output(i));
            cout << "refLocation[" << i << "]: " << q_output(i)<< endl;
        }
    }

    for (int j = 0; j < P_MAX_ID; j++)
    {
        jointController[j]->RKfun(robot.feedback_position[j], robot.feedback_velocity[j], &robot.goal_position[j], robot.feedback_current[j]);
        cout << "theta = " << robot.goal_position[j] << endl;
        if(j == 0)
            robot.goal_position[j] = -robot.goal_position[j];
            // robot.goal_position[j] = 0;
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
    gettimeofday(&time_new, NULL);
    cout << "run times : " << time_new.tv_usec - time_old.tv_usec << " us " << endl;

    t += (double)TIME_STEP / 1000.0;
  }

  // Enter here exit cleanup code.
  return 0;
}

void
parameterConfig()
{
    dt = TIME_STEP;
    Tf1 = 300;
    maxSpeed = 0.3;
    velocity_x = 0.3;
    velocity_y = 0;
    freq_walk = 0.6;
    phShifts << 0.35, 0.5, 0.4, 0.9;
    Duty =0.7;
    q_input.setZero();
    q_output.setZero();
    girdleCpgOutput.setZero();
    legPhase << 0., 0., 0., 0.;
    inputTurningCurvature = 0;

    forVelocity.setZero();
    girdleVelocity.setZero();
    forAngularVelocity.setZero();
    girdleAngularVelocity.setZero();
    forVelocity_filtered.setZero();
    forAngularVelocity_filtered.setZero();
    girdleVelocity_filtered.setZero(); 
    girdleAngularVelocity_filtered.setZero();
    

    l_trunk << 0.08, 0.155, 0.155, 0.075;//已更新
    IG = 0.465;
    length_FL << 
        0, 0.051, 0,
        0, 0, 0,
        0, 0.1474, 0,
        0, 0, 0,
        0, 0, -0.132;
    length_FR << 
        0, -0.051, 0,
        0, 0, 0,
        0, -0.1474, 0,
        0, 0, 0,
        0, 0, -0.132;
    length_HL << 
        0, 0.051, 0,
        0, 0, 0,
        0, 0.1474, 0,
        0, 0, 0,
        0, 0, -0.132;
    length_HR << 
        0, -0.051, 0,
        0, 0, 0,
        0, -0.1474, 0,
        0, 0, 0,
        0, 0, -0.132;

    l0 = 0.051;
    l1 = 0;
    l2 = 0.1474;
    l3 = 0;
    l4 = 0.132;
    ellipse_a = 0.1;
    ellipse_b = 0.05;
    midstance << 0, 0.0, 0., 0.,
                 0.1984, -0.23, 0.23, -0.23,
                 -0.132, -0.132, -0.132, -0.132;

    swing_height << 0.05 , 0.07, 0.07, 0.07;
    spineCPGscaling = 0.15;
    constrS = my_pi/6.;
    qNULL << 0,0,0,0,
            0,0,0,0,
            0,0,0,0,
            0,0,0,0;
    ikin_lam << 0.01, 0.01;
    ikin_M <<0.01,	0,	0,	0,
            0,	0.01,	0,	0,
            0,	0,	0.01,	0,
            0,	0,	0,	0.01;
    ikin_max_dist =0.03;
    ikin_tol=0.001;
    ikin_maxIter=4;
    ikin_constr_penalty<<0.001,2,1.01;

    constrFL << -50., -75., -50., -20.,
                50.,  75.,	50., 75.;

    // constrFL << -103., -75., -100., -20.,
    //             103.,  75.,	190., 75.;

    constrFR << -103., -75., -100., -75.,
                103.,  75.,	190., 20.;

    constrHL << -103., -75., -190., -75.,
                103.,  75.,	100., 20.;

    constrHR << -103., -75., -190., -20.,
                103.,  75.,	100., 75.;     
}


void forward_kinematics()
{
    static MatrixXd feetLocations_old(3,4);
    // Front girdle
    Fgird.setZero();
    Fgird.block<3,3>(0, 0)=MatrixXd::Identity(3,3);
    Fgird(3,3)=1;

    //Trunk,calculate hgirdle frame-from front to hind
    Rtr0<<  cos(q_input(16)), -sin(q_input(16)), 0, -l_trunk(0),
              sin(q_input(16)), cos(q_input(16)), 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
    Rtr_g = Rtr0;

    Rtr1<<cos(q_input(17)), -sin(q_input(17)), 0, -l_trunk(1),
            sin(q_input(17)), cos(q_input(17)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    Rtr_g=Rtr_g*Rtr1;

    Rtr2<<cos(q_input(18)), -sin(q_input(18)), 0, -l_trunk(2),
            sin(q_input(18)), cos(q_input(18)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    Rtr_g=Rtr_g*Rtr2;

    Rtr3<<1, 0, 0, -l_trunk(number_trunkmotor),
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;   
    Rtr_g=Rtr_g*Rtr3;
    //key calc, get hgirdle frame
    Hgird=Rtr_g;      
}


std::vector<Matrix<double, 4, 4>> legKinematics(Vector4d q, int leg)
{

    static vector<Matrix4d> H(5);

    if(leg==0){     //FL
        H[0] << cos(q(0)), -sin(q(0)), 0, length_FL(0,0),
                sin(q(0)), cos(q(0)), 0, length_FL(0,1),
                0, 0, 1, length_FL(0,2),
                0, 0, 0, 1;
        H[1] << 1, 0, 0, length_FL(1,0),
                0, cos(q(1)), -sin(q(1)), length_FL(1,1),
                0, sin(q(1)), cos(q(1)), length_FL(1,2),
                0, 0, 0, 1;
        H[2] << cos(q(2)), 0, sin(q(2)), length_FL(2,0),
                0,  1, 0, length_FL(2,1),
                -sin(q(2)), 0, cos(q(2)), length_FL(2,2),
                0, 0, 0, 1;
        H[3] << 1, 0, 0, length_FL(3,0),
                0, cos(q(3)), -sin(q(3)), length_FL(3,1),
                0, sin(q(3)), cos(q(3)), length_FL(3,2),
                0, 0, 0, 1;
        H[4] << 1, 0, 0, length_FL(4,0),
                0, 1, 0, length_FL(4,1),
                0, 0, 1, length_FL(4,2),
                0, 0, 0, 1;
    }

    else if (leg==1){       //FR
        H[0] << cos(q(0)), -sin(q(0)), 0, length_FR(0,0),
                sin(q(0)), cos(q(0)), 0, length_FR(0,1),
                0, 0, 1, length_FR(0,2),
                0, 0, 0, 1;
        H[1] << 1, 0, 0, length_FR(1,0),
                0, cos(q(1)), -sin(q(1)), length_FR(1,1),
                0, sin(q(1)), cos(q(1)), length_FR(1,2),
                0, 0, 0, 1;
        H[2] << cos(q(2)), 0, sin(q(2)), length_FR(2,0),
                0,  1, 0, length_FR(2,1),
                -sin(q(2)), 0, cos(q(2)), length_FR(2,2),
                0, 0, 0, 1;
        H[3] << 1, 0, 0, length_FR(3,0),
                0, cos(q(3)), -sin(q(3)), length_FR(3,1),
                0, sin(q(3)), cos(q(3)), length_FR(3,2),
                0, 0, 0, 1;
        H[4] << 1, 0, 0, length_FR(4,0),
                0, 1, 0, length_FR(4,1),
                0, 0, 1, length_FR(4,2),
                0, 0, 0, 1;
    }
    else if (leg==2){       //HL
        H[0] << cos(q(0)), -sin(q(0)), 0, length_HL(0,0),
                sin(q(0)), cos(q(0)), 0, length_HL(0,1),
                0, 0, 1, length_HL(0,2),
                0, 0, 0, 1;
        H[1] << 1, 0, 0, length_HL(1,0),
                0, cos(q(1)), -sin(q(1)), length_HL(1,1),
                0, sin(q(1)), cos(q(1)), length_HL(1,2),
                0, 0, 0, 1;
        H[2] << cos(q(2)), 0, sin(q(2)), length_HL(2,0),
                0,  1, 0, length_HL(2,1),
                -sin(q(2)), 0, cos(q(2)), length_HL(2,2),
                0, 0, 0, 1;
        H[3] << 1, 0, 0, length_HL(3,0),
                0, cos(q(3)), -sin(q(3)), length_HL(3,1),
                0, sin(q(3)), cos(q(3)), length_HL(3,2),
                0, 0, 0, 1;
        H[4] << 1, 0, 0, length_HL(4,0),
                0, 1, 0, length_HL(4,1),
                0, 0, 1, length_HL(4,2),
                0, 0, 0, 1;
    } 
    else if (leg==3){       //HR
        H[0] << cos(q(0)), -sin(q(0)), 0, length_HR(0,0),
                sin(q(0)), cos(q(0)), 0, length_HR(0,1),
                0, 0, 1, length_HR(0,2),
                0, 0, 0, 1;
        H[1] << 1, 0, 0, length_HR(1,0),
                0, cos(q(1)), -sin(q(1)), length_HR(1,1),
                0, sin(q(1)), cos(q(1)), length_HR(1,2),
                0, 0, 0, 1;
        H[2] << cos(q(2)), 0, sin(q(2)), length_HR(2,0),
                0,  1, 0, length_HR(2,1),
                -sin(q(2)), 0, cos(q(2)), length_HR(2,2),
                0, 0, 0, 1;
        H[3] << 1, 0, 0, length_HR(3,0),
                0, cos(q(3)), -sin(q(3)), length_HR(3,1),
                0, sin(q(3)), cos(q(3)), length_HR(3,2),
                0, 0, 0, 1;
        H[4] << 1, 0, 0, length_HR(4,0),
                0, 1, 0, length_HR(4,1),
                0, 0, 1, length_HR(4,2),
                0, 0, 0, 1;
    }
    // feet location in girdle body frame
    // return H[0]*H[1]*H[2]*H[3];
    return H;
}

/* Returns jacobian matrix of one leg */
Matrix<double, 3, 4> Jacob(Vector4d q, int leg)
{
    static MatrixXd J(3, 4);
    static double S0, S1, S2, S3, C0, C1, C2, C3;
    S0 = sin(q(0));
    S1 = sin(q(1));
    S2 = sin(q(2));
    S3 = sin(q(3));
    C0 = cos(q(0));
    C1 = cos(q(1));
    C2 = cos(q(2));
    C3 = cos(q(3));

    if(leg==0){     //FL
        J(0) = - l4*(C1*C0*S3 + C3*(-S0*S2 + C2*C0*S1))-C1*l2*C0;
        J(1) = - l4*(-S1*S0*S3 + C3*(C2*S0*C1))+S1*l2*S0;
        J(2) = - l4*(C3*(C0*C2 - S2*S0*S1));
        J(3) = - l4*(C1*S0*C3 - S3*(C0*S2 + C2*S0*S1));
        J(4) = - l4*(S0*C1*S3 + C3*(S0*C2*S1 + C0*S2)) - S0*C1*l2;
        J(5) = - l4*(C0*S1*S3 + C3*(-C0*C2*C1)) - C0*S1*l2;
        J(6) = - l4*(C3*(C0*S2*S1 + S0*C2));
        J(7) = - l4*(-C0*C1*C3 - S3*(-C0*C2*S1 + S0*S2));
        J(8) = 0;
        J(9) = -l4*(-C1*S3-S1*C2*C3) + l2*C1;
        J(10) = -l4*(-C1*S2*C3);
        J(11) = -l4*(-S1*C3-C1*C2*S3);
    }
    else if (leg==1){       //FR
        J(0) = -l4*(C0*C1*S3+(-S0*S2+C0*S1*S2)*C3) + l2*C0*C1;
        J(1) = -l4*(-S0*S1*S3+(S0*C1*S2)*C3) - l2*S0*S1;
        J(2) = -l4*(0 +(C0*C2+S0*S1*C2)*C3);
        J(3) = -l4*(S0*C1*C3 - (C0*S2+S0*S1*S2)*S3);
        J(4) = -l4*(S0*C1*S3+(C0*S2+S0*S1*S2)*C3) + l2*S0*C1 ;
        J(5) = -l4*(C0*S1*S3+(-C0*C1*S2)*C3) + l2*C0*S1 ;
        J(6) = -l4*(0+(S0*C2-C0*S1*C2)*C3) ;
        J(7) = -l4*(-C0*C1*C3-(S0*S2-C0*S1*S2)*S3);
        J(8) = 0;
        J(9) = -l4*(-C1*S3-S1*C2*C3) - l2*C1;
        J(10) = -l4*(-C1*S2*C3);
        J(11) = -l4*(-S1*C3-C1*C2*S3);
    }
    else if (leg==2){       //HL
        J(0) = -l4*(C0*C1*S3+(-S0*S2+C0*S1*S2)*C3) - l2*C0*C1;
        J(1) = -l4*(-S0*S1*S3+(S0*C1*S2)*C3) + l2*S0*S1;
        J(2) = -l4*(0 +(C0*C2+S0*S1*C2)*C3);
        J(3) = -l4*(S0*C1*C3 - (C0*S2+S0*S1*S2)*S3);
        J(4) = -l4*(S0*C1*S3+(C0*S2+S0*S1*S2)*C3) - l2*S0*C1 ;
        J(5) = -l4*(C0*S1*S3+(-C0*C1*S2)*C3) - l2*C0*S1 ;
        J(6) = -l4*(0+(S0*C2-C0*S1*C2)*C3) ;
        J(7) = -l4*(-C0*C1*C3-(S0*S2-C0*S1*S2)*S3);
        J(8) = 0;
        J(9) = -l4*(-C1*S3-S1*C2*C3) + l2*C1;
        J(10) = -l4*(-C1*S2*C3);
        J(11) = -l4*(-S1*C3-C1*C2*S3);
    }
    else if (leg==3){       //HR
        J(0) = -l4*(C0*C1*S3+(-S0*S2+C0*S1*S2)*C3) + l2*C0*C1;
        J(1) = -l4*(-S0*S1*S3+(S0*C1*S2)*C3) - l2*S0*S1;
        J(2) = -l4*(0 +(C0*C2+S0*S1*C2)*C3);
        J(3) = -l4*(S0*C1*C3 - (C0*S2+S0*S1*S2)*S3);
        J(4) = -l4*(S0*C1*S3+(C0*S2+S0*S1*S2)*C3) + l2*S0*C1 ;
        J(5) = -l4*(C0*S1*S3+(-C0*C1*S2)*C3) + l2*C0*S1 ;
        J(6) = -l4*(0+(S0*C2-C0*S1*C2)*C3) ;
        J(7) = -l4*(-C0*C1*C3-(S0*S2-C0*S1*S2)*S3);
        J(8) = 0;
        J(9) = -l4*(-C1*S3-S1*C2*C3) - l2*C1;
        J(10) = -l4*(-C1*S2*C3);
        J(11) = -l4*(-S1*C3-C1*C2*S3);
    }

    return J;
}

void legPhaseDynamics()
{
  static MatrixXd legCPGtheta=MatrixXd::Ones(4,2)*0.5*2*my_pi; 

    //===================================== LEG CPG ===================================================
	for(int i=0; i<4; i++){
        legPhase[i] = freq_walk*t+phShifts(i);
        legPhase[i]=fmod(legPhase[i], 1.);
    }

    //============================== DETECT STANCE AND SWING TRANSITIONS =============================
    for(int i=0; i<4; i++){
        if(legPhase[i]<Duty)
        {
            legs_stance(i)=1;
        }
        else{
            legs_stance(i)=0;
        }
    }
    legs_stance_old=legs_stance;
}

void leg_spine_generator()
{
  for(int i=0; i<LEG_NUM; i++){
		  if(legs_stance(i)==0){
            
        moveSwingLeg(i);
		  }
      else{
        moveStanceLeg(i);
      }
	  }
    girdleTrajectories(spineCPGscaling);
    
    // for(int i=0;i<LEG_NUM;i++)
    // {
    // feetReference_G.block(0,i,3,1) = AngleAxisd(-girdleCpgOutput(i/2), Vector3d::UnitZ())*feetReference.block(0,i,3,1);
    // }
    //need to recalculate
    calcLegAngles = leginversekinematics();
}

/* Takes care of a swing phase of the swinging leg */
bool moveSwingLeg(int leg_index)
{
    static Vector3d trajPoint;
    static MatrixXd initPoint(3,4), middlePoint(3,4), finalPoint(3,4);
    //feet end point position in the girdle coordinate, so Fgird and Hgird are for, not be fixed to body

    //state transition from stance to swing
    initPoint.block<3,1>(0,leg_index) << midstance(0,leg_index)-ellipse_a, midstance(1,leg_index), midstance(2,leg_index);
    finalPoint.block<3,1>(0,leg_index) << midstance(0,leg_index)+ellipse_a, midstance(1,leg_index), midstance(2,leg_index);
    
    stanceStart.block(0,leg_index,3,1)=finalPoint.block(0,leg_index,3,1);
    stanceEstEnd.block(0,leg_index,3,1)=initPoint.block(0,leg_index,3,1);

    middlePoint.block<3,1>(0,leg_index)=midstance.block<3,1>(0,leg_index);
    middlePoint(2,leg_index) += swing_height(leg_index);

    //calc swing phase, see swing as a complete circloe
    swingPhase(leg_index)=(legPhase[leg_index]-Duty)/(1.-Duty);

    
    trajPoint=getSwingTrajectory(initPoint.block<3,1>(0,leg_index), middlePoint.block<3,1>(0,leg_index), finalPoint.block<3,1>(0,leg_index), swingPhase(leg_index));
    
    
    if(leg_index>1){
        trajPoint(0) += -IG-Hgird(0, 3);
    }    
    // get feetreference in girdle body frame
    feetReference.block<3,1>(0,leg_index) = trajPoint;

    if(swingPhase(leg_index)>=1){
        return true;
    }
    return false;
    
}

Vector3d getSwingTrajectory(Vector3d initPoint, Vector3d middlePoint, Vector3d finalPoint, double phase)
{   
    Matrix<double, 3, 1> Scurve; 
    // cout << "index  "<< leg_index << endl;
    double s = phase*(1.-Duty);
    double T0 = 1.-Duty;

    if(phase < 0.5)
    {
        Scurve <<
        initPoint(0) + (finalPoint(0)-initPoint(0))
        * (s / T0 - 1. / 2. / my_pi * sin(2. * my_pi * s / T0)),
        midstance(1),
        midstance(2)+ swing_height(0)*
        (2 * s/T0 - 1./2./my_pi*sin(4 *my_pi * s /T0));
    }
    else
    {
        Scurve <<
        initPoint(0) + (finalPoint(0)-initPoint(0)) 
        * (s / T0 - 1. / 2. / my_pi * sin(2 * my_pi * s / T0)),
        midstance(1),
        midstance(2)+ swing_height(0)* 
        (2. - 2 * s / T0 + 1./2./my_pi * sin(4 * my_pi * s / T0));
    }

    // Scurve << initPoint(0,leg_index)+(finalPoint(0,leg_index)-initPoint(0,leg_index))*phase,
    //             initPoint(1,leg_index)+(finalPoint(1,leg_index)-initPoint(1,leg_index))*phase,
    //             midstance(2,leg_index)+abs(swing_height(leg_index)*sin(phase*my_pi));
    return Scurve;

} 

/* Takes care of a stance phase of the stancing leg */
bool moveStanceLeg(int leg_index)
{
    static Vector3d trajPoint;
    static Matrix<double,3,4> initPoint, middlePoint, finalPoint;
    //midstance, stanceStart, stanceEstEnd;
    middlePoint = midstance;
    finalPoint.block(0,leg_index,3,1) << midstance(0,leg_index)-ellipse_a, midstance(1,leg_index), midstance(2,leg_index);
    initPoint.block(0,leg_index,3,1) <<  midstance(0,leg_index)+ellipse_a, midstance(1,leg_index), midstance(2,leg_index);

    stanceStart.block(0,leg_index,3,1)=initPoint.block(0,leg_index,3,1);
    stanceEstEnd.block(0,leg_index,3,1)=finalPoint.block(0,leg_index,3,1);

    stancePhase(leg_index)=legPhase[leg_index]/Duty;

    trajPoint <<
    initPoint(0,leg_index) + (finalPoint(0,leg_index)-initPoint(0,leg_index)) * 
    (stancePhase(leg_index) - 1./2./ my_pi *sin(2 * my_pi * stancePhase(leg_index))),
    middlePoint(1,leg_index),
    middlePoint(2,leg_index);

    // trajPoint=initPoint.block(0, leg_index, 3,1) + stancePhase(leg_index) * (finalPoint.block(0, leg_index, 3,1) - initPoint.block(0, leg_index, 3,1));
        
    if(leg_index>1){
        trajPoint(0) += -IG-Hgird(0, 3);
    }

    // get feetreference in girdle body frame
    feetReference.block<3,1>(0,leg_index) = trajPoint;  

    if(stancePhase(leg_index)>=1){
        return true;
    }
    return false;
    
}

/* calculates girdle trajctories / velocities for given input commands - linear and angular velocity of the front girdle */
void girdleTrajectories(double spineCPGscaling)
{
	// -------------------------------------- GIRDLE OSCILLATIONS -----------------------------------------
    girdleOscillations(spineCPGscaling);
    // -------------------------------------- SPINE INVERSE KINEMATICS -----------------------------------------
	// get all motors output q_trunk
    trunkInverseKinematics();
}


/* Calculate girdle oscillations to follow leg movements */
void girdleOscillations(double spineCPGscaling)
{
    int N=60;
	double pred_dt=0.05;
	// predict trajectories
	std::vector<Matrix<double,3,4>> predictedFootsteps;
	predictedFootsteps.resize(N);

	static MatrixXd predLegPhase(N,4);
	predictedFootsteps=predictTrajectories(N, pred_dt, &predLegPhase);
	static MatrixXd scaled_phases(4,N);
	scaled_phases=predLegPhase.transpose();

	//------------------------------- MESSY FOOTSTEP ANGLE PREDICTION -------------------------------
	double footX, footY, tmp;
	for(int i=0; i<N; i++)
	{
		for(int j=0; j<4; j++)
		{
			if(predLegPhase(i,j)<0)
					predLegPhase(i,j)+=1;

			if(predLegPhase(i, j)<Duty)
			{
				tmp=(predLegPhase(i,j))/(Duty);

				footX=(1-tmp)*stanceStart(0,0) + tmp*stanceEstEnd(0,0);
				footY=(1-tmp)*stanceStart(1,0) + tmp*stanceEstEnd(1,0);
			}
			else
			{

				tmp=(predLegPhase(i,j)-Duty)/(1-Duty);

				footX=(1-tmp)*stanceEstEnd(0,0) + tmp*stanceStart(0,0);
				footY=(1-tmp)*stanceEstEnd(1,0) + tmp*stanceStart(1,0);
			}

			scaled_phases(j,i)=atan2(footX, footY);
		}
	}
	// one side legs have to be transformed
	scaled_phases.block(0,0,1,N)*=-1;
	scaled_phases.block(2,0,1,N)*=-1;

	// get reference for both girdles, the predicted phi_mean
	static MatrixXd girdleRefFromLegs(2,N);
	girdleRefFromLegs.block(0,0,1,N) = spineCPGscaling*(scaled_phases.block(0,0,1,N) + scaled_phases.block(1,0,1,N));
	girdleRefFromLegs.block(1,0,1,N) = spineCPGscaling*(scaled_phases.block(2,0,1,N) + scaled_phases.block(3,0,1,N));

	// init cpg
	double a=1;
	//按行操作，求Rmax
	Vector2d R=girdleRefFromLegs.rowwise().maxCoeff();
	static MatrixXd theta=MatrixXd::Zero(2,2), r=MatrixXd::Zero(2,2);
	Vector2d dr, dtheta;

	//----------------------------- DFT PHASE ESTIMATION ------------------------------
	double re, im;
	static MatrixXd phase_est=MatrixXd::Zero(2,2);
	// double phase_diff;
	static Vector2d phase_correction;
	for(int i=0; i<2; i++){
		re=0; im=0;
		for(int k=0; k<N; k++){
			re = re + girdleRefFromLegs(i,k)*cos(2*my_pi*freq_walk*k*pred_dt);
			im = im - girdleRefFromLegs(i,k)*sin(2*my_pi*freq_walk*k*pred_dt);
		}
		phase_est(i, 0)=phase_est(i, 1);
		phase_est(i, 1)=atan2(im, re);

	//the second term is a modification
		phase_correction(i)=phase_est(i,0)+ round((theta(i,0)-phase_est(i,0))/(2*my_pi))*2*my_pi - theta(i,0);
    
	}

	//------------------------------------- RUN CPG -------------------------------------

	dtheta = 2*my_pi*freq_walk*MatrixXd::Ones(2,1) + 2*phase_correction;

	// update radius
	// R=R*(1-abs(turning_curvature))*abs(cos(walkingDirection));
	dr = a*(R-r.block(0,0,2,1));

	// euler integration
	theta.block(0,1,2,1)=theta.block(0,0,2,1) + dt*dtheta/1000.;
	r.block(0,1,2,1)=r.block(0,0,2,1) + dt*dr/1000.;

	// angle output
	girdleCpgOutput(0)=r(0,0)*cos(theta(0,0));
	girdleCpgOutput(1)=r(1,0)*cos(theta(1,0));

	// udpate old values
	theta.block(0,0,2,1)=theta.block(0,1,2,1);
	r.block(0,0,2,1)=r.block(0,1,2,1);

	return;

}

std::vector<Matrix<double,3,4>>
predictTrajectories(int N, double time_step, MatrixXd *predLegPhase_in)
{

	std::vector<Matrix<double,3,4>> predictedFootsteps;
	predictedFootsteps.resize(N);

	// double omega=2*my_pi*freq_walk;
	MatrixXd predLegPhase(N,4);
	predLegPhase.block(0,0,1,4)=legPhase.transpose();

	// initial (current) points (all in girdle frame reference)
	predictedFootsteps[0].block<3,1>(0,0)=feetReference.block<3,1>(0,0);
	predictedFootsteps[0].block<3,1>(0,1)=feetReference.block<3,1>(0,1);
	predictedFootsteps[0].block<3,1>(0,2)=feetReference.block<3,1>(0,2);
	predictedFootsteps[0].block<3,1>(0,3)=feetReference.block<3,1>(0,3);

	// starting point for stance phase
	MatrixXd stanceStartingPoint(3,4);

	stanceStartingPoint=stanceStart;
	Transform<double,3,Affine> Girdle_transformation;


	// if swing phase, footsteps invalid
	for(int j=0; j<4; j++){
		if(predLegPhase(0,j)>Duty){
			predictedFootsteps[0].block<3,1>(0,j) << -9999,
							                   -9999,
			   				                   -9999;
		}
	}

	// LOOK INTO THE FUTURE
	for(int i=1; i<N; i++){


		// run legPhaseDynamics and determine if swing or stance
		predLegPhase.block(i,0,1,4)=predLegPhase.block(i-1,0,1,4)+freq_walk*time_step*MatrixXd::Ones(1,4);
		for(int j=0; j<4; j++){
			predLegPhase(i,j)=predLegPhase(i,j)>1 ? (0+(predLegPhase(i,j)-1)) : predLegPhase(i,j);
		}

		// when in stance move the leg with current speed profiles
		for(int j=0; j<4; j++){

			// ignore if in swing phase
			if(predLegPhase(i,j)>Duty){
				predictedFootsteps[i].block<3,1>(0,j) << -9999,
								                   -9999,
				   				                   -9999;
               	continue;
			}
            
			// if comming back from the swing phase (landing)
			if(predictedFootsteps[i-1](2,j)==-9999){
				predictedFootsteps[i].block<3,1>(0,j)=stanceStartingPoint.block<3,1>(0,j);
				continue;
			}
			// otherwise, move leg with a current speed
            
			// Girdle_transformation = AngleAxisd(-forAngularVelocity_filtered(j/2)*time_step, Vector3d::UnitZ()) *
			// 									Translation3d((-forVelocity_filtered.block<3,1>(0,j/2))*time_step);

			predictedFootsteps[i].block<3,1>(0,j)=Girdle_transformation*predictedFootsteps[i-1].block<3,1>(0,j);

		}

	}


	*predLegPhase_in=predLegPhase;
	return predictedFootsteps;
}

/* Solves spine inverse kinematics */
void trunkInverseKinematics()
{
    // ------------------------------------------ RUNNING GIRDLE CPG AND SPLINING STUFF -------------------------------
    // spline parameter
    static MatrixXd t_spline(1, number_trunkmotor+2);
    static MatrixXd p_spline(2, t_spline.size());
    static Vector2d old_solution;

    // spline points and vectors
	double m0_spline, m1_spline;

    //girdleCpgOutput*=(1-abs(gamma));
    //fortraj: absolute frame to get absolute vector m0, m1;
    m0_spline = girdleCpgOutput(0);
    m1_spline = girdleCpgOutput(1);
    // get trunk angles
    // static double const scl=0.4;
    for(int i=0; i<number_trunkmotor; i++){
        q0_trunk_from_spline(i)=(m0_spline - m1_spline) / (double)number_trunkmotor;
    }
    for(int i=0; i<q0_trunk_from_spline.size(); i++){
        q0_trunk_from_spline(i) = q0_trunk_from_spline(i) > constrS ? constrS : q0_trunk_from_spline(i);
        q0_trunk_from_spline(i) = q0_trunk_from_spline(i) <-constrS ?-constrS : q0_trunk_from_spline(i);
    }

}

Matrix<double, 4,4> leginversekinematics()
{
    static MatrixXd qSol = MatrixXd::Zero(4,4);
    qSol.block<4,1>(0,0)=iKinQpOases(0, feetReference.block<3,1>(0,0), qNULL.block<4,1>(0,0), qSol.block<4,1>(0,0), ikin_lam, ikin_M, ikin_max_dist, ikin_maxIter, ikin_tol, constrFL, ikin_constr_penalty);
    qSol.block<4,1>(0,1)=iKinQpOases(1, feetReference.block<3,1>(0,1), qNULL.block<4,1>(0,1), qSol.block<4,1>(0,1), ikin_lam, ikin_M, ikin_max_dist, ikin_maxIter, ikin_tol, constrFR, ikin_constr_penalty);
    qSol.block<4,1>(0,2)=iKinQpOases(2, feetReference.block<3,1>(0,2), qNULL.block<4,1>(0,2), qSol.block<4,1>(0,2), ikin_lam, ikin_M, ikin_max_dist, ikin_maxIter, ikin_tol, constrHL, ikin_constr_penalty);
    qSol.block<4,1>(0,3)=iKinQpOases(3, feetReference.block<3,1>(0,3), qNULL.block<4,1>(0,3), qSol.block<4,1>(0,3), ikin_lam, ikin_M, ikin_max_dist, ikin_maxIter, ikin_tol, constrHR, ikin_constr_penalty);
    return qSol;
}

/* Solves inverse kinematics for one leg by using Damped Inverse Jacobian method */
Vector4d iKinQpOases(int leg, Vector3d pref, Vector4d qref, Vector4d q0, Vector2d lam, 
Matrix4d M, double max_dist, int maxIter, double tol, MatrixXd constr, Vector3d constr_penalty)
{
    static MatrixXd J(3,4), H(4,4), D(4,4), U(3,3), V(4,4), S(3,1), Jnull(4,4);
    static MatrixXd H_qp(4,4), h_qp(4,1), lb(4,1), ub(4,1), A_qp(2,4), lbA(2,1), ubA(2,1);
    static qpOASES::real_t H_qpoases[4*4], h_qpoases[4], lb_qpoases[4], ub_qpoases[4], A_qpoases[2*4], lbA_qpoases[2], ubA_qpoases[2];
    static Vector4d dqref, dq, q0_first;
    static Vector3d dpref, p0;
    static double norm_dp;
    static vector<bool> is_init={false, false, false, false};
    vector<Matrix4d> H_vec(5);  
    q0_first = q0;
    // for(int k=0; k<maxIter; k++){
        // get current position
        H_vec=legKinematics(q0, leg);
        H=H_vec[0]*H_vec[1]*H_vec[2]*H_vec[3]*H_vec[4];
        p0=H.block<3, 1>(0, 3);
        if(leg == 0)
        printf("p0x: %f, p0y: %f, p0z: %f\n", p0(0), p0(1), p0(2));
        // get Jacobians
        J=Jacob(q0, leg);
        JacobiSVD<MatrixXd> svd(J, ComputeFullU | ComputeFullV);
        S=svd.singularValues();
        U=svd.matrixU();
        V=svd.matrixV();
        D=Matrix4d::Zero();
        D(3,3)=1;
        Jnull=V*D*V.transpose();
        // get reference velocities
        dqref=qref-q0;
        dpref=pref-p0;
        // limit maximum linear velocity
        norm_dp=sqrt(dpref(0)*dpref(0)+dpref(1)*dpref(1)+dpref(2)*dpref(2));
        if(norm_dp>max_dist){
            dpref=max_dist/norm_dp*dpref;
        }
        //=========================================== QP ================================================
        // construct qp matrices
        H_qp=J.transpose()*J + M + lam(0)*MatrixXd::Identity(4,4);
        h_qp=-J.transpose()*dpref - M*Jnull*dqref;
        lb=constr.block<1,4>(0,0).transpose() - q0;
        ub=constr.block<1,4>(1,0).transpose() - q0;
        Map<MatrixXd>( &H_qpoases[0], H_qp.cols(), H_qp.rows() ) = H_qp.transpose();
        Map<MatrixXd>( &h_qpoases[0], h_qp.rows(), h_qp.cols() ) = h_qp;
        Map<MatrixXd>( &lb_qpoases[0], lb.rows(), lb.cols() ) = lb;
        Map<MatrixXd>( &ub_qpoases[0], ub.rows(), ub.cols() ) = ub;

        A_qp << 1,1,0,0,
                1,-1,0,0;
        lbA <<  constr(0,0)*0 -13500*my_pi/180. -(q0(0)+q0(1)), 
                constr(0,1)*0 -13500*my_pi/180. -(q0(0)-q0(1));                  
        ubA <<  constr(1,0)*0 +13500*my_pi/180. -(q0(0)+q0(1)), 
                constr(1,1)*0 +13500*my_pi/180. -(q0(0)-q0(1));         
        Map<MatrixXd>( &A_qpoases[0], A_qp.cols(), A_qp.rows() ) = A_qp.transpose();
        Map<MatrixXd>( &lbA_qpoases[0], lbA.rows(), lbA.cols() ) = lbA;
        Map<MatrixXd>( &ubA_qpoases[0], ubA.rows(), ubA.cols() ) = ubA;
        // init qp solver
        qpOASES::int_t nWSR = 300;
        if(!is_init[leg]){
            qpOASES::QProblem tmpSolver(4, 2);
            ikinQpSolver.resize(LEG_NUM, qpOASES::QProblem(4, 2));
            ikinQpSolver[leg]=tmpSolver;
            qpOASES::Options myOptions;
            myOptions.printLevel = qpOASES::PL_NONE;
            ikinQpSolver[leg].setOptions(myOptions);
        }
        ikinQpSolver[leg].init( H_qpoases, h_qpoases, A_qpoases, lb_qpoases, ub_qpoases, lbA_qpoases, ubA_qpoases, nWSR, 0 );
        // get solution
        qpOASES::real_t sol_qpoases[4];
        ikinQpSolver[leg].getPrimalSolution(sol_qpoases);
        dq(0)=sol_qpoases[0];
        dq(1)=sol_qpoases[1];
        dq(2)=sol_qpoases[2];
        dq(3)=sol_qpoases[3];

        q0+=dq;
    return q0;
}


void calcMotorPosition()
{
  for(int i=0; i<LEG_NUM; i++){
    q_output.block<4,1>(4*i,0) = calcLegAngles.block<4,1>(0,i);
  }
    q_output.block<3,1>(16, 0) = q0_trunk_from_spline;
}
