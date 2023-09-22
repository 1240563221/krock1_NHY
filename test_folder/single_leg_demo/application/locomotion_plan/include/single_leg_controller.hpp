// File:            main_toflobot_controller.cpp
// Date:		    2023/09/18
// Description:	    single_leg locomotion controller demo
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
#include <algorithm>
#include <pthread.h>
#include <vector>
#include <thread>
#include <qpOASES.hpp>

#include <iostream>
#include "yaml-cpp/yaml.h"
#include <vector>
// #include "robot.h"
#include <sys/time.h>
#include <signal.h>
#include "gait_parameters.hpp"
// #include "musclemodel.h"
// #include "yaml_read.h"

using namespace std;
using namespace Eigen;
// YamlDataTPDF yamlData;
// muscleModelTPDF muscleRead[P_MAX_ID];

#define LEG_NUM 4
#define TIME_STEP 16
#define my_pi     3.141592653589793
#define number_motor 4
#define number_trunkmotor 3

#define P_MAX_ID 4

class Locomotion_controller
{
    public:

    double dt;
    double Tf1;
    double velocity_x, velocity_y, inputTurningCurvature;
    double walkingDirection, walking_forward_velocity;
    // double freq_walk, turning_curvature, walking_angular_velocity;
    Vector4d l_trunk;

    // gait parameters
    double IG;
    double l0, l1, l2, l3, l4;
    double front_angle, hind_angle;
    double maxSpeed;
    Matrix<double, 3, 4> stanceStart, stanceEstEnd;
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
    // Matrix<double,4,4> qNULL;
    Vector2d ikin_lam;
    Matrix4d ikin_M;
    double ikin_max_dist, ikin_tol;
    int ikin_maxIter;
    Matrix<double,2,4> constrFL, constrHL, constrFR, constrHR;
    Vector3d ikin_constr_penalty;
    vector<qpOASES::QProblem> ikinQpSolver;
    
    gaitParams GP1;
    Vector3d p_ref, p_cur;
    Matrix<double,10,1> traj_log;
    
    //functions:
    void run_step(double* input, double* output, double t);
    Matrix<double,10,1> get_traj_log();

    void variable_initial();
    void constant_initial();
    void parameterConfig(); 

    void forward_kinematics();
    std::vector<Matrix<double, 4, 4>> legKinematics(Vector4d q, int leg);
    Matrix<double, 3, 4> Jacob(Vector4d q, int leg);
    Vector4d iKinQpOases(int leg, Vector3d pref, Vector4d qref, Vector4d q0, Vector2d lam, 
    Matrix4d M, double max_dist, int maxIter, double tol, MatrixXd constr, 
    Vector3d constr_penalty);
    void legPhaseDynamics(double t_cur);
    void leg_traj_generator();
    void girdleTrajectories(double spineCPGscaling);
    void girdleOscillations(double spineCPGscaling);
    void trunkInverseKinematics();
    Vector3d trunkForwardKinematics(Vector3d fgird_pos, MatrixXd q_trunk_in);
    std::vector<Matrix<double,3,4>> predictTrajectories(int N, double time_step, MatrixXd *predLegPhase);
    bool moveSwingLeg(int leg);
    Vector3d getSwingTrajectory(Vector3d initPoint, Vector3d middlePoint, Vector3d finalPoint, double phase);
    bool moveStanceLeg(int leg);
    Matrix<double, 4,4> leginversekinematics();

    void getposition(double *positon_input);
    void setposition(double *positon_output);
    void calcMotorPosition();

};