#include "single_leg_controller.hpp"

void Locomotion_controller::run_step(double* input, double* output, double t)
{
    getposition(input);
    forward_kinematics();
    legPhaseDynamics(t);
    leg_traj_generator();
    //确认电机位置q_input
    calcMotorPosition();
    setposition(output);

    get_traj_log();
    
}