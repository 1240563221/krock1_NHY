#include "single_leg_controller.hpp"

void Locomotion_controller::variable_initial()
{
    dt = TIME_STEP;
    Tf1 = 300;
    maxSpeed = 0.3;

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
}

void Locomotion_controller::constant_initial()
{
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
    constrS = 30.;
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

    constrHL << -103., -75., -190., -75.,
                103.,  75.,	100., 20.;
}