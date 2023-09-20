#include "musclemodel.h"
#include <iostream>
#include <cmath>
using namespace std;


VAAM_Model_CLASS::VAAM_Model_CLASS( double h, muscleModelTPDF param)
{
    m_n = 1;
    m_h = h;

    muscleParameters.D = param.D;
    muscleParameters.K = param.K;
    muscleParameters.m = param.m;
    muscleParameters.L = param.L;
    muscleParameters.G = param.G;
    muscleParameters.refLocation = param.refLocation;
    muscleParameters.I = muscleParameters.m * muscleParameters.L * muscleParameters.L;
}

double VAAM_Model_CLASS::fun_z(double t, double y, double z)
{
    return z;
}

double VAAM_Model_CLASS::fun_y(double t, double y, double z)
{
    double T_k, T_d;
    T_k = (muscleParameters.refLocation-y) * muscleParameters.K / muscleParameters.I;
    T_d = -z*muscleParameters.D / muscleParameters.I;

    return -muscleParameters.G * (1. / muscleParameters.L) * sin(y-muscleParameters.refLocation) - currentTorque  + T_k + T_d;
}

void VAAM_Model_CLASS::RKfun(double y, double z, double *out_y, double inputTorque)
{
    double K1y, K2y, K3y, K4y, K1z, K2z, K3z, K4z;

    double y_next = y;
    double z_next = z;

    double x_used, y_used, z_used;
    
    setTorque(inputTorque);

    for (int i = 0; i < m_n; i++)
    {
        x_used = 0;
        y_used = y_next;
        z_used = z_next;

        K1z = fun_z(x_used, y_used,  z_used);
        K1y = fun_y(x_used, y_used,  z_used);
        K2z = fun_z(x_used + m_h/2., y_used + K1z * m_h/2., z_used + K1y * m_h/2.);
        K2y = fun_y(x_used + m_h/2., y_used + K1z * m_h/2., z_used + K1y * m_h/2.);
        K3z = fun_z(x_used + m_h/2., y_used + K2z * m_h/2., z_used + K2y * m_h/2.);
        K3y = fun_y(x_used + m_h/2., y_used + K2z * m_h/2., z_used + K2y * m_h/2.);
        K4z = fun_z(x_used + m_h, y_used + K3z * m_h, z_used + K3y * m_h);
        K4y = fun_y(x_used + m_h, y_used + K3z * m_h, z_used + K3y * m_h);

        y_next = y_used + m_h/6.0 * (K1z + 2*K2z + 2*K3z + K4z);
        z_next = z_used + m_h/6.0 * (K1y + 2*K2y + 2*K3y + K4y);    
    }
    *out_y = y_next;
}


int VAAM_Model_CLASS::getNumber(void)
{
    return m_n;
}

double VAAM_Model_CLASS::getCalculateStride(void)
{
    return m_h;
}

double VAAM_Model_CLASS::getGoalPosition(void)
{
    return muscleParameters.refLocation;
}

void VAAM_Model_CLASS::setTorque(double extTorque)
{
    currentTorque = extTorque * TORQUE_COEFFICIENT;
}

void VAAM_Model_CLASS::setCalculateStride(double timeInterval)
{
    m_h = timeInterval;
}

void VAAM_Model_CLASS::setGoalPosition(double goalPosition)
{
    muscleParameters.refLocation = goalPosition;
}

void VAAM_Model_CLASS::showMuscleParameters(void)
{
    cout << "muscle_K" << muscleParameters.K << endl;
    cout << "muscle_D" << muscleParameters.D << endl;
    cout << "muscle_m" << muscleParameters.m << endl;
    cout << "muscle_L" << muscleParameters.L << endl;
    cout << "muscle_I" << muscleParameters.I << endl;
    cout << "muscle_G" << muscleParameters.G << endl;
    cout << "muscle_refLocation" << muscleParameters.refLocation << endl;
}