#include "musclemodel.h"
#include <iostream>
#include <cmath>
using namespace std;


Fourth_RK_CLASS::Fourth_RK_CLASS(int n, double h, muscleModelTPDF *param, int index)
{
    m_n = n;
    m_h = h;

    for (int i = 0; i < index; i++)
    {
        muscleParameters[i].D = param[i].D;
        muscleParameters[i].K = param[i].K;
        muscleParameters[i].m = param[i].m;
        muscleParameters[i].L = param[i].L;
        muscleParameters[i].G = param[i].G;
        muscleParameters[i].refLocation = param[i].refLocation;
        muscleParameters[i].I = muscleParameters[i].m * muscleParameters[i].L * muscleParameters[i].L;
    }

}

double Fourth_RK_CLASS::fun_z(double t, double y, double z)
{
    return z;
}

double Fourth_RK_CLASS::fun_y(double t, double y, double z, int index)
{
    double T_s1=0, T_s2=0, T_spring=0,D1=0,D2=0,D=0;
    if (muscleParameters[index].refLocation < y)
    {
        T_s1 = (muscleParameters[index].refLocation-y) * muscleParameters[index].K;
    }
        
    if (muscleParameters[index].refLocation > y)
    {
        T_s2 = (muscleParameters[index].refLocation-y) * muscleParameters[index].K;
    }
    T_spring = (T_s1+T_s2) / muscleParameters[index].I;
    
    if (z > 0)
    {
        D1 = -z*muscleParameters[index].D;
    }
    if (z < 0)
    {
        D2 = -z*muscleParameters[index].D;
    }  
    D = (D1 + D2) / muscleParameters[index].I;
    return -muscleParameters[index].G * (1. / muscleParameters[index].L) * sin(y-muscleParameters[index].refLocation) - currentTorque[index]*0.03  + T_spring + D;
}

void Fourth_RK_CLASS::RKfun(double *y, double *z, double *out_y, int RKfun_index)
{
    double K1y, K2y, K3y, K4y, K1z, K2z, K3z, K4z;
    K1y = K2y = K3y = K4y = K1z = K2z = K3z = K4z = 0.0;

    double y_next = y[RKfun_index];
    double z_next = z[RKfun_index];

    double x_used, y_used, z_used;

    for (int i = 0; i < m_n; i++)
    {
        x_used = 0;
        y_used = y_next;
        z_used = z_next;

        K1z = fun_z(x_used, y_used,  z_used);
        K1y = fun_y(x_used, y_used,  z_used, RKfun_index);
        K2z = fun_z(x_used + m_h/2., y_used + K1z * m_h/2., z_used + K1y * m_h/2.);
        K2y = fun_y(x_used + m_h/2., y_used + K1z * m_h/2., z_used + K1y * m_h/2., RKfun_index);
        K3z = fun_z(x_used + m_h/2., y_used + K2z * m_h/2., z_used + K2y * m_h/2.);
        K3y = fun_y(x_used + m_h/2., y_used + K2z * m_h/2., z_used + K2y * m_h/2., RKfun_index);
        K4z = fun_z(x_used + m_h, y_used + K3z * m_h, z_used + K3y * m_h);
        K4y = fun_y(x_used + m_h, y_used + K3z * m_h, z_used + K3y * m_h, RKfun_index);

        y_next = y_used + m_h/6.0 * (K1z + 2*K2z + 2*K3z + K4z);
        z_next = z_used + m_h/6.0 * (K1y + 2*K2y + 2*K3y + K4y);    
    }
    out_y[RKfun_index] = y_next;
}

void Fourth_RK_CLASS::calculateMuscleOutput(double *inputY, double *inputZ, double *outputY, double *inputTorque, int numbers)
{
    for (int  i = 0; i < numbers; i++)
    {
        currentTorque[i] = inputTorque[i];
        RKfun(inputY, inputZ, outputY, i);
    }
}