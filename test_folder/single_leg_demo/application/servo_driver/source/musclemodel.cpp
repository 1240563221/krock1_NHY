/** 
* @file [musclemodel.cpp]
* @brief 
* @author <chen chen>
* @date <2023.9.21>
* @version <V1.0>
*
* detailed description for the file.
*/

#include "musclemodel.h"
#include <iostream>
#include <cmath>
using namespace std;


/**  
*   @brief      the construct of class
*   @param-in   set the stride length
*   @param-in   set the parameters of the virtual muscle model
*   @return     none 
    */
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

/**  
*   @brief      define a function z;   z=y'
*   @param-in   the variable t
*   @param-in   the variable y
*   @param-in   the variable z
*   @return     the value of z 
    */
double VAAM_Model_CLASS::fun_z(double t, double y, double z)
{
    return z;
}

/**  
*   @brief      define a function y;   I*y'' = -mg*sin(delta_y)*L + K*delta_y + D*(-y')
*   @param-in   the variable t
*   @param-in   the variable y
*   @param-in   the variable z
*   @return     the value of z 
    */
double VAAM_Model_CLASS::fun_y(double t, double y, double z)
{
    double T_k, T_d;
    T_k = (muscleParameters.refLocation-y) * muscleParameters.K / muscleParameters.I;
    T_d = -z*muscleParameters.D / muscleParameters.I;

    return -muscleParameters.G * (1. / muscleParameters.L) * sin(y-muscleParameters.refLocation) - currentTorque  + T_k + T_d;
}

/**  
*   @brief      the fourth RK differential solver
*   @param-in   the position y
*   @param-in   the velocity z
*   @param-out  the modify position out_y
*   @param-in   the external force
*   @return     none 
    */
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


/**  
*   @brief      get the number of times the differential solver 
*   @return     the number of times the differential solver 
    */
int VAAM_Model_CLASS::getNumber(void)
{
    return m_n;
}

/**  
*   @brief      get the stride length of the differential solver 
*   @return     the stride length of the differential solver 
    */
double VAAM_Model_CLASS::getCalculateStride(void)
{
    return m_h;
}

/**  
*   @brief      get the current goal position(reference position) of the virtual muscle model 
*   @return     the current goal position(reference position) of the virtual muscle model 
    */
double VAAM_Model_CLASS::getGoalPosition(void)
{
    return muscleParameters.refLocation;
}

/**  
*   @brief      set the external force of the virtual muscle model 
*   @param-in   the actual external force of the model
*   @return     none
    */
void VAAM_Model_CLASS::setTorque(double extTorque)
{
    currentTorque = extTorque * TORQUE_COEFFICIENT;
}

/**  
*   @brief      set the stride length of the differential solver 
*   @param-in   the desired stride length of the differential solver
*   @return     none
    */
void VAAM_Model_CLASS::setCalculateStride(double timeInterval)
{
    m_h = timeInterval;
}

/**  
*   @brief      set the current goal position(reference position) of the virtual muscle model 
*   @param-in   the desired goal position(reference position) of the virtual muscle model
*   @return     none    
    */
void VAAM_Model_CLASS::setGoalPosition(double goalPosition)
{
    muscleParameters.refLocation = goalPosition;
}

/**  
*   @brief      show all the parameter values of the muscle model 
*   @return     none
    */
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