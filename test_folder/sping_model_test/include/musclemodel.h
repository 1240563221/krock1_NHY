#ifndef __MUSCLE_MODEL_H__
#define __MUSCLE_MODEL_H__

#include "control_table_constants.hpp"

typedef struct 
{
    double m_x;
	double m_y;
	double m_z;

	double m_r;
	double m_K;
	double m_D;
	double m_ref;
	double m_I;
	double m_m;
	double m_L;
	double m_G;
	double m_torque;
    double m_Fext;
}muscleModelTPDF;


class Fourth_RK_CLASS
{
public:
	double m_x;
	double m_y;
	double m_z;

    muscleModelTPDF muscleParameters[P_MAX_ID];

	Fourth_RK_CLASS(int n, double h);
	//y' = z	y'' = 2*r**2*(Ky+Dy')  ---->  z' = 2*r**2*(Ky + Dz)
	double fun_z(double t, double y, double z);
	double Fourth_RK_CLASS::fun_y(double t, double y, double z, int index);
	void RKfun(double *input, double *torque);

private:
	double m_h;
	int m_n;

};



#endif