#ifndef __MUSCLE_MODEL_H__
#define __MUSCLE_MODEL_H__

#include "control_table_constants.hpp"

typedef struct 
{
	double K;
	double D;
	double refLocation;
	double I;
	double m;
	double L;
	double G;
}muscleModelTPDF;


class Fourth_RK_CLASS
{
public:
	double currentTorque[P_MAX_ID];
    muscleModelTPDF muscleParameters[P_MAX_ID];

	Fourth_RK_CLASS(int n, double h, muscleModelTPDF *param, int index);
	double fun_z(double t, double y, double z);
	double fun_y(double t, double y, double z, int index);
	void RKfun(double *y, double *z, double *out_y, int RKfun_index); //y' = z	y'' = 2*r**2*(Ky+Dy')  ---->  z' = 2*r**2*(Ky + Dz)
	void calculateMuscleOutput(double *inputY, double *inputZ, double *outputY, double *inputTorque, int numbers);

	int getNumber(void);

private:

	double m_h;
	int m_n;

};

#endif