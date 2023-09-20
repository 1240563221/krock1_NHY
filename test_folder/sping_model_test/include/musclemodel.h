#ifndef __MUSCLE_MODEL_H__
#define __MUSCLE_MODEL_H__

#include "control_table_constants.hpp"

#define TORQUE_COEFFICIENT 			0.03
#define MAX_LIMIT_POSITION_SERVO 	4095
#define TIME_INTERVAL				0.047


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


class VAAM_Model_CLASS
{
public:
	VAAM_Model_CLASS( double h, muscleModelTPDF param);
	double fun_z(double t, double y, double z);
	double fun_y(double t, double y, double z);
	void RKfun(double y, double z, double *out_y, double inputTorque); //y' = z	y'' = 2*r**2*(Ky+Dy')  ---->  z' = 2*r**2*(Ky + Dz)

	// get parameters
	int getNumber(void);
	double getCalculateStride(void);
	double getGoalPosition(void);

	//set parameters
	void setTorque(double extTorque);
	void setCalculateStride(double timeInterval);
	void setGoalPosition(double goalPosition);

	void showMuscleParameters(void);

private:
	int m_n;
	double m_h;
	double currentTorque;
	muscleModelTPDF muscleParameters;
};

#endif