/** 
* @file [musclemodel.h]
* @brief 
* @author <chen chen>
* @date <2023.9.21>
* @version <V1.0>
*
* detailed description for the file.
*/
#ifndef __MUSCLE_MODEL_H__
#define __MUSCLE_MODEL_H__	

#include "control_table_constants.hpp"

#define TORQUE_COEFFICIENT 			0.03
#define MAX_LIMIT_POSITION_SERVO 	4095
#define TIME_INTERVAL				0.047


//the struct of muscle model
typedef struct 
{
	double K;				//stiffness
	double D;				//damper
	double refLocation;		//the stable location of muscle model
	double I;				//moment of inertia
	double m;				//object mass
	double L;				//the arm of force
	double G;				//acceleration of gravity
}muscleModelTPDF;


//the class of vaam model
class VAAM_Model_CLASS
{
public:
	//construct function
	VAAM_Model_CLASS( double h, muscleModelTPDF param);
	double fun_z(double t, double y, double z);
	double fun_y(double t, double y, double z);
	
	//fourth RK solver 
	void RKfun(double y, double z, double *out_y, double inputTorque); //y' = z	y'' = 2*r**2*(Ky+Dy')  ---->  z' = 2*r**2*(Ky + Dz)

	// get parameters
	int getNumber(void);
	double getCalculateStride(void);
	double getGoalPosition(void);

	//set parameters
	void setTorque(double extTorque);
	void setCalculateStride(double timeInterval);
	void setGoalPosition(double goalPosition);

	//show the parameters of muscle model
	void showMuscleParameters(void);

private:
	int m_n;							//the number of times the differential solver is called
	double m_h;							//the stride length of the differential solver
	double currentTorque;				//the external force
	muscleModelTPDF muscleParameters;
};

#endif