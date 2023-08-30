/*****************************************************************************
* Project: AgnathaX robot
* Copyright 2021-2023 Laura Paez Coy and Kamilo Melo
* This code is under MIT licence, you can find the complete file here: https://opensource.org/licenses/MIT
* Laura.Paez@KM-RoBota.com, 04-2023
* Kamilo.Melo@KM-RoBota.com, 04-2023
*****************************************************************************/

#include "utils.hpp"


// using namespace std;

#ifndef PI
#define PI 3.14159f
#endif

/**
 * @brief angle2Position
 * Converts angle input into position data (based on motor model provided) code only for 1 model
 * @param angle Angle to be converted
 * @param Motor model, as encoded on Dynamixel motor's control table
 * @param isRadians Boolean flag for unit used in expressing the angle, true if radians (radians by default)
 * @return Position value corresponding to the joint angle (mid-position is 0 degrees/rad)
 */
int angle2Position(double angle, int model, bool isRadians)
{
	int position=2048;
    double angleInDegree = isRadians ? angle*180/PI : angle;
    if (model== 1030 || model== 1000 || model== 311){
    	int Model_max_position=4095;
    	int range=360;
		position = (angleInDegree + range/2) * (double)Model_max_position/range + 0.5;
		if (position > Model_max_position)
			position = Model_max_position;
		else if (position < 0)
			position = 0;    	
    }

    return position;
}

/**
 * @brief position2Angle
 * Converts position input into angle data (based on motor model provided) code only for 1 model
 * @param position to be converted
 * @param Motor model, as encoded on Dynamixel motor's control table
 * @param inRadians Boolean flag for unit used in expressing the angle, true if desire output is in radians (radians by default)
 * @return angle value corresponding to the position data (mid-position is 0 degrees/rad)
 */

double position2Angle(int position, int model, bool inRadians)
{	
	double angle=0;
	if (model== 1030 || model== 1000 || model== 311){
    	int Model_max_position=4095;
    	double modelCenter=Model_max_position/2.;
    	angle=(((double)position - modelCenter)*3.14159/modelCenter);
	}
    angle = inRadians ? angle : angle*180/PI;
    return angle;
}

/* Read current time */
double get_real_time()
{
    struct timespec real_time;

    if (clock_gettime(CLOCK_REALTIME, &real_time) == -1 )
    {
        perror("clock gettime");
        exit( EXIT_FAILURE );
    }

    return((double)real_time.tv_sec + (double)(real_time.tv_nsec/BILLION));
}

/* Read current time - this one is being used*/
double get_timestamp()
{
    struct timeval now;
    gettimeofday (&now, NULL);
    return  (now.tv_usec + (unsigned long long)now.tv_sec * 1000000)/1000000.;
}

/* PT1 filter for scalars */
double
pt1(double u, double y, double T, double Ts)
{
    y=exp(-Ts/T)*y+(1-exp(-Ts/T))*u;
    return y;
}
