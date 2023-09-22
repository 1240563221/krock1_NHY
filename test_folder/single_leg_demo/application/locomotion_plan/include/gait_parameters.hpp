#ifndef __GAIT_PARAMETERS_H__
#define __GAIT_PARAMETERS_H__

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <stdlib.h>
#include "yaml-cpp/yaml.h"

using namespace std;

struct gaitParams {
    double freq_walk;
  	double Duty;
    Eigen::Matrix<double, 4, 1> phShifts;
	Eigen::Matrix<double, 3, 4> midstance;
	double ellipse_a, ellipse_b;
	Eigen::Vector4d swing_height, swing_width;
	double spineCPGscaling;
    // Eigen::Matrix<double, 3, 4> nSurf;
    // Eigen::Matrix<double, 3, 4> nLO, nTD;
	// Eigen::Matrix<double, 3, 2>  bezierScaling;
	// double tSclSwing;
 	Eigen::Matrix<double, 4, 4> qNULL;
};

// void parameterConfig(gaitParams *GP);

#endif