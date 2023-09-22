/*****************************************************************************
* Project: AgnathaX robot
* Copyright 2021-2023 Laura Paez Coy and Kamilo Melo
* This code is under MIT licence, you can find the complete file here: https://opensource.org/licenses/MIT
* Laura.Paez@KM-RoBota.com, 04-2023
* Kamilo.Melo@KM-RoBota.com, 04-2023
*****************************************************************************/

/* Robot UTILITIES */
#ifndef UTILS_HPP
#define UTILS_HPP
#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h> 


#define BILLION          1000000000.0
#define MILLION          1000000.0

// #include <string>

using namespace std;

int angle2Position(double angle, int model, bool isRadian = true);
double position2Angle(int position, int model, bool inRadian = true);
double get_real_time();
double get_timestamp();
double pt1(double u, double y, double T, double Ts);

#endif 
