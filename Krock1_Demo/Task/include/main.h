#ifndef __MAIN_H__
#define __MAIN_H__

#include "robot.h"
#include "gamepad.hpp"


#define MAX_JOYSTICK_VALUE  32767
#define M_2PI   3.1415925

typedef struct 
{
    /* data */
    float offset_gain;
    float amp_gain;
    float freq_gain;
}
robotParameter;



#endif