/*****************************************************************************
* Project: AgnathaX robot
* Copyright 2021-2023 Laura Paez Coy and Kamilo Melo
* This code is under MIT licence: https://opensource.org/licenses/MIT
* Forked from https://www.keithlantz.net/2011/10/a-linux-c-joystick-object/
* Laura.Paez@KM-RoBota.com, 04-2023
* Kamilo.Melo@KM-RoBota.com, 05-2023
*****************************************************************************/

#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <iostream>
#include <fcntl.h>
#include <pthread.h>
#include <math.h>
#include <linux/joystick.h>
#include <vector>
#include <unistd.h>

#define GAMEPAD_DEV "/dev/input/js0"

// Structure for holding the state
struct gamepad_state {
    std::vector<signed short> button;
    std::vector<signed short> axis;
};

// Structure for returning the events (axesPosition)
struct axis_position {
    float offset_js,
          amp_js,
          freq_ax;
};

// Structure for returning the events (buttonPress)
struct button_pressed {
    bool stop_btn,
         pause_btn;
         // shutdown_btn;
};

// Declaration of the joystick object
class cGamepad {
private:
    pthread_t thread;
    bool active;
    int gamepad_fd;
    js_event *gamepad_ev;
    // gamepad_state *gamepad_st;
    __u32 version;
    __u8 axes;
    __u8 buttons;
    char name[256];

protected:
public:
    gamepad_state *gamepad_st;
    cGamepad();
    ~cGamepad();
    static void* loop(void* obj);
    void readEv();
    axis_position axisPosition(int n);
    button_pressed buttonPressed(int n);
};

#endif
