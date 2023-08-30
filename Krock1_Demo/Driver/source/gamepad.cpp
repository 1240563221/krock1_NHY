/*****************************************************************************
* Project: AgnathaX robot
* Copyright 2021-2023 Laura Paez Coy and Kamilo Melo
* This code is under MIT licence: https://opensource.org/licenses/MIT
* Forked from https://www.keithlantz.net/2011/10/a-linux-c-joystick-object/
* Laura.Paez@KM-RoBota.com, 04-2023
* Kamilo.Melo@KM-RoBota.com, 05-2023
*****************************************************************************/

#include "gamepad.hpp"

// Constructor
cGamepad::cGamepad()
{
    active = false;
    gamepad_fd = 0;
    gamepad_ev = new js_event();
    gamepad_st = new gamepad_state();
    gamepad_fd = open(GAMEPAD_DEV, O_RDONLY | O_NONBLOCK);
    if (gamepad_fd > 0) {
        ioctl(gamepad_fd, JSIOCGNAME(256), name);
        ioctl(gamepad_fd, JSIOCGVERSION, &version);
        ioctl(gamepad_fd, JSIOCGAXES, &axes);
        ioctl(gamepad_fd, JSIOCGBUTTONS, &buttons);
        std::cout << "   Name: " << name << std::endl;
        std::cout << "Version: " << version << std::endl;
        std::cout << "   Axes: " << (int)axes << std::endl;
        std::cout << "Buttons: " << (int)buttons << std::endl;
        gamepad_st->axis.reserve(axes);                             //modify the maximum number of gamepad_st->axis
        gamepad_st->button.reserve(buttons);                        //modify the maximum number of gamepad_st->button
        active = true;
        pthread_create(&thread, 0, &cGamepad::loop, this);          //create a thread to run -- cGamepad::loop
    }
}

// Destructor
cGamepad::~cGamepad()
{
    if (gamepad_fd > 0) {
        active = false;
        pthread_join(thread, 0);                                    //block,waiting for thread end
        close(gamepad_fd);
    }
    delete gamepad_st;
    delete gamepad_ev;
    gamepad_fd = 0;
}

// Loop to read events form the device
void* cGamepad::loop(void *obj)
{
    while (reinterpret_cast<cGamepad *>(obj)->active)
        reinterpret_cast<cGamepad *>(obj)->readEv();
    return 0;
}

void cGamepad::readEv()
{
    int bytes = read(gamepad_fd, gamepad_ev, sizeof(*gamepad_ev));
    if (bytes > 0) {
        gamepad_ev->type &= ~JS_EVENT_INIT;                                 //clear gamepad_ev->type bit 8
        if (gamepad_ev->type & JS_EVENT_BUTTON) {                           //judge gamepad_ev->type bit 1  ------- Button
            gamepad_st->button[gamepad_ev->number] = gamepad_ev->value;     //value to gamepad_st->button[gamepad_ev->number]
        }
        if (gamepad_ev->type & JS_EVENT_AXIS) {                             //judge gamepad_ev->type bit 2  ------- AXIS
            gamepad_st->axis[gamepad_ev->number] = gamepad_ev->value;
        }
    }
}

// Method to query the state of an axis
axis_position cGamepad::axisPosition(int n)
{
    axis_position pos;
    // Horizontal left stick axis 0. left to right -32767 0 +32767
    pos.offset_js = gamepad_st->axis[0];
    // Vertical right stick axis 4. up to down -32767 0 +32767
    pos.amp_js = gamepad_st->axis[4];
    // R2 button axis 5. unpress to fully pressed -32767 +32767
    pos.freq_ax = gamepad_st->axis[5];
    return pos;
}

// Method to query the state of a button
button_pressed cGamepad::buttonPressed(int n)
{
    button_pressed btn;
    // Create button 8. Toggle play and pause
    btn.stop_btn=gamepad_st->button[8];
    // Options button 9. Toggle start and stop
    btn.pause_btn=gamepad_st->button[9];
    // PlayStation button 10. Shutdown the robot // Never used
    // pos.shutdown_btn=gamepad_st->button[10];
    // return n > -1 && n < buttons ? gamepad_st->button[n] : 0;
    return btn;
}
