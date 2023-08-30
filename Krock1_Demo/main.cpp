#include "robot.h"
#include "gamepad.hpp"

int id[3]={1,2,3};
double present_position[P_MAX_ID]={0};
int a=0;
int main(void)
{
    if (true) {
        int fd = open(GAMEPAD_DEV, O_NONBLOCK);
        if (fd == -1)
            std::cerr << "Failed to open gamepad device" << std::endl;
    }
    cGamepad gp; axis_position ap[1]; button_pressed bp[1];

    cout << "normal running!" << endl;
    while(1)
    {
        cout << " button[0] = " << gp.gamepad_st->button[0] <<
                " button[1] = " << gp.gamepad_st->button[1] <<
                " button[2] = " << gp.gamepad_st->button[2] <<
                " button[3] = " << gp.gamepad_st->button[3] <<
                " button[4] = " << gp.gamepad_st->button[4] <<
                " button[5] = " << gp.gamepad_st->button[5] <<
                " button[6] = " << gp.gamepad_st->button[6] <<
                " button[7] = " << gp.gamepad_st->button[7] <<
                " button[8] = " << gp.gamepad_st->button[8] <<
                " button[9] = " << gp.gamepad_st->button[9] <<
                " button[10] = " << gp.gamepad_st->button[10] <<
                " button[11] = " << gp.gamepad_st->button[11] <<
                " button[12] = " << gp.gamepad_st->button[12] <<
                " button[13] = " << gp.gamepad_st->button[13] << endl;

        cout << " axis[0] = " << gp.gamepad_st->axis[0] <<
                " axis[1] = " << gp.gamepad_st->axis[1] <<
                " axis[2] = " << gp.gamepad_st->axis[2] <<
                " axis[3] = " << gp.gamepad_st->axis[3] <<
                " axis[4] = " << gp.gamepad_st->axis[4] <<
                " axis[5] = " << gp.gamepad_st->axis[5] <<
                " axis[6] = " << gp.gamepad_st->axis[6] <<
                " axis[7] = " << gp.gamepad_st->axis[7] << endl;

        sleep(0.3);
        // printf("Please input a number(0-4096):\n");
        // scanf("%d",&a);
        // if(a<-4096 || a>4096)
        // {
        //     printf("The number out of limit!  Please input again!\n");
        //     continue;
        // }
        // else
        // {
          
        //     // Add Dynamixel#1 goal position value to the Syncwrite storage
        //     for(uint8_t i=0; i<=P_MAX_ID; i++)
        //     {
        //         goal_position[i-1] = a;
        //         cout << "a = " << a << endl;
        //     }

        //     robot.setAllPositions(goal_position, id);
        // }

        // do
        // {
        //     /* code */

        //     robot.getAllPositions(present_position, id);
        //     for (uint8_t i = 1; i <= P_MAX_ID; i++)
        //     {
        //         /* code */
        //         printf("[ID:%03d] Present Position : %.3f \t  Goal Position : %.3f\n", i, present_position[i-1], goal_position[i-1]);
            
        //     }
        //     sleep(0.2);
            
        //     printf("------------------%.3f-----------------------\n", present_position[0]);
        // } while (abs(a-present_position[0]) > 20);
        
        // for(uint8_t j=0; j<=50; j++)
        // {
        //     /* code */

        //     robot.getAllCurrents(present_position, id);
        //     for (uint8_t i = 1; i <= P_MAX_ID; i++)
        //     {
        //         /* code */
        //         printf("[ID:%03d] Present Current : %.3f \t  Goal Current : %.3f\n", i, present_position[i-1], goal_position[i-1]);
            
        //     }
        //     sleep(0.4);
            
        //     printf("------------------%.3f-----------------------\n", present_position[0]);
        // } 
    }
}