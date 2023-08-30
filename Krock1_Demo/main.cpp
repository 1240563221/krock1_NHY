#include "robot.h"

int id[3]={1,2,3};
double present_position[P_MAX_ID]={0};
int a=0;
int main(void)
{

    Robot robot(3, 57600, id, "/dev/ttyUSB1");
    double goal_position[P_MAX_ID]={0};

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    bool dxl_getdata_result = false; 

    robot.Anglelimits(0, 4095, id);
    robot.setAllJointPositionControlMode(id);
    robot.CurrentLimit((float)100, id);

    while(1)
    {
        printf("Please input a number(0-4096):\n");
        scanf("%d",&a);
        if(a<-4096 || a>4096)
        {
            printf("The number out of limit!  Please input again!\n");
            continue;
        }
        else
        {
          
            // Add Dynamixel#1 goal position value to the Syncwrite storage
            for(uint8_t i=0; i<=P_MAX_ID; i++)
            {
                goal_position[i-1] = a;
                cout << "a = " << a << endl;
            }

            robot.setAllPositions(goal_position, id);
        }

        do
        {
            /* code */

            robot.getAllPositions(present_position, id);
            for (uint8_t i = 1; i <= P_MAX_ID; i++)
            {
                /* code */
                printf("[ID:%03d] Present Position : %.3f \t  Goal Position : %.3f\n", i, present_position[i-1], goal_position[i-1]);
            
            }
            sleep(0.2);
            
            printf("------------------%.3f-----------------------\n", present_position[0]);
        } while (abs(a-present_position[0]) > 20);
        
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